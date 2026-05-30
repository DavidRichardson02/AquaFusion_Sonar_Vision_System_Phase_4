`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// oled_telemetry_pack_sys
//------------------------------------------------------------------------------
// PURPOSE
//   Build four fixed-width ASCII telemetry lines in the SYS clock domain for
//   later rendering by an OLED text console.
//
// HIGH-LEVEL ROLE
//   This module is a formatting / presentation block, not a sensor-processing
//   block. Its job is to take already-available telemetry/state signals and
//   convert them into four packed ASCII line buses:
//
//       line0_ascii
//       line1_ascii
//       line2_ascii
//       line3_ascii
//
//   Each line contains LINE_CHARS characters, packed one ASCII byte per
//   character position.
//
//   The formatted lines are intended to feed a downstream text renderer such as
//   an SSD1306/OLED page-byte generator.
//
// DISPLAY / TEXT MODEL
//   - Four text rows are produced.
//   - Each row is exactly LINE_CHARS characters wide.
//   - Character 0 is stored in bits [7:0] of the corresponding line bus.
//   - Character 1 is stored in bits [15:8].
//   - And so on.
//
// EXAMPLE OUTPUT FORMS
//   Row 0: "S1 123IN A005 OK    "
//   Row 1: "S2 087IN A014 ST    "
//   Row 2: "CAM BUSY IDOK E00   "
//   Row 3: "L1 H1 HB0 F0123     "
//
// UPDATE POLICY
//   The line outputs are refreshed under two conditions:
//
//     1) Immediate event-driven refresh
//        If any tracked telemetry input changes, the four output lines are
//        republished immediately on the next rising clock edge.
//
//     2) Periodic refresh
//        Even if no telemetry state changes, the lines are republished at
//        REFRESH_HZ. This is useful for display paths that benefit from a
//        periodic "still alive" update pulse.
//
//   On each publish event, the module asserts:
//
//       line_upd = 1
//
//   for exactly one clock cycle.
//
// ARCHITECTURAL ORGANIZATION
//   The design is partitioned into three conceptual layers:
//
//     A) Telemetry-state capture abstraction
//        Current input fields are packed into one comparison vector,
//        telemetry_state_now.
//
//     B) Combinational formatting
//        line0_next..line3_next are built from current inputs.
//
//     C) Sequential publication
//        The next-line buses are copied into registered outputs when either:
//          - telemetry_state_now differs from telemetry_state_prev, or
//          - the periodic refresh counter expires
//
// DESIGN STYLE
//   - Combinational next-value generation
//   - Registered output publication
//   - One-cycle update pulse
//   - Deterministic bounded work per clock
//
// IMPORTANT SCOPE NOTE
//   This block formats telemetry for human visibility. It does not perform any
//   filtering, debouncing, validity derivation, or error latching beyond what
//   is already represented in its inputs.
//
//==============================================================================
module oled_telemetry_pack_sys #(
    parameter integer CLK_HZ      = 100_000_000,
    parameter integer REFRESH_HZ  = 5,
    parameter integer LINE_CHARS  = 21
)(
    input  wire                      clk,
    input  wire                      rst,

    //--------------------------------------------------------------------------
    // Sonar 1 telemetry inputs
    //--------------------------------------------------------------------------
    input  wire [9:0]                sonar1_distance_in,
    input  wire                      sonar1_valid,
    input  wire                      sonar1_stale,
    input  wire                      sonar1_timeout_err,
    input  wire [15:0]               sonar1_age_ticks,

    //--------------------------------------------------------------------------
    // Sonar 2 telemetry inputs
    //--------------------------------------------------------------------------
    input  wire [9:0]                sonar2_distance_in,
    input  wire                      sonar2_valid,
    input  wire                      sonar2_stale,
    input  wire                      sonar2_timeout_err,
    input  wire [15:0]               sonar2_age_ticks,

    //--------------------------------------------------------------------------
    // Camera subsystem telemetry inputs
    //--------------------------------------------------------------------------
    input  wire                      cam_busy,
    input  wire                      cam_init_done,
    input  wire                      cam_init_fail,
    input  wire                      cam_sensor_id_ok,
    input  wire [7:0]                cam_last_err,

    //--------------------------------------------------------------------------
    // System / display / heartbeat / frame indicators
    //--------------------------------------------------------------------------
    input  wire                      sys_locked,
    input  wire                      hdmi_hpd,
    input  wire                      heartbeat,
    input  wire [15:0]               frame_count_lsb,

    //--------------------------------------------------------------------------
    // Packed ASCII output lines
    //--------------------------------------------------------------------------
    output reg  [(LINE_CHARS*8)-1:0] line0_ascii,
    output reg  [(LINE_CHARS*8)-1:0] line1_ascii,
    output reg  [(LINE_CHARS*8)-1:0] line2_ascii,
    output reg  [(LINE_CHARS*8)-1:0] line3_ascii,

    //--------------------------------------------------------------------------
    // One-cycle update pulse asserted when the line outputs are republished
    //--------------------------------------------------------------------------
    output reg                       line_upd
);

    //--------------------------------------------------------------------------
    // Periodic refresh divider
    //--------------------------------------------------------------------------
    // A refresh event is generated every REFRESH_DIV clock cycles.
    //
    // Example:
    //   CLK_HZ     = 100_000_000
    //   REFRESH_HZ = 5
    //
    // Then:
    //   REFRESH_DIV = 20_000_000
    //
    // meaning a periodic refresh every 0.2 seconds.
    //--------------------------------------------------------------------------
    localparam integer REFRESH_DIV = (CLK_HZ / REFRESH_HZ);

    //--------------------------------------------------------------------------
    // Flattened telemetry comparison state width
    //--------------------------------------------------------------------------
    // This vector is used only to detect whether any tracked telemetry field
    // has changed since the previous published state.
    //
    // Field breakdown:
    //   sonar1: 10 + 1 + 1 + 1 + 16 = 29
    //   sonar2: 10 + 1 + 1 + 1 + 16 = 29
    //   camera: 1 + 1 + 1 + 1 + 8   = 12
    //   flags : 1 + 1 + 1           = 3
    //   frame : 16
    //   total = 89
    //--------------------------------------------------------------------------
    localparam integer TELEMETRY_STATE_W = 89;

    //--------------------------------------------------------------------------
    // Periodic refresh counter
    //--------------------------------------------------------------------------
    reg [31:0] refresh_ctr;

    //--------------------------------------------------------------------------
    // Flattened current and previous telemetry-state vectors
    //--------------------------------------------------------------------------
    // telemetry_state_now
    //   Combinational snapshot of current inputs.
    //
    // telemetry_state_prev
    //   Last state that was actually published to the output line registers.
    //
    // Their comparison drives event-based refresh.
    //--------------------------------------------------------------------------
    reg [TELEMETRY_STATE_W-1:0] telemetry_state_now;
    reg [TELEMETRY_STATE_W-1:0] telemetry_state_prev;

    //--------------------------------------------------------------------------
    // Combinational next-value line buffers
    //--------------------------------------------------------------------------
    // These are the formatted line values derived from the current inputs.
    // They are copied into the registered outputs only when a publish event
    // occurs.
    //--------------------------------------------------------------------------
    reg [(LINE_CHARS*8)-1:0] line0_next;
    reg [(LINE_CHARS*8)-1:0] line1_next;
    reg [(LINE_CHARS*8)-1:0] line2_next;
    reg [(LINE_CHARS*8)-1:0] line3_next;

    //--------------------------------------------------------------------------
    // Local formatted / saturated intermediate fields
    //--------------------------------------------------------------------------
    // Distances and ages are clamped to fit the visible decimal field widths.
    //--------------------------------------------------------------------------
    reg [9:0]  s1_dist_sat;
    reg [9:0]  s2_dist_sat;
    reg [15:0] s1_age_sat;
    reg [15:0] s2_age_sat;

    //--------------------------------------------------------------------------
    // Two-character sonar status abbreviations
    //--------------------------------------------------------------------------
    // Examples:
    //   "OK" = valid and not stale/timeout
    //   "ST" = stale
    //   "TO" = timeout
    //   "--" = neither valid nor flagged
    //--------------------------------------------------------------------------
    reg [7:0] s1_stat0;
    reg [7:0] s1_stat1;
    reg [7:0] s2_stat0;
    reg [7:0] s2_stat1;

    //--------------------------------------------------------------------------
    // Four-character camera state word
    //--------------------------------------------------------------------------
    // Examples:
    //   "BUSY"
    //   "FAIL"
    //   "DONE"
    //   "INIT"
    //--------------------------------------------------------------------------
    reg [7:0] cam0;
    reg [7:0] cam1;
    reg [7:0] cam2;
    reg [7:0] cam3;

    //--------------------------------------------------------------------------
    // Temporary working registers used during numeric formatting
    //--------------------------------------------------------------------------
    reg [9:0]  dtmp;
    reg [15:0] atmp;
    reg [15:0] ftmp;

    //--------------------------------------------------------------------------
    // Function: asc_digit
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Convert a 4-bit value in the range 0..9 into its ASCII decimal digit.
    //
    // METHOD
    //   ASCII '0' = 8'h30, so:
    //
    //       ASCII digit = 8'h30 + numeric digit
    //
    // IMPORTANT ASSUMPTION
    //   This function is intended to be called only with values 0..9.
    //   If a larger value is supplied, the result will no longer be a valid
    //   decimal digit character.
    //--------------------------------------------------------------------------
    function [7:0] asc_digit;
        input [3:0] val;
        begin
            asc_digit = 8'h30 + val[3:0];
        end
    endfunction

    //--------------------------------------------------------------------------
    // Function: asc_hex
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Convert a 4-bit nibble into its uppercase hexadecimal ASCII character.
    //
    // EXAMPLES
    //   4'h0 -> "0"
    //   4'h9 -> "9"
    //   4'hA -> "A"
    //   4'hF -> "F"
    //--------------------------------------------------------------------------
    function [7:0] asc_hex;
        input [3:0] val;
        begin
            case (val)
                4'h0: asc_hex = "0";
                4'h1: asc_hex = "1";
                4'h2: asc_hex = "2";
                4'h3: asc_hex = "3";
                4'h4: asc_hex = "4";
                4'h5: asc_hex = "5";
                4'h6: asc_hex = "6";
                4'h7: asc_hex = "7";
                4'h8: asc_hex = "8";
                4'h9: asc_hex = "9";
                4'hA: asc_hex = "A";
                4'hB: asc_hex = "B";
                4'hC: asc_hex = "C";
                4'hD: asc_hex = "D";
                4'hE: asc_hex = "E";
                default: asc_hex = "F";
            endcase
        end
    endfunction

    //--------------------------------------------------------------------------
    // Task: set_char
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Write one ASCII character into position `idx` of a packed line bus.
    //
    // PACKING CONVENTION
    //   Character i occupies bits:
    //
    //       [(i*8)+7 : (i*8)]
    //
    // STEP-BY-STEP
    //   If idx = 0:
    //       line_bus[7:0]   = ch
    //
    //   If idx = 1:
    //       line_bus[15:8]  = ch
    //
    //   etc.
    //
    // WHY THIS TASK EXISTS
    //   It hides repetitive packed-bus indexing syntax and makes the line
    //   formatting code read more like string assembly.
    //--------------------------------------------------------------------------
    task set_char;
        inout [(LINE_CHARS*8)-1:0] line_bus;
        input integer idx;
        input [7:0] ch;
        begin
            line_bus[(idx*8)+7 -: 8] = ch;
        end
    endtask

    //--------------------------------------------------------------------------
    // Task: clear_line
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Initialize an entire packed line bus to ASCII spaces.
    //
    // STEP-BY-STEP
    //   1) The line bus is first zeroed.
    //   2) Every character slot is then overwritten with 8'h20 (space).
    //
    // WHY THIS TASK EXISTS
    //   It guarantees that any character positions not explicitly written later
    //   in the formatting logic remain blank rather than undefined.
    //--------------------------------------------------------------------------
    task clear_line;
        output [(LINE_CHARS*8)-1:0] line_bus;
        integer i;
        begin
            line_bus = {(LINE_CHARS*8){1'b0}};
            for (i = 0; i < LINE_CHARS; i = i + 1)
                line_bus[(i*8)+7 -: 8] = 8'h20;
        end
    endtask

    //--------------------------------------------------------------------------
    // Combinational flattening of current telemetry state
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Pack all tracked input fields into a single comparison vector so that
    //   the sequential publication block can detect any telemetry change using
    //   one inequality comparison:
    //
    //       telemetry_state_now != telemetry_state_prev
    //
    // IMPORTANT SEMANTIC NOTE
    //   This vector is not itself an output protocol. It exists only as an
    //   internal change-detection abstraction.
    //--------------------------------------------------------------------------
    always @(*) begin
        telemetry_state_now = {
            sonar1_distance_in[9:0],
            sonar1_valid,
            sonar1_stale,
            sonar1_timeout_err,
            sonar1_age_ticks,

            sonar2_distance_in[9:0],
            sonar2_valid,
            sonar2_stale,
            sonar2_timeout_err,
            sonar2_age_ticks,

            cam_busy,
            cam_init_done,
            cam_init_fail,
            cam_sensor_id_ok,
            cam_last_err,

            sys_locked,
            hdmi_hpd,
            heartbeat,
            frame_count_lsb
        };
    end

    //--------------------------------------------------------------------------
    // Combinational line formatting
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Build the next telemetry lines from the current inputs.
    //
    // ORGANIZATION
    //   The formatting sequence is:
    //
    //     1) Clear all four lines to spaces
    //     2) Saturate numeric fields to visible decimal width
    //     3) Derive short status abbreviations
    //     4) Write Row 0 (Sonar 1)
    //     5) Write Row 1 (Sonar 2)
    //     6) Derive camera 4-character state word
    //     7) Write Row 2 (Camera)
    //     8) Write Row 3 (system / HDMI / heartbeat / frame count)
    //
    // OUTPUTS
    //   line0_next..line3_next are fully determined combinationally.
    //--------------------------------------------------------------------------
    always @(*) begin
        //----------------------------------------------------------------------
        // Start by clearing every line to spaces.
        // This ensures unwritten character positions remain blank.
        //----------------------------------------------------------------------
        clear_line(line0_next);
        clear_line(line1_next);
        clear_line(line2_next);
        clear_line(line3_next);

        //----------------------------------------------------------------------
        // Saturate distance and age values to visible field widths.
        //
        // Distances are displayed using 3 decimal digits:
        //   000..999
        //
        // Age fields are displayed using 3 decimal digits:
        //   000..999
        //
        // Any larger value is clamped so the text formatting remains stable and
        // fixed-width.
        //----------------------------------------------------------------------
        s1_dist_sat = (sonar1_distance_in > 10'd999) ? 10'd999 : sonar1_distance_in;
        s2_dist_sat = (sonar2_distance_in > 10'd999) ? 10'd999 : sonar2_distance_in;
        s1_age_sat  = (sonar1_age_ticks   > 16'd999) ? 16'd999 : sonar1_age_ticks;
        s2_age_sat  = (sonar2_age_ticks   > 16'd999) ? 16'd999 : sonar2_age_ticks;

        //----------------------------------------------------------------------
        // Derive 2-character status abbreviations for Sonar 1.
        //
        // Priority:
        //   timeout_err > stale > valid > unknown
        //
        // Resulting codes:
        //   "TO" timeout
        //   "ST" stale
        //   "OK" valid and healthy
        //   "--" otherwise
        //----------------------------------------------------------------------
        if (sonar1_timeout_err) begin
            s1_stat0 = "T";
            s1_stat1 = "O";
        end else if (sonar1_stale) begin
            s1_stat0 = "S";
            s1_stat1 = "T";
        end else if (sonar1_valid) begin
            s1_stat0 = "O";
            s1_stat1 = "K";
        end else begin
            s1_stat0 = "-";
            s1_stat1 = "-";
        end

        //----------------------------------------------------------------------
        // Derive 2-character status abbreviations for Sonar 2.
        //----------------------------------------------------------------------
        if (sonar2_timeout_err) begin
            s2_stat0 = "T";
            s2_stat1 = "O";
        end else if (sonar2_stale) begin
            s2_stat0 = "S";
            s2_stat1 = "T";
        end else if (sonar2_valid) begin
            s2_stat0 = "O";
            s2_stat1 = "K";
        end else begin
            s2_stat0 = "-";
            s2_stat1 = "-";
        end

        //======================================================================
        // Row 0 formatting
        // Intended appearance:
        //
        //   "S1 123IN A005 OK    "
        //
        // Field breakdown:
        //   [0:1]   "S1"
        //   [3:5]   3-digit distance
        //   [6:7]   "IN"
        //   [9]     "A"
        //   [10:12] 3-digit age
        //   [14:15] 2-character status
        //======================================================================
        set_char(line0_next,  0, "S");
        set_char(line0_next,  1, "1");
        set_char(line0_next,  2, " ");

        dtmp = s1_dist_sat;
        set_char(line0_next,  3, asc_digit((dtmp / 10'd100) % 10));
        set_char(line0_next,  4, asc_digit((dtmp / 10'd10)  % 10));
        set_char(line0_next,  5, asc_digit(dtmp % 10));

        set_char(line0_next,  6, "I");
        set_char(line0_next,  7, "N");
        set_char(line0_next,  8, " ");
        set_char(line0_next,  9, "A");

        atmp = s1_age_sat;
        set_char(line0_next, 10, asc_digit((atmp / 16'd100) % 10));
        set_char(line0_next, 11, asc_digit((atmp / 16'd10)  % 10));
        set_char(line0_next, 12, asc_digit(atmp % 10));

        set_char(line0_next, 13, " ");
        set_char(line0_next, 14, s1_stat0);
        set_char(line0_next, 15, s1_stat1);

        //======================================================================
        // Row 1 formatting
        // Intended appearance:
        //
        //   "S2 087IN A014 ST    "
        //
        // Same structure as Row 0, but for Sonar 2.
        //======================================================================
        set_char(line1_next,  0, "S");
        set_char(line1_next,  1, "2");
        set_char(line1_next,  2, " ");

        dtmp = s2_dist_sat;
        set_char(line1_next,  3, asc_digit((dtmp / 10'd100) % 10));
        set_char(line1_next,  4, asc_digit((dtmp / 10'd10)  % 10));
        set_char(line1_next,  5, asc_digit(dtmp % 10));

        set_char(line1_next,  6, "I");
        set_char(line1_next,  7, "N");
        set_char(line1_next,  8, " ");
        set_char(line1_next,  9, "A");

        atmp = s2_age_sat;
        set_char(line1_next, 10, asc_digit((atmp / 16'd100) % 10));
        set_char(line1_next, 11, asc_digit((atmp / 16'd10)  % 10));
        set_char(line1_next, 12, asc_digit(atmp % 10));

        set_char(line1_next, 13, " ");
        set_char(line1_next, 14, s2_stat0);
        set_char(line1_next, 15, s2_stat1);

        //----------------------------------------------------------------------
        // Derive 4-character camera state word.
        //
        // Priority:
        //   BUSY > FAIL > DONE > INIT
        //
        // This produces one of:
        //   "BUSY"
        //   "FAIL"
        //   "DONE"
        //   "INIT"
        //----------------------------------------------------------------------
        if (cam_busy) begin
            cam0 = "B"; cam1 = "U"; cam2 = "S"; cam3 = "Y";
        end else if (cam_init_fail) begin
            cam0 = "F"; cam1 = "A"; cam2 = "I"; cam3 = "L";
        end else if (cam_init_done) begin
            cam0 = "D"; cam1 = "O"; cam2 = "N"; cam3 = "E";
        end else begin
            cam0 = "I"; cam1 = "N"; cam2 = "I"; cam3 = "T";
        end

        //======================================================================
        // Row 2 formatting
        // Intended appearance:
        //
        //   "CAM BUSY IDOK E00   "
        //
        // Field breakdown:
        //   [0:2]   "CAM"
        //   [4:7]   4-char camera state
        //   [9:10]  "ID"
        //   [11:12] "OK" or "--"
        //   [14]    "E"
        //   [15:16] 2 hex digits of cam_last_err
        //======================================================================
        set_char(line2_next,  0, "C");
        set_char(line2_next,  1, "A");
        set_char(line2_next,  2, "M");
        set_char(line2_next,  3, " ");

        set_char(line2_next,  4, cam0);
        set_char(line2_next,  5, cam1);
        set_char(line2_next,  6, cam2);
        set_char(line2_next,  7, cam3);

        set_char(line2_next,  8, " ");
        set_char(line2_next,  9, "I");
        set_char(line2_next, 10, "D");

        if (cam_sensor_id_ok) begin
            set_char(line2_next, 11, "O");
            set_char(line2_next, 12, "K");
        end else begin
            set_char(line2_next, 11, "-");
            set_char(line2_next, 12, "-");
        end

        set_char(line2_next, 13, " ");
        set_char(line2_next, 14, "E");
        set_char(line2_next, 15, asc_hex(cam_last_err[7:4]));
        set_char(line2_next, 16, asc_hex(cam_last_err[3:0]));

        //======================================================================
        // Row 3 formatting
        // Intended appearance:
        //
        //   "L1 H1 HB0 F0123     "
        //
        // Field breakdown:
        //   [0]     "L"
        //   [1]     sys_locked bit as ASCII 0/1
        //   [3]     "H"
        //   [4]     hdmi_hpd bit as ASCII 0/1
        //   [6:7]   "HB"
        //   [8]     heartbeat bit as ASCII 0/1
        //   [10]    "F"
        //   [11:14] frame count lower 4 decimal digits
        //======================================================================
        set_char(line3_next,  0, "L");
        set_char(line3_next,  1, sys_locked ? "1" : "0");
        set_char(line3_next,  2, " ");

        set_char(line3_next,  3, "H");
        set_char(line3_next,  4, hdmi_hpd ? "1" : "0");
        set_char(line3_next,  5, " ");

        set_char(line3_next,  6, "H");
        set_char(line3_next,  7, "B");
        set_char(line3_next,  8, heartbeat ? "1" : "0");
        set_char(line3_next,  9, " ");

        set_char(line3_next, 10, "F");

        ftmp = frame_count_lsb;
        if (ftmp > 16'd9999)
            ftmp = 16'd9999;

        set_char(line3_next, 11, asc_digit((ftmp / 16'd1000) % 10));
        set_char(line3_next, 12, asc_digit((ftmp / 16'd100)  % 10));
        set_char(line3_next, 13, asc_digit((ftmp / 16'd10)   % 10));
        set_char(line3_next, 14, asc_digit(ftmp % 10));
    end

    //--------------------------------------------------------------------------
    // Sequential publication and refresh control
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Register the formatted line outputs and generate a one-cycle line_upd
    //   pulse when the lines are republished.
    //
    // STEP-BY-STEP
    //   On every rising edge:
    //
    //     1) If reset is asserted:
    //          - clear refresh counter
    //          - clear previous state vector
    //          - clear output lines
    //          - clear line_upd
    //
    //     2) Otherwise:
    //          - default line_upd low
    //          - advance refresh counter
    //          - if either:
    //                a) telemetry state changed, or
    //                b) periodic refresh expired
    //            then:
    //                * update telemetry_state_prev
    //                * publish line*_next into line*_ascii
    //                * pulse line_upd
    //
    // SEMANTIC NOTE
    //   telemetry_state_prev tracks the last published state, not merely the
    //   previous cycle's raw input state. This is the correct choice for
    //   event-driven publication logic.
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            refresh_ctr          <= 32'd0;
            telemetry_state_prev <= {TELEMETRY_STATE_W{1'b0}};

            line0_ascii          <= {(LINE_CHARS*8){1'b0}};
            line1_ascii          <= {(LINE_CHARS*8){1'b0}};
            line2_ascii          <= {(LINE_CHARS*8){1'b0}};
            line3_ascii          <= {(LINE_CHARS*8){1'b0}};

            line_upd             <= 1'b0;
        end else begin
            //------------------------------------------------------------------
            // Default pulse behavior:
            // line_upd is asserted only on a publication cycle.
            //------------------------------------------------------------------
            line_upd <= 1'b0;

            //------------------------------------------------------------------
            // Free-running periodic refresh counter.
            //
            // When the terminal count is reached, the counter wraps to zero.
            //------------------------------------------------------------------
            if (refresh_ctr >= REFRESH_DIV-1)
                refresh_ctr <= 32'd0;
            else
                refresh_ctr <= refresh_ctr + 32'd1;

            //------------------------------------------------------------------
            // Publish condition:
            //   - immediate publish on any tracked telemetry change
            //   - periodic publish on refresh expiration
            //------------------------------------------------------------------
            if ((telemetry_state_now != telemetry_state_prev) ||
                (refresh_ctr >= REFRESH_DIV-1)) begin

                //--------------------------------------------------------------
                // Record the state that is now being published.
                //--------------------------------------------------------------
                telemetry_state_prev <= telemetry_state_now;

                //--------------------------------------------------------------
                // Publish the newly formatted lines.
                //--------------------------------------------------------------
                line0_ascii <= line0_next;
                line1_ascii <= line1_next;
                line2_ascii <= line2_next;
                line3_ascii <= line3_next;

                //--------------------------------------------------------------
                // One-cycle update pulse.
                //--------------------------------------------------------------
                line_upd <= 1'b1;
            end
        end
    end

endmodule

`default_nettype wire