`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// pmod_cls_debug
//------------------------------------------------------------------------------
// ROLE
//   Conservative write-only UART transport for the Digilent Pmod CLS.
//
// PURPOSE
//   Transmit two frozen 16-character lines to the Pmod CLS using a deliberately
//   paced escape-sequence script. This version is intended both for normal
//   operation and for low-risk hardware bring-up.
//
// WHY THIS REWRITE EXISTS
//   The earlier transport emitted bytes back-to-back whenever the UART became
//   ready. That is logically sufficient for a UART transmitter, but it gives
//   the Pmod CLS very little command-processing slack. The Pmod CLS is driven
//   by an on-board ATmega48 microcontroller that interprets escape sequences,
//   so a conservative transport is appropriate during bring-up and fault
//   isolation.
//
// DISPLAY SCRIPT
//   Depending on CLEAR_EACH_TRANSFER and whether the current transfer is the
//   first one after reset, the transmitted script is:
//
//     Optional clear/home:
//       ESC [ j
//
//     Always:
//       ESC [ 1 ; 1 H
//       16 chars of line 0
//       ESC [ 2 ; 1 H
//       16 chars of line 1
//
// SNAPSHOT POLICY
//   line0_text and line1_text are captured only when a new transfer starts.
//   The entire transfer therefore uses a frozen text image.
//
// REQUEST POLICY
//   - In MODE_ONE_SHOT != 0:
//       exactly one transfer is emitted after startup delay, then all later
//       refresh_req and periodic refresh activity are ignored.
//   - In MODE_ONE_SHOT == 0:
//       transfers may be started by:
//         * startup completion (first transfer only)
//         * refresh_req, if ENABLE_REFRESH_REQ != 0
//         * periodic timer, if ENABLE_PERIODIC_REFRESH != 0
//
// PACING POLICY
//   - Each byte is launched only when the UART is ready.
//   - After each launched byte, an inter-byte gap is inserted.
//   - After command-group terminal bytes ('j', 'H', 'H'), an additional
//     command gap is inserted.
//
// PORTS
//   clk         : system clock
//   rst         : synchronous active-high reset
//   line0_text  : 16-char line, char0 in [127:120], char15 in [7:0]
//   line1_text  : 16-char line, char0 in [127:120], char15 in [7:0]
//   refresh_req : optional one-cycle or level request for refresh
//   cls_txd     : UART transmit line to Pmod CLS RXD
//
// INTEGRATION NOTE
//   The Pmod CLS manual states that UART communication is supported and that
//   J2 pin 4 is RXD on the module side; mode jumpers MD2,MD1,MD0 = 0,1,0 select
//   UART at 9600 baud on Rev E boards. 
//==============================================================================
module pmod_cls_debug #(
    parameter integer CLK_HZ                  = 100_000_000,
    parameter integer BAUD                    = 9600,
    parameter integer STARTUP_MS              = 100,
    parameter integer REFRESH_MS              = 250,

    // Conservative pacing controls
    parameter integer INTER_BYTE_GAP_US       = 300,
    parameter integer CMD_EXTRA_GAP_US        = 2000,

    // Bring-up / policy controls
    parameter integer MODE_ONE_SHOT           = 0,
    parameter integer ENABLE_PERIODIC_REFRESH = 1,
    parameter integer ENABLE_REFRESH_REQ      = 1,
    parameter integer CLEAR_EACH_TRANSFER     = 1
)(
    input  wire         clk,
    input  wire         rst,
    input  wire [127:0] line0_text,
    input  wire [127:0] line1_text,
    input  wire         refresh_req,
    output wire         cls_txd
);

    //--------------------------------------------------------------------------
    // Timing constants
    //--------------------------------------------------------------------------
    localparam integer STARTUP_CYCLES =
        (CLK_HZ / 1000) * STARTUP_MS;

    localparam integer REFRESH_CYCLES =
        (CLK_HZ / 1000) * REFRESH_MS;

    localparam integer INTER_BYTE_GAP_CYCLES =
        (CLK_HZ / 1000000) * INTER_BYTE_GAP_US;

    localparam integer CMD_EXTRA_GAP_CYCLES =
        (CLK_HZ / 1000000) * CMD_EXTRA_GAP_US;

    //--------------------------------------------------------------------------
    // Script sizing
    //--------------------------------------------------------------------------
    localparam integer SCRIPT_LEN_WITH_CLEAR = 47;
    localparam integer SCRIPT_LEN_NO_CLEAR   = 44;

    //--------------------------------------------------------------------------
    // FSM encoding
    //--------------------------------------------------------------------------
    localparam [2:0]
        ST_STARTUP   = 3'd0,
        ST_IDLE      = 3'd1,
        ST_LOAD_BYTE = 3'd2,
        ST_LAUNCH    = 3'd3,
        ST_WAIT_GAP  = 3'd4,
        ST_ADVANCE   = 3'd5;

    //--------------------------------------------------------------------------
    // State / control
    //--------------------------------------------------------------------------
    reg [2:0]  state;

    reg [31:0] startup_ctr;
    reg [31:0] refresh_ctr;
    reg [31:0] gap_ctr;

    reg        first_transfer_done;
    reg        transfer_active;
    reg        transfer_use_clear;
    reg [5:0]  byte_idx;
    reg [5:0]  last_idx;

    reg [127:0] snap_line0;
    reg [127:0] snap_line1;

    reg [7:0] tx_data;
    reg       tx_valid;
    wire      tx_ready;

    reg       refresh_req_d;
    wire      refresh_req_rise;

    reg       start_transfer;
    reg [31:0] applied_gap_cycles;

    //--------------------------------------------------------------------------
    // Character extraction helper
    //--------------------------------------------------------------------------
    function [7:0] char_at16;
        input [127:0] text;
        input [4:0]   idx;
        begin
            case (idx)
                5'd0:  char_at16 = text[127:120];
                5'd1:  char_at16 = text[119:112];
                5'd2:  char_at16 = text[111:104];
                5'd3:  char_at16 = text[103:96];
                5'd4:  char_at16 = text[95:88];
                5'd5:  char_at16 = text[87:80];
                5'd6:  char_at16 = text[79:72];
                5'd7:  char_at16 = text[71:64];
                5'd8:  char_at16 = text[63:56];
                5'd9:  char_at16 = text[55:48];
                5'd10: char_at16 = text[47:40];
                5'd11: char_at16 = text[39:32];
                5'd12: char_at16 = text[31:24];
                5'd13: char_at16 = text[23:16];
                5'd14: char_at16 = text[15:8];
                default: char_at16 = text[7:0];
            endcase
        end
    endfunction

    //--------------------------------------------------------------------------
    // Edge detector for refresh_req
    //--------------------------------------------------------------------------
    assign refresh_req_rise = refresh_req & ~refresh_req_d;

    //--------------------------------------------------------------------------
    // Determine whether a byte terminates a CLS command group.
    //
    // With clear:
    //   idx  2 = 'j'     end of ESC [ j
    //   idx  8 = 'H'     end of ESC [ 1 ; 1 H
    //   idx 30 = 'H'     end of ESC [ 2 ; 1 H
    //
    // Without clear:
    //   idx  5 = 'H'     end of ESC [ 1 ; 1 H
    //   idx 27 = 'H'     end of ESC [ 2 ; 1 H
    //--------------------------------------------------------------------------
    function is_cmd_terminal;
        input        use_clear;
        input [5:0]  idx;
        begin
            if (use_clear) begin
                case (idx)
                    6'd2,
                    6'd8,
                    6'd30: is_cmd_terminal = 1'b1;
                    default: is_cmd_terminal = 1'b0;
                endcase
            end else begin
                case (idx)
                    6'd5,
                    6'd27: is_cmd_terminal = 1'b1;
                    default: is_cmd_terminal = 1'b0;
                endcase
            end
        end
    endfunction

    //--------------------------------------------------------------------------
    // Script ROM
    //--------------------------------------------------------------------------
    function [7:0] script_byte;
        input       use_clear;
        input [5:0] idx;
        begin
            if (use_clear) begin
                case (idx)
                    // ESC [ j
                    6'd0:  script_byte = 8'h1B;
                    6'd1:  script_byte = 8'h5B;
                    6'd2:  script_byte = 8'h6A;

                    // ESC [ 1 ; 1 H
                    6'd3:  script_byte = 8'h1B;
                    6'd4:  script_byte = 8'h5B;
                    6'd5:  script_byte = 8'h31;
                    6'd6:  script_byte = 8'h3B;
                    6'd7:  script_byte = 8'h31;
                    6'd8:  script_byte = 8'h48;

                    // line 0
                    6'd9:  script_byte = char_at16(snap_line0,  5'd0);
                    6'd10: script_byte = char_at16(snap_line0,  5'd1);
                    6'd11: script_byte = char_at16(snap_line0,  5'd2);
                    6'd12: script_byte = char_at16(snap_line0,  5'd3);
                    6'd13: script_byte = char_at16(snap_line0,  5'd4);
                    6'd14: script_byte = char_at16(snap_line0,  5'd5);
                    6'd15: script_byte = char_at16(snap_line0,  5'd6);
                    6'd16: script_byte = char_at16(snap_line0,  5'd7);
                    6'd17: script_byte = char_at16(snap_line0,  5'd8);
                    6'd18: script_byte = char_at16(snap_line0,  5'd9);
                    6'd19: script_byte = char_at16(snap_line0,  5'd10);
                    6'd20: script_byte = char_at16(snap_line0,  5'd11);
                    6'd21: script_byte = char_at16(snap_line0,  5'd12);
                    6'd22: script_byte = char_at16(snap_line0,  5'd13);
                    6'd23: script_byte = char_at16(snap_line0,  5'd14);
                    6'd24: script_byte = char_at16(snap_line0,  5'd15);

                    // ESC [ 2 ; 1 H
                    6'd25: script_byte = 8'h1B;
                    6'd26: script_byte = 8'h5B;
                    6'd27: script_byte = 8'h32;
                    6'd28: script_byte = 8'h3B;
                    6'd29: script_byte = 8'h31;
                    6'd30: script_byte = 8'h48;

                    // line 1
                    6'd31: script_byte = char_at16(snap_line1,  5'd0);
                    6'd32: script_byte = char_at16(snap_line1,  5'd1);
                    6'd33: script_byte = char_at16(snap_line1,  5'd2);
                    6'd34: script_byte = char_at16(snap_line1,  5'd3);
                    6'd35: script_byte = char_at16(snap_line1,  5'd4);
                    6'd36: script_byte = char_at16(snap_line1,  5'd5);
                    6'd37: script_byte = char_at16(snap_line1,  5'd6);
                    6'd38: script_byte = char_at16(snap_line1,  5'd7);
                    6'd39: script_byte = char_at16(snap_line1,  5'd8);
                    6'd40: script_byte = char_at16(snap_line1,  5'd9);
                    6'd41: script_byte = char_at16(snap_line1,  5'd10);
                    6'd42: script_byte = char_at16(snap_line1,  5'd11);
                    6'd43: script_byte = char_at16(snap_line1,  5'd12);
                    6'd44: script_byte = char_at16(snap_line1,  5'd13);
                    6'd45: script_byte = char_at16(snap_line1,  5'd14);
                    default: script_byte = char_at16(snap_line1, 5'd15);
                endcase
            end else begin
                case (idx)
                    // ESC [ 1 ; 1 H
                    6'd0:  script_byte = 8'h1B;
                    6'd1:  script_byte = 8'h5B;
                    6'd2:  script_byte = 8'h31;
                    6'd3:  script_byte = 8'h3B;
                    6'd4:  script_byte = 8'h31;
                    6'd5:  script_byte = 8'h48;

                    // line 0
                    6'd6:  script_byte = char_at16(snap_line0,  5'd0);
                    6'd7:  script_byte = char_at16(snap_line0,  5'd1);
                    6'd8:  script_byte = char_at16(snap_line0,  5'd2);
                    6'd9:  script_byte = char_at16(snap_line0,  5'd3);
                    6'd10: script_byte = char_at16(snap_line0,  5'd4);
                    6'd11: script_byte = char_at16(snap_line0,  5'd5);
                    6'd12: script_byte = char_at16(snap_line0,  5'd6);
                    6'd13: script_byte = char_at16(snap_line0,  5'd7);
                    6'd14: script_byte = char_at16(snap_line0,  5'd8);
                    6'd15: script_byte = char_at16(snap_line0,  5'd9);
                    6'd16: script_byte = char_at16(snap_line0,  5'd10);
                    6'd17: script_byte = char_at16(snap_line0,  5'd11);
                    6'd18: script_byte = char_at16(snap_line0,  5'd12);
                    6'd19: script_byte = char_at16(snap_line0,  5'd13);
                    6'd20: script_byte = char_at16(snap_line0,  5'd14);
                    6'd21: script_byte = char_at16(snap_line0,  5'd15);

                    // ESC [ 2 ; 1 H
                    6'd22: script_byte = 8'h1B;
                    6'd23: script_byte = 8'h5B;
                    6'd24: script_byte = 8'h32;
                    6'd25: script_byte = 8'h3B;
                    6'd26: script_byte = 8'h31;
                    6'd27: script_byte = 8'h48;

                    // line 1
                    6'd28: script_byte = char_at16(snap_line1,  5'd0);
                    6'd29: script_byte = char_at16(snap_line1,  5'd1);
                    6'd30: script_byte = char_at16(snap_line1,  5'd2);
                    6'd31: script_byte = char_at16(snap_line1,  5'd3);
                    6'd32: script_byte = char_at16(snap_line1,  5'd4);
                    6'd33: script_byte = char_at16(snap_line1,  5'd5);
                    6'd34: script_byte = char_at16(snap_line1,  5'd6);
                    6'd35: script_byte = char_at16(snap_line1,  5'd7);
                    6'd36: script_byte = char_at16(snap_line1,  5'd8);
                    6'd37: script_byte = char_at16(snap_line1,  5'd9);
                    6'd38: script_byte = char_at16(snap_line1,  5'd10);
                    6'd39: script_byte = char_at16(snap_line1,  5'd11);
                    6'd40: script_byte = char_at16(snap_line1,  5'd12);
                    6'd41: script_byte = char_at16(snap_line1,  5'd13);
                    6'd42: script_byte = char_at16(snap_line1,  5'd14);
                    default: script_byte = char_at16(snap_line1, 5'd15);
                endcase
            end
        end
    endfunction

    //--------------------------------------------------------------------------
    // UART transmitter
    //--------------------------------------------------------------------------
    uart_tx_8n1 #(
        .CLK_HZ (CLK_HZ),
        .BAUD   (BAUD)
    ) u_uart_tx_8n1 (
        .clk        (clk),
        .rst        (rst),
        .data_in    (tx_data),
        .data_valid (tx_valid),
        .ready      (tx_ready),
        .txd        (cls_txd)
    );

    //--------------------------------------------------------------------------
    // Main control
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state               <= ST_STARTUP;
            startup_ctr         <= 32'd0;
            refresh_ctr         <= 32'd0;
            gap_ctr             <= 32'd0;
            first_transfer_done <= 1'b0;
            transfer_active     <= 1'b0;
            transfer_use_clear  <= 1'b1;
            byte_idx            <= 6'd0;
            last_idx            <= 6'd0;
            snap_line0          <= 128'd0;
            snap_line1          <= 128'd0;
            tx_data             <= 8'h20;
            tx_valid            <= 1'b0;
            refresh_req_d       <= 1'b0;
            start_transfer      <= 1'b0;
            applied_gap_cycles  <= 32'd0;
        end else begin
            tx_valid       <= 1'b0;
            start_transfer <= 1'b0;
            refresh_req_d  <= refresh_req;

            case (state)
                //------------------------------------------------------------------
                // Startup holdoff
                //------------------------------------------------------------------
                ST_STARTUP: begin
                    if (STARTUP_CYCLES == 0) begin
                        state <= ST_IDLE;
                    end else if (startup_ctr >= (STARTUP_CYCLES - 1)) begin
                        startup_ctr <= startup_ctr;
                        state       <= ST_IDLE;
                    end else begin
                        startup_ctr <= startup_ctr + 1'b1;
                    end
                end

                //------------------------------------------------------------------
                // Idle / start qualification
                //------------------------------------------------------------------
                ST_IDLE: begin
                    transfer_active <= 1'b0;

                    if (!first_transfer_done) begin
                        start_transfer <= 1'b1;
                    end else if ((MODE_ONE_SHOT == 0) &&
                                 (ENABLE_REFRESH_REQ != 0) &&
                                 refresh_req_rise) begin
                        start_transfer <= 1'b1;
                    end else if ((MODE_ONE_SHOT == 0) &&
                                 (ENABLE_PERIODIC_REFRESH != 0)) begin
                        if (REFRESH_CYCLES == 0) begin
                            start_transfer <= 1'b1;
                        end else if (refresh_ctr >= (REFRESH_CYCLES - 1)) begin
                            start_transfer <= 1'b1;
                        end else begin
                            refresh_ctr <= refresh_ctr + 1'b1;
                        end
                    end

                    if (start_transfer || (!first_transfer_done)) begin
                        transfer_active <= 1'b1;
                        byte_idx        <= 6'd0;
                        snap_line0      <= line0_text;
                        snap_line1      <= line1_text;
                        refresh_ctr     <= 32'd0;

                        if (!first_transfer_done)
                            transfer_use_clear <= 1'b1;
                        else if (CLEAR_EACH_TRANSFER != 0)
                            transfer_use_clear <= 1'b1;
                        else
                            transfer_use_clear <= 1'b0;

                        if ((!first_transfer_done) || (CLEAR_EACH_TRANSFER != 0))
                            last_idx <= SCRIPT_LEN_WITH_CLEAR - 1;
                        else
                            last_idx <= SCRIPT_LEN_NO_CLEAR - 1;

                        state <= ST_LOAD_BYTE;
                    end
                end

                //------------------------------------------------------------------
                // Present next script byte to UART input holding register
                //------------------------------------------------------------------
                ST_LOAD_BYTE: begin
                    tx_data <= script_byte(transfer_use_clear, byte_idx);
                    state   <= ST_LAUNCH;
                end

                //------------------------------------------------------------------
                // Launch exactly one UART byte when transmitter reports ready
                //------------------------------------------------------------------
                ST_LAUNCH: begin
                    if (tx_ready) begin
                        tx_valid <= 1'b1;

                        if (is_cmd_terminal(transfer_use_clear, byte_idx))
                            applied_gap_cycles <= INTER_BYTE_GAP_CYCLES + CMD_EXTRA_GAP_CYCLES;
                        else
                            applied_gap_cycles <= INTER_BYTE_GAP_CYCLES;

                        gap_ctr <= 32'd0;
                        state   <= ST_WAIT_GAP;
                    end
                end

                //------------------------------------------------------------------
                // Conservative post-byte wait
                //------------------------------------------------------------------
                ST_WAIT_GAP: begin
                    if (applied_gap_cycles == 0) begin
                        state <= ST_ADVANCE;
                    end else if (gap_ctr >= (applied_gap_cycles - 1)) begin
                        state <= ST_ADVANCE;
                    end else begin
                        gap_ctr <= gap_ctr + 1'b1;
                    end
                end

                //------------------------------------------------------------------
                // Advance script index or retire transfer
                //------------------------------------------------------------------
                ST_ADVANCE: begin
                    if (byte_idx >= last_idx) begin
                        first_transfer_done <= 1'b1;
                        transfer_active     <= 1'b0;
                        byte_idx            <= 6'd0;

                        if (MODE_ONE_SHOT != 0)
                            refresh_ctr <= 32'd0;
                        else
                            refresh_ctr <= 32'd0;

                        state <= ST_IDLE;
                    end else begin
                        byte_idx <= byte_idx + 1'b1;
                        state    <= ST_LOAD_BYTE;
                    end
                end

                default: begin
                    state <= ST_STARTUP;
                end
            endcase
        end
    end

endmodule

`default_nettype wire