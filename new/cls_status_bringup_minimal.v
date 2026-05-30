`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// cls_static_bringup_minimal
//------------------------------------------------------------------------------
// ROLE
//   Minimal one-shot Pmod CLS bring-up transmitter.
//
// PURPOSE
//   Remove all higher-level telemetry and formatting logic from the path and
//   prove the basic UART -> CLS transport with a fixed two-line message.
//
// TRANSMIT POLICY
//   After reset:
//
//     1) wait STARTUP_DELAY_MS
//     2) send ESC [ j              // clear/home
//     3) send ESC [ 1 ; 1 H        // line 1, column 1
//     4) send 16 bytes of LINE0
//     5) send ESC [ 2 ; 1 H        // line 2, column 1
//     6) send 16 bytes of LINE1
//     7) stop and hold done=1
//
// WHY THIS MODULE EXISTS
//   Character corruption on the CLS can originate from many layers. This
//   module deliberately collapses the problem into the smallest possible,
//   deterministic transaction script so the physical UART path can be proven
//   independently of telemetry formatting or runtime page updates.
//
// TIMING POLICY
//   - Conservative post-reset startup delay.
//   - Conservative inter-byte gap.
//   - No continuous refresh.
//   - No page flip logic.
//   - No dynamic text generation.
//
// INTEGRATION NOTES
//   - Connect cls_txd_o to the exact FPGA pin that drives the CLS UART RX path.
//   - Use the same board-level routing intended by the existing design.
//   - Probe cls_txd_o and the physical CLS RX line with a logic analyzer.
//
// EXPECTED DISPLAY CONTENT
//   Line 1: "HELLO CLS TEST  "
//   Line 2: "0123456789ABCDEF"
//
// PORTS
//   clk        : system clock
//   rst        : synchronous reset, active high
//   cls_txd_o  : UART TX output to CLS receive path
//   busy       : high while script is being transmitted
//   done       : high after full script transmission completes
//   fault      : reserved sticky error flag; currently always 0 unless the
//                script FSM enters an impossible state
//==============================================================================
module cls_static_bringup_minimal #(
    parameter integer CLK_HZ            = 100_000_000,
    parameter integer BAUD              = 9600,
    parameter integer STARTUP_DELAY_MS  = 100,
    parameter integer BYTE_GAP_US       = 300
)(
    input  wire clk,
    input  wire rst,

    output wire cls_txd_o,
    output reg  busy,
    output reg  done,
    output reg  fault
);

    //--------------------------------------------------------------------------
    // Fixed display payload
    //--------------------------------------------------------------------------
    localparam [127:0] LINE0 = {
        8'h48, 8'h45, 8'h4C, 8'h4C, 8'h4F, 8'h20, 8'h43, 8'h4C,
        8'h53, 8'h20, 8'h54, 8'h45, 8'h53, 8'h54, 8'h20, 8'h20
    }; // "HELLO CLS TEST  "

    localparam [127:0] LINE1 = {
        8'h30, 8'h31, 8'h32, 8'h33, 8'h34, 8'h35, 8'h36, 8'h37,
        8'h38, 8'h39, 8'h41, 8'h42, 8'h43, 8'h44, 8'h45, 8'h46
    }; // "0123456789ABCDEF"

    //--------------------------------------------------------------------------
    // Script length
    //--------------------------------------------------------------------------
    //  0.. 2 : ESC [ j
    //  3.. 8 : ESC [ 1 ; 1 H
    //  9..24 : LINE0[0..15]
    // 25..30 : ESC [ 2 ; 1 H
    // 31..46 : LINE1[0..15]
    localparam integer SCRIPT_LEN = 47;

    //--------------------------------------------------------------------------
    // Timing constants
    //--------------------------------------------------------------------------
    localparam integer STARTUP_DELAY_CYCLES =
        (CLK_HZ / 1000) * STARTUP_DELAY_MS;

    localparam integer BYTE_GAP_CYCLES =
        ((CLK_HZ / 1000000) * BYTE_GAP_US);

    //--------------------------------------------------------------------------
    // FSM encoding
    //--------------------------------------------------------------------------
    localparam [2:0]
        ST_STARTUP  = 3'd0,
        ST_PREPARE  = 3'd1,
        ST_STROBE   = 3'd2,
        ST_WAIT_GAP = 3'd3,
        ST_DONE     = 3'd4,
        ST_FAULT    = 3'd5;

    reg [2:0]  state;
    reg [31:0] timer_ctr;
    reg [7:0]  tx_data;
    reg        tx_valid;
    reg [5:0]  script_idx;

    wire       tx_ready;

    //--------------------------------------------------------------------------
    // UART transmitter
    //--------------------------------------------------------------------------
    uart_tx_8n1 #(
        .CLK_HZ(CLK_HZ),
        .BAUD  (BAUD)
    ) u_uart_tx_8n1 (
        .clk       (clk),
        .rst       (rst),
        .data_in   (tx_data),
        .data_valid(tx_valid),
        .ready     (tx_ready),
        .txd       (cls_txd_o)
    );

    //--------------------------------------------------------------------------
    // Fixed-line character extraction helpers
    //--------------------------------------------------------------------------
    function [7:0] line0_char;
        input [4:0] idx;
        begin
            case (idx)
                5'd0:  line0_char = LINE0[127:120];
                5'd1:  line0_char = LINE0[119:112];
                5'd2:  line0_char = LINE0[111:104];
                5'd3:  line0_char = LINE0[103:96];
                5'd4:  line0_char = LINE0[95:88];
                5'd5:  line0_char = LINE0[87:80];
                5'd6:  line0_char = LINE0[79:72];
                5'd7:  line0_char = LINE0[71:64];
                5'd8:  line0_char = LINE0[63:56];
                5'd9:  line0_char = LINE0[55:48];
                5'd10: line0_char = LINE0[47:40];
                5'd11: line0_char = LINE0[39:32];
                5'd12: line0_char = LINE0[31:24];
                5'd13: line0_char = LINE0[23:16];
                5'd14: line0_char = LINE0[15:8];
                5'd15: line0_char = LINE0[7:0];
                default: line0_char = 8'h20;
            endcase
        end
    endfunction

    function [7:0] line1_char;
        input [4:0] idx;
        begin
            case (idx)
                5'd0:  line1_char = LINE1[127:120];
                5'd1:  line1_char = LINE1[119:112];
                5'd2:  line1_char = LINE1[111:104];
                5'd3:  line1_char = LINE1[103:96];
                5'd4:  line1_char = LINE1[95:88];
                5'd5:  line1_char = LINE1[87:80];
                5'd6:  line1_char = LINE1[79:72];
                5'd7:  line1_char = LINE1[71:64];
                5'd8:  line1_char = LINE1[63:56];
                5'd9:  line1_char = LINE1[55:48];
                5'd10: line1_char = LINE1[47:40];
                5'd11: line1_char = LINE1[39:32];
                5'd12: line1_char = LINE1[31:24];
                5'd13: line1_char = LINE1[23:16];
                5'd14: line1_char = LINE1[15:8];
                5'd15: line1_char = LINE1[7:0];
                default: line1_char = 8'h20;
            endcase
        end
    endfunction

    //--------------------------------------------------------------------------
    // Script ROM
    //--------------------------------------------------------------------------
    function [7:0] script_byte;
        input [5:0] idx;
        begin
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

                // LINE0
                6'd9:  script_byte = line0_char(5'd0);
                6'd10: script_byte = line0_char(5'd1);
                6'd11: script_byte = line0_char(5'd2);
                6'd12: script_byte = line0_char(5'd3);
                6'd13: script_byte = line0_char(5'd4);
                6'd14: script_byte = line0_char(5'd5);
                6'd15: script_byte = line0_char(5'd6);
                6'd16: script_byte = line0_char(5'd7);
                6'd17: script_byte = line0_char(5'd8);
                6'd18: script_byte = line0_char(5'd9);
                6'd19: script_byte = line0_char(5'd10);
                6'd20: script_byte = line0_char(5'd11);
                6'd21: script_byte = line0_char(5'd12);
                6'd22: script_byte = line0_char(5'd13);
                6'd23: script_byte = line0_char(5'd14);
                6'd24: script_byte = line0_char(5'd15);

                // ESC [ 2 ; 1 H
                6'd25: script_byte = 8'h1B;
                6'd26: script_byte = 8'h5B;
                6'd27: script_byte = 8'h32;
                6'd28: script_byte = 8'h3B;
                6'd29: script_byte = 8'h31;
                6'd30: script_byte = 8'h48;

                // LINE1
                6'd31: script_byte = line1_char(5'd0);
                6'd32: script_byte = line1_char(5'd1);
                6'd33: script_byte = line1_char(5'd2);
                6'd34: script_byte = line1_char(5'd3);
                6'd35: script_byte = line1_char(5'd4);
                6'd36: script_byte = line1_char(5'd5);
                6'd37: script_byte = line1_char(5'd6);
                6'd38: script_byte = line1_char(5'd7);
                6'd39: script_byte = line1_char(5'd8);
                6'd40: script_byte = line1_char(5'd9);
                6'd41: script_byte = line1_char(5'd10);
                6'd42: script_byte = line1_char(5'd11);
                6'd43: script_byte = line1_char(5'd12);
                6'd44: script_byte = line1_char(5'd13);
                6'd45: script_byte = line1_char(5'd14);
                6'd46: script_byte = line1_char(5'd15);

                default: script_byte = 8'h20;
            endcase
        end
    endfunction

    //--------------------------------------------------------------------------
    // One-shot script transmitter FSM
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state      <= ST_STARTUP;
            timer_ctr  <= 32'd0;
            tx_data    <= 8'h00;
            tx_valid   <= 1'b0;
            script_idx <= 6'd0;
            busy       <= 1'b1;
            done       <= 1'b0;
            fault      <= 1'b0;
        end else begin
            // default one-cycle strobe behavior
            tx_valid <= 1'b0;

            case (state)
                ST_STARTUP: begin
                    busy <= 1'b1;
                    done <= 1'b0;

                    if (timer_ctr >= (STARTUP_DELAY_CYCLES - 1)) begin
                        timer_ctr  <= 32'd0;
                        script_idx <= 6'd0;
                        state      <= ST_PREPARE;
                    end else begin
                        timer_ctr <= timer_ctr + 1'b1;
                    end
                end

                ST_PREPARE: begin
                    busy <= 1'b1;

                    if (script_idx >= SCRIPT_LEN[5:0]) begin
                        state <= ST_DONE;
                    end else begin
                        tx_data <= script_byte(script_idx);
                        state   <= ST_STROBE;
                    end
                end

                ST_STROBE: begin
                    busy <= 1'b1;

                    if (tx_ready) begin
                        tx_valid  <= 1'b1;
                        timer_ctr <= 32'd0;
                        state     <= ST_WAIT_GAP;
                    end
                end

                ST_WAIT_GAP: begin
                    busy <= 1'b1;

                    if (timer_ctr >= (BYTE_GAP_CYCLES - 1)) begin
                        timer_ctr  <= 32'd0;
                        script_idx <= script_idx + 1'b1;
                        state      <= ST_PREPARE;
                    end else begin
                        timer_ctr <= timer_ctr + 1'b1;
                    end
                end

                ST_DONE: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                end

                ST_FAULT: begin
                    busy  <= 1'b0;
                    done  <= 1'b0;
                    fault <= 1'b1;
                end

                default: begin
                    state <= ST_FAULT;
                end
            endcase
        end
    end

endmodule

`default_nettype wire