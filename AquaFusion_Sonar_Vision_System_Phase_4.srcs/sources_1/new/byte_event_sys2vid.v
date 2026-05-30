`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// byte_event_sys2vid
//------------------------------------------------------------------------------
// Handshake-based SYS->VID byte event transfer.
//
// Source side:
//   byte_sys / byte_vld_sys
//     One-byte event in clk_sys.
//
// Destination side:
//   byte_vid / byte_vld_vid
//     One-cycle pulse in clk_vid carrying the transferred byte.
//
// Contract:
//   - One event in flight at a time.
//   - Source holds the published byte in a source register until an ack
//     toggle returns from the destination.
//==============================================================================
module byte_event_sys2vid (
    input  wire       clk_sys,
    input  wire       rst_sys,
    input  wire [7:0] byte_sys,
    input  wire       byte_vld_sys,

    input  wire       clk_vid,
    input  wire       rst_vid,
    output reg  [7:0] byte_vid,
    output reg        byte_vld_vid,
    output wire       busy_sys
);
    reg [7:0] hold_byte_sys;
    reg       req_tgl_sys;
    reg       src_busy_sys;

    reg ack_sync1_sys;
    reg ack_sync2_sys;
    reg ack_seen_sys;

    reg req_sync1_vid;
    reg req_sync2_vid;
    reg req_seen_vid;
    reg ack_tgl_vid;

    assign busy_sys = src_busy_sys;

    always @(posedge clk_sys) begin
        if (rst_sys) begin
            hold_byte_sys <= 8'd0;
            req_tgl_sys   <= 1'b0;
            src_busy_sys  <= 1'b0;
            ack_sync1_sys <= 1'b0;
            ack_sync2_sys <= 1'b0;
            ack_seen_sys  <= 1'b0;
        end else begin
            ack_sync1_sys <= ack_tgl_vid;
            ack_sync2_sys <= ack_sync1_sys;

            if (src_busy_sys && (ack_sync2_sys != ack_seen_sys)) begin
                ack_seen_sys <= ack_sync2_sys;
                src_busy_sys <= 1'b0;
            end

            if (byte_vld_sys && !src_busy_sys) begin
                hold_byte_sys <= byte_sys;
                req_tgl_sys   <= ~req_tgl_sys;
                src_busy_sys  <= 1'b1;
            end
        end
    end

    always @(posedge clk_vid) begin
        if (rst_vid) begin
            req_sync1_vid <= 1'b0;
            req_sync2_vid <= 1'b0;
            req_seen_vid  <= 1'b0;
            ack_tgl_vid   <= 1'b0;
            byte_vid      <= 8'd0;
            byte_vld_vid  <= 1'b0;
        end else begin
            byte_vld_vid  <= 1'b0;
            req_sync1_vid <= req_tgl_sys;
            req_sync2_vid <= req_sync1_vid;

            if (req_sync2_vid != req_seen_vid) begin
                req_seen_vid <= req_sync2_vid;
                byte_vid     <= hold_byte_sys;
                byte_vld_vid <= 1'b1;
                ack_tgl_vid  <= ~ack_tgl_vid;
            end
        end
    end
endmodule

`default_nettype wire