`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// sonar_snapshot_pack_rich_sys
//------------------------------------------------------------------------------
// Rich 128-bit SYS-domain snapshot for hud_sonar_tile (renamed basic tile kept
// separate from this richer format).
//
// Field map used by hud_sonar_tile input contract:
//   [  7:  0] raw inches
//   [ 15:  8] filtered inches
//   [ 31: 16] age_ms
//   [ 63: 32] update_count
//   [ 65: 64] source id
//   [ 66]     fresh
//   [ 67]     err_seen
// Upper bits are reserved and zero-filled.
//==============================================================================
module sonar_snapshot_pack_rich_sys #(
    parameter integer CLK_HZ = 100_000_000,
    parameter integer SNAP_W = 128,
    parameter [1:0]   SRC_ID = 2'd0
)(
    input  wire              clk_sys,
    input  wire              rst_sys,
    input  wire [9:0]        raw_in,
    input  wire [9:0]        filt_in,
    input  wire              stale,
    input  wire              timeout_err,
    input  wire              parse_err_sticky,
    input  wire [15:0]       age_ticks,
    input  wire [31:0]       update_count,
    output reg  [SNAP_W-1:0] snap_data,
    output reg               snap_upd
);
    reg [SNAP_W-1:0] snap_next;
    reg [SNAP_W-1:0] snap_prev;

    wire [31:0] age_ms_u32 = (age_ticks * 32'd1000) / CLK_HZ;
    wire [15:0] age_ms_u16 = (age_ms_u32 > 32'h0000_FFFF) ? 16'hFFFF : age_ms_u32[15:0];
    wire [7:0]  raw_u8     = (raw_in  > 10'd255) ? 8'hFF : raw_in[7:0];
    wire [7:0]  filt_u8    = (filt_in > 10'd255) ? 8'hFF : filt_in[7:0];
    wire        fresh      = ~stale;
    wire        err_seen   = timeout_err | parse_err_sticky;

    always @* begin
        snap_next = {SNAP_W{1'b0}};
        snap_next[7:0]   = raw_u8;
        snap_next[15:8]  = filt_u8;
        snap_next[31:16] = age_ms_u16;
        snap_next[63:32] = update_count;
        snap_next[65:64] = SRC_ID;
        snap_next[66]    = fresh;
        snap_next[67]    = err_seen;
    end

    always @(posedge clk_sys) begin
        if (rst_sys) begin
            snap_data <= {SNAP_W{1'b0}};
            snap_prev <= {SNAP_W{1'b0}};
            snap_upd  <= 1'b0;
        end else begin
            snap_data <= snap_next;
            snap_upd  <= 1'b0;
            if (snap_next != snap_prev) begin
                snap_prev <= snap_next;
                snap_upd  <= 1'b1;
            end
        end
    end
endmodule

`default_nettype wire