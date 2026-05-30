`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// occupancy_map_dp_bram
//------------------------------------------------------------------------------
// Simple true dual-port map memory:
//   - Port A: SYS write-only
//   - Port B: VID synchronous read
//
// RD_LAT assumption for sonar_map_renderer:
//   This implementation provides a 1-cycle registered read on port B.
//==============================================================================
module occupancy_map_dp_bram #(
    parameter integer MAP_W   = 256,
    parameter integer MAP_H   = 256,
    parameter integer DATA_W  = 8,
    parameter integer ADDR_W  = 16
)(
    input  wire                  clk_a,
    input  wire                  we_a,
    input  wire [ADDR_W-1:0]     addr_a,
    input  wire [DATA_W-1:0]     din_a,

    input  wire                  clk_b,
    input  wire [ADDR_W-1:0]     addr_b,
    output reg  [DATA_W-1:0]     dout_b
);
    localparam integer DEPTH = MAP_W * MAP_H;

    reg [DATA_W-1:0] mem [0:DEPTH-1];

    integer i;
    initial begin
        for (i = 0; i < DEPTH; i = i + 1)
            mem[i] = {DATA_W{1'b0}};
    end

    always @(posedge clk_a) begin
        if (we_a)
            mem[addr_a] <= din_a;
    end

    always @(posedge clk_b) begin
        dout_b <= mem[addr_b];
    end
endmodule

`default_nettype wire