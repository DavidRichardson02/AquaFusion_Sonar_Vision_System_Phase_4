`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// sonar_map_mem_tdp_dc
//------------------------------------------------------------------------------
// Simple dual-clock map memory:
//   - SYS write port
//   - VID read port
//
// This is a pragmatic bring-up memory shell intended to match the current
// painter/renderer split. Device-specific RAM replacement remains possible.
//==============================================================================
module sonar_map_mem_tdp_dc #(
    parameter integer AW = 16,
    parameter integer DW = 8
)(
    input  wire          clk_sys,
    input  wire          we_sys,
    input  wire [AW-1:0] addr_sys,
    input  wire [DW-1:0] din_sys,

    input  wire          clk_vid,
    input  wire [AW-1:0] addr_vid,
    output reg  [DW-1:0] dout_vid
);
    reg [DW-1:0] mem [0:(1<<AW)-1];
    integer init_i;

    initial begin
        for (init_i = 0; init_i < (1<<AW); init_i = init_i + 1)
            mem[init_i] = {DW{1'b0}};
    end

    always @(posedge clk_sys) begin
        if (we_sys)
            mem[addr_sys] <= din_sys;
    end

    always @(posedge clk_vid) begin
        dout_vid <= mem[addr_vid];
    end
endmodule

`default_nettype wire
