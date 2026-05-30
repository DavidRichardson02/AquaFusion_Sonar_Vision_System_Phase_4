`timescale 1ns/1ps
`default_nettype none

// Minimal placeholder SCCB slave model kept in design sources so the project
// owns a single, correctly named camera-support module. The focused camera
// sequencer testbench uses direct handshake mocking rather than bit-accurate
// bus simulation.
module ov5640_sccb_slave_model #(
    parameter [6:0] DEV_ADDR_7B = 7'h3C
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       wr_en,
    input  wire       rd_en,
    input  wire [15:0]reg_addr,
    input  wire [7:0] wr_data,
    output reg  [7:0] rd_data
);

    reg [7:0] reg_file [0:65535];

    always @(posedge clk) begin
        if (rst) begin
            reg_file[16'h300A] <= 8'h56;
            reg_file[16'h300B] <= 8'h40;
            rd_data            <= 8'h00;
        end else begin
            if (wr_en)
                reg_file[reg_addr] <= wr_data;
            if (rd_en)
                rd_data <= reg_file[reg_addr];
        end
    end

    wire _unused_addr;
    assign _unused_addr = DEV_ADDR_7B[0];

endmodule

`default_nettype wire
