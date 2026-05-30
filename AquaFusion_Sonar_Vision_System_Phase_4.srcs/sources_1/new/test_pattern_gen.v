`timescale 1ns/1ps
`default_nettype none


//==============================================================================
// test_pattern_gen
//==============================================================================

module test_pattern_gen (
    input  wire [11:0] pix_x,
    input  wire [11:0] pix_y,
    input  wire        de,
    output reg  [23:0] rgb_out
);
    always @(*) begin
        if (!de) begin
            rgb_out = 24'h000000;
        end else if (pix_x < 12'd160) begin
            rgb_out = 24'hFF0000;
        end else if (pix_x < 12'd320) begin
            rgb_out = 24'h00FF00;
        end else if (pix_x < 12'd480) begin
            rgb_out = 24'h0000FF;
        end else begin
            rgb_out = 24'h404040;
        end

        if (de && ((pix_x[5:0] == 6'd0) || (pix_y[5:0] == 6'd0)))
            rgb_out = 24'hFFFFFF;
    end
endmodule

`default_nettype wire