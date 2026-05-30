`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// cls_ascii_u32_6d_mod
//------------------------------------------------------------------------------
// ROLE
//   Convert an unsigned value into six ASCII decimal digits.
//
// POLICY
//   - Display is modulo 1,000,000.
//   - This keeps the field width fixed for the CLS second line.
//==============================================================================

module cls_ascii_u32_6d_mod (
    input  wire [31:0] value,
    output reg  [47:0] ascii_digits
);
    integer q;
    integer d5;
    integer d4;
    integer d3;
    integer d2;
    integer d1;
    integer d0;

    always @* begin
        q  = value % 1000000;

        d5 = q / 100000;
        q  = q - (d5 * 100000);
        d4 = q / 10000;
        q  = q - (d4 * 10000);
        d3 = q / 1000;
        q  = q - (d3 * 1000);
        d2 = q / 100;
        q  = q - (d2 * 100);
        d1 = q / 10;
        d0 = q - (d1 * 10);

        ascii_digits[47:40] = 8'd48 + d5;
        ascii_digits[39:32] = 8'd48 + d4;
        ascii_digits[31:24] = 8'd48 + d3;
        ascii_digits[23:16] = 8'd48 + d2;
        ascii_digits[15:8]  = 8'd48 + d1;
        ascii_digits[7:0]   = 8'd48 + d0;
    end
endmodule

`default_nettype wire