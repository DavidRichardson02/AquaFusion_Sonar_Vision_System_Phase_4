`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// cls_ascii_u8_3d
//------------------------------------------------------------------------------
// ROLE
//   Convert an 8-bit unsigned value into three ASCII decimal digits.
//
// FORMAT
//   0   -> "000"
//   7   -> "007"
//   42  -> "042"
//   255 -> "255"
//==============================================================================

module cls_ascii_u8_3d (
    input  wire [7:0]  value,
    output reg  [23:0] ascii_digits
);
    integer q;
    integer d2;
    integer d1;
    integer d0;

    always @* begin
        q  = value;
        d2 = q / 100;
        q  = q - (d2 * 100);
        d1 = q / 10;
        d0 = q - (d1 * 10);

        ascii_digits[23:16] = 8'd48 + d2;
        ascii_digits[15:8]  = 8'd48 + d1;
        ascii_digits[7:0]   = 8'd48 + d0;
    end
endmodule

`default_nettype wire