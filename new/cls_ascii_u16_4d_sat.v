`timescale 1ns/1ps
`default_nettype none


//==============================================================================
// cls_ascii_u16_4d_sat
//------------------------------------------------------------------------------
// ROLE
//   Convert an unsigned value into four ASCII decimal digits.
//
// POLICY
//   - Values above 9999 are saturated to "9999".
//==============================================================================

module cls_ascii_u16_4d_sat (
    input  wire [15:0] value,
    output reg  [31:0] ascii_digits
);
    integer q;
    integer d3;
    integer d2;
    integer d1;
    integer d0;

    always @* begin
        q = value;
        if (q > 9999)
            q = 9999;

        d3 = q / 1000;
        q  = q - (d3 * 1000);
        d2 = q / 100;
        q  = q - (d2 * 100);
        d1 = q / 10;
        d0 = q - (d1 * 10);

        ascii_digits[31:24] = 8'd48 + d3;
        ascii_digits[23:16] = 8'd48 + d2;
        ascii_digits[15:8]  = 8'd48 + d1;
        ascii_digits[7:0]   = 8'd48 + d0;
    end
endmodule

`default_nettype wire