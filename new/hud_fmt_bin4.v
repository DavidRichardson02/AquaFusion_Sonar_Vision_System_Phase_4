`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// hud_fmt_bin4.v
//------------------------------------------------------------------------------
// Formats a 4-bit field into exactly four ASCII binary digits.
//
// Example:
//   4'b0001 -> "0001"
//   4'b1010 -> "1010"
//
// Purely combinational.
//==============================================================================

module hud_fmt_bin4 (
    input  wire [3:0] value_in,
    output wire [31:0] ascii4
);

    assign ascii4[31:24] = value_in[3] ? 8'h31 : 8'h30;
    assign ascii4[23:16] = value_in[2] ? 8'h31 : 8'h30;
    assign ascii4[15: 8] = value_in[1] ? 8'h31 : 8'h30;
    assign ascii4[ 7: 0] = value_in[0] ? 8'h31 : 8'h30;

endmodule

`default_nettype wire