`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// hud_fmt_dec3_sat16.v
//------------------------------------------------------------------------------
// Formats a 16-bit unsigned value into exactly three ASCII decimal digits.
//
// Behavior:
//   - value is saturated to MAX_VAL
//   - output is always three digits, zero-padded
//
// Examples:
//   0   -> "000"
//   7   -> "007"
//   47  -> "047"
//   123 -> "123"
//
// Notes:
//   - Purely combinational
//   - No RAM
//   - No division operator
//   - Bounded subtract implementation
//==============================================================================

module hud_fmt_dec3_sat16 #(
    parameter integer MAX_VAL = 999
)(
    input  wire [15:0] value_in,
    output reg  [23:0] ascii3
);

    integer value_sat;
    integer rem_i;
    integer hundreds_i;
    integer tens_i;
    integer ones_i;
    integer k;

    reg [7:0] ascii_h;
    reg [7:0] ascii_t;
    reg [7:0] ascii_o;

    always @* begin
        //----------------------------------------------------------------------
        // Saturate
        //----------------------------------------------------------------------
        if (value_in > MAX_VAL[15:0])
            value_sat = MAX_VAL;
        else
            value_sat = value_in;

        //----------------------------------------------------------------------
        // Hundreds
        //----------------------------------------------------------------------
        rem_i      = value_sat;
        hundreds_i = 0;
        for (k = 0; k < 9; k = k + 1) begin
            if ((hundreds_i < 9) && (rem_i >= 100)) begin
                rem_i      = rem_i - 100;
                hundreds_i = hundreds_i + 1;
            end
        end

        //----------------------------------------------------------------------
        // Tens
        //----------------------------------------------------------------------
        tens_i = 0;
        for (k = 0; k < 9; k = k + 1) begin
            if ((tens_i < 9) && (rem_i >= 10)) begin
                rem_i  = rem_i - 10;
                tens_i = tens_i + 1;
            end
        end

        //----------------------------------------------------------------------
        // Ones
        //----------------------------------------------------------------------
        ones_i = rem_i;

        //----------------------------------------------------------------------
        // ASCII pack
        //----------------------------------------------------------------------
        ascii_h = 8'h30 + hundreds_i[7:0];
        ascii_t = 8'h30 + tens_i[7:0];
        ascii_o = 8'h30 + ones_i[7:0];

        ascii3 = {ascii_h, ascii_t, ascii_o};
    end

endmodule

`default_nettype wire