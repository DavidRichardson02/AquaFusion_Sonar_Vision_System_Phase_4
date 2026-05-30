`timescale 1ns/1ps
`default_nettype none

// ============================================================================
// doppler_tint_pix
// ----------------------------------------------------------------------------
// Computes tint selection and magnitude coefficient from innovation or signed
// delta. Intended for modulating dot brightness/hue.
// ============================================================================
module doppler_tint_pix #(
    parameter integer USE_INNOV = 0,

    // magnitude shaping (shift then clamp to 0..15)
    parameter integer MAG_SHIFT_INNOV = 6,   // |innov_mm| >> shift
    parameter integer MAG_SHIFT_DELTA = 0,   // |delta_px| >> shift

    parameter integer MAG_MAX = 15
) (
    input  wire        vld,   // commit-qualified (or always 1 if desired)

    input  wire signed [15:0] innov_mm,
    input  wire signed [15:0] delta_signed_px,

    output reg  [1:0]  tint_sel, // 0 neutral, 1 warm(approach), 2 cool(recede)
    output reg  [3:0]  mag_k     // 0..15
);
    function signed [15:0] abs_s16;
        input signed [15:0] v;
        begin
            abs_s16 = (v < 0) ? -v : v;
        end
    endfunction

    reg signed [15:0] src;
    reg signed [15:0] src_abs;
    reg [15:0]        mag_u16;

    always @* begin
        tint_sel = 2'd0;
        mag_k    = 4'd0;

        if (vld) begin
            src = (USE_INNOV != 0) ? innov_mm : delta_signed_px;
            src_abs = abs_s16(src);

            if (USE_INNOV != 0)
                mag_u16 = $unsigned(src_abs) >> MAG_SHIFT_INNOV;
            else
                mag_u16 = $unsigned(src_abs) >> MAG_SHIFT_DELTA;

            if (mag_u16 > MAG_MAX[15:0]) mag_k = MAG_MAX[3:0];
            else mag_k = mag_u16[3:0];

            if (src < 0)
                tint_sel = 2'd1; // approaching => warm
            else if (src > 0)
                tint_sel = 2'd2; // receding => cool
            else
                tint_sel = 2'd0;
        end
    end

endmodule

`default_nettype wire