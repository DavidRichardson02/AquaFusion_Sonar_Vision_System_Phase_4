`timescale 1ns/1ps
`default_nettype none

// ============================================================================
// front_sweep_line_pix
// ----------------------------------------------------------------------------
// Thin "front sweep" predicate derived from beam-space quantities.
// ============================================================================
module front_sweep_line_pix #(
    parameter integer FRONT_W_PX = 1
) (
    input  wire        in_ray_segment,
    input  wire signed [15:0] cross_i,
    output wire        front_ink
);
    function signed [15:0] abs_s16;
        input signed [15:0] v;
        begin
            abs_s16 = (v < 0) ? -v : v;
        end
    endfunction

    assign front_ink = in_ray_segment && (abs_s16(cross_i) <= $signed(FRONT_W_PX));

endmodule

`default_nettype wire