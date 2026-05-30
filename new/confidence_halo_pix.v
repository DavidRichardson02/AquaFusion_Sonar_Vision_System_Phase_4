`timescale 1ns/1ps
`default_nettype none

// ============================================================================
// confidence_halo_pix
// ----------------------------------------------------------------------------
// Ring band around (ex_s, ey_s) indicating uncertainty.
// - Uses p_q16 when enabled, otherwise uses conf_k fallback.
// - Pure combinational per pixel.
// ============================================================================
module confidence_halo_pix #(
    parameter integer USE_P_Q16     = 0,

    // p_q16 mapping
    parameter integer P_SHIFT       = 16,  // halo_r_px = p_q16 >> P_SHIFT
    parameter integer HALO_R_MIN_PX = 3,
    parameter integer HALO_R_MAX_PX = 18,

    // ring thickness
    parameter integer HALO_THICK_PX = 1
) (
    input  wire [9:0]  pix_x,
    input  wire [9:0]  pix_y,

    input  wire signed [15:0] ex_s,
    input  wire signed [15:0] ey_s,

    input  wire        in_widget,

    // preferred uncertainty input
    input  wire [31:0] p_q16,

    // fallback confidence (0..15), higher means more confident
    input  wire [3:0]  conf_k,

    output wire        halo_ink
);
    // ------------------------------------------------------------
    // Radius selection
    // ------------------------------------------------------------
    wire [15:0] halo_r_raw_p = p_q16[31:P_SHIFT];
    wire [15:0] halo_r_p =
        (halo_r_raw_p < HALO_R_MIN_PX) ? HALO_R_MIN_PX[15:0] :
        (halo_r_raw_p > HALO_R_MAX_PX) ? HALO_R_MAX_PX[15:0] :
                                          halo_r_raw_p;

    // fallback mapping: conf_k high => small radius
    wire [4:0] inv_conf = 5'd15 - {1'b0, conf_k};
    wire [15:0] halo_r_f =
        HALO_R_MIN_PX[15:0] +
        ((inv_conf * (HALO_R_MAX_PX - HALO_R_MIN_PX)) / 15);

    wire [15:0] halo_r_px = (USE_P_Q16 != 0) ? halo_r_p : halo_r_f;

    // ------------------------------------------------------------
    // Ring band test using squared distances
    // ------------------------------------------------------------
    wire signed [15:0] hx = $signed({1'b0, pix_x}) - ex_s;
    wire signed [15:0] hy = $signed({1'b0, pix_y}) - ey_s;

    wire signed [31:0] h2 = ($signed(hx) * $signed(hx)) + ($signed(hy) * $signed(hy));

    wire [15:0] r0 = (halo_r_px > HALO_THICK_PX[15:0]) ? (halo_r_px - HALO_THICK_PX[15:0]) : 16'd0;
    wire [15:0] r1 = halo_r_px + HALO_THICK_PX[15:0];

    wire signed [31:0] r0_2 = $signed(r0) * $signed(r0);
    wire signed [31:0] r1_2 = $signed(r1) * $signed(r1);

    assign halo_ink = in_widget && (h2 >= r0_2) && (h2 <= r1_2);

endmodule

`default_nettype wire