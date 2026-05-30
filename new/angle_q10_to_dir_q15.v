`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// angle_q10_to_dir_q15
//------------------------------------------------------------------------------
// Convert a 10-bit turn-space angle into signed Q1.15 direction components.
//
// Angle convention:
//   angle_q10 = 0..1023 spans one full turn.
//   0         ->   0 deg
//   256       ->  90 deg
//   512       -> 180 deg
//   768       -> 270 deg
//
// Output convention:
//   dir_x_q15 = cos(theta)
//   dir_y_q15 = sin(theta)
//
// Implementation:
//   1) Convert turn-space into integer degrees 0..359 using
//        deg = floor(angle_q10 * 360 / 1024)
//   2) Look up sin/cos from sincos_lut_180_q15.
//==============================================================================
module angle_q10_to_dir_q15 #(
    parameter         LUT_INIT_FILE     = "sincos180_q15.hex",
    parameter integer REGISTER_OUTPUTS  = 1,
    parameter integer CLAMP_DEGREE_IDX  = 1
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire [9:0]        angle_q10,
    output reg signed [15:0] dir_x_q15,
    output reg signed [15:0] dir_y_q15
);
    wire [18:0] ang_mul_360 = angle_q10 * 9'd360;
    wire [8:0]  deg_idx_u9  = ang_mul_360[18:10];

    wire signed [15:0] cos_q15_w;
    wire signed [15:0] sin_q15_w;

    sincos_lut_180_q15 #(
        .INIT_FILE       (LUT_INIT_FILE),
        .CLAMP_IDX       (CLAMP_DEGREE_IDX),
        .REGISTER_OUTPUT (0)
    ) u_sincos_lut_180_q15 (
        .clk      (clk),
        .idx      (deg_idx_u9),
        .cos_q15  (cos_q15_w),
        .sin_q15  (sin_q15_w)
    );

    generate
        if (REGISTER_OUTPUTS != 0) begin : GEN_REG_OUT
            always @(posedge clk) begin
                if (!rst_n) begin
                    dir_x_q15 <= 16'sd32767;
                    dir_y_q15 <= 16'sd0;
                end else begin
                    dir_x_q15 <= cos_q15_w;
                    dir_y_q15 <= sin_q15_w;
                end
            end
        end else begin : GEN_COMB_OUT
            always @* begin
                dir_x_q15 = cos_q15_w;
                dir_y_q15 = sin_q15_w;
            end
        end
    endgenerate
endmodule

`default_nettype wire