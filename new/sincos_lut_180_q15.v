`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// sincos_lut_180_q15.v  (FINAL)
//------------------------------------------------------------------------------
// ROLE
//   Full-circle (0..359 degree) sin/cos source in signed Q1.15, implemented
//   using a canonical 0..179 ROM plus deterministic 180-degree symmetry.
//
// ROM DATA CONTRACT
//   - ROM contains 180 entries for degrees 0..179 inclusive
//   - rom[i][31:16] = cos(i) in Q1.15
//   - rom[i][15:0]  = sin(i) in Q1.15
//
// FULL-CIRCLE DERIVATION
//   For idx in 180..359:
//     sin(idx) = -sin(idx-180)
//     cos(idx) = -cos(idx-180)
//
// INDEX POLICY
//   - idx valid range: 0..359
//   - CLAMP_IDX=1: idx>=360 clamps to 359
//   - CLAMP_IDX=0: idx>=360 returns fallback constants
//
// TIMING
//   REGISTER_OUTPUT=0 => combinational outputs
//   REGISTER_OUTPUT=1 => 1-cycle registered outputs
//
// SYNTHESIS
//   - Verilog-2001
//   - No latches
//   - Bounded logic
//==============================================================================
module sincos_lut_180_q15 #(
    parameter         INIT_FILE = "sincos180_q15.hex",

    parameter integer CLAMP_IDX = 1,

    parameter signed [15:0] FB_COS_Q15 = 16'sd32767,
    parameter signed [15:0] FB_SIN_Q15 = 16'sd0,

    parameter integer REGISTER_OUTPUT = 0
)(
    input  wire        clk,
    input  wire [8:0]  idx,      // 0..359 valid
    output reg  signed [15:0] cos_q15,
    output reg  signed [15:0] sin_q15
);

    // ROM: 0..179 degrees (180 entries)
    reg [31:0] rom [0:179];

    // ROM init
    initial begin
        $readmemh(INIT_FILE, rom);
    end

    // Validity check and optional clamp
    wire idx_valid_360 = (idx < 9'd360);
    wire [8:0] idx_eff_360 =
        idx_valid_360 ? idx :
        ((CLAMP_IDX != 0) ? 9'd359 : 9'd0);

    wire upper_half = (idx_eff_360 >= 9'd180);
    wire [8:0] idx_minus_180 = idx_eff_360 - 9'd180;

    // Base index 0..179 into ROM
    wire [7:0] idx_base_180 = upper_half ? idx_minus_180[7:0] : idx_eff_360[7:0];

    wire signed [15:0] cos_base = $signed(rom[idx_base_180][31:16]);
    wire signed [15:0] sin_base = $signed(rom[idx_base_180][15:0]);

    wire signed [15:0] cos_full = upper_half ? (-cos_base) : cos_base;
    wire signed [15:0] sin_full = upper_half ? (-sin_base) : sin_base;

    generate
        if (REGISTER_OUTPUT != 0) begin : GEN_REG
            always @(posedge clk) begin
                if (idx_valid_360 || (CLAMP_IDX != 0)) begin
                    cos_q15 <= cos_full;
                    sin_q15 <= sin_full;
                end else begin
                    cos_q15 <= FB_COS_Q15;
                    sin_q15 <= FB_SIN_Q15;
                end
            end
        end else begin : GEN_COMB
            always @* begin
                if (idx_valid_360 || (CLAMP_IDX != 0)) begin
                    cos_q15 = cos_full;
                    sin_q15 = sin_full;
                end else begin
                    cos_q15 = FB_COS_Q15;
                    sin_q15 = FB_SIN_Q15;
                end
            end
        end
    endgenerate

endmodule

`default_nettype wire