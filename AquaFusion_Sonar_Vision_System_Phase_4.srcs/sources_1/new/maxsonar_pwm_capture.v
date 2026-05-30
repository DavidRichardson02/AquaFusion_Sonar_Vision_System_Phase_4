`timescale 1ns/1ps
`default_nettype none

// ============================================================================
// maxsonar_pwm_capture.v
// ----------------------------------------------------------------------------
// Captures the LV-MaxSonar PWM pulse-width output and converts it to range.
//
// Sensor contract:
//   - PWM high time is 147 us per inch.
//   - A very long high pulse is treated as no-target/saturation.
//
// Output contract:
//   - dist_valid is a one-cycle clk pulse after an accepted falling edge or
//     high-pulse timeout.
//   - dist_in is whole inches, saturated to 255.
//   - dist_mm is the same sample converted to millimeters.
// ============================================================================

module maxsonar_pwm_capture #(
    parameter integer CLK_HZ            = 100_000_000,
    parameter integer MIN_HIGH_US       = 50,
    parameter integer HIGH_TIMEOUT_MS   = 50,
    parameter integer STREAM_TIMEOUT_MS = 200
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        pwm_in,

    output reg  [7:0]  dist_in,
    output reg  [15:0] dist_mm,
    output reg         dist_valid
);

    localparam [63:0] CYC_PER_US =
        ((64'd0 + CLK_HZ) / 64'd1_000_000);

    localparam [63:0] CYCLES_PER_INCH =
        ((64'd0 + CLK_HZ) * 64'd147) / 64'd1_000_000;

    localparam [63:0] CYCLES_NO_TARGET =
        ((64'd0 + CLK_HZ) * 64'd375) / 64'd10_000;

    localparam [63:0] CYCLES_MIN_HIGH_64 =
        (MIN_HIGH_US <= 0) ? 64'd0 :
        (CYC_PER_US * (64'd0 + MIN_HIGH_US));

    localparam [63:0] CYCLES_HIGH_TIMEOUT_64 =
        (HIGH_TIMEOUT_MS <= 0) ? 64'd0 :
        (((64'd0 + CLK_HZ) / 64'd1000) * (64'd0 + HIGH_TIMEOUT_MS));

    localparam [63:0] CYCLES_STREAM_TIMEOUT_64 =
        (STREAM_TIMEOUT_MS <= 0) ? 64'd0 :
        (((64'd0 + CLK_HZ) / 64'd1000) * (64'd0 + STREAM_TIMEOUT_MS));

    function [31:0] sat64_to_u32;
        input [63:0] x;
        begin
            sat64_to_u32 = (x[63:32] != 32'd0) ? 32'hFFFF_FFFF : x[31:0];
        end
    endfunction

    localparam [31:0] CYCLES_MIN_HIGH_32 =
        sat64_to_u32(CYCLES_MIN_HIGH_64);

    localparam [31:0] CYCLES_HIGH_TIMEOUT_32 =
        sat64_to_u32(CYCLES_HIGH_TIMEOUT_64);

    localparam [31:0] CYCLES_STREAM_TIMEOUT_32 =
        sat64_to_u32(CYCLES_STREAM_TIMEOUT_64);

    wire div_ok_inch = (CYCLES_PER_INCH != 64'd0);

    reg pwm_ff0;
    reg pwm_ff1;
    reg pwm_d;

    always @(posedge clk) begin
        if (rst) begin
            pwm_ff0 <= 1'b0;
            pwm_ff1 <= 1'b0;
            pwm_d   <= 1'b0;
        end else begin
            pwm_ff0 <= pwm_in;
            pwm_ff1 <= pwm_ff0;
            pwm_d   <= pwm_ff1;
        end
    end

    wire pwm  = pwm_ff1;
    wire rise =  pwm_ff1 & ~pwm_d;
    wire fall = ~pwm_ff1 &  pwm_d;

    reg [31:0] hi_cnt;
    reg        in_high;
    reg [31:0] high_watchdog;
    reg [31:0] stream_watchdog;

    function [31:0] div_round_u32;
        input [31:0] num;
        input [31:0] den;
        reg   [31:0] half;
        begin
            if (den == 32'd0) begin
                div_round_u32 = 32'd0;
            end else begin
                half = den >> 1;
                div_round_u32 = (num + half) / den;
            end
        end
    endfunction

    function [7:0] cycles_to_inches_round_sat;
        input [31:0] cyc;
        reg   [31:0] q;
        reg   [31:0] den32;
        begin
            if ({32'd0, cyc} >= CYCLES_NO_TARGET) begin
                cycles_to_inches_round_sat = 8'd255;
            end else if (!div_ok_inch) begin
                cycles_to_inches_round_sat = 8'd0;
            end else begin
                den32 = CYCLES_PER_INCH[31:0];
                q = div_round_u32(cyc, den32);
                cycles_to_inches_round_sat = (q > 32'd255) ? 8'd255 : q[7:0];
            end
        end
    endfunction

    function [15:0] cycles_to_mm_round_sat;
        input [31:0] cyc;
        reg   [63:0] num;
        reg   [63:0] den;
        reg   [63:0] q;
        begin
            if ({32'd0, cyc} >= CYCLES_NO_TARGET) begin
                cycles_to_mm_round_sat = 16'd6477;
            end else if (!div_ok_inch) begin
                cycles_to_mm_round_sat = 16'd0;
            end else begin
                num = (64'd0 + cyc) * 64'd254;
                den = CYCLES_PER_INCH * 64'd10;
                q = (den == 64'd0) ? 64'd0 : ((num + (den >> 1)) / den);
                cycles_to_mm_round_sat = (q > 64'd65535) ? 16'hFFFF : q[15:0];
            end
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            hi_cnt          <= 32'd0;
            in_high         <= 1'b0;
            high_watchdog   <= 32'd0;
            stream_watchdog <= 32'd0;
            dist_in         <= 8'd0;
            dist_mm         <= 16'd0;
            dist_valid      <= 1'b0;
        end else begin
            dist_valid <= 1'b0;

            if (CYCLES_STREAM_TIMEOUT_32 != 32'd0) begin
                if (rise) begin
                    stream_watchdog <= 32'd0;
                end else if (stream_watchdog != 32'hFFFF_FFFF) begin
                    stream_watchdog <= stream_watchdog + 32'd1;
                end
            end else begin
                stream_watchdog <= 32'd0;
            end

            if (rise) begin
                in_high       <= 1'b1;
                hi_cnt        <= 32'd1;
                high_watchdog <= 32'd0;
            end

            if (in_high) begin
                if ((CYCLES_HIGH_TIMEOUT_32 != 32'd0) &&
                    (high_watchdog >= CYCLES_HIGH_TIMEOUT_32)) begin
                    in_high    <= 1'b0;
                    dist_in    <= 8'd255;
                    dist_mm    <= 16'd6477;
                    dist_valid <= 1'b1;
                end else if (fall) begin
                    in_high <= 1'b0;
                    if (hi_cnt >= CYCLES_MIN_HIGH_32) begin
                        dist_in    <= cycles_to_inches_round_sat(hi_cnt);
                        dist_mm    <= cycles_to_mm_round_sat(hi_cnt);
                        dist_valid <= 1'b1;
                    end
                end else if (pwm) begin
                    if (hi_cnt != 32'hFFFF_FFFF)
                        hi_cnt <= hi_cnt + 32'd1;

                    if ((CYCLES_HIGH_TIMEOUT_32 != 32'd0) &&
                        (high_watchdog != 32'hFFFF_FFFF)) begin
                        high_watchdog <= high_watchdog + 32'd1;
                    end
                end
            end
        end
    end

endmodule

`default_nettype wire
