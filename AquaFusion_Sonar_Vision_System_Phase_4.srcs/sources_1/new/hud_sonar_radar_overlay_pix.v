`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// hud_sonar_radar_overlay_pix.v
//==============================================================================
// Combines ALL functionality appearing in either provided module variant:
//
// Core radar (PIX-domain, deterministic):
//   - Continuous sweep coherence (frame_tick-driven internal sweep angle)
//   - Optional external angle selection (SWEEP_USE_EXT_ANGLE)
//
// Measurement staging (tear-free):
//   - sample_upd_pix -> staged -> committed on next cycle (sample_upd_d1)
//
// Beam rendering:
//   - Wedge core + soft edge (beam-space proj/cross math)
//   - Thin front sweep line predicate (front_sweep_line_pix) above wedge
//
// Target rendering:
//   - Endpoint dot (beam-space box test)
//   - Confidence halo ring (confidence_halo_pix) below dot
//   - Doppler-ish tint (doppler_tint_pix) commit-latched, frame-stable
//
// History / persistence:
//   - Trail shift-register (frame_tick or commit-driven, selectable)
//   - Optional BRAM-backed phosphor persistence plane (radar_phosphor_plane_pix)
//     with sweep glow deposits (sweep_glow_depositor_pix)
//   - 1-cycle painter to align BRAM sample (phos_I_d1/in_phos_d1)
//
// Telemetry visuals (bottom strip):
//   - Age bar, delta bar, stale blink marker
//   - Sparkline (SPARK_N), envelope (min/max/peak-hold), histogram (HIST_BINS)
//   - Compact 3x5 labels ("AGE", "DLT", "SPK", "HST", "STL") (no font ROM)
//
// Border visuals:
//   - Border + corner ticks
//
// Telemetry packet output:
//   - 64-bit packet emitted on commit (sample_upd_d1)
//
// External dependencies assumed present:
//   - angle_q10_to_dir_q15
//   - radar_phosphor_plane_pix
//   - sweep_glow_depositor_pix
//   - front_sweep_line_pix
//   - confidence_halo_pix
//   - doppler_tint_pix
//==============================================================================

module hud_sonar_radar_overlay_pix #(
    // Screen/widget placement
    parameter integer X0 = 16,
    parameter integer Y0 = 16,
    parameter integer W  = 256,
    parameter integer H  = 256,

    // Radar geometry
    parameter integer R_MAX_PX = 120,

    // Beam wedge controls
    parameter integer BEAM_EN        = 1,
    parameter integer BEAM_W_MAX_PX  = 24,
    parameter integer BEAM_SOFT_EDGE = 1,

    // Ring geometry (runtime enable still required)
    parameter integer RING_STEP_PX   = 20,

    // Distance-to-pixel scaling: r_px = dist_mm >> MM_PER_PX_SHIFT
    parameter integer MM_PER_PX_SHIFT = 5,

    // Endpoint dot thickness in (proj,cross) space
    parameter integer DOT_THICK_PX = 2,

    // History trail storage (runtime enable still required)
    parameter integer TRAIL_N            = 16,
    parameter integer TRAIL_DOT_THICK_PX = 1,

    // Trail staging policy
    parameter integer TRAIL_FOLLOWS_SWEEP = 1,  // 1: frame_tick, 0: sample commit

    // Stale tint enable
    parameter integer STALE_TINT_EN = 1,

    // Optional telemetry output enable
    parameter integer TELEMETRY_EN = 1,

    // ---------------- Telemetry visuals ----------------
    parameter integer EN_VIS_BORDER      = 1,
    parameter integer EN_VIS_TELEM_STRIP = 1,

    // Bottom telemetry strip geometry (inside widget)
    parameter integer TELEM_STRIP_H      = 32,
    parameter integer TELEM_PAD_X        = 4,
    parameter integer TELEM_PAD_Y        = 3,

    // Age bar controls (frames since last sample commit)
    parameter integer AGE_SAT_FRAMES     = 120,

    // Sparkline controls
    parameter integer SPARK_N            = 64,
    parameter integer EN_VIS_SPARKLINE   = 1,

    // Delta bar scaling (abs jump in r_px pixels -> bar height)
    parameter integer DELTA_SAT_PX       = 32,

    // ---------------- Confidence tint ----------------
    parameter integer EN_CONF_TINT       = 1,
    parameter integer CONF_SEV_MAX       = 15,   // internal 0..15 scale

    // ---------------- Inter-arrival histogram ----------------
    parameter integer EN_VIS_HIST        = 1,
    parameter integer HIST_BINS          = 8,
    parameter integer HIST_MAX_FRAMES    = 120,  // clamp inter-arrival measurement
    parameter integer HIST_BAR_W         = 3,    // pixels per bin
    parameter integer HIST_MAX_COUNT     = 15,   // saturating count per bin (4-bit)
    parameter integer HIST_DECAY_PERIOD  = 8,    // frames per global decay step

    // ---------------- Spark envelope ----------------
    parameter integer EN_VIS_ENVELOPE    = 1,
    parameter integer PEAK_DECAY_PERIOD  = 6,    // frames per peak-hold decay step

    // ---------------- Range gate bands ----------------
    parameter integer EN_RANGE_GATES     = 1,
    parameter integer GATE0_PX           = 40,   // near threshold radius
    parameter integer GATE1_PX           = 80,   // mid threshold radius

    // ---------------- Continuous sweep policy ----------------
    parameter integer SWEEP_EN              = 1,
    parameter [9:0]   SWEEP_STEP_Q10        = 10'd3,
    parameter integer SWEEP_USE_EXT_ANGLE   = 0,  // 0: internal sweep drives direction
                                                 // 1: angle_q10_pix drives direction

    // ---------------- Labels (3x5) ----------------
    parameter integer EN_LABELS          = 1,
    parameter [11:0]  LABEL_RGB         = 12'hEEE,
    parameter integer LABEL_SCALE        = 1,      // integer scale (>=1)

    // ---------------- Phosphor persistence plane ----------------
    parameter integer EN_PHOS          = 1,
    parameter integer PHOS_W           = 128,
    parameter integer PHOS_H           = 128,
    parameter integer PHOS_AW          = 14,   // >= log2(PHOS_W*PHOS_H)
    parameter [7:0]   PHOS_DECAY       = 8'd2,
    parameter [7:0]   PHOS_HIT_ADD     = 8'd80,
    parameter [7:0]   PHOS_SWEEP_ADD   = 8'd6,
    parameter integer PHOS_MAINT_K     = 256,  // decay cells per frame

    // Optional phosphor blending knobs (present in second variant)
    parameter [11:0]  PHOS_RGB_MAX     = 12'h0F0, // green phosphor peak (used as a cap cue)
    parameter integer PHOS_BLEND_SHIFT = 4,       // intensity scale -> nibble add

    // ---------------- Halo ----------------
    parameter integer EN_HALO          = 1,
    parameter integer HALO_THICK_PX    = 1,
    parameter integer HALO_R_MIN_PX    = 3,
    parameter integer HALO_R_MAX_PX    = 18
)(
    input  wire        clk_pix,
    input  wire        rst_pix,
    input  wire [9:0]  pix_x,
    input  wire [9:0]  pix_y,
    input  wire        active_video,
    input  wire        frame_tick,

    // Tear-free sample commit pulse (1-cycle)
    input  wire        sample_upd_pix,

    // Sample data (assumed stable on sample_upd_pix)
    input  wire [15:0] dist_mm_pix,
    input  wire [9:0]  angle_q10_pix,
    input  wire        dist_stale_pix,

    // Runtime enables
    input  wire        ring_en_pix,
    input  wire        trail_en_pix,

    input  wire [11:0] rgb_bg,
    output reg  [11:0] rgb_out,

    // -------------------------------------------------------------------------
    // Telemetry outputs (optional; safe to leave unconnected)
    // -------------------------------------------------------------------------
    output reg  [63:0] radar_telem_pix,
    output reg         radar_telem_vld_pix
);

    //--------------------------------------------------------------------------
    // 0) Widget bounds and center
    //--------------------------------------------------------------------------
    localparam integer X1 = X0 + W - 1;
    localparam integer Y1 = Y0 + H - 1;
    localparam integer CX = X0 + (W/2);
    localparam integer CY = Y0 + (H/2);

    localparam [9:0] X0_10 = X0[9:0];
    localparam [9:0] X1_10 = X1[9:0];
    localparam [9:0] Y0_10 = Y0[9:0];
    localparam [9:0] Y1_10 = Y1[9:0];

    wire in_widget =
        active_video &&
        (pix_x >= X0_10) && (pix_x <= X1_10) &&
        (pix_y >= Y0_10) && (pix_y <= Y1_10);

    //--------------------------------------------------------------------------
    // 1) Helper functions (pure combinational)
    //--------------------------------------------------------------------------
    function signed [15:0] abs_s16;
        input signed [15:0] v;
        begin
            abs_s16 = (v < 0) ? -v : v;
        end
    endfunction

    function [15:0] u16_min;
        input [15:0] a;
        input [15:0] b;
        begin
            u16_min = (a < b) ? a : b;
        end
    endfunction

    function [7:0] u8_min;
        input [7:0] a;
        input [7:0] b;
        begin
            u8_min = (a < b) ? a : b;
        end
    endfunction

    function [3:0] mix4;
        input [3:0] a;
        input [3:0] b;
        input [3:0] k;
        reg [9:0] num;
        begin
            num = (a * (4'd15 - k)) + (b * k) + 10'd7;
            mix4 = num / 10'd15;
        end
    endfunction

    function [3:0] scale4;
        input [3:0] a;
        input [3:0] k;
        reg [8:0] num;
        begin
            num = (a * k) + 9'd7;
            scale4 = num / 9'd15;
        end
    endfunction

    function [3:0] add_sat4;
        input [3:0] a;
        input [3:0] b;
        reg [4:0] s;
        begin
            s = a + b;
            add_sat4 = s[4] ? 4'hF : s[3:0];
        end
    endfunction

    //--------------------------------------------------------------------------
    // 2) Continuous sweep angle (PIX, independent of sample commits)
    //--------------------------------------------------------------------------
    reg [9:0] angle_q10_sweep;

    always @(posedge clk_pix) begin
        if (rst_pix) begin
            angle_q10_sweep <= 10'd0;
        end else if (SWEEP_EN != 0) begin
            if (frame_tick) begin
                angle_q10_sweep <= angle_q10_sweep + SWEEP_STEP_Q10;
            end
        end
    end

    // Unsigned turn-space angle used by the helper conversion block.
    wire [9:0] angle_q10_dir_u =
        (SWEEP_USE_EXT_ANGLE != 0) ? angle_q10_pix : angle_q10_sweep;

    //--------------------------------------------------------------------------
    // 4) Angle -> direction (Q1.15), registered outputs
    //--------------------------------------------------------------------------
    wire signed [15:0] dir_x_q15; // cos
    wire signed [15:0] dir_y_q15; // sin

    angle_q10_to_dir_q15 #(
        .REGISTER_OUTPUTS(1)
    ) u_dir_pix (
        .clk       (clk_pix),
        .rst_n     (~rst_pix),
        .angle_q10 (angle_q10_dir_u),
        .dir_x_q15 (dir_x_q15),
        .dir_y_q15 (dir_y_q15)
    );

    // Keep committed telemetry observability in the original turn-space format.
    // Replace prior angle capture references accordingly:
    //   angle_q10_s <= angle_q10_dir_u;
    
    
    
    //--------------------------------------------------------------------------
    // 3) Sample staging and aligned commit (commit-driven telemetry preserved)
    //--------------------------------------------------------------------------
    reg        sample_upd_d1;

    reg [15:0] dist_mm_s;
    reg [9:0]  angle_q10_s;     // captured for telemetry observability
    reg        dist_stale_s;
    reg [15:0] r_px_s_u16;

    reg [15:0] dist_mm_r;
    reg [9:0]  angle_q10_r;     // committed for telemetry observability
    reg        dist_stale_r;
    reg [15:0] r_px_u16;

    always @(posedge clk_pix) begin
        if (rst_pix) begin
            sample_upd_d1 <= 1'b0;

            dist_mm_s     <= 16'd0;
            angle_q10_s   <= 10'd0;
            dist_stale_s  <= 1'b0;
            r_px_s_u16    <= 16'd0;

            dist_mm_r     <= 16'd0;
            angle_q10_r   <= 10'd0;
            dist_stale_r  <= 1'b0;
            r_px_u16      <= 16'd0;
        end else begin
            sample_upd_d1 <= sample_upd_pix;

            if (sample_upd_pix) begin
                dist_mm_s    <= dist_mm_pix;
                angle_q10_s  <= angle_q10_dir_u; // capture bearing at sample epoch
                dist_stale_s <= dist_stale_pix;
                r_px_s_u16   <= u16_min((dist_mm_pix >> MM_PER_PX_SHIFT), R_MAX_PX[15:0]);
            end

            if (sample_upd_d1) begin
                dist_mm_r    <= dist_mm_s;
                angle_q10_r  <= angle_q10_s;
                dist_stale_r <= dist_stale_s;
                r_px_u16     <= r_px_s_u16;
            end
        end
    end


    //--------------------------------------------------------------------------
    // 5) Endpoint in pixel space (screen coordinates)
    //--------------------------------------------------------------------------
    wire signed [31:0] dx_end_q15 = $signed({1'b0, r_px_u16}) * $signed(dir_x_q15);
    wire signed [31:0] dy_end_q15 = $signed({1'b0, r_px_u16}) * $signed(dir_y_q15);
    wire signed [15:0] dx_end_i   = dx_end_q15 >>> 15;
    wire signed [15:0] dy_end_i   = dy_end_q15 >>> 15;

    wire signed [15:0] ex_s = $signed(CX) + dx_end_i;
    wire signed [15:0] ey_s = $signed(CY) - dy_end_i;

    //--------------------------------------------------------------------------
    // 6) History trail shift-register (selectable update source)
    //--------------------------------------------------------------------------
    integer ti;
    reg signed [15:0] trail_x [0:TRAIL_N-1];
    reg signed [15:0] trail_y [0:TRAIL_N-1];

    wire trail_shift_evt =
        (TRAIL_FOLLOWS_SWEEP != 0) ? frame_tick : sample_upd_d1;

    always @(posedge clk_pix) begin
        if (rst_pix) begin
            for (ti = 0; ti < TRAIL_N; ti = ti + 1) begin
                trail_x[ti] <= 16'sd0;
                trail_y[ti] <= 16'sd0;
            end
        end else if (trail_en_pix && trail_shift_evt) begin
            trail_x[0] <= ex_s;
            trail_y[0] <= ey_s;
            for (ti = 1; ti < TRAIL_N; ti = ti + 1) begin
                trail_x[ti] <= trail_x[ti-1];
                trail_y[ti] <= trail_y[ti-1];
            end
        end
    end

    //--------------------------------------------------------------------------
    // 7) Per-pixel vectors and beam-space coordinates
    //--------------------------------------------------------------------------
    wire signed [15:0] vx_s = $signed({1'b0, pix_x}) - $signed(CX);
    wire signed [15:0] vy_s = $signed(CY) - $signed({1'b0, pix_y});

    wire signed [31:0] proj_q15 =
        ($signed(vx_s) * $signed(dir_x_q15)) +
        ($signed(vy_s) * $signed(dir_y_q15));
    wire signed [15:0] proj_i = proj_q15 >>> 15;

    wire signed [31:0] cross_q15 =
        ($signed(vx_s) * $signed(dir_y_q15)) -
        ($signed(vy_s) * $signed(dir_x_q15));
    wire signed [15:0] cross_i = cross_q15 >>> 15;

    wire in_ray_segment =
        in_widget &&
        (proj_i >= 16'sd0) &&
        (proj_i <= $signed(R_MAX_PX));

    //--------------------------------------------------------------------------
    // 8) Beam wedge classification
    //--------------------------------------------------------------------------
    wire signed [15:0] cross_abs = abs_s16(cross_i);

    wire signed [31:0] lhs_cross = $signed(cross_abs) * $signed(R_MAX_PX);
    wire signed [31:0] rhs_proj  = $signed(proj_i)    * $signed(BEAM_W_MAX_PX);

    wire beam_core_ink =
        (BEAM_EN != 0) &&
        in_ray_segment &&
        (lhs_cross <= rhs_proj);

    wire signed [31:0] rhs_proj_soft = rhs_proj + $signed(R_MAX_PX);

    wire beam_edge_ink =
        (BEAM_EN != 0) &&
        (BEAM_SOFT_EDGE != 0) &&
        in_ray_segment &&
        (lhs_cross <= rhs_proj_soft) &&
        (lhs_cross >  rhs_proj);

    reg [1:0] beam_band;
    always @* begin
        beam_band = 2'd0;
        if (proj_i > $signed((R_MAX_PX*2)/3))      beam_band = 2'd2;
        else if (proj_i > $signed(R_MAX_PX/3))     beam_band = 2'd1;
        else                                       beam_band = 2'd0;
    end

    //--------------------------------------------------------------------------
    // 9) Dot + center marker
    //--------------------------------------------------------------------------
    wire signed [15:0] r_px_s = $signed({1'b0, r_px_u16});

    wire dot_ink =
        in_ray_segment &&
        (abs_s16(proj_i - r_px_s) <= $signed(DOT_THICK_PX)) &&
        (abs_s16(cross_i)         <= $signed(DOT_THICK_PX));

    wire center_ink =
        in_widget &&
        (abs_s16($signed({1'b0, pix_x}) - $signed(CX)) <= 16'sd1) &&
        (abs_s16($signed({1'b0, pix_y}) - $signed(CY)) <= 16'sd1);

    //--------------------------------------------------------------------------
    // 10) Rings via squared-distance banding
    //--------------------------------------------------------------------------
    wire signed [31:0] r2 =
        ($signed(vx_s) * $signed(vx_s)) +
        ($signed(vy_s) * $signed(vy_s));

    reg ring_ink;
    integer rk;
    integer rk2;
    integer band;

    always @* begin
        ring_ink = 1'b0;
        if (ring_en_pix && in_widget) begin
            band = (RING_STEP_PX * 2);
            for (rk = RING_STEP_PX; rk <= R_MAX_PX; rk = rk + RING_STEP_PX) begin
                rk2 = rk * rk;
                if ($signed(r2) >= $signed(rk2 - band) && $signed(r2) <= $signed(rk2 + band))
                    ring_ink = 1'b1;
            end
        end
    end

    //--------------------------------------------------------------------------
    // 11) Trail dots
    //--------------------------------------------------------------------------
    reg trail_ink;
    integer tj;
    reg signed [15:0] dx_t;
    reg signed [15:0] dy_t;

    always @* begin
        trail_ink = 1'b0;
        if (trail_en_pix && in_widget) begin
            for (tj = 0; tj < TRAIL_N; tj = tj + 1) begin
                dx_t = $signed({1'b0, pix_x}) - trail_x[tj];
                dy_t = $signed({1'b0, pix_y}) - trail_y[tj];
                if ((abs_s16(dx_t) <= $signed(TRAIL_DOT_THICK_PX)) &&
                    (abs_s16(dy_t) <= $signed(TRAIL_DOT_THICK_PX)))
                    trail_ink = 1'b1;
            end
        end
    end

    //==========================================================================
    // 12) Telemetry state (commit-driven)
    //==========================================================================
    reg [7:0] age_frames_u8;

    always @(posedge clk_pix) begin
        if (rst_pix) begin
            age_frames_u8 <= 8'hFF;
        end else begin
            if (sample_upd_d1) begin
                age_frames_u8 <= 8'd0;
            end else if (frame_tick) begin
                if (age_frames_u8 != 8'hFF) begin
                    if (age_frames_u8 < AGE_SAT_FRAMES[7:0])
                        age_frames_u8 <= age_frames_u8 + 8'd1;
                end
            end
        end
    end

    reg [15:0] r_px_prev_u16;
    reg [7:0]  delta_px_u8;

    wire [15:0] r_px_now_u16 = r_px_u16;
    wire [15:0] r_px_diff_u16 =
        (r_px_now_u16 >= r_px_prev_u16) ? (r_px_now_u16 - r_px_prev_u16) :
                                          (r_px_prev_u16 - r_px_now_u16);

    always @(posedge clk_pix) begin
        if (rst_pix) begin
            r_px_prev_u16 <= 16'd0;
            delta_px_u8   <= 8'd0;
        end else if (sample_upd_d1) begin
            r_px_prev_u16 <= r_px_now_u16;
            delta_px_u8   <= u8_min(r_px_diff_u16[7:0], DELTA_SAT_PX[7:0]);
        end
    end

    // Signed delta for doppler tint
    wire signed [15:0] delta_signed_px =
        (r_px_now_u16 >= r_px_prev_u16) ? $signed({1'b0, (r_px_now_u16 - r_px_prev_u16)}) :
                                          -$signed({1'b0, (r_px_prev_u16 - r_px_now_u16)});

    // Inter-arrival frames
    reg [7:0] ia_frames_u8;
    reg [7:0] ia_counter_u8;

    always @(posedge clk_pix) begin
        if (rst_pix) begin
            ia_frames_u8  <= 8'd0;
            ia_counter_u8 <= 8'd0;
        end else begin
            if (sample_upd_d1) begin
                ia_frames_u8  <= u8_min(ia_counter_u8, HIST_MAX_FRAMES[7:0]);
                ia_counter_u8 <= 8'd0;
            end else if (frame_tick) begin
                if (ia_counter_u8 < HIST_MAX_FRAMES[7:0])
                    ia_counter_u8 <= ia_counter_u8 + 8'd1;
            end
        end
    end

    // Histogram (decayed)
    reg [3:0] hist [0:HIST_BINS-1];
    integer hi;
    reg [7:0] hist_decay_ctr;
    wire hist_decay_tick = (frame_tick && (hist_decay_ctr == (HIST_DECAY_PERIOD[7:0] - 8'd1)));
    integer bin_i;

    always @(posedge clk_pix) begin
        if (rst_pix) begin
            hist_decay_ctr <= 8'd0;
            for (hi = 0; hi < HIST_BINS; hi = hi + 1)
                hist[hi] <= 4'd0;
        end else begin
            if (frame_tick) begin
                if (hist_decay_ctr == (HIST_DECAY_PERIOD[7:0] - 8'd1))
                    hist_decay_ctr <= 8'd0;
                else
                    hist_decay_ctr <= hist_decay_ctr + 8'd1;
            end

            if (hist_decay_tick && (EN_VIS_HIST != 0)) begin
                for (hi = 0; hi < HIST_BINS; hi = hi + 1) begin
                    if (hist[hi] != 4'd0)
                        hist[hi] <= hist[hi] - 4'd1;
                end
            end

            if ((EN_VIS_HIST != 0) && sample_upd_d1) begin
                bin_i = (ia_frames_u8 * HIST_BINS) / (HIST_MAX_FRAMES + 1);
                if (bin_i < 0) bin_i = 0;
                if (bin_i > (HIST_BINS-1)) bin_i = (HIST_BINS-1);

                if (hist[bin_i] < HIST_MAX_COUNT[3:0])
                    hist[bin_i] <= hist[bin_i] + 4'd1;
            end
        end
    end

    // Blink for stale indicator
    reg [5:0] blink_ctr;
    always @(posedge clk_pix) begin
        if (rst_pix) blink_ctr <= 6'd0;
        else if (frame_tick) blink_ctr <= blink_ctr + 6'd1;
    end
    wire blink_on = blink_ctr[5];

    // Sparkline shift register (range samples)
    reg [7:0] spark [0:SPARK_N-1];
    integer si;

    always @(posedge clk_pix) begin
        if (rst_pix) begin
            for (si = 0; si < SPARK_N; si = si + 1)
                spark[si] <= 8'd0;
        end else if ((EN_VIS_SPARKLINE != 0) && sample_upd_d1) begin
            spark[0] <= u8_min(r_px_u16[7:0], R_MAX_PX[7:0]);
            for (si = 1; si < SPARK_N; si = si + 1)
                spark[si] <= spark[si-1];
        end
    end

    // Envelope min/max/peak
    reg [7:0] spark_min_u8;
    reg [7:0] spark_max_u8;
    reg [7:0] spark_peak_u8;

    reg [7:0] peak_decay_ctr;

    integer es;
    reg [7:0] min_tmp;
    reg [7:0] max_tmp;

    always @(posedge clk_pix) begin
        if (rst_pix) begin
            spark_min_u8    <= 8'd0;
            spark_max_u8    <= 8'd0;
            spark_peak_u8   <= 8'd0;
            peak_decay_ctr  <= 8'd0;
        end else if (frame_tick && (EN_VIS_ENVELOPE != 0)) begin
            min_tmp = 8'hFF;
            max_tmp = 8'h00;
            for (es = 0; es < SPARK_N; es = es + 1) begin
                if (spark[es] < min_tmp) min_tmp = spark[es];
                if (spark[es] > max_tmp) max_tmp = spark[es];
            end
            spark_min_u8 <= min_tmp;
            spark_max_u8 <= max_tmp;

            if (max_tmp >= spark_peak_u8) begin
                spark_peak_u8 <= max_tmp;
            end else begin
                if (peak_decay_ctr == (PEAK_DECAY_PERIOD[7:0] - 8'd1)) begin
                    peak_decay_ctr <= 8'd0;
                    if (spark_peak_u8 != 8'd0)
                        spark_peak_u8 <= spark_peak_u8 - 8'd1;
                end else begin
                    peak_decay_ctr <= peak_decay_ctr + 8'd1;
                end
            end
        end
    end

    //==========================================================================
    // 13) Confidence tint computation
    //==========================================================================
    wire [7:0] age_num_u8_for_conf =
        (age_frames_u8 == 8'hFF) ? AGE_SAT_FRAMES[7:0] : age_frames_u8;

    wire [3:0] sev_age =
        (AGE_SAT_FRAMES != 0) ? ((age_num_u8_for_conf * CONF_SEV_MAX) / AGE_SAT_FRAMES[7:0]) : 4'd0;

    wire [3:0] sev_delta =
        (DELTA_SAT_PX != 0) ? ((delta_px_u8 * CONF_SEV_MAX) / DELTA_SAT_PX[7:0]) : 4'd0;

    wire [3:0] conf_age   = 4'd15 - sev_age;
    wire [3:0] conf_delta = 4'd15 - sev_delta;

    wire [3:0] conf_k     = (conf_age < conf_delta) ? conf_age : conf_delta;
    wire [3:0] delta_hue_k = sev_delta;

    //==========================================================================
    // 14) Range gate shading (squared radius comparisons)
    //==========================================================================
    localparam integer RMAX2 = (R_MAX_PX * R_MAX_PX);
    localparam integer G0_2  = (GATE0_PX * GATE0_PX);
    localparam integer G1_2  = (GATE1_PX * GATE1_PX);

    wire in_circle = in_widget && ($signed(r2) <= $signed(RMAX2));

    wire gate_near = (EN_RANGE_GATES != 0) && in_circle && ($signed(r2) <= $signed(G0_2));
    wire gate_mid  = (EN_RANGE_GATES != 0) && in_circle && ($signed(r2) >  $signed(G0_2)) && ($signed(r2) <= $signed(G1_2));
    wire gate_far  = (EN_RANGE_GATES != 0) && in_circle && ($signed(r2) >  $signed(G1_2));

    //==========================================================================
    // 15) Telemetry strip geometry + per-pixel classification
    //==========================================================================
    localparam integer STRIP_Y0_I = (Y0 + H - TELEM_STRIP_H);
    localparam integer STRIP_Y1_I = (Y0 + H - 1);

    localparam [9:0] STRIP_Y0_10 = STRIP_Y0_I[9:0];
    localparam [9:0] STRIP_Y1_10 = STRIP_Y1_I[9:0];

    wire in_strip =
        (EN_VIS_TELEM_STRIP != 0) &&
        in_widget &&
        (pix_y >= STRIP_Y0_10) && (pix_y <= STRIP_Y1_10);

    wire [9:0] strip_lx = pix_x - X0_10;
    wire [9:0] strip_ly = pix_y - STRIP_Y0_10;

    // ---- Age bar ----
    localparam integer AGE_BAR_Y   = TELEM_PAD_Y;
    localparam integer AGE_BAR_H   = 4;
    localparam integer AGE_BAR_X0  = TELEM_PAD_X;
    localparam integer AGE_BAR_X1  = (W - TELEM_PAD_X - 1);

    localparam integer AGE_BAR_Y1EX_I = (AGE_BAR_Y + AGE_BAR_H);
    localparam [9:0]   AGE_BAR_Y0_10  = AGE_BAR_Y[9:0];
    localparam [9:0]   AGE_BAR_Y1EX_10= AGE_BAR_Y1EX_I[9:0];
    localparam [9:0]   AGE_BAR_X0_10  = AGE_BAR_X0[9:0];

    wire [15:0] age_span_u16 = (AGE_BAR_X1 - AGE_BAR_X0 + 1);
    wire [7:0]  age_num_u8   = age_num_u8_for_conf;

    wire [15:0] age_len_u16  =
        (AGE_SAT_FRAMES != 0) ? ((age_span_u16 * age_num_u8) / AGE_SAT_FRAMES[7:0]) : 16'd0;

    wire age_bar_ink =
        in_strip &&
        (strip_ly >= AGE_BAR_Y0_10) && (strip_ly < AGE_BAR_Y1EX_10) &&
        (strip_lx >= AGE_BAR_X0_10) &&
        (strip_lx <  (AGE_BAR_X0_10 + age_len_u16[9:0]));

    // ---- Delta bar ----
    localparam integer DELTA_BAR_W_I    = 6;
    localparam integer DELTA_BAR_X0_I   = (W - TELEM_PAD_X - DELTA_BAR_W_I);
    localparam integer DELTA_BAR_X1EX_I = (DELTA_BAR_X0_I + DELTA_BAR_W_I);
    localparam integer DELTA_BAR_Y1_I   = (TELEM_STRIP_H - TELEM_PAD_Y - 1);

    localparam [9:0] DELTA_BAR_X0_10   = DELTA_BAR_X0_I[9:0];
    localparam [9:0] DELTA_BAR_X1EX_10 = DELTA_BAR_X1EX_I[9:0];
    localparam [9:0] DELTA_BAR_Y1_10   = DELTA_BAR_Y1_I[9:0];

    wire [9:0] delta_h_px =
        (DELTA_SAT_PX != 0) ? ((delta_px_u8 * (TELEM_STRIP_H - 2*TELEM_PAD_Y - 1)) / DELTA_SAT_PX[7:0]) : 10'd0;

    wire delta_bar_ink =
        in_strip &&
        (strip_lx >= DELTA_BAR_X0_10) && (strip_lx < DELTA_BAR_X1EX_10) &&
        (strip_ly <= DELTA_BAR_Y1_10) &&
        (strip_ly >= (DELTA_BAR_Y1_10 - delta_h_px));

    // ---- Stale marker ----
    localparam integer STALE_DOT_X_I  = (W - TELEM_PAD_X - 2);
    localparam integer STALE_DOT_Y_I  = (AGE_BAR_Y + AGE_BAR_H + 2);
    localparam integer STALE_DOT_X1_I = (STALE_DOT_X_I + 1);
    localparam integer STALE_DOT_Y1_I = (STALE_DOT_Y_I + 1);

    localparam [9:0] STALE_DOT_X_10  = STALE_DOT_X_I[9:0];
    localparam [9:0] STALE_DOT_X1_10 = STALE_DOT_X1_I[9:0];
    localparam [9:0] STALE_DOT_Y_10  = STALE_DOT_Y_I[9:0];
    localparam [9:0] STALE_DOT_Y1_10 = STALE_DOT_Y1_I[9:0];

    wire stale_dot_ink =
        in_strip &&
        dist_stale_r &&
        blink_on &&
        (strip_lx >= STALE_DOT_X_10)  && (strip_lx <= STALE_DOT_X1_10) &&
        (strip_ly >= STALE_DOT_Y_10)  && (strip_ly <= STALE_DOT_Y1_10);

    // ---- Sparkline area ----
    localparam integer SPARK_X0_I   = TELEM_PAD_X;
    localparam integer SPARK_Y0_I   = (AGE_BAR_Y + AGE_BAR_H + 6);
    localparam integer SPARK_H_I    = (TELEM_STRIP_H - SPARK_Y0_I - TELEM_PAD_Y);
    localparam integer SPARK_X1EX_I = (SPARK_X0_I + SPARK_N);
    localparam integer SPARK_Y1EX_I = (SPARK_Y0_I + SPARK_H_I);
    localparam integer SPARK_HM1_I  = (SPARK_H_I - 1);

    localparam [9:0] SPARK_X0_10   = SPARK_X0_I[9:0];
    localparam [9:0] SPARK_Y0_10   = SPARK_Y0_I[9:0];
    localparam [9:0] SPARK_X1EX_10 = SPARK_X1EX_I[9:0];
    localparam [9:0] SPARK_Y1EX_10 = SPARK_Y1EX_I[9:0];
    localparam [9:0] SPARK_HM1_10  = SPARK_HM1_I[9:0];

    wire in_spark_area =
        in_strip &&
        (EN_VIS_SPARKLINE != 0) &&
        (strip_lx >= SPARK_X0_10) &&
        (strip_lx <  SPARK_X1EX_10) &&
        (strip_ly >= SPARK_Y0_10) &&
        (strip_ly <  SPARK_Y1EX_10);

    wire [9:0] spark_dx10 = strip_lx - SPARK_X0_10;
    wire [7:0] spark_idx  = spark_dx10[7:0];

    wire [7:0] spark_val_u8 = spark[spark_idx];

    wire [15:0] spark_scaled =
        (R_MAX_PX != 0) ? ((spark_val_u8 * SPARK_HM1_I) / R_MAX_PX[7:0]) : 16'd0;

    wire [9:0] spark_y_plot =
        SPARK_Y0_10 + SPARK_HM1_10 - spark_scaled[9:0];

    wire spark_ink =
        in_spark_area &&
        ((strip_ly == spark_y_plot) || (strip_ly + 10'd1 == spark_y_plot));

    // ---- Envelope lines ----
    wire [15:0] min_scaled =
        (R_MAX_PX != 0) ? ((spark_min_u8 * SPARK_HM1_I) / R_MAX_PX[7:0]) : 16'd0;
    wire [15:0] max_scaled =
        (R_MAX_PX != 0) ? ((spark_max_u8 * SPARK_HM1_I) / R_MAX_PX[7:0]) : 16'd0;
    wire [15:0] peak_scaled =
        (R_MAX_PX != 0) ? ((spark_peak_u8 * SPARK_HM1_I) / R_MAX_PX[7:0]) : 16'd0;

    wire [9:0] y_min_line  = SPARK_Y0_10 + SPARK_HM1_10 - min_scaled[9:0];
    wire [9:0] y_max_line  = SPARK_Y0_10 + SPARK_HM1_10 - max_scaled[9:0];
    wire [9:0] y_peak_line = SPARK_Y0_10 + SPARK_HM1_10 - peak_scaled[9:0];

    wire env_min_ink  = in_spark_area && (EN_VIS_ENVELOPE != 0) && (strip_ly == y_min_line);
    wire env_max_ink  = in_spark_area && (EN_VIS_ENVELOPE != 0) && (strip_ly == y_max_line);
    wire env_peak_ink = in_spark_area && (EN_VIS_ENVELOPE != 0) && (strip_ly == y_peak_line);

    // ---- Histogram area ----
    localparam integer HIST_WPX_I = (HIST_BINS * HIST_BAR_W);
    localparam integer HIST_X1_I  = (DELTA_BAR_X0_I - 2);
    localparam integer HIST_X0_I  = (HIST_X1_I - HIST_WPX_I + 1);

    localparam integer HIST_Y0_I  = (AGE_BAR_Y + AGE_BAR_H + 2);
    localparam integer HIST_Y1_I  = (TELEM_STRIP_H - TELEM_PAD_Y - 1);

    localparam [9:0] HIST_X0_10 = HIST_X0_I[9:0];
    localparam [9:0] HIST_X1_10 = HIST_X1_I[9:0];
    localparam [9:0] HIST_Y0_10 = HIST_Y0_I[9:0];
    localparam [9:0] HIST_Y1_10 = HIST_Y1_I[9:0];

    wire in_hist_area =
        in_strip &&
        (EN_VIS_HIST != 0) &&
        (strip_lx >= HIST_X0_10) && (strip_lx <= HIST_X1_10) &&
        (strip_ly >= HIST_Y0_10) && (strip_ly <= HIST_Y1_10);

    wire [9:0] hist_lx  = strip_lx - HIST_X0_10;
    wire [7:0] hist_bin = (HIST_BAR_W != 0) ? (hist_lx / HIST_BAR_W) : 8'd0;

    localparam integer HIST_HPX_I  = (HIST_Y1_I - HIST_Y0_I + 1);
    localparam integer HIST_HM1_I  = (HIST_HPX_I - 1);
    localparam [9:0]   HIST_HM1_10 = HIST_HM1_I[9:0];

    wire [3:0] hist_cnt = hist[hist_bin];
    wire [9:0] hist_bar_h =
        (HIST_MAX_COUNT != 0) ? ((hist_cnt * (HIST_HPX_I-1)) / HIST_MAX_COUNT[3:0]) : 10'd0;

    wire [9:0] hist_ly = strip_ly - HIST_Y0_10;

    wire hist_bar_ink =
        in_hist_area &&
        (hist_ly >= (HIST_HM1_10 - hist_bar_h));

    //==========================================================================
    // 16) 3x5 LABELS (NO EXTERNAL ROM)
    //==========================================================================
    function [2:0] glyph3x5_row;
        input [7:0] ch;
        input [2:0] row; // 0..4
        begin
            glyph3x5_row = 3'b000;
            case (ch)
                // Digits
                "0": case (row) 0: glyph3x5_row=3'b111; 1: glyph3x5_row=3'b101; 2: glyph3x5_row=3'b101; 3: glyph3x5_row=3'b101; 4: glyph3x5_row=3'b111; endcase
                "1": case (row) 0: glyph3x5_row=3'b010; 1: glyph3x5_row=3'b110; 2: glyph3x5_row=3'b010; 3: glyph3x5_row=3'b010; 4: glyph3x5_row=3'b111; endcase
                "2": case (row) 0: glyph3x5_row=3'b111; 1: glyph3x5_row=3'b001; 2: glyph3x5_row=3'b111; 3: glyph3x5_row=3'b100; 4: glyph3x5_row=3'b111; endcase
                "3": case (row) 0: glyph3x5_row=3'b111; 1: glyph3x5_row=3'b001; 2: glyph3x5_row=3'b111; 3: glyph3x5_row=3'b001; 4: glyph3x5_row=3'b111; endcase
                "4": case (row) 0: glyph3x5_row=3'b101; 1: glyph3x5_row=3'b101; 2: glyph3x5_row=3'b111; 3: glyph3x5_row=3'b001; 4: glyph3x5_row=3'b001; endcase
                "5": case (row) 0: glyph3x5_row=3'b111; 1: glyph3x5_row=3'b100; 2: glyph3x5_row=3'b111; 3: glyph3x5_row=3'b001; 4: glyph3x5_row=3'b111; endcase
                "6": case (row) 0: glyph3x5_row=3'b111; 1: glyph3x5_row=3'b100; 2: glyph3x5_row=3'b111; 3: glyph3x5_row=3'b101; 4: glyph3x5_row=3'b111; endcase
                "7": case (row) 0: glyph3x5_row=3'b111; 1: glyph3x5_row=3'b001; 2: glyph3x5_row=3'b010; 3: glyph3x5_row=3'b010; 4: glyph3x5_row=3'b010; endcase
                "8": case (row) 0: glyph3x5_row=3'b111; 1: glyph3x5_row=3'b101; 2: glyph3x5_row=3'b111; 3: glyph3x5_row=3'b101; 4: glyph3x5_row=3'b111; endcase
                "9": case (row) 0: glyph3x5_row=3'b111; 1: glyph3x5_row=3'b101; 2: glyph3x5_row=3'b111; 3: glyph3x5_row=3'b001; 4: glyph3x5_row=3'b111; endcase

                // Letters
                "A": case (row) 0: glyph3x5_row=3'b111; 1: glyph3x5_row=3'b101; 2: glyph3x5_row=3'b111; 3: glyph3x5_row=3'b101; 4: glyph3x5_row=3'b101; endcase
                "D": case (row) 0: glyph3x5_row=3'b110; 1: glyph3x5_row=3'b101; 2: glyph3x5_row=3'b101; 3: glyph3x5_row=3'b101; 4: glyph3x5_row=3'b110; endcase
                "E": case (row) 0: glyph3x5_row=3'b111; 1: glyph3x5_row=3'b100; 2: glyph3x5_row=3'b111; 3: glyph3x5_row=3'b100; 4: glyph3x5_row=3'b111; endcase
                "G": case (row) 0: glyph3x5_row=3'b111; 1: glyph3x5_row=3'b100; 2: glyph3x5_row=3'b101; 3: glyph3x5_row=3'b101; 4: glyph3x5_row=3'b111; endcase
                "H": case (row) 0: glyph3x5_row=3'b101; 1: glyph3x5_row=3'b101; 2: glyph3x5_row=3'b111; 3: glyph3x5_row=3'b101; 4: glyph3x5_row=3'b101; endcase
                "I": case (row) 0: glyph3x5_row=3'b111; 1: glyph3x5_row=3'b010; 2: glyph3x5_row=3'b010; 3: glyph3x5_row=3'b010; 4: glyph3x5_row=3'b111; endcase
                "L": case (row) 0: glyph3x5_row=3'b100; 1: glyph3x5_row=3'b100; 2: glyph3x5_row=3'b100; 3: glyph3x5_row=3'b100; 4: glyph3x5_row=3'b111; endcase
                "P": case (row) 0: glyph3x5_row=3'b111; 1: glyph3x5_row=3'b101; 2: glyph3x5_row=3'b111; 3: glyph3x5_row=3'b100; 4: glyph3x5_row=3'b100; endcase
                "S": case (row) 0: glyph3x5_row=3'b111; 1: glyph3x5_row=3'b100; 2: glyph3x5_row=3'b111; 3: glyph3x5_row=3'b001; 4: glyph3x5_row=3'b111; endcase
                "T": case (row) 0: glyph3x5_row=3'b111; 1: glyph3x5_row=3'b010; 2: glyph3x5_row=3'b010; 3: glyph3x5_row=3'b010; 4: glyph3x5_row=3'b010; endcase
                "K": case (row) 0: glyph3x5_row=3'b101; 1: glyph3x5_row=3'b110; 2: glyph3x5_row=3'b100; 3: glyph3x5_row=3'b110; 4: glyph3x5_row=3'b101; endcase
                " ": glyph3x5_row = 3'b000;
                default: glyph3x5_row = 3'b000;
            endcase
        end
    endfunction

    function label3_ink;
        input [9:0] lx;     // strip-local x
        input [9:0] ly;     // strip-local y
        input integer x0i;
        input integer y0i;
        input [7:0] c0;
        input [7:0] c1;
        input [7:0] c2;
        reg [9:0] dx0;
        reg [9:0] dy0;
        reg [9:0] sx;
        reg [9:0] sy;
        reg [1:0] cell_col;
        reg [1:0] ch_idx;
        reg [7:0] ch;
        reg [2:0] row;
        reg [2:0] bits;
        reg       bit_on;
        begin
            label3_ink = 1'b0;

            if (LABEL_SCALE < 1) begin
                label3_ink = 1'b0;
            end else if ((lx >= x0i[9:0]) && (ly >= y0i[9:0])) begin
                dx0 = lx - x0i[9:0];
                dy0 = ly - y0i[9:0];

                sx = dx0 / LABEL_SCALE;
                sy = dy0 / LABEL_SCALE;

                // 3 chars * 4 pitch = 12 columns, 5 rows
                if ((sx < 10'd12) && (sy < 10'd5)) begin
                    ch_idx   = sx[3:2];   // /4
                    cell_col = sx[1:0];   // %4
                    row      = sy[2:0];

                    // Column 3 is spacing
                    if (cell_col != 2'd3) begin
                        case (ch_idx)
                            2'd0: ch = c0;
                            2'd1: ch = c1;
                            default: ch = c2;
                        endcase

                        bits = glyph3x5_row(ch, row);

                        case (cell_col)
                            2'd0: bit_on = bits[2];
                            2'd1: bit_on = bits[1];
                            default: bit_on = bits[0];
                        endcase

                        label3_ink = bit_on;
                    end
                end
            end
        end
    endfunction

    // Labels inside strip
    localparam integer L_AGE_X = 1;
    localparam integer L_AGE_Y = 1;

    localparam integer L_DLT_X = 1;
    localparam integer L_DLT_Y = (AGE_BAR_Y + AGE_BAR_H + 1);

    localparam integer L_SPK_X = 1;
    localparam integer L_SPK_Y = (SPARK_Y0_I);

    localparam integer L_HST_X = (HIST_X0_I);
    localparam integer L_HST_Y = (HIST_Y0_I - 6);

    localparam integer L_STL_X = (W - TELEM_PAD_X - 12);
    localparam integer L_STL_Y = (STALE_DOT_Y_I);

    wire label_age_ink = in_strip && (EN_LABELS != 0) && label3_ink(strip_lx, strip_ly, L_AGE_X, L_AGE_Y, "A","G","E");
    wire label_dlt_ink = in_strip && (EN_LABELS != 0) && label3_ink(strip_lx, strip_ly, L_DLT_X, L_DLT_Y, "D","L","T");
    wire label_spk_ink = in_strip && (EN_LABELS != 0) && label3_ink(strip_lx, strip_ly, L_SPK_X, L_SPK_Y, "S","P","K");
    wire label_hst_ink = in_strip && (EN_LABELS != 0) && label3_ink(strip_lx, strip_ly, L_HST_X, L_HST_Y, "H","S","T");
    wire label_stl_ink = in_strip && (EN_LABELS != 0) && label3_ink(strip_lx, strip_ly, L_STL_X, L_STL_Y, "S","T","L");

    wire labels_ink = label_age_ink | label_dlt_ink | label_spk_ink | label_hst_ink | label_stl_ink;

    //==========================================================================
    // 17) Border + corner ticks
    //==========================================================================
    localparam integer X0_P6_I = (X0 + 6);
    localparam integer Y0_P6_I = (Y0 + 6);
    localparam integer X1_M6_I = (X1 - 6);
    localparam integer Y1_M6_I = (Y1 - 6);

    localparam [9:0] X0_P6_10 = X0_P6_I[9:0];
    localparam [9:0] Y0_P6_10 = Y0_P6_I[9:0];
    localparam [9:0] X1_M6_10 = X1_M6_I[9:0];
    localparam [9:0] Y1_M6_10 = Y1_M6_I[9:0];

    wire border_ink =
        (EN_VIS_BORDER != 0) &&
        in_widget &&
        (
            (pix_x == X0_10) || (pix_x == X1_10) ||
            (pix_y == Y0_10) || (pix_y == Y1_10)
        );

    wire corner_tick_ink =
        (EN_VIS_BORDER != 0) &&
        in_widget &&
        (
            (((pix_x >= X0_10) && (pix_x <= X0_P6_10) && (pix_y == Y0_10)) ||
             ((pix_y >= Y0_10) && (pix_y <= Y0_P6_10) && (pix_x == X0_10))) ||
            (((pix_x <= X1_10) && (pix_x >= X1_M6_10) && (pix_y == Y0_10)) ||
             ((pix_y >= Y0_10) && (pix_y <= Y0_P6_10) && (pix_x == X1_10))) ||
            (((pix_x >= X0_10) && (pix_x <= X0_P6_10) && (pix_y == Y1_10)) ||
             ((pix_y <= Y1_10) && (pix_y >= Y1_M6_10) && (pix_x == X0_10))) ||
            (((pix_x <= X1_10) && (pix_x >= X1_M6_10) && (pix_y == Y1_10)) ||
             ((pix_y <= Y1_10) && (pix_y >= Y1_M6_10) && (pix_x == X1_10)))
        );

    //==========================================================================
    // 18) Telemetry packet generation (optional)
    //==========================================================================
    reg [7:0] sample_count_u8;

    always @(posedge clk_pix) begin
        if (rst_pix) begin
            sample_count_u8     <= 8'd0;
            radar_telem_pix     <= 64'd0;
            radar_telem_vld_pix <= 1'b0;
        end else begin
            radar_telem_vld_pix <= 1'b0;

            if ((TELEMETRY_EN != 0) && sample_upd_d1) begin
                sample_count_u8 <= sample_count_u8 + 8'd1;

                radar_telem_pix <= {
                    8'hA5,
                    sample_count_u8,
                    dist_stale_r,
                    ring_en_pix,
                    trail_en_pix,
                    (BEAM_EN != 0),
                    conf_k,
                    sev_delta,
                    age_frames_u8,
                    ia_frames_u8,
                    angle_q10_r,
                    r_px_u16[9:0]
                };

                radar_telem_vld_pix <= 1'b1;
            end else if (TELEMETRY_EN == 0) begin
                radar_telem_pix <= 64'd0;
            end
        end
    end

    //==========================================================================
    // 19) Colors and confidence-tinted beam colors
    //==========================================================================
    localparam [11:0] C_RING        = 12'h222;
    localparam [11:0] C_TRAIL       = 12'h048;
    localparam [11:0] C_CENTER      = 12'hFFF;

    localparam [11:0] C_BEAM0       = 12'h033;
    localparam [11:0] C_BEAM1       = 12'h055;
    localparam [11:0] C_BEAM2       = 12'h077;
    localparam [11:0] C_BEAM_EDGE   = 12'h0AA;

    localparam [11:0] C_BEAM_WARN0  = 12'h220;
    localparam [11:0] C_BEAM_WARN1  = 12'h640;
    localparam [11:0] C_BEAM_WARN2  = 12'hA60;
    localparam [11:0] C_BEAM_WEDGEW = 12'hF80;

    localparam [11:0] C_STALE_TINT  = 12'h211;

    localparam [11:0] C_BORDER      = 12'h0F0;
    localparam [11:0] C_TLM_BG      = 12'h012;
    localparam [11:0] C_TLM_AGE     = 12'h0A4;
    localparam [11:0] C_TLM_DELTA   = 12'hA22;
    localparam [11:0] C_TLM_SPARK   = 12'h4AF;
    localparam [11:0] C_TLM_STALE   = 12'hF00;

    localparam [11:0] C_TLM_ENV_MIN = 12'h0C4;
    localparam [11:0] C_TLM_ENV_MAX = 12'hC40;
    localparam [11:0] C_TLM_ENV_PK  = 12'hF0F;

    localparam [11:0] C_TLM_HIST    = 12'h0B2;

    localparam [11:0] C_GATE_NEAR   = 12'h012;
    localparam [11:0] C_GATE_MID    = 12'h011;
    localparam [11:0] C_GATE_FAR    = 12'h010;

    // Front sweep + halo + dot tint palettes
    localparam integer FRONT_W_PX = 1;
    localparam [11:0]  C_FRONT    = 12'h0FF;
    localparam [11:0]  C_HALO     = 12'h0C0;

    localparam [11:0]  C_DOT_NEUT = 12'hFA0;
    localparam [11:0]  C_DOT_WARM = 12'hF40;
    localparam [11:0]  C_DOT_COOL = 12'h4FF;

    reg [11:0] beam_base;
    reg [11:0] beam_warn;

    always @* begin
        beam_base = C_BEAM0;
        beam_warn = C_BEAM_WARN0;

        if (beam_band == 2'd2) begin
            beam_base = C_BEAM2;
            beam_warn = C_BEAM_WARN2;
        end else if (beam_band == 2'd1) begin
            beam_base = C_BEAM1;
            beam_warn = C_BEAM_WARN1;
        end
    end

    wire [11:0] edge_base = C_BEAM_EDGE;
    wire [11:0] edge_warn = C_BEAM_WEDGEW;

    wire [11:0] beam_hue_mix = {
        mix4(beam_base[11:8], beam_warn[11:8], delta_hue_k),
        mix4(beam_base[7:4],  beam_warn[7:4],  delta_hue_k),
        mix4(beam_base[3:0],  beam_warn[3:0],  delta_hue_k)
    };

    wire [11:0] edge_hue_mix = {
        mix4(edge_base[11:8], edge_warn[11:8], delta_hue_k),
        mix4(edge_base[7:4],  edge_warn[7:4],  delta_hue_k),
        mix4(edge_base[3:0],  edge_warn[3:0],  delta_hue_k)
    };

    wire [11:0] beam_conf_color =
        (EN_CONF_TINT != 0) ? {
            scale4(beam_hue_mix[11:8], conf_k),
            scale4(beam_hue_mix[7:4],  conf_k),
            scale4(beam_hue_mix[3:0],  conf_k)
        } : beam_base;

    wire [11:0] edge_conf_color =
        (EN_CONF_TINT != 0) ? {
            scale4(edge_hue_mix[11:8], conf_k),
            scale4(edge_hue_mix[7:4],  conf_k),
            scale4(edge_hue_mix[3:0],  conf_k)
        } : edge_base;

    //==========================================================================
    // 19.5) Feature blocks: front sweep, halo, doppler tint, phosphor plane
    //==========================================================================

    // Thin front sweep predicate
    wire front_ink;
    front_sweep_line_pix #(
        .FRONT_W_PX(FRONT_W_PX)
    ) u_front (
        .in_ray_segment(in_ray_segment),
        .cross_i       (cross_i),
        .front_ink     (front_ink)
    );

    // Halo predicate (fallback mode via conf_k; p_q16 not present here yet)
    wire halo_ink;
    confidence_halo_pix #(
        .USE_P_Q16     (0),
        .P_SHIFT       (16),
        .HALO_R_MIN_PX (HALO_R_MIN_PX),
        .HALO_R_MAX_PX (HALO_R_MAX_PX),
        .HALO_THICK_PX (HALO_THICK_PX)
    ) u_halo (
        .pix_x     (pix_x),
        .pix_y     (pix_y),
        .ex_s      (ex_s),
        .ey_s      (ey_s),
        .in_widget (in_widget),
        .p_q16     (32'd0),
        .conf_k    (conf_k),
        .halo_ink  (halo_ink)
    );

    // Doppler-ish tint coefficients (commit-validated)
    wire [1:0] dop_tint_sel_w;
    wire [3:0] dop_mag_k_w;

    doppler_tint_pix #(
        .USE_INNOV(0),
        .MAG_SHIFT_INNOV(6),
        .MAG_SHIFT_DELTA(0),
        .MAG_MAX(15)
    ) u_dop (
        .vld             (sample_upd_d1),
        .innov_mm        (16'sd0),
        .delta_signed_px (delta_signed_px),
        .tint_sel        (dop_tint_sel_w),
        .mag_k           (dop_mag_k_w)
    );

    // Commit-latched tint coefficients (frame-stable)
    reg [1:0] dop_tint_sel_r;
    reg [3:0] dop_mag_k_r;

    always @(posedge clk_pix) begin
        if (rst_pix) begin
            dop_tint_sel_r <= 2'd0;
            dop_mag_k_r    <= 4'd0;
        end else if (sample_upd_d1) begin
            dop_tint_sel_r <= dop_tint_sel_w;
            dop_mag_k_r    <= dop_mag_k_w;
        end
    end

    reg [11:0] dot_tint_target;
    always @* begin
        dot_tint_target = C_DOT_NEUT;
        if (dop_tint_sel_r == 2'd1) dot_tint_target = C_DOT_WARM;
        else if (dop_tint_sel_r == 2'd2) dot_tint_target = C_DOT_COOL;
    end

    wire [11:0] dot_tinted_color = {
        mix4(C_DOT_NEUT[11:8], dot_tint_target[11:8], dop_mag_k_r),
        mix4(C_DOT_NEUT[7:4],  dot_tint_target[7:4],  dop_mag_k_r),
        mix4(C_DOT_NEUT[3:0],  dot_tint_target[3:0],  dop_mag_k_r)
    };

    // Sweep glow deposit generator (bounded)
    wire        sweep_dep_pulse;
    wire [7:0]  sweep_dep_u;
    wire [7:0]  sweep_dep_v;

    sweep_glow_depositor_pix #(
        .X0(X0), .Y0(Y0), .W(W), .H(H),
        .R_MAX_PX(R_MAX_PX),
        .PHOS_W(PHOS_W),
        .PHOS_H(PHOS_H),
        .DEPOSITS_PER_FRAME(8),
        .R_STEP_PX(4),
        .ENABLE(1)
    ) u_sweep_dep (
        .clk_pix       (clk_pix),
        .rst_pix       (rst_pix),
        .frame_tick    (frame_tick),
        .dir_x_q15     (dir_x_q15),
        .dir_y_q15     (dir_y_q15),
        .deposit_pulse (sweep_dep_pulse),
        .deposit_u     (sweep_dep_u),
        .deposit_v     (sweep_dep_v)
    );

    // Phosphor plane sample (aligned by 1 cycle)
    wire [7:0] phos_I_d1;
    wire       in_phos_d1;

    radar_phosphor_plane_pix #(
        .X0(X0), .Y0(Y0), .W(W), .H(H),
        .PHOS_W(PHOS_W),
        .PHOS_H(PHOS_H),
        .PHOS_AW(PHOS_AW),
        .PHOS_DECAY(PHOS_DECAY),
        .PHOS_HIT_ADD(PHOS_HIT_ADD),
        .PHOS_SWEEP_ADD(PHOS_SWEEP_ADD),
        .PHOS_MAINT_K(PHOS_MAINT_K)
    ) u_phos_plane (
        .clk_pix          (clk_pix),
        .rst_pix          (rst_pix),
        .pix_x            (pix_x),
        .pix_y            (pix_y),
        .active_video     (active_video),
        .frame_tick       (frame_tick),
        .commit_pulse     (sample_upd_d1),
        .ex_s             (ex_s),
        .ey_s             (ey_s),
        .sample_en_mask   (in_circle),
        .sweep_deposit_en (sweep_dep_pulse),
        .sweep_u          (sweep_dep_u),
        .sweep_v          (sweep_dep_v),
        .phos_I_d1        (phos_I_d1),
        .in_phos_d1       (in_phos_d1)
    );

    //==========================================================================
    // 20) 1-cycle painter (base -> register -> finalize with phosphor)
    //==========================================================================

    reg [11:0] rgb_base_next;
    reg [11:0] rgb_base_d1;

    // Combinational base painter for current pixel
    always @* begin
        rgb_base_next = rgb_bg;

        if (in_widget) begin
            if (gate_near)      rgb_base_next = C_GATE_NEAR;
            else if (gate_mid)  rgb_base_next = C_GATE_MID;
            else if (gate_far)  rgb_base_next = C_GATE_FAR;

            if ((STALE_TINT_EN != 0) && dist_stale_r)
                rgb_base_next = C_STALE_TINT;

            if (ring_ink)       rgb_base_next = C_RING;

            if (beam_core_ink)  rgb_base_next = beam_conf_color;
            if (beam_edge_ink)  rgb_base_next = edge_conf_color;

            if (front_ink)      rgb_base_next = C_FRONT;

            if (trail_ink)      rgb_base_next = C_TRAIL;

            if ((EN_HALO != 0) && halo_ink) rgb_base_next = C_HALO;
            if (dot_ink)        rgb_base_next = dot_tinted_color;

            if (center_ink)     rgb_base_next = C_CENTER;

            if (in_strip && (EN_VIS_TELEM_STRIP != 0))
                rgb_base_next = C_TLM_BG;

            if (age_bar_ink)    rgb_base_next = C_TLM_AGE;
            if (delta_bar_ink)  rgb_base_next = C_TLM_DELTA;
            if (spark_ink)      rgb_base_next = C_TLM_SPARK;
            if (stale_dot_ink)  rgb_base_next = C_TLM_STALE;

            if (env_min_ink)    rgb_base_next = C_TLM_ENV_MIN;
            if (env_max_ink)    rgb_base_next = C_TLM_ENV_MAX;
            if (env_peak_ink)   rgb_base_next = C_TLM_ENV_PK;

            if (hist_bar_ink)   rgb_base_next = C_TLM_HIST;

            if (labels_ink)     rgb_base_next = LABEL_RGB;

            if (border_ink)       rgb_base_next = C_BORDER;
            if (corner_tick_ink)  rgb_base_next = C_BORDER;
        end
    end

    // Phosphor addend derived from PHOS_BLEND_SHIFT (clamped to 0..15)
    wire [7:0] phos_scaled_u8 =
        (PHOS_BLEND_SHIFT < 8) ? (phos_I_d1 >> PHOS_BLEND_SHIFT) : 8'd0;

    wire [3:0] phos_add4 =
        (phos_scaled_u8[7:4] != 4'd0) ? 4'hF : phos_scaled_u8[3:0];

    // Final stage: add phosphor glow (green add) aligned to previous pixel
    reg [11:0] rgb_final_next;
    always @* begin
        rgb_final_next = rgb_base_d1;

        if ((EN_PHOS != 0) && in_phos_d1) begin
            // Add green glow; PHOS_RGB_MAX is a semantic “peak” but final nibble saturates anyway.
            rgb_final_next = {
                rgb_base_d1[11:8],
                add_sat4(rgb_base_d1[7:4], phos_add4),
                rgb_base_d1[3:0]
            };
        end
    end

    always @(posedge clk_pix) begin
        if (rst_pix) begin
            rgb_base_d1 <= 12'h000;
            rgb_out     <= 12'h000;
        end else begin
            rgb_base_d1 <= rgb_base_next;
            rgb_out     <= rgb_final_next;
        end
    end

endmodule

`default_nettype wire