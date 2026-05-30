`timescale 1ns/1ps
`default_nettype none

// ============================================================================
// sonar_occupancy_painter.v
// ----------------------------------------------------------------------------
// Unified implementation covering:
//
// (A) SYS clock-domain occupancy-map "ray painter"
//     - Consumes dist_mm_sys + dist_vld_sys and direction sin/cos (Q1.15).
//     - Emits a write-only map port: map_we_sys/map_addr_sys/map_din_sys.
//     - Emits painter_telem_sys[63:0] + painter_telem_vld_sys once per ray.
//
// (B) PIX clock-domain VGA HUD widget for SONAR telemetry (RAW or BUS mode)
//     - Receives already-snapshotted inputs (no CDC performed internally).
//     - Renders a compact SONAR tile over rgb_bg into rgb_out.
//
// Single-switch policy hook (external):
//   - brush_mode_sys : runtime footprint selection (cross vs ROM) in SYS domain
//   - hud_mode_pix   : runtime HUD mode selection (basic vs advanced) in PIX domain
//
// Brush selection
// ---------------
// - Built-in 5-tap cross (center + 4-neighbors), or
// - beam_brush_rom footprint (radius/shape), indexed sequentially.
//
// For deterministic synthesis behavior, beam_brush_rom is instantiated
// unconditionally and its outputs are only consumed when selected.
//
// Telemetry notes
// ---------------
// painter_telem_sys format:
//   [63:56] seq
//   [55]    no_target
//   [54]    clamped
//   [53]    oob
//   [52:37] ray_len_cells
//   [36:21] free_writes
//   [20:13] hit_writes
//   [12:5]  brush_writes
//   [4:0]   busy_cycles>>5 sat
//
// HUD robustness notes
// --------------------
// - HUD "basic" mode disables the spark plot and advanced counters to reduce
//   clutter and eliminate pegged plots from max-range / no-target readings.
// - Spark ingestion/draw is gated both by hud_mode_pix and by a local
//   no-target detector for common sonar max-range behavior.
//
// ============================================================================
 
module sonar_occupancy_painter #(
    // =========================================================================
    // (A) OCCUPANCY MAP PAINTER PARAMETERS
    // =========================================================================
    parameter integer MAP_W = 256,
    parameter integer MAP_H = 256,

    // Origin in map coordinates (cell units)
    parameter integer ORIGIN_X = (MAP_W/2),
    parameter integer ORIGIN_Y = (MAP_H/2),

    // Cell size in mm: CELL_MM = 2^CELL_MM_SHIFT
    parameter integer CELL_MM_SHIFT = 5,     // 32 mm/cell default

    // Ray stepping bound (cells)
    parameter integer MAX_RAY_STEPS = 200,

    // Write values (tokens)
    parameter [7:0] FREE_VAL = 8'd10,
    parameter [7:0] HIT_VAL  = 8'd255,

    // Optional features
    parameter integer DRAW_FREE_EN  = 1,
    parameter integer DRAW_HIT_EN   = 1,
    parameter integer HIT_BRUSH_EN  = 1,
    parameter integer FREE_SKIP     = 0,     // 0 => write every step; N => skip N cells per write

    // Brush selection policy:
    //   BRUSH_USE_ROM != 0 forces ROM footprint (compile-time).
    //   BRUSH_USE_ROM == 0 allows runtime selection via brush_mode_sys.
    parameter integer BRUSH_USE_ROM = 0,     // 0 => runtime selection, 1 => force ROM

    // beam_brush_rom parameters
    parameter integer BRUSH_RADIUS = 1,
    parameter integer BRUSH_SHAPE  = 1,     // 0=cross 1=square 2=diamond
    parameter integer BRUSH_DXW    = 4,     // signed width for dx/dy (supports radius up to 7 with DXW=4)

    // Painter telemetry policy
    parameter integer NO_TARGET_REJECT_EN = 1,
    parameter [15:0]  NO_TARGET_MM_VALUE = 16'hFFFF,

    // =========================================================================
    // (B) SONAR HUD OVERLAY PARAMETERS
    // =========================================================================
    parameter integer RIGHT_PANEL_X0 = 280,
    parameter integer RIGHT_PANEL_W  = 160,
    parameter integer SCREEN_H       = 480,
    parameter integer GRID_ROWS      = 4,
    parameter integer GRID_COLS      = 2,
    parameter integer ROW_IDX        = 3,
    parameter integer COL_IDX        = 1,

    parameter integer STALE_MS = 250,

    parameter integer SPARK_N         = 64,
    parameter integer SPARK_MM_MAX    = 6000,
    parameter integer SPARK_AUTOSCALE = 1,
    parameter integer SPARK_BOX_EN    = 1,
    parameter integer SPARK_MIDLINE   = 1,

    parameter integer BORDER_EN     = 0,
    parameter integer INCH_ROUNDING = 1,

    parameter integer TELEM_MODE = 1   // 0=RAW, 1=BUS
)(
    // =========================================================================
    // SYS CLOCK DOMAIN (occupancy painter)
    // =========================================================================
    input  wire        clk_sys,
    input  wire        rst_sys,

    input  wire [15:0] dist_mm_sys,
    input  wire        dist_vld_sys,

    input  wire signed [15:0] sin_q15_sys,
    input  wire signed [15:0] cos_q15_sys,

    // Runtime brush selection (SYS domain)
    //   0 => built-in 5-tap cross
    //   1 => beam_brush_rom footprint
    input  wire        brush_mode_sys,

    output reg         busy_sys,
    output reg         done_pulse_sys,

    output reg         map_we_sys,
    output reg  [15:0] map_addr_sys,
    output reg  [7:0]  map_din_sys,

    output reg  [63:0] painter_telem_sys,
    output reg         painter_telem_vld_sys,

    // =========================================================================
    // PIX CLOCK DOMAIN (HUD overlay)
    // =========================================================================
    input  wire        pix_clk,
    input  wire        pix_rst,

    // Runtime HUD mode selection (PIX domain)
    //   0 => basic HUD (spark off, minimal counters)
    //   1 => advanced HUD (spark on, counters enabled via flags)
    input  wire        hud_mode_pix,

    input  wire [9:0]  hcount,
    input  wire [9:0]  vcount,
    input  wire        active_video,
    input  wire        frame_tick,

    // RAW inputs (TELEM_MODE=0)
    input  wire [15:0] sonar_mm_pix,
    input  wire [7:0]  sonar_update_pix,

    // BUS inputs (TELEM_MODE=1)
    input  wire [47:0] sonar_telem_pix,
    input  wire        sonar_telem_upd_pix,

    input  wire [1:0]  src_pix,
    input  wire        uart_pkt_pulse_pix,
    input  wire        uart_crc_err_pulse_pix,
    input  wire [7:0]  hud_flags,
    
    
    // Painter telemetry snapshot already transferred into PIX domain
    input  wire [63:0] painter_telem_pix,
    input  wire        painter_telem_upd_pix,
    

    input  wire [11:0] rgb_bg,
    output reg  [11:0] rgb_out
);

    // =========================================================================
    // (A) SYS-DOMAIN OCCUPANCY MAP PAINTER
    // =========================================================================

    function [15:0] clamp_coord;
        input integer x;
        input integer maxv;
    begin
        if (x < 0)          clamp_coord = 16'd0;
        else if (x >= maxv) clamp_coord = (maxv-1);
        else                clamp_coord = x[15:0];
    end
    endfunction

    function [15:0] addr_of;
        input [15:0] x;
        input [15:0] y;
        integer tmp;
    begin
        tmp = (y * MAP_W) + x;
        addr_of = tmp[15:0];
    end
    endfunction

    function oob_coord;
        input integer xi;
        input integer yi;
        begin
            oob_coord = (xi < 0) || (xi >= MAP_W) || (yi < 0) || (yi >= MAP_H);
        end
    endfunction

    function [4:0] cost5_from_cycles;
        input [15:0] cyc;
        reg [10:0] q;
        begin
            q = {1'b0, cyc[15:5]};
            if (q >= 11'd31) cost5_from_cycles = 5'd31;
            else             cost5_from_cycles = q[4:0];
        end
    endfunction


    // ------------------------------------------------------------------------
    // Painter telemetry state (PIX domain)
    // ------------------------------------------------------------------------
    reg [7:0]  p_seq_r;
    reg        p_no_target_r;
    reg        p_clamped_r;
    reg        p_oob_r;
    reg [15:0] p_ray_len_cells_r;
    reg [15:0] p_free_writes_r;
    reg [7:0]  p_hit_writes_r;
    reg [7:0]  p_brush_writes_r;
    reg [4:0]  p_cost5_r;

    reg [7:0]  p_seq_prev_r;
    reg [7:0]  p_evt_meter_r;

    wire upd_painter = painter_telem_upd_pix;

    wire [7:0]  p_seq_next           = painter_telem_pix[63:56];
    wire        p_no_target_next     = painter_telem_pix[55];
    wire        p_clamped_next       = painter_telem_pix[54];
    wire        p_oob_next           = painter_telem_pix[53];
    wire [15:0] p_ray_len_cells_next = painter_telem_pix[52:37];
    wire [15:0] p_free_writes_next   = painter_telem_pix[36:21];
    wire [7:0]  p_hit_writes_next    = painter_telem_pix[20:13];
    wire [7:0]  p_brush_writes_next  = painter_telem_pix[12:5];
    wire [4:0]  p_cost5_next         = painter_telem_pix[4:0];


    reg [7:0]  paint_seq_sys;

    reg        tlm_no_target_sys;
    reg        tlm_clamped_sys;
    reg        tlm_oob_sys;

    reg [15:0] tlm_ray_len_cells_sys;
    reg [15:0] tlm_free_writes_sys;
    reg [7:0]  tlm_hit_writes_sys;
    reg [7:0]  tlm_brush_writes_sys;
    reg [15:0] tlm_skipped_steps_sys;

    reg [15:0] tlm_busy_cycles_sys;
    reg [15:0] tlm_busy_cycles_max_sys;

    wire no_target_evt_sys = (dist_mm_sys == NO_TARGET_MM_VALUE);
    wire accept_paint_sys  = dist_vld_sys && ((NO_TARGET_REJECT_EN == 0) || !no_target_evt_sys);

    // Distance converted into cell units and bounded by MAX_RAY_STEPS
    reg [15:0] dist_cells_sys;

    always @(posedge clk_sys) begin
        if (rst_sys) begin
            dist_cells_sys    <= 16'd0;
            tlm_clamped_sys   <= 1'b0;
            tlm_no_target_sys <= 1'b0;
        end else begin
            if (dist_vld_sys && !busy_sys) begin
                tlm_no_target_sys <= no_target_evt_sys;
                tlm_clamped_sys   <= 1'b0;

                dist_cells_sys <= (dist_mm_sys >> CELL_MM_SHIFT);

                if ((dist_mm_sys >> CELL_MM_SHIFT) > MAX_RAY_STEPS) begin
                    dist_cells_sys  <= MAX_RAY_STEPS[15:0];
                    tlm_clamped_sys <= 1'b1;
                end
            end
        end
    end

    // Q1.15 -> Q16.16 delta per step
    // <<<1: Q1.15 becomes Q2.16, then treated as signed Q16.16 step increments.
    reg signed [31:0] dx_q16_sys;
    reg signed [31:0] dy_q16_sys;

    reg signed [31:0] x_q16_sys;
    reg signed [31:0] y_q16_sys;

    reg [15:0] step_idx_sys;
    reg [15:0] last_step_sys;
    reg [7:0]  free_skip_cnt_sys;

    reg [15:0] hit_x_sys;
    reg [15:0] hit_y_sys;

    wire signed [15:0] cell_x_s_sys = x_q16_sys[31:16];
    wire signed [15:0] cell_y_s_sys = y_q16_sys[31:16];

    integer cx_i_sys;
    integer cy_i_sys;

    // ------------------------------------------------------------------------
    // Brush option A: built-in 5-tap cross
    // ------------------------------------------------------------------------
    reg [2:0]  brush_k_sys;
    reg [15:0] brush_cell_x_sys;
    reg [15:0] brush_cell_y_sys;

    // ------------------------------------------------------------------------
    // Brush option B: beam_brush_rom-driven footprint
    // ------------------------------------------------------------------------
    reg  [7:0]  brush_idx_sys;
    wire signed [BRUSH_DXW-1:0] brush_dx_sys;
    wire signed [BRUSH_DXW-1:0] brush_dy_sys;
    wire        brush_valid_sys;
    wire [7:0]  brush_count_sys;

    // Unconditional ROM instantiation (outputs consumed only when selected)
    beam_brush_rom #(
        .RADIUS(BRUSH_RADIUS),
        .SHAPE (BRUSH_SHAPE),
        .DXW   (BRUSH_DXW)
    ) u_brush (
        .idx   (brush_idx_sys),
        .dx    (brush_dx_sys),
        .dy    (brush_dy_sys),
        .valid (brush_valid_sys),
        .count (brush_count_sys)
    );

    // Selection policy:
    // - BRUSH_USE_ROM != 0 forces ROM
    // - otherwise runtime via brush_mode_sys
    wire use_rom_sys = (BRUSH_USE_ROM != 0) ? 1'b1 : brush_mode_sys;

    localparam [2:0]
        S_IDLE_SYS   = 3'd0,
        S_INIT_SYS   = 3'd1,
        S_FREE_SYS   = 3'd2,
        S_HIT_SYS    = 3'd3,
        S_BRUSH0_SYS = 3'd4,
        S_BRUSH1_SYS = 3'd5,
        S_DONE_SYS   = 3'd6;

    reg [2:0] state_sys;

    always @(posedge clk_sys) begin
        if (rst_sys) begin
            state_sys         <= S_IDLE_SYS;
            busy_sys          <= 1'b0;
            done_pulse_sys    <= 1'b0;

            map_we_sys        <= 1'b0;
            map_addr_sys      <= 16'd0;
            map_din_sys       <= 8'd0;

            dx_q16_sys        <= 32'sd0;
            dy_q16_sys        <= 32'sd0;

            x_q16_sys         <= 32'sd0;
            y_q16_sys         <= 32'sd0;

            step_idx_sys      <= 16'd0;
            last_step_sys     <= 16'd0;
            free_skip_cnt_sys <= 8'd0;

            hit_x_sys         <= 16'd0;
            hit_y_sys         <= 16'd0;

            brush_k_sys       <= 3'd0;
            brush_cell_x_sys  <= 16'd0;
            brush_cell_y_sys  <= 16'd0;

            brush_idx_sys     <= 8'd0;

            painter_telem_sys     <= 64'd0;
            painter_telem_vld_sys <= 1'b0;

            paint_seq_sys         <= 8'd0;

            tlm_oob_sys           <= 1'b0;
            tlm_ray_len_cells_sys <= 16'd0;
            tlm_free_writes_sys   <= 16'd0;
            tlm_hit_writes_sys    <= 8'd0;
            tlm_brush_writes_sys  <= 8'd0;
            tlm_skipped_steps_sys <= 16'd0;

            tlm_busy_cycles_sys     <= 16'd0;
            tlm_busy_cycles_max_sys <= 16'd0;
        end else begin
            done_pulse_sys        <= 1'b0;
            map_we_sys            <= 1'b0;
            painter_telem_vld_sys <= 1'b0;

            if (busy_sys) begin
                if (tlm_busy_cycles_sys != 16'hFFFF)
                    tlm_busy_cycles_sys <= tlm_busy_cycles_sys + 16'd1;
            end

            case (state_sys)
                S_IDLE_SYS: begin
                    busy_sys <= 1'b0;
                    if (accept_paint_sys) begin
                        busy_sys  <= 1'b1;
                        state_sys <= S_INIT_SYS;
                    end
                end

                S_INIT_SYS: begin
                    dx_q16_sys <= $signed(cos_q15_sys) <<< 1;
                    dy_q16_sys <= $signed(sin_q15_sys) <<< 1;

                    x_q16_sys <= $signed(ORIGIN_X <<< 16);
                    y_q16_sys <= $signed(ORIGIN_Y <<< 16);

                    step_idx_sys      <= 16'd0;
                    free_skip_cnt_sys <= 8'd0;

                    last_step_sys <= dist_cells_sys;

                    tlm_oob_sys           <= 1'b0;
                    tlm_ray_len_cells_sys <= dist_cells_sys;
                    tlm_free_writes_sys   <= 16'd0;
                    tlm_hit_writes_sys    <= 8'd0;
                    tlm_brush_writes_sys  <= 8'd0;
                    tlm_skipped_steps_sys <= 16'd0;
                    tlm_busy_cycles_sys   <= 16'd0;

                    brush_k_sys   <= 3'd0;
                    brush_idx_sys <= 8'd0;

                    state_sys <= (DRAW_FREE_EN != 0) ? S_FREE_SYS : S_HIT_SYS;
                end

                // FREE writes: steps [0 .. last_step-1]
                S_FREE_SYS: begin
                    if (step_idx_sys >= last_step_sys) begin
                        state_sys <= S_HIT_SYS;
                    end else begin
                        if (FREE_SKIP != 0) begin
                            if (free_skip_cnt_sys < FREE_SKIP[7:0]) begin
                                free_skip_cnt_sys     <= free_skip_cnt_sys + 8'd1;
                                tlm_skipped_steps_sys <= tlm_skipped_steps_sys + 16'd1;

                                x_q16_sys    <= x_q16_sys + dx_q16_sys;
                                y_q16_sys    <= y_q16_sys + dy_q16_sys;
                                step_idx_sys <= step_idx_sys + 16'd1;
                            end else begin
                                free_skip_cnt_sys <= 8'd0;

                                cx_i_sys = cell_x_s_sys;
                                cy_i_sys = cell_y_s_sys;

                                if (oob_coord(cx_i_sys, cy_i_sys))
                                    tlm_oob_sys <= 1'b1;

                                map_we_sys   <= 1'b1;
                                map_addr_sys <= addr_of(clamp_coord(cx_i_sys, MAP_W), clamp_coord(cy_i_sys, MAP_H));
                                map_din_sys  <= FREE_VAL;

                                tlm_free_writes_sys <= tlm_free_writes_sys + 16'd1;

                                x_q16_sys    <= x_q16_sys + dx_q16_sys;
                                y_q16_sys    <= y_q16_sys + dy_q16_sys;
                                step_idx_sys <= step_idx_sys + 16'd1;
                            end
                        end else begin
                            cx_i_sys = cell_x_s_sys;
                            cy_i_sys = cell_y_s_sys;

                            if (oob_coord(cx_i_sys, cy_i_sys))
                                tlm_oob_sys <= 1'b1;

                            map_we_sys   <= 1'b1;
                            map_addr_sys <= addr_of(clamp_coord(cx_i_sys, MAP_W), clamp_coord(cy_i_sys, MAP_H));
                            map_din_sys  <= FREE_VAL;

                            tlm_free_writes_sys <= tlm_free_writes_sys + 16'd1;

                            x_q16_sys    <= x_q16_sys + dx_q16_sys;
                            y_q16_sys    <= y_q16_sys + dy_q16_sys;
                            step_idx_sys <= step_idx_sys + 16'd1;
                        end
                    end
                end

                // HIT cell: endpoint at current cell
                S_HIT_SYS: begin
                    cx_i_sys = cell_x_s_sys;
                    cy_i_sys = cell_y_s_sys;

                    hit_x_sys <= clamp_coord(cx_i_sys, MAP_W);
                    hit_y_sys <= clamp_coord(cy_i_sys, MAP_H);

                    if (oob_coord(cx_i_sys, cy_i_sys))
                        tlm_oob_sys <= 1'b1;

                    if (DRAW_HIT_EN != 0) begin
                        map_we_sys   <= 1'b1;
                        map_addr_sys <= addr_of(clamp_coord(cx_i_sys, MAP_W), clamp_coord(cy_i_sys, MAP_H));
                        map_din_sys  <= HIT_VAL;

                        tlm_hit_writes_sys <= tlm_hit_writes_sys + 8'd1;
                    end

                    brush_k_sys   <= 3'd0;
                    brush_idx_sys <= 8'd0;

                    state_sys <= (HIT_BRUSH_EN != 0) ? S_BRUSH0_SYS : S_DONE_SYS;
                end

                // BRUSH: stage 0 computes target cell, stage 1 performs write
                S_BRUSH0_SYS: begin
                    if (use_rom_sys) begin
                        // ROM-driven footprint: compute candidate from dx/dy
                        if (brush_valid_sys) begin
                            cx_i_sys = $signed({1'b0, hit_x_sys}) + $signed(brush_dx_sys);
                            cy_i_sys = $signed({1'b0, hit_y_sys}) + $signed(brush_dy_sys);

                            brush_cell_x_sys <= clamp_coord(cx_i_sys, MAP_W);
                            brush_cell_y_sys <= clamp_coord(cy_i_sys, MAP_H);

                            if (oob_coord(cx_i_sys, cy_i_sys))
                                tlm_oob_sys <= 1'b1;

                            state_sys <= S_BRUSH1_SYS;
                        end else begin
                            // No valid points => finish
                            state_sys <= S_DONE_SYS;
                        end
                    end else begin
                        // Built-in 5-tap cross:
                        //   k=0: (0,0)
                        //   k=1: (+1,0)
                        //   k=2: (-1,0)
                        //   k=3: (0,+1)
                        //   k=4: (0,-1)
                        brush_cell_x_sys <= hit_x_sys;
                        brush_cell_y_sys <= hit_y_sys;

                        if (brush_k_sys == 3'd1)
                            brush_cell_x_sys <= (hit_x_sys == (MAP_W-1)) ? hit_x_sys : (hit_x_sys + 16'd1);
                        if (brush_k_sys == 3'd2)
                            brush_cell_x_sys <= (hit_x_sys == 16'd0)     ? hit_x_sys : (hit_x_sys - 16'd1);
                        if (brush_k_sys == 3'd3)
                            brush_cell_y_sys <= (hit_y_sys == (MAP_H-1)) ? hit_y_sys : (hit_y_sys + 16'd1);
                        if (brush_k_sys == 3'd4)
                            brush_cell_y_sys <= (hit_y_sys == 16'd0)     ? hit_y_sys : (hit_y_sys - 16'd1);

                        state_sys <= S_BRUSH1_SYS;
                    end
                end

                S_BRUSH1_SYS: begin
                    map_we_sys   <= 1'b1;
                    map_addr_sys <= addr_of(brush_cell_x_sys, brush_cell_y_sys);
                    map_din_sys  <= HIT_VAL;

                    tlm_brush_writes_sys <= tlm_brush_writes_sys + 8'd1;

                    if (use_rom_sys) begin
                        // Advance until brush_idx reaches brush_count-1
                        if (brush_count_sys == 0) begin
                            state_sys <= S_DONE_SYS;
                        end else if (brush_idx_sys >= (brush_count_sys - 1)) begin
                            state_sys <= S_DONE_SYS;
                        end else begin
                            brush_idx_sys <= brush_idx_sys + 8'd1;
                            state_sys     <= S_BRUSH0_SYS;
                        end
                    end else begin
                        // Built-in cross: brush_k 0..4
                        if (brush_k_sys >= 3'd4) begin
                            state_sys <= S_DONE_SYS;
                        end else begin
                            brush_k_sys <= brush_k_sys + 3'd1;
                            state_sys   <= S_BRUSH0_SYS;
                        end
                    end
                end

                S_DONE_SYS: begin
                    busy_sys       <= 1'b0;
                    done_pulse_sys <= 1'b1;

                    if (tlm_busy_cycles_sys > tlm_busy_cycles_max_sys)
                        tlm_busy_cycles_max_sys <= tlm_busy_cycles_sys;

                    painter_telem_sys <= {
                        paint_seq_sys,
                        tlm_no_target_sys,
                        tlm_clamped_sys,
                        tlm_oob_sys,
                        tlm_ray_len_cells_sys,
                        tlm_free_writes_sys,
                        tlm_hit_writes_sys,
                        tlm_brush_writes_sys,
                        cost5_from_cycles(tlm_busy_cycles_sys)
                    };
                    painter_telem_vld_sys <= 1'b1;
                    paint_seq_sys <= paint_seq_sys + 8'd1;

                    state_sys <= S_IDLE_SYS;
                end

                default: state_sys <= S_IDLE_SYS;
            endcase
        end
    end

    // =========================================================================
    // (B) PIX-DOMAIN HUD OVERLAY
    // =========================================================================

    localparam integer CELL_W = (RIGHT_PANEL_W / GRID_COLS);
    localparam integer CELL_H = (SCREEN_H      / GRID_ROWS);

    localparam integer X0 = RIGHT_PANEL_X0 + (COL_IDX * CELL_W);
    localparam integer Y0 =                (ROW_IDX * CELL_H);
    localparam integer X1 = X0 + CELL_W - 1;
    localparam integer Y1 = Y0 + CELL_H - 1;

    localparam integer PAD_L = 4;
    localparam integer PAD_T = 4;

    localparam integer TITLE_H = 12;
    localparam integer TEXT_H  = 10;

    localparam integer _SPARK_H_RAW = (CELL_H - TITLE_H - 3*TEXT_H - 2*PAD_T);
    localparam integer _SPARK_W_RAW = (CELL_W - 2*PAD_L);
    localparam integer SPARK_H = (_SPARK_H_RAW < 8)  ? 8  : _SPARK_H_RAW;
    localparam integer SPARK_W = (_SPARK_W_RAW < 16) ? 16 : _SPARK_W_RAW;

    localparam integer SPARK_X0 = X0 + PAD_L;
    localparam integer SPARK_Y0 = Y0 + PAD_T + TITLE_H + (2*TEXT_H);
    localparam integer SPARK_X1 = SPARK_X0 + SPARK_W - 1;
    localparam integer SPARK_Y1 = SPARK_Y0 + SPARK_H - 1;

    // ------------------------------------------------------------------------
    // HUD feature flags (effective enables)
    // - Basic mode uses a simplified view, advanced mode uses hud_flags.
    // ------------------------------------------------------------------------
    wire en_master = hud_flags[7];

    wire en_uart_eff  = (hud_mode_pix != 0) ? hud_flags[6] : 1'b0;
    wire en_stale_eff = (hud_mode_pix != 0) ? hud_flags[5] : 1'b1;

    wire en_mmx_eff   = (hud_mode_pix != 0) ? hud_flags[4] : 1'b0;
    wire en_hz_eff    = (hud_mode_pix != 0) ? hud_flags[3] : 1'b0;
    wire en_age_eff   = (hud_mode_pix != 0) ? hud_flags[2] : 1'b0;

    // Basic mode forces inch display for quick sanity readout.
    wire en_in_eff    = (hud_mode_pix != 0) ? hud_flags[1] : 1'b1;

    wire en_act_eff   = (hud_mode_pix != 0) ? hud_flags[0] : 1'b0;

    // ------------------------------------------------------------------------
    // HUD state registers
    // ------------------------------------------------------------------------
    reg [15:0] dist_mm_r;
    reg [9:0]  age_ms_r;
    reg [9:0]  period_ms_r;
    reg [5:0]  drop_r;
    reg [5:0]  nt_r;

    reg [7:0] raw_seq_prev;
    reg [7:0] act_meter;

    reg [15:0] spark_scale_mm;
    reg [15:0] spark_peak_mm;

    reg [15:0] raw_stale_ctr;

    wire stale_bus = (age_ms_r >= STALE_MS[9:0]);
    wire stale_raw = (raw_stale_ctr >= 16'd60);
    wire stale     = (TELEM_MODE != 0) ? stale_bus : stale_raw;

    localparam integer SPARK_AW =
        (SPARK_N <= 2)   ? 1 :
        (SPARK_N <= 4)   ? 2 :
        (SPARK_N <= 8)   ? 3 :
        (SPARK_N <= 16)  ? 4 :
        (SPARK_N <= 32)  ? 5 :
        (SPARK_N <= 64)  ? 6 :
        (SPARK_N <= 128) ? 7 : 8;

    reg [15:0] spark_mem [0:SPARK_N-1];
    reg [SPARK_AW-1:0] spark_wr;

    integer i;
    initial begin
        for (i = 0; i < SPARK_N; i = i + 1)
            spark_mem[i] = 16'd0;
    end

    wire upd_bus = (TELEM_MODE != 0) && sonar_telem_upd_pix;
    wire upd_raw = (TELEM_MODE == 0) && (sonar_update_pix != raw_seq_prev);
    wire upd_any = upd_bus | upd_raw;

    wire [15:0] dist_mm_next_bus = sonar_telem_pix[47:32];
    wire [15:0] dist_mm_next_raw = sonar_mm_pix;
    wire [15:0] dist_mm_next     = upd_bus ? dist_mm_next_bus :
                                   upd_raw ? dist_mm_next_raw :
                                   dist_mm_r;

    function [15:0] mm_to_inch;
        input [15:0] mm;
        reg [31:0] num;
        begin
            num = mm * 32'd100;
            if (INCH_ROUNDING != 0) mm_to_inch = (num + 32'd127) / 32'd254;
            else                    mm_to_inch = (num) / 32'd254;
        end
    endfunction

    // ------------------------------------------------------------------------
    // Spark "no target" detector for common sonar max-range reports.
    // - In inch mode: 255 in is treated as no-target.
    // - In mm mode: 6477 mm (255*25.4) and above treated as no-target.
    // This logic is local to HUD only and does not change SYS painting.
    // ------------------------------------------------------------------------
    localparam [15:0] HUD_NO_TARGET_INCH = 16'd255;
    localparam [15:0] HUD_NO_TARGET_MM   = 16'd6477;

    wire [15:0] dist_in_next = mm_to_inch(dist_mm_next);
    wire no_target_pix = en_in_eff ? (dist_in_next == HUD_NO_TARGET_INCH)
                                   : (dist_mm_next >= HUD_NO_TARGET_MM);

    // Spark updates only in advanced mode and only when not no-target.
    wire spark_upd_qual = (hud_mode_pix != 0) && upd_any && !no_target_pix;

    // ------------------------------------------------------------------------
    // 3x5 font helpers
    // ------------------------------------------------------------------------
    function glyph3x5;
        input [7:0] ch;
        input [1:0] gx;
        input [2:0] gy;
        reg [2:0] rowbits;
        begin
            rowbits = 3'b000;
            case (ch)
                "0": case (gy) 0: rowbits=3'b111; 1: rowbits=3'b101; 2: rowbits=3'b101; 3: rowbits=3'b101; 4: rowbits=3'b111; default: rowbits=3'b000; endcase
                "1": case (gy) 0: rowbits=3'b010; 1: rowbits=3'b110; 2: rowbits=3'b010; 3: rowbits=3'b010; 4: rowbits=3'b111; default: rowbits=3'b000; endcase
                "2": case (gy) 0: rowbits=3'b111; 1: rowbits=3'b001; 2: rowbits=3'b111; 3: rowbits=3'b100; 4: rowbits=3'b111; default: rowbits=3'b000; endcase
                "3": case (gy) 0: rowbits=3'b111; 1: rowbits=3'b001; 2: rowbits=3'b111; 3: rowbits=3'b001; 4: rowbits=3'b111; default: rowbits=3'b000; endcase
                "4": case (gy) 0: rowbits=3'b101; 1: rowbits=3'b101; 2: rowbits=3'b111; 3: rowbits=3'b001; 4: rowbits=3'b001; default: rowbits=3'b000; endcase
                "5": case (gy) 0: rowbits=3'b111; 1: rowbits=3'b100; 2: rowbits=3'b111; 3: rowbits=3'b001; 4: rowbits=3'b111; default: rowbits=3'b000; endcase
                "6": case (gy) 0: rowbits=3'b111; 1: rowbits=3'b100; 2: rowbits=3'b111; 3: rowbits=3'b101; 4: rowbits=3'b111; default: rowbits=3'b000; endcase
                "7": case (gy) 0: rowbits=3'b111; 1: rowbits=3'b001; 2: rowbits=3'b001; 3: rowbits=3'b001; 4: rowbits=3'b001; default: rowbits=3'b000; endcase
                "8": case (gy) 0: rowbits=3'b111; 1: rowbits=3'b101; 2: rowbits=3'b111; 3: rowbits=3'b101; 4: rowbits=3'b111; default: rowbits=3'b000; endcase
                "9": case (gy) 0: rowbits=3'b111; 1: rowbits=3'b101; 2: rowbits=3'b111; 3: rowbits=3'b001; 4: rowbits=3'b111; default: rowbits=3'b000; endcase

                "S": case (gy) 0: rowbits=3'b111; 1: rowbits=3'b100; 2: rowbits=3'b111; 3: rowbits=3'b001; 4: rowbits=3'b111; default: rowbits=3'b000; endcase
                "O": case (gy) 0: rowbits=3'b111; 1: rowbits=3'b101; 2: rowbits=3'b101; 3: rowbits=3'b101; 4: rowbits=3'b111; default: rowbits=3'b000; endcase
                "N": case (gy) 0: rowbits=3'b101; 1: rowbits=3'b111; 2: rowbits=3'b111; 3: rowbits=3'b111; 4: rowbits=3'b101; default: rowbits=3'b000; endcase
                "A": case (gy) 0: rowbits=3'b111; 1: rowbits=3'b101; 2: rowbits=3'b111; 3: rowbits=3'b101; 4: rowbits=3'b101; default: rowbits=3'b000; endcase
                "R": case (gy) 0: rowbits=3'b110; 1: rowbits=3'b101; 2: rowbits=3'b110; 3: rowbits=3'b101; 4: rowbits=3'b101; default: rowbits=3'b000; endcase
                "H": case (gy) 0: rowbits=3'b101; 1: rowbits=3'b101; 2: rowbits=3'b111; 3: rowbits=3'b101; 4: rowbits=3'b101; default: rowbits=3'b000; endcase
                "Z": case (gy) 0: rowbits=3'b111; 1: rowbits=3'b001; 2: rowbits=3'b010; 3: rowbits=3'b100; 4: rowbits=3'b111; default: rowbits=3'b000; endcase
                "D": case (gy) 0: rowbits=3'b110; 1: rowbits=3'b101; 2: rowbits=3'b101; 3: rowbits=3'b101; 4: rowbits=3'b110; default: rowbits=3'b000; endcase
                "P": case (gy) 0: rowbits=3'b111; 1: rowbits=3'b101; 2: rowbits=3'b111; 3: rowbits=3'b100; 4: rowbits=3'b100; default: rowbits=3'b000; endcase
                "T": case (gy) 0: rowbits=3'b111; 1: rowbits=3'b010; 2: rowbits=3'b010; 3: rowbits=3'b010; 4: rowbits=3'b010; default: rowbits=3'b000; endcase
                "I": case (gy) 0: rowbits=3'b111; 1: rowbits=3'b010; 2: rowbits=3'b010; 3: rowbits=3'b010; 4: rowbits=3'b111; default: rowbits=3'b000; endcase
                "U": case (gy) 0: rowbits=3'b101; 1: rowbits=3'b101; 2: rowbits=3'b101; 3: rowbits=3'b101; 4: rowbits=3'b111; default: rowbits=3'b000; endcase
                "F": case (gy) 0: rowbits=3'b111; 1: rowbits=3'b100; 2: rowbits=3'b111; 3: rowbits=3'b100; 4: rowbits=3'b100; default: rowbits=3'b000; endcase

                ":": case (gy) 0: rowbits=3'b000; 1: rowbits=3'b010; 2: rowbits=3'b000; 3: rowbits=3'b010; 4: rowbits=3'b000; default: rowbits=3'b000; endcase
                ".": case (gy) 0: rowbits=3'b000; 1: rowbits=3'b000; 2: rowbits=3'b000; 3: rowbits=3'b000; 4: rowbits=3'b010; default: rowbits=3'b000; endcase
                "-": case (gy) 0: rowbits=3'b000; 1: rowbits=3'b000; 2: rowbits=3'b111; 3: rowbits=3'b000; 4: rowbits=3'b000; default: rowbits=3'b000; endcase
                " ": rowbits = 3'b000;
                default: rowbits = 3'b000;
            endcase

            glyph3x5 = rowbits[2-gx];
        end
    endfunction

    function text3x5_ink;
        input integer px;
        input integer py;
        input integer x_text0;
        input integer y_text0;
        input integer max_chars;
        input [8*16-1:0] str16;
        integer relx, rely;
        integer char_idx;
        integer cx, cy;
        integer msb_char;
        integer byte_index_from_msb;
        integer sh;
        reg [7:0] ch;
        begin
            text3x5_ink = 1'b0;

            relx = px - x_text0;
            rely = py - y_text0;

            if (relx < 0 || rely < 0) begin
                text3x5_ink = 1'b0;
            end else if (rely >= 6) begin
                text3x5_ink = 1'b0;
            end else begin
                char_idx = relx / 4;
                cx       = relx % 4;
                cy       = rely;

                if (char_idx < 0 || char_idx >= max_chars) begin
                    text3x5_ink = 1'b0;
                end else if (cx >= 3 || cy >= 5) begin
                    text3x5_ink = 1'b0;
                end else begin
                    msb_char = (16 - max_chars) + char_idx;
                    byte_index_from_msb = msb_char;

                    sh = (15 - byte_index_from_msb) * 8;
                    ch = (str16 >> sh) & 8'hFF;

                    text3x5_ink = glyph3x5(ch, cx[1:0], cy[2:0]);
                end
            end
        end
    endfunction

    // ------------------------------------------------------------------------
    // PIX sequential: latch telemetry and maintain activity meters + spark buffer
    // ------------------------------------------------------------------------
    always @(posedge pix_clk) begin
        if (pix_rst) begin
            dist_mm_r      <= 16'd0;
            age_ms_r       <= 10'd0;
            period_ms_r    <= 10'd0;
            drop_r         <= 6'd0;
            nt_r           <= 6'd0;

            raw_seq_prev   <= 8'd0;
            raw_stale_ctr  <= 16'd0;

            act_meter      <= 8'd0;

            spark_wr       <= {SPARK_AW{1'b0}};
            spark_scale_mm <= (SPARK_MM_MAX != 0) ? SPARK_MM_MAX[15:0] : 16'd1;
            spark_peak_mm  <= 16'd1;

            p_seq_r            <= 8'd0;
            p_no_target_r      <= 1'b0;
            p_clamped_r        <= 1'b0;
            p_oob_r            <= 1'b0;
            p_ray_len_cells_r  <= 16'd0;
            p_free_writes_r    <= 16'd0;
            p_hit_writes_r     <= 8'd0;
            p_brush_writes_r   <= 8'd0;
            p_cost5_r          <= 5'd0;
            p_seq_prev_r       <= 8'd0;
            p_evt_meter_r      <= 8'd0;
        end else begin
            // ------------------------------------------------------------
            // Activity meter decay
            // ------------------------------------------------------------
            if (frame_tick) begin
                if (act_meter != 0)
                    act_meter <= act_meter - 8'd1;

                if (p_evt_meter_r != 0)
                    p_evt_meter_r <= p_evt_meter_r - 8'd1;
            end

            // UART pulses contribute to activity only in advanced mode
            if (en_uart_eff && uart_pkt_pulse_pix) begin
                if (act_meter < 8'd250) act_meter <= act_meter + 8'd6;
                else                    act_meter <= 8'd255;
            end

            // Sonar update pulses contribute to activity in both modes
            if (upd_any) begin
                if (act_meter < 8'd240) act_meter <= act_meter + 8'd12;
                else                    act_meter <= 8'd255;
            end

            // Painter telemetry updates contribute to separate painter-event meter
            if (upd_painter) begin
                if (p_evt_meter_r < 8'd240) p_evt_meter_r <= p_evt_meter_r + 8'd12;
                else                        p_evt_meter_r <= 8'd255;
            end

            // ------------------------------------------------------------
            // RAW staleness counter
            // ------------------------------------------------------------
            if (frame_tick) begin
                if (upd_raw) raw_stale_ctr <= 16'd0;
                else if (raw_stale_ctr != 16'hFFFF) raw_stale_ctr <= raw_stale_ctr + 16'd1;
            end

            // ------------------------------------------------------------
            // Sonar telemetry latch
            // ------------------------------------------------------------
            if (upd_bus) begin
                dist_mm_r   <= sonar_telem_pix[47:32];
                age_ms_r    <= sonar_telem_pix[31:22];
                period_ms_r <= sonar_telem_pix[21:12];
                drop_r      <= sonar_telem_pix[11:6];
                nt_r        <= sonar_telem_pix[5:0];
            end

            if (upd_raw) begin
                dist_mm_r    <= sonar_mm_pix;
                age_ms_r     <= 10'd0;
                period_ms_r  <= 10'd0;
                drop_r       <= 6'd0;
                nt_r         <= 6'd0;
                raw_seq_prev <= sonar_update_pix;
            end

            // ------------------------------------------------------------
            // Painter telemetry latch
            // ------------------------------------------------------------
            if (upd_painter) begin
                p_seq_r           <= p_seq_next;
                p_no_target_r     <= p_no_target_next;
                p_clamped_r       <= p_clamped_next;
                p_oob_r           <= p_oob_next;
                p_ray_len_cells_r <= p_ray_len_cells_next;
                p_free_writes_r   <= p_free_writes_next;
                p_hit_writes_r    <= p_hit_writes_next;
                p_brush_writes_r  <= p_brush_writes_next;
                p_cost5_r         <= p_cost5_next;
                p_seq_prev_r      <= p_seq_next;
            end

            // ------------------------------------------------------------
            // Spark ingestion (advanced-only, and no-target suppressed)
            // ------------------------------------------------------------
            if (spark_upd_qual) begin
                spark_mem[spark_wr] <= dist_mm_next;
                spark_wr <= spark_wr + {{(SPARK_AW-1){1'b0}},1'b1};

                if (dist_mm_next > spark_peak_mm) spark_peak_mm <= dist_mm_next;
                else if (frame_tick && spark_peak_mm > 16'd1) spark_peak_mm <= spark_peak_mm - 16'd1;

                if (SPARK_AUTOSCALE != 0) begin
                    if (en_mmx_eff) begin
                        if (spark_peak_mm < 16'd256) spark_scale_mm <= 16'd256;
                        else if (spark_peak_mm > SPARK_MM_MAX[15:0]) spark_scale_mm <= SPARK_MM_MAX[15:0];
                        else spark_scale_mm <= spark_peak_mm;
                    end else begin
                        spark_scale_mm <= (SPARK_MM_MAX != 0) ? SPARK_MM_MAX[15:0] : 16'd1;
                    end
                end else begin
                    spark_scale_mm <= (SPARK_MM_MAX != 0) ? SPARK_MM_MAX[15:0] : 16'd1;
                end
            end
        end
    end
    
    
    // ------------------------------------------------------------------------
    // Spark sample lookup: maps horizontal pixel into SPARK_N history samples.
    // ------------------------------------------------------------------------
    function [15:0] spark_sample_at;
        input integer x_pix;
        integer xr;
        integer idx;
        integer rd;
        integer mask;
        begin
            xr = x_pix - SPARK_X0;
            if (xr < 0) xr = 0;
            if (xr > (SPARK_W-1)) xr = (SPARK_W-1);

            idx = (xr * SPARK_N) / (SPARK_W == 0 ? 1 : SPARK_W);
            if (idx < 0) idx = 0;
            if (idx > (SPARK_N-1)) idx = (SPARK_N-1);

            mask = (SPARK_N - 1);

            if ((SPARK_N & (SPARK_N-1)) == 0)
                rd = (spark_wr + idx) & mask;
            else
                rd = (spark_wr + idx) % SPARK_N;

            spark_sample_at = spark_mem[rd];
        end
    endfunction

    // ------------------------------------------------------------------------
    // HUD colors
    // ------------------------------------------------------------------------
    localparam [11:0] C_TEXT   = 12'hFFF;
    localparam [11:0] C_DIM    = 12'h888;
    localparam [11:0] C_BORDER = 12'hAAA;

    localparam [11:0] C_OK     = 12'h0F0;
    localparam [11:0] C_WARN   = 12'hFF0;
    localparam [11:0] C_BAD    = 12'hF00;

    localparam [11:0] C_SRC0   = 12'h888;
    localparam [11:0] C_SRC1   = 12'h0AF;
    localparam [11:0] C_SRC2   = 12'hFA0;
    localparam [11:0] C_SRC3   = 12'h0F0;

    function [11:0] src_color;
        input [1:0] s;
        begin
            case (s)
                2'd1: src_color = C_SRC1;
                2'd2: src_color = C_SRC2;
                2'd3: src_color = C_SRC3;
                default: src_color = C_SRC0;
            endcase
        end
    endfunction

    // ------------------------------------------------------------------------
    // HUD region decode
    // ------------------------------------------------------------------------
    wire in_widget = active_video &&
                     (hcount >= X0) && (hcount <= X1) &&
                     (vcount >= Y0) && (vcount <= Y1);

    wire on_border = (BORDER_EN != 0) && in_widget &&
                     ((hcount == X0) || (hcount == X1) || (vcount == Y0) || (vcount == Y1));

    // Spark region enabled only in advanced HUD mode
    wire in_spark = in_widget && (hud_mode_pix != 0) && (SPARK_BOX_EN != 0) &&
                    (hcount >= SPARK_X0) && (hcount <= SPARK_X1) &&
                    (vcount >= SPARK_Y0) && (vcount <= SPARK_Y1);

    wire on_spark_border = in_spark &&
                           ((hcount == SPARK_X0) || (hcount == SPARK_X1) ||
                            (vcount == SPARK_Y0) || (vcount == SPARK_Y1));

    wire on_midline = in_spark && (SPARK_MIDLINE != 0) &&
                      (vcount == (SPARK_Y0 + (SPARK_H/2)));

    // Spark geometry -> top-of-fill
    wire [15:0] s_mm   = spark_sample_at(hcount);
    wire [15:0] denom  = (spark_scale_mm == 0) ? 16'd1 : spark_scale_mm;
    wire [31:0] s_mul  = s_mm * (SPARK_H > 2 ? (SPARK_H-2) : 1);
    wire [15:0] s_h    = s_mul / denom;



    wire [15:0] ray_cl = (p_ray_len_cells_r > 16'd999) ? 16'd999 : p_ray_len_cells_r;
    wire [15:0] fr_cl  = (p_free_writes_r   > 16'd999) ? 16'd999 : p_free_writes_r;
    wire [15:0] hw_cl  = (p_hit_writes_r    > 16'd99)  ? 16'd99  : p_hit_writes_r;
    wire [15:0] bw_cl  = (p_brush_writes_r  > 16'd99)  ? 16'd99  : p_brush_writes_r;
    wire [15:0] c5_cl  = p_cost5_r;

    wire [3:0] r_hu = (ray_cl / 16'd100) % 10;
    wire [3:0] r_te = (ray_cl / 16'd10)  % 10;
    wire [3:0] r_on = (ray_cl / 16'd1)   % 10;

    wire [3:0] f_hu = (fr_cl / 16'd100) % 10;
    wire [3:0] f_te = (fr_cl / 16'd10)  % 10;
    wire [3:0] f_on = (fr_cl / 16'd1)   % 10;

    wire [3:0] h_te2 = (hw_cl / 16'd10) % 10;
    wire [3:0] h_on2 = (hw_cl / 16'd1)  % 10;

    wire [3:0] b_te2 = (bw_cl / 16'd10) % 10;
    wire [3:0] b_on2 = (bw_cl / 16'd1)  % 10;

    wire [3:0] c_te2 = (c5_cl / 16'd10) % 10;
    wire [3:0] c_on2 = (c5_cl / 16'd1)  % 10;



    integer y_top_i;
    always @* begin
        y_top_i = SPARK_Y1 - 1;
        if (s_h >= (SPARK_H-2)) y_top_i = SPARK_Y0 + 1;
        else y_top_i = (SPARK_Y1 - 1) - s_h;
    end

    // Spark draw suppressed for local no-target
    wire spark_ink = in_spark && !no_target_pix &&
                     (hcount > SPARK_X0) && (hcount < SPARK_X1) &&
                     (vcount >= y_top_i) && (vcount < SPARK_Y1);

    wire stale_tint = en_stale_eff && stale;

    // ------------------------------------------------------------------------
    // HUD text strings
    // ------------------------------------------------------------------------
    wire title_ink = text3x5_ink(hcount, vcount,
                                 X0 + PAD_L, Y0 + PAD_T,
                                 5, "SONAR           ");

    wire [15:0] dist_in    = mm_to_inch(dist_mm_r);
    wire [15:0] dist_disp  = en_in_eff ? dist_in : dist_mm_r;
    wire [15:0] dist_clamp = (dist_disp > 16'd9999) ? 16'd9999 : dist_disp;

    wire [3:0] d_th = (dist_clamp / 16'd1000) % 10;
    wire [3:0] d_hu = (dist_clamp / 16'd100)  % 10;
    wire [3:0] d_te = (dist_clamp / 16'd10)   % 10;
    wire [3:0] d_on = (dist_clamp / 16'd1)    % 10;

    wire [8*16-1:0] dist_str = {
        "D",":",
        (d_th + "0"),
        (d_hu + "0"),
        (d_te + "0"),
        (d_on + "0"),
        (en_in_eff ? "I" : "M"),
        (en_in_eff ? "N" : "M"),
        "        "
    };

    wire dist_ink = text3x5_ink(hcount, vcount,
                                X0 + PAD_L, Y0 + PAD_T + TITLE_H,
                                10, dist_str);

    wire [9:0] age_cl = (age_ms_r > 10'd999) ? 10'd999 : age_ms_r;
    wire [3:0] a_hu = (age_cl / 10'd100) % 10;
    wire [3:0] a_te = (age_cl / 10'd10)  % 10;
    wire [3:0] a_on = (age_cl / 10'd1)   % 10;

    wire [8*16-1:0] age_str = {
        "A",":",
        (a_hu + "0"),
        (a_te + "0"),
        (a_on + "0"),
        "M","S",
        "         "
    };

    wire age_ink = en_age_eff && text3x5_ink(hcount, vcount,
                                             X0 + PAD_L, Y0 + PAD_T + TITLE_H + TEXT_H,
                                             9, age_str);

    wire [15:0] hz_val = (period_ms_r == 0) ? 16'd0 : (16'd1000 / period_ms_r);
    wire [15:0] hz_cl  = (hz_val > 16'd999) ? 16'd999 : hz_val;

    wire [3:0] h_hu = (hz_cl / 16'd100) % 10;
    wire [3:0] h_te = (hz_cl / 16'd10)  % 10;
    wire [3:0] h_on = (hz_cl / 16'd1)   % 10;

    wire [8*16-1:0] hz_str = {
        "H",":",
        (h_hu + "0"),
        (h_te + "0"),
        (h_on + "0"),
        "H","Z",
        "         "
    };

    wire hz_ink = en_hz_eff && text3x5_ink(hcount, vcount,
                                           X0 + PAD_L, Y0 + PAD_T + TITLE_H + 2*TEXT_H,
                                           9, hz_str);

    wire [3:0] dp_te  = (drop_r / 10) % 10;
    wire [3:0] dp_on  = (drop_r / 1)  % 10;
    wire [3:0] nt_te2 = (nt_r   / 10) % 10;
    wire [3:0] nt_on2 = (nt_r   / 1)  % 10;

    wire [8*16-1:0] dn_str = {
        "D","P",
        (dp_te + "0"),
        (dp_on + "0"),
        " ",
        "N","T",
        (nt_te2 + "0"),
        (nt_on2 + "0"),
        "       "
    };


    wire [8*16-1:0] ray_str = {
        "R",":",
        (r_hu + "0"),
        (r_te + "0"),
        (r_on + "0"),
        "C",
        " ",
        "F",":",
        (f_hu + "0"),
        (f_te + "0"),
        (f_on + "0"),
        "   "
    };

    wire [8*16-1:0] hitb_str = {
        "H",":",
        (h_te2 + "0"),
        (h_on2 + "0"),
        " ",
        "B",":",
        (b_te2 + "0"),
        (b_on2 + "0"),
        " ",
        "C",":",
        (c_te2 + "0"),
        (c_on2 + "0"),
        " "
    };

    wire [8*16-1:0] stat_str = {
        (p_no_target_r ? "N" : "-"),
        (p_clamped_r   ? "C" : "-"),
        (p_oob_r       ? "O" : "-"),
        " ",
        "S",":",
        ((p_seq_r / 8'd100) % 10 + "0"),
        ((p_seq_r / 8'd10)  % 10 + "0"),
        ((p_seq_r / 8'd1)   % 10 + "0"),
        "       "
    };
    
    


    wire dn_ink = (hud_mode_pix != 0) && text3x5_ink(hcount, vcount,
                                                     X0 + PAD_L, Y1 - 10,
                                                     12, dn_str);

    // ------------------------------------------------------------------------
    // HUD indicators (src dot, uart dot, activity bar)
    // ------------------------------------------------------------------------
    wire src_dot = in_widget &&
                   (hcount >= (X1 - 10)) && (hcount <= (X1 - 6)) &&
                   (vcount >= (Y0 + 6))  && (vcount <= (Y0 + 10));

    wire uart_dot = in_widget &&
                    (hcount >= (X1 - 10)) && (hcount <= (X1 - 6)) &&
                    (vcount >= (Y0 + 14)) && (vcount <= (Y0 + 18));

    wire act_bar = in_widget && en_act_eff &&
                   (hcount >= (X0 + PAD_L)) && (hcount <= (X0 + PAD_L + 80)) &&
                   (vcount == (Y1 - 3)) &&
                   ((hcount - (X0 + PAD_L)) < (act_meter >> 2));

    wire ray_ink = (hud_mode_pix != 0) && text3x5_ink(
                       hcount, vcount,
                       X0 + PAD_L, Y0 + PAD_T + TITLE_H + 3*TEXT_H,
                       12, ray_str
                   );
               
                   wire hitb_ink = (hud_mode_pix != 0) && text3x5_ink(
                       hcount, vcount,
                       X0 + PAD_L, Y0 + PAD_T + TITLE_H + 4*TEXT_H,
                       13, hitb_str
                   );
               
                   wire stat_ink = (hud_mode_pix != 0) && text3x5_ink(
                       hcount, vcount,
                       X0 + PAD_L, Y0 + PAD_T + TITLE_H + 5*TEXT_H,
                       10, stat_str
                   );



    wire p_evt_bar = in_widget && (hud_mode_pix != 0) &&
                                    (hcount >= (X0 + PAD_L)) && (hcount <= (X0 + PAD_L + 80)) &&
                                    (vcount == (Y1 - 6)) &&
                                    ((hcount - (X0 + PAD_L)) < (p_evt_meter_r >> 2));


    // ------------------------------------------------------------------------
    // Final HUD compositor
    // - Priority: border -> widget tint -> text -> spark -> indicators
    // ------------------------------------------------------------------------
    always @* begin
        rgb_out = rgb_bg;

        if (!en_master) begin
            rgb_out = rgb_bg;
        end else if (on_border) begin
            rgb_out = C_BORDER;
        end else if (in_widget) begin
            if (stale_tint)
                rgb_out = 12'h211;

            if (title_ink) rgb_out = C_TEXT;
            if (dist_ink)  rgb_out = C_TEXT;

            if (age_ink)   rgb_out = stale ? C_WARN : C_DIM;
            if (hz_ink)    rgb_out = C_DIM;
            if (dn_ink)    rgb_out = C_DIM;

            if (ray_ink)   rgb_out = C_DIM;
            if (hitb_ink)  rgb_out = C_DIM;
            if (stat_ink)  rgb_out = p_oob_r ? C_BAD :
                                      p_clamped_r ? C_WARN :
                                      p_no_target_r ? 12'h0AF :
                                      C_OK;

            if (in_spark) begin
                if (on_spark_border) rgb_out = 12'h555;
                else if (on_midline) rgb_out = 12'h333;
                else if (spark_ink)  rgb_out = stale ? 12'hA44 : 12'h4AF;
            end

            if (src_dot)
                rgb_out = src_color(src_pix);

            if (en_uart_eff && uart_dot) begin
                rgb_out = uart_crc_err_pulse_pix ? C_BAD :
                          uart_pkt_pulse_pix     ? C_OK  : 12'h222;
            end

            if (p_evt_bar)
                rgb_out = p_oob_r ? C_BAD : C_WARN;

            if (act_bar)
                rgb_out = stale ? C_WARN : C_OK;
        end
    end
endmodule

`default_nettype wire
