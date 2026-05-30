`timescale 1ns/1ps
`default_nettype none

// ============================================================================
// sonar_map_renderer.v  (BRAM-latency aligned + telemetry + robustness)
//------------------------------------------------------------------------------
// PIX-domain map panel renderer with deterministic BRAM read alignment and
// built-in observability.
//
// Core behavior
//   - Computes map address from a BRAM-latency lookahead coordinate when inside
//     the configured panel.
//   - Expects map_rd_data to correspond to the address presented RD_LAT cycles
//     earlier (registered BRAM read latency).
//   - painter_valid / painter_rgb are aligned to the current HDMI pixel while
//     map_rd_addr is driven early enough to absorb BRAM/read-register latency.
//
// Robustness extensions
//   - Explicit RD_LAT parameter (>=1) with inside/border/grid pipelines.
//   - Optional divider strategy: shift when power-of-two, otherwise divide.
//   - Optional grid/border overlay to make geometry/scale diagnosable.
//   - Defensive clamps and event counters.
//
// Telemetry extensions (frame-scoped, PIX domain)
//   - Counts: inside pixels, read requests, clamp events, nonzero pixels.
//   - Tracks: last requested address and last cell coordinates.
//   - Tracks: per-frame max intensity nibble.
//   - Emits a compact snap bus once per frame_tick.
//
// Notes
//   - DATA_W must be >=4 (top nibble plus nonzero detection drive color).
//   - MAP_W*MAP_H should fit in 16-bit address space for addr32_now[15:0] use.
//   - If MAP_W*MAP_H can exceed 65535, widen map_rd_addr and addr truncation.
// ============================================================================

module sonar_map_renderer #(
    // ---------------- Map geometry ----------------
    parameter integer MAP_W  = 256,
    parameter integer MAP_H  = 256,
    parameter integer DATA_W = 8,

    // ---------------- Panel placement ----------------
    parameter integer PANEL_X0  = 16,
    parameter integer PANEL_Y0  = 16,
    parameter integer PANEL_WPX = 256,
    parameter integer PANEL_HPX = 256,

    // ---------------- Pixel-per-cell scale ----------------
    // If CELL_PX_* are power-of-two, providing *_SHIFT enables shift-based div.
    parameter integer CELL_PX_W       = 1,
    parameter integer CELL_PX_H       = 1,
    parameter integer CELL_PX_W_SHIFT = 0,  // valid if CELL_PX_W == (1<<SHIFT)
    parameter integer CELL_PX_H_SHIFT = 0,  // valid if CELL_PX_H == (1<<SHIFT)

    // ---------------- Full-map fit mode ----------------
    // When enabled, local panel coordinates are mapped across the full MAP_W x
    // MAP_H address space. For power-of-two ratios, FIT_*_SHIFT keeps the scale
    // as a shift instead of a general multiply/divide.
    parameter integer FIT_TO_PANEL = 0,
    parameter integer FIT_X_SHIFT  = 0,
    parameter integer FIT_Y_SHIFT  = 0,

    // ---------------- BRAM read latency ----------------
    parameter integer RD_LAT = 1,  // registered read latency cycles (>=1)

    // ---------------- Visual debug overlays ----------------
    parameter integer EN_BORDER = 0,
    parameter integer EN_GRID   = 0,

    // Border color (RGB444)
    parameter [11:0] BORDER_RGB = 12'hAAA,

    // Grid policy: draw a 1-pixel line at cell boundaries
    parameter [11:0] GRID_RGB   = 12'h444,

    // ---------------- Telemetry ----------------
    parameter integer EN_TELEM  = 1
)(
    input  wire              pix_clk,
    input  wire              pix_rst,

    input  wire [9:0]        hcount,
    input  wire [9:0]        vcount,
    input  wire              active_video,

    // Frame tick (one pulse per frame) used to snapshot/reset frame telemetry.
    input  wire              frame_tick,

    // Map BRAM read port
    output reg  [15:0]       map_rd_addr,
    input  wire [DATA_W-1:0] map_rd_data,

    // Renderer output (valid is aligned to map_rd_data)
    output reg               painter_valid,
    output reg  [11:0]       painter_rgb,

    // Optional per-frame telemetry snapshot
    output reg  [63:0]       renderer_telem_pix,
    output reg               renderer_telem_upd_pix
);

    // ------------------------------------------------------------------------
    // Compile-time bounds (exclusive). Localparams are integers; comparisons
    // use explicit casts to fixed widths where needed.
    // ------------------------------------------------------------------------
    localparam integer PANEL_X1 = PANEL_X0 + PANEL_WPX; // exclusive
    localparam integer PANEL_Y1 = PANEL_Y0 + PANEL_HPX; // exclusive

    // ------------------------------------------------------------------------
    // Occupancy mapping.
    //
    // The painter writes low nonzero FREE_VAL cells and high HIT_VAL cells.
    // A pure top-nibble grayscale map makes FREE_VAL=10 render black, hiding
    // the ray/free-space path. Keep zero cells black, but make any nonzero low
    // value visible as dim cyan/green and high values bright.
    // ------------------------------------------------------------------------
    function [11:0] v_to_rgb444;
        input [DATA_W-1:0] v;
        reg [3:0] n;
        begin
            n = v[DATA_W-1 : DATA_W-4];
            if (v == {DATA_W{1'b0}})
                v_to_rgb444 = 12'h000;
            else if (n == 4'h0)
                v_to_rgb444 = 12'h034;
            else if (n < 4'h4)
                v_to_rgb444 = 12'h068;
            else if (n < 4'h8)
                v_to_rgb444 = 12'h0B6;
            else if (n < 4'hC)
                v_to_rgb444 = 12'hFC0;
            else
                v_to_rgb444 = 12'hFFF;
        end
    endfunction

    function [3:0] v_to_nibble;
        input [DATA_W-1:0] v;
        begin
            v_to_nibble = v[DATA_W-1 : DATA_W-4];
        end
    endfunction

    function [3:0] v_to_telem_nibble;
        input [DATA_W-1:0] v;
        reg [3:0] n;
        begin
            n = v_to_nibble(v);
            v_to_telem_nibble = ((n == 4'd0) && (v != {DATA_W{1'b0}})) ? 4'd1 : n;
        end
    endfunction

    // ------------------------------------------------------------------------
    // Small saturating helpers (synth-friendly)
    // ------------------------------------------------------------------------
    function [5:0] sat6_from_u32;
        input [31:0] v;
        begin
            sat6_from_u32 = (v > 32'd63) ? 6'd63 : v[5:0];
        end
    endfunction

    function [3:0] sat4_from_u32;
        input [31:0] v;
        begin
            sat4_from_u32 = (v > 32'd15) ? 4'd15 : v[3:0];
        end
    endfunction

`ifndef SYNTHESIS
    initial begin
        if (DATA_W < 4) begin
            $display("sonar_map_renderer: ERROR: DATA_W must be >= 4");
        end
        if (CELL_PX_W <= 0 || CELL_PX_H <= 0) begin
            $display("sonar_map_renderer: ERROR: CELL_PX_W/H must be >= 1");
        end
        if (RD_LAT <= 0) begin
            $display("sonar_map_renderer: ERROR: RD_LAT must be >= 1");
        end
        if ((MAP_W * MAP_H) > 65535) begin
            $display("sonar_map_renderer: WARNING: MAP_W*MAP_H > 65535; map_rd_addr truncates.");
        end
    end
`endif

    // ------------------------------------------------------------------------
    // Read-request coordinate.
    //
    // map_rd_addr is registered here, the map memory registers dout, and this
    // module registers painter_rgb. The request therefore has to be issued
    // RD_LAT pixels early so painter_rgb corresponds to the current HDMI pixel.
    // ------------------------------------------------------------------------
    localparam [10:0] RD_LAT_U11 = RD_LAT;

    wire [10:0] hcount_req_w = {1'b0, hcount} + RD_LAT_U11;
    wire [9:0]  hcount_req   = hcount_req_w[9:0];

    wire inside_now =
        active_video &&
        !hcount_req_w[10] &&
        (hcount_req >= PANEL_X0[9:0]) && (hcount_req < PANEL_X1[9:0]) &&
        (vcount >= PANEL_Y0[9:0]) && (vcount < PANEL_Y1[9:0]);

    // Local panel coordinates (unsigned)
    wire [9:0] lx_now = hcount_req - PANEL_X0[9:0];
    wire [9:0] ly_now = vcount - PANEL_Y0[9:0];

    // ------------------------------------------------------------------------
    // Cell coordinate computation
    //   - Default mode: panel pixels address cells using CELL_PX_* scale.
    //   - Fit mode   : panel pixels span the complete map extent.
    //   - Prefer shift paths when the configured ratios are powers of two.
    // ------------------------------------------------------------------------
    wire use_shift_w = (CELL_PX_W == (1 << CELL_PX_W_SHIFT));
    wire use_shift_h = (CELL_PX_H == (1 << CELL_PX_H_SHIFT));

    wire use_fit_shift_w =
        (FIT_TO_PANEL != 0) && (MAP_W == (PANEL_WPX << FIT_X_SHIFT));
    wire use_fit_shift_h =
        (FIT_TO_PANEL != 0) && (MAP_H == (PANEL_HPX << FIT_Y_SHIFT));

    wire [31:0] cx_fit_u32 =
        (PANEL_WPX <= 1) ? 32'd0 :
        (use_fit_shift_w) ? ({22'd0, lx_now} << FIT_X_SHIFT) :
                            (({22'd0, lx_now} * MAP_W) / PANEL_WPX);

    wire [31:0] cy_fit_u32 =
        (PANEL_HPX <= 1) ? 32'd0 :
        (use_fit_shift_h) ? ({22'd0, ly_now} << FIT_Y_SHIFT) :
                            (({22'd0, ly_now} * MAP_H) / PANEL_HPX);

    wire [15:0] cx_panel_scaled =
        (CELL_PX_W == 1) ? {6'd0, lx_now} :
        (use_shift_w)    ? ({6'd0, lx_now} >> CELL_PX_W_SHIFT) :
                           ({6'd0, lx_now} / CELL_PX_W);

    wire [15:0] cy_panel_scaled =
        (CELL_PX_H == 1) ? {6'd0, ly_now} :
        (use_shift_h)    ? ({6'd0, ly_now} >> CELL_PX_H_SHIFT) :
                           ({6'd0, ly_now} / CELL_PX_H);

    wire [15:0] cx_div =
        (FIT_TO_PANEL != 0) ? cx_fit_u32[15:0] : cx_panel_scaled;

    wire [15:0] cy_div =
        (FIT_TO_PANEL != 0) ? cy_fit_u32[15:0] : cy_panel_scaled;

    // Clamp to map bounds
    wire cx_oob = (cx_div >= MAP_W[15:0]);
    wire cy_oob = (cy_div >= MAP_H[15:0]);

    wire [15:0] cx_clamp = cx_oob ? (MAP_W[15:0] - 16'd1) : cx_div;
    wire [15:0] cy_clamp = cy_oob ? (MAP_H[15:0] - 16'd1) : cy_div;

    // Address = cy*MAP_W + cx (truncate to 16-bit)
    wire [31:0] addr32_now = (cy_clamp * MAP_W[15:0]) + cx_clamp;
    wire [15:0] addr_now   = addr32_now[15:0];

      
    wire [9:0] temp_panel_x1 = PANEL_X1-1;
    wire [9:0] temp_panel_y1 = PANEL_Y1-1;
    wire on_border_now =
        (EN_BORDER != 0) &&
        inside_now &&
        ((hcount_req == PANEL_X0[9:0]) || (hcount_req == temp_panel_x1[9:0]) ||
         (vcount == PANEL_Y0[9:0]) || (vcount == temp_panel_y1[9:0]));

    // Grid at cell boundaries: line at lx%CELL_PX_W==0 or ly%CELL_PX_H==0
    wire [9:0] lx_mod_w =
        (CELL_PX_W == 1) ? 10'd1 : // "not on grid" when scale 1
        (use_shift_w)    ? (lx_now & ((1 << CELL_PX_W_SHIFT) - 1)) :
                           (lx_now % CELL_PX_W);

    wire [9:0] ly_mod_h =
        (CELL_PX_H == 1) ? 10'd1 :
        (use_shift_h)    ? (ly_now & ((1 << CELL_PX_H_SHIFT) - 1)) :
                           (ly_now % CELL_PX_H);

    wire on_grid_now =
        (EN_GRID != 0) &&
        inside_now &&
        ( ((CELL_PX_W != 1) && (lx_mod_w == 10'd0)) ||
          ((CELL_PX_H != 1) && (ly_mod_h == 10'd0)) );

    // ------------------------------------------------------------------------
    // RD_LAT alignment pipelines (safe for RD_LAT==1)
    // ------------------------------------------------------------------------
    reg [RD_LAT-1:0] inside_pipe;
    reg [RD_LAT-1:0] border_pipe;
    reg [RD_LAT-1:0] grid_pipe;

    generate
        if (RD_LAT == 1) begin : g_lat1
            always @(posedge pix_clk) begin
                if (pix_rst) begin
                    inside_pipe <= 1'b0;
                    border_pipe <= 1'b0;
                    grid_pipe   <= 1'b0;
                end else begin
                    inside_pipe <= inside_now;
                    border_pipe <= on_border_now;
                    grid_pipe   <= on_grid_now;
                end
            end
        end else begin : g_latn
            always @(posedge pix_clk) begin
                if (pix_rst) begin
                    inside_pipe <= {RD_LAT{1'b0}};
                    border_pipe <= {RD_LAT{1'b0}};
                    grid_pipe   <= {RD_LAT{1'b0}};
                end else begin
                    inside_pipe <= {inside_pipe[RD_LAT-2:0], inside_now};
                    border_pipe <= {border_pipe[RD_LAT-2:0], on_border_now};
                    grid_pipe   <= {grid_pipe[RD_LAT-2:0], on_grid_now};
                end
            end
        end
    endgenerate

    // ------------------------------------------------------------------------
    // Captured request metadata (telemetry)
    // ------------------------------------------------------------------------
    reg [15:0] last_addr_req;
    reg [15:0] last_cx_req;
    reg [15:0] last_cy_req;

    // ------------------------------------------------------------------------
    // Telemetry counters (frame-scoped)
    // ------------------------------------------------------------------------
    reg [31:0] ctr_inside_pix;
    reg [31:0] ctr_rd_req;
    reg [31:0] ctr_clamp_evt;
    reg [31:0] ctr_nonzero_pix;
    reg [3:0]  max_nibble;

    // ------------------------------------------------------------------------
    // Main sequential logic: address, telemetry, output compositor
    // ------------------------------------------------------------------------
    always @(posedge pix_clk) begin
        if (pix_rst) begin
            map_rd_addr            <= 16'd0;

            painter_valid          <= 1'b0;
            painter_rgb            <= 12'h000;

            last_addr_req          <= 16'd0;
            last_cx_req            <= 16'd0;
            last_cy_req            <= 16'd0;

            ctr_inside_pix         <= 32'd0;
            ctr_rd_req             <= 32'd0;
            ctr_clamp_evt          <= 32'd0;
            ctr_nonzero_pix        <= 32'd0;
            max_nibble             <= 4'd0;

            renderer_telem_pix     <= 64'd0;
            renderer_telem_upd_pix <= 1'b0;
        end else begin
            renderer_telem_upd_pix <= 1'b0;

            // ----------------------------------------------------------------
            // Address drive: request meaningful address only when inside.
            // Outside: drive 0 to keep downstream BRAM activity predictable.
            // ----------------------------------------------------------------
            map_rd_addr <= inside_now ? addr_now : 16'd0;

            // Record last request info (only when issuing a meaningful request)
            if (inside_now) begin
                last_addr_req <= addr_now;
                last_cx_req   <= cx_clamp;
                last_cy_req   <= cy_clamp;
            end

            // ----------------------------------------------------------------
            // Telemetry accumulation
            // ----------------------------------------------------------------
            if (EN_TELEM != 0) begin
                if (inside_now) begin
                    if (ctr_inside_pix != 32'hFFFF_FFFF) ctr_inside_pix <= ctr_inside_pix + 32'd1;
                    if (ctr_rd_req     != 32'hFFFF_FFFF) ctr_rd_req     <= ctr_rd_req     + 32'd1;

                    if (cx_oob || cy_oob) begin
                        if (ctr_clamp_evt != 32'hFFFF_FFFF) ctr_clamp_evt <= ctr_clamp_evt + 32'd1;
                    end
                end

                // Count nonzero pixels aligned to BRAM data valid phase
                if (inside_pipe[RD_LAT-1]) begin
                    if (map_rd_data != {DATA_W{1'b0}}) begin
                        if (ctr_nonzero_pix != 32'hFFFF_FFFF) ctr_nonzero_pix <= ctr_nonzero_pix + 32'd1;
                    end
                    if (v_to_telem_nibble(map_rd_data) > max_nibble) begin
                        max_nibble <= v_to_telem_nibble(map_rd_data);
                    end
                end

                // Snapshot and reset on frame_tick
                if (frame_tick) begin
                    // Packed telemetry (64-bit)
                    // [63:60] max_nibble
                    // [59:44] last_addr_req (16)
                    // [43:32] last_cx_req[11:0]
                    // [31:20] last_cy_req[11:0]
                    // [19:14] clamp_evt[5:0]   (sat)
                    // [13:8]  nonzero[5:0]     (sat)
                    // [7:4]   rd_req[3:0]      (sat coarse)
                    // [3:0]   inside[3:0]      (sat coarse)
                    renderer_telem_pix <= {
                        max_nibble,
                        last_addr_req,
                        last_cx_req[11:0],
                        last_cy_req[11:0],
                        sat6_from_u32(ctr_clamp_evt),
                        sat6_from_u32(ctr_nonzero_pix),
                        sat4_from_u32(ctr_rd_req),
                        sat4_from_u32(ctr_inside_pix)
                    };
                    renderer_telem_upd_pix <= 1'b1;

                    ctr_inside_pix  <= 32'd0;
                    ctr_rd_req      <= 32'd0;
                    ctr_clamp_evt   <= 32'd0;
                    ctr_nonzero_pix <= 32'd0;
                    max_nibble      <= 4'd0;
                end
            end

            // ----------------------------------------------------------------
            // Output aligned to BRAM data latency
            // ----------------------------------------------------------------
            painter_valid <= inside_pipe[RD_LAT-1];

            // Base grayscale from map data
            painter_rgb <= inside_pipe[RD_LAT-1] ? v_to_rgb444(map_rd_data) : 12'h000;

            // Debug overlays (priority over grayscale)
            if (inside_pipe[RD_LAT-1]) begin
                if (border_pipe[RD_LAT-1]) painter_rgb <= BORDER_RGB;
                else if (grid_pipe[RD_LAT-1]) painter_rgb <= GRID_RGB;
            end
        end
    end

endmodule

`default_nettype wire
