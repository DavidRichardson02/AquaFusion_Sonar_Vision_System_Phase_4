`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// hud_debug_panel_rich_numeric.v
//------------------------------------------------------------------------------
// Rich deterministic telemetry panel with live numeric text fields.
//
// Text additions:
//   RNG:###
//   AGE:###
//   FLT:####
//
// Contracts:
//   - Rendering is purely raster-combinational from stable status inputs.
//   - Text is rendered through vga_textline_3x5.
//   - No RAM or framebuffer is introduced.
//   - Formatter outputs must come from stable snapshot-qualified telemetry.
//==============================================================================

module hud_debug_panel_rich_numeric #(
    parameter integer PANEL_X0 = 12,
    parameter integer PANEL_Y0 = 8,
    parameter integer PANEL_W  = 224,
    parameter integer PANEL_H  = 112,

    parameter integer TEXT_REGISTER_SELECT = 0,
    parameter integer TEXT_REGISTER_GLYPH  = 0
)(
    input  wire        clk_pix,
    input  wire        rst_pix,

    input  wire [11:0] pix_x,
    input  wire [11:0] pix_y,
    input  wire        de,

    input  wire        clk_locked,
    input  wire        sonar_valid,
    input  wire        sonar_stale,
    input  wire [15:0] sonar_age_ms,
    input  wire [8:0]  sonar_distance_in,
    input  wire        sonar_update_toggle,
    input  wire [7:0]  frame_ctr_lsb,
    input  wire [3:0]  fault_flags,

    output reg  [23:0] rgb_out
);

    //--------------------------------------------------------------------------
    // Raster truncation for text modules
    //--------------------------------------------------------------------------
    wire [9:0] hcount10 = pix_x[9:0];
    wire [9:0] vcount10 = pix_y[9:0];

    //--------------------------------------------------------------------------
    // Geometry
    //--------------------------------------------------------------------------
    localparam integer PANEL_X1 = PANEL_X0 + PANEL_W;
    localparam integer PANEL_Y1 = PANEL_Y0 + PANEL_H;

    localparam integer BORDER_W = 2;
    localparam integer HEADER_H = 12;

    localparam integer TILE_Y0  = PANEL_Y0 + 20;
    localparam integer TILE_Y1  = TILE_Y0 + 18;

    localparam integer CLK_X0   = PANEL_X0 + 10;
    localparam integer CLK_X1   = CLK_X0 + 28;

    localparam integer VAL_X0   = PANEL_X0 + 44;
    localparam integer VAL_X1   = VAL_X0 + 28;

    localparam integer ACT_X0   = PANEL_X0 + 78;
    localparam integer ACT_X1   = ACT_X0 + 28;

    localparam integer DIST_X0  = PANEL_X0 + 10;
    localparam integer DIST_X1  = PANEL_X0 + 134;
    localparam integer DIST_Y0  = PANEL_Y0 + 50;
    localparam integer DIST_Y1  = DIST_Y0 + 10;
    localparam integer DIST_W   = DIST_X1 - DIST_X0;

    localparam integer AGE_X0   = PANEL_X0 + 10;
    localparam integer AGE_X1   = PANEL_X0 + 134;
    localparam integer AGE_Y0   = PANEL_Y0 + 70;
    localparam integer AGE_Y1   = AGE_Y0 + 10;
    localparam integer AGE_W    = AGE_X1 - AGE_X0;

    localparam integer FAULT_Y0  = PANEL_Y0 + 90;
    localparam integer FAULT_Y1  = FAULT_Y0 + 10;
    localparam integer FAULT_W   = 12;
    localparam integer FAULT_GAP = 4;
    localparam integer FAULT0_X0 = PANEL_X0 + 10;
    localparam integer FAULT1_X0 = FAULT0_X0 + FAULT_W + FAULT_GAP;
    localparam integer FAULT2_X0 = FAULT1_X0 + FAULT_W + FAULT_GAP;
    localparam integer FAULT3_X0 = FAULT2_X0 + FAULT_W + FAULT_GAP;

    localparam integer WIN_X0   = PANEL_X0 + 148;
    localparam integer WIN_X1   = PANEL_X0 + 214;
    localparam integer WIN_Y0   = PANEL_Y0 + 20;
    localparam integer WIN_Y1   = PANEL_Y0 + 96;
    localparam integer WIN_W    = WIN_X1 - WIN_X0;
    localparam integer WIN_H    = WIN_Y1 - WIN_Y0;

    //--------------------------------------------------------------------------
    // Text placement
    //--------------------------------------------------------------------------
    localparam [9:0] TXT_HDR_X = PANEL_X0 + 10;
    localparam [9:0] TXT_HDR_Y = PANEL_Y0 + 3;

    localparam [9:0] TXT_CLK_X = CLK_X0;
    localparam [9:0] TXT_CLK_Y = PANEL_Y0 + 14;

    localparam [9:0] TXT_VAL_X = VAL_X0;
    localparam [9:0] TXT_VAL_Y = PANEL_Y0 + 14;

    localparam [9:0] TXT_RNG_X = DIST_X0;
    localparam [9:0] TXT_RNG_Y = DIST_Y0 - 7;

    localparam [9:0] TXT_AGE_X = AGE_X0;
    localparam [9:0] TXT_AGE_Y = AGE_Y0 - 7;

    localparam [9:0] TXT_FLT_X = FAULT0_X0;
    localparam [9:0] TXT_FLT_Y = FAULT_Y0 - 7;

    //--------------------------------------------------------------------------
    // Static packed strings
    //--------------------------------------------------------------------------
    localparam [127:0] TXT_HDR =
        {8'h53,8'h4F,8'h4E,8'h41,8'h52,8'h20,8'h53,8'h54,
         8'h41,8'h54,8'h55,8'h53,8'h20,8'h20,8'h20,8'h20}; // SONAR STATUS

    localparam [127:0] TXT_CLK =
        {8'h43,8'h4C,8'h4B,{13{8'h20}}}; // CLK

    localparam [127:0] TXT_VAL =
        {8'h56,8'h41,8'h4C,{13{8'h20}}}; // VAL

    //--------------------------------------------------------------------------
    // Live numeric formatters
    //--------------------------------------------------------------------------
    wire [23:0] rng_ascii3;
    wire [23:0] age_ascii3;
    wire [31:0] flt_ascii4;

    hud_fmt_dec3_sat16 #(
        .MAX_VAL(255)
    ) u_fmt_rng (
        .value_in ({7'd0, sonar_distance_in}),
        .ascii3   (rng_ascii3)
    );

    hud_fmt_dec3_sat16 #(
        .MAX_VAL(999)
    ) u_fmt_age (
        .value_in (sonar_age_ms),
        .ascii3   (age_ascii3)
    );

    hud_fmt_bin4 u_fmt_flt (
        .value_in (fault_flags),
        .ascii4   (flt_ascii4)
    );

    //--------------------------------------------------------------------------
    // Live packed strings
    //--------------------------------------------------------------------------
    wire [127:0] TXT_RNG_LIVE =
        {8'h52,8'h4E,8'h47,8'h3A, rng_ascii3, 72'h20_20_20_20_20_20_20_20_20};

    wire [127:0] TXT_AGE_LIVE =
        {8'h41,8'h47,8'h45,8'h3A, age_ascii3, 72'h20_20_20_20_20_20_20_20_20};

    wire [127:0] TXT_FLT_LIVE =
        {8'h46,8'h4C,8'h54,8'h3A, flt_ascii4, 64'h20_20_20_20_20_20_20_20};

    //--------------------------------------------------------------------------
    // Region tests
    //--------------------------------------------------------------------------
    wire in_panel;
    wire in_header;
    wire in_body;
    wire in_border;

    assign in_panel  = de &&
                       (pix_x >= PANEL_X0) && (pix_x < PANEL_X1) &&
                       (pix_y >= PANEL_Y0) && (pix_y < PANEL_Y1);

    assign in_header = in_panel &&
                       (pix_y >= PANEL_Y0) &&
                       (pix_y <  (PANEL_Y0 + HEADER_H));

    assign in_body   = in_panel &&
                       (pix_y >= (PANEL_Y0 + HEADER_H)) &&
                       (pix_y <  PANEL_Y1);

    assign in_border = in_panel &&
                       (
                           (pix_x <  (PANEL_X0 + BORDER_W)) ||
                           (pix_x >= (PANEL_X1 - BORDER_W)) ||
                           (pix_y <  (PANEL_Y0 + BORDER_W)) ||
                           (pix_y >= (PANEL_Y1 - BORDER_W))
                       );

    //--------------------------------------------------------------------------
    // Derived telemetry metrics
    //--------------------------------------------------------------------------
    integer dist_clamped_in;
    integer dist_fill_px;
    integer dist_marker_24;
    integer dist_marker_72;
    integer dist_marker_144;

    integer age_clamped_ms;
    integer age_fill_px;
    integer age_marker_50;
    integer age_marker_100;
    integer age_marker_150;

    integer win_cursor_x;
    integer header_glow_x0;

    reg [23:0] border_color;
    reg [23:0] clock_color;
    reg [23:0] val_color;
    reg [23:0] dist_color;
    reg [23:0] age_color;
    reg [23:0] win_color;

    reg [23:0] hdr_text_color;
    reg [23:0] clk_text_color;
    reg [23:0] val_text_color;
    reg [23:0] rng_text_color;
    reg [23:0] age_text_color;
    reg [23:0] flt_text_color;

    always @* begin
        //----------------------------------------------------------------------
        // Distance scaling
        //----------------------------------------------------------------------
        if (sonar_distance_in < 9'd6)
            dist_clamped_in = 6;
        else if (sonar_distance_in > 9'd255)
            dist_clamped_in = 255;
        else
            dist_clamped_in = sonar_distance_in;

        if (!sonar_valid)
            dist_fill_px = 0;
        else
            dist_fill_px = ((dist_clamped_in - 6) * DIST_W) / 249;

        dist_marker_24  = DIST_X0 + ((24  - 6) * DIST_W) / 249;
        dist_marker_72  = DIST_X0 + ((72  - 6) * DIST_W) / 249;
        dist_marker_144 = DIST_X0 + ((144 - 6) * DIST_W) / 249;

        //----------------------------------------------------------------------
        // Age scaling
        //----------------------------------------------------------------------
        if (sonar_age_ms > 16'd255)
            age_clamped_ms = 255;
        else
            age_clamped_ms = sonar_age_ms;

        if (!sonar_valid)
            age_fill_px = AGE_W;
        else
            age_fill_px = (age_clamped_ms * AGE_W) / 255;

        age_marker_50  = AGE_X0 + ( 50 * AGE_W) / 255;
        age_marker_100 = AGE_X0 + (100 * AGE_W) / 255;
        age_marker_150 = AGE_X0 + (150 * AGE_W) / 255;

        //----------------------------------------------------------------------
        // Range window cursor
        //----------------------------------------------------------------------
        win_cursor_x = WIN_X0 + 4 + ((dist_clamped_in - 6) * (WIN_W - 8)) / 249;

        //----------------------------------------------------------------------
        // Header accent position
        //----------------------------------------------------------------------
        header_glow_x0 = PANEL_X0 + 8 + frame_ctr_lsb;

        //----------------------------------------------------------------------
        // Semantic colors
        //----------------------------------------------------------------------
        if (!clk_locked)
            border_color = 24'hA00000;
        else if (!sonar_valid)
            border_color = 24'hA04000;
        else if (sonar_stale)
            border_color = 24'hC08000;
        else if (fault_flags != 4'b0000)
            border_color = 24'hB0B000;
        else
            border_color = 24'h00A060;

        clock_color = clk_locked ? 24'h00FF66 : 24'h600000;

        if (!sonar_valid)
            val_color = 24'hA00000;
        else if (sonar_stale)
            val_color = 24'hFF9000;
        else
            val_color = 24'h00A0FF;

        if (!sonar_valid)
            dist_color = 24'h505050;
        else if (dist_clamped_in <= 24)
            dist_color = 24'hFF3030;
        else if (dist_clamped_in <= 72)
            dist_color = 24'hFFB020;
        else
            dist_color = 24'h40E060;

        if (!sonar_valid)
            age_color = 24'hA00000;
        else if (age_clamped_ms >= 150)
            age_color = 24'hFF3030;
        else if (age_clamped_ms >= 75)
            age_color = 24'hFFC020;
        else
            age_color = 24'h00C070;

        if (!sonar_valid)
            win_color = 24'h808080;
        else if (dist_clamped_in <= 24)
            win_color = 24'hFF4040;
        else if (dist_clamped_in <= 72)
            win_color = 24'hFFC040;
        else
            win_color = 24'h40D0FF;

        hdr_text_color = clk_locked ? 24'hD8FFF0 : 24'hFFC0C0;
        clk_text_color = clk_locked ? 24'hC8FFD8 : 24'hFF9090;

        if (!sonar_valid)
            val_text_color = 24'hFF9090;
        else if (sonar_stale)
            val_text_color = 24'hFFD090;
        else
            val_text_color = 24'hA0D8FF;

        if (!sonar_valid)
            rng_text_color = 24'hB0B0B0;
        else if (dist_clamped_in <= 24)
            rng_text_color = 24'hFFC0C0;
        else if (dist_clamped_in <= 72)
            rng_text_color = 24'hFFE0A0;
        else
            rng_text_color = 24'hC8FFD0;

        if (!sonar_valid)
            age_text_color = 24'hFFB0B0;
        else if (age_clamped_ms >= 150)
            age_text_color = 24'hFFC0C0;
        else if (age_clamped_ms >= 75)
            age_text_color = 24'hFFE0A0;
        else
            age_text_color = 24'hC8FFD8;

        flt_text_color = (fault_flags != 4'b0000) ? 24'hFFE090 : 24'hA0FFC0;
    end

    //--------------------------------------------------------------------------
    // Text renderers
    //--------------------------------------------------------------------------
    wire txt_hdr_on;
    wire txt_clk_on;
    wire txt_val_on;
    wire txt_rng_on;
    wire txt_age_on;
    wire txt_flt_on;

    vga_textline_3x5 #(
        .REGISTER_GLYPH  (TEXT_REGISTER_GLYPH),
        .REGISTER_SELECT (TEXT_REGISTER_SELECT)
    ) u_txt_hdr (
        .clk_pix      (clk_pix),
        .rst_pix      (rst_pix),
        .hcount       (hcount10),
        .vcount       (vcount10),
        .active_video (de),
        .x0           (TXT_HDR_X),
        .y0           (TXT_HDR_Y),
        .scale        (4'd1),
        .str16        (TXT_HDR),
        .pixel_on     (txt_hdr_on)
    );

    vga_textline_3x5 #(
        .REGISTER_GLYPH  (TEXT_REGISTER_GLYPH),
        .REGISTER_SELECT (TEXT_REGISTER_SELECT)
    ) u_txt_clk (
        .clk_pix      (clk_pix),
        .rst_pix      (rst_pix),
        .hcount       (hcount10),
        .vcount       (vcount10),
        .active_video (de),
        .x0           (TXT_CLK_X),
        .y0           (TXT_CLK_Y),
        .scale        (4'd1),
        .str16        (TXT_CLK),
        .pixel_on     (txt_clk_on)
    );

    vga_textline_3x5 #(
        .REGISTER_GLYPH  (TEXT_REGISTER_GLYPH),
        .REGISTER_SELECT (TEXT_REGISTER_SELECT)
    ) u_txt_val (
        .clk_pix      (clk_pix),
        .rst_pix      (rst_pix),
        .hcount       (hcount10),
        .vcount       (vcount10),
        .active_video (de),
        .x0           (TXT_VAL_X),
        .y0           (TXT_VAL_Y),
        .scale        (4'd1),
        .str16        (TXT_VAL),
        .pixel_on     (txt_val_on)
    );

    vga_textline_3x5 #(
        .REGISTER_GLYPH  (TEXT_REGISTER_GLYPH),
        .REGISTER_SELECT (TEXT_REGISTER_SELECT)
    ) u_txt_rng (
        .clk_pix      (clk_pix),
        .rst_pix      (rst_pix),
        .hcount       (hcount10),
        .vcount       (vcount10),
        .active_video (de),
        .x0           (TXT_RNG_X),
        .y0           (TXT_RNG_Y),
        .scale        (4'd1),
        .str16        (TXT_RNG_LIVE),
        .pixel_on     (txt_rng_on)
    );

    vga_textline_3x5 #(
        .REGISTER_GLYPH  (TEXT_REGISTER_GLYPH),
        .REGISTER_SELECT (TEXT_REGISTER_SELECT)
    ) u_txt_age (
        .clk_pix      (clk_pix),
        .rst_pix      (rst_pix),
        .hcount       (hcount10),
        .vcount       (vcount10),
        .active_video (de),
        .x0           (TXT_AGE_X),
        .y0           (TXT_AGE_Y),
        .scale        (4'd1),
        .str16        (TXT_AGE_LIVE),
        .pixel_on     (txt_age_on)
    );

    vga_textline_3x5 #(
        .REGISTER_GLYPH  (TEXT_REGISTER_GLYPH),
        .REGISTER_SELECT (TEXT_REGISTER_SELECT)
    ) u_txt_flt (
        .clk_pix      (clk_pix),
        .rst_pix      (rst_pix),
        .hcount       (hcount10),
        .vcount       (vcount10),
        .active_video (de),
        .x0           (TXT_FLT_X),
        .y0           (TXT_FLT_Y),
        .scale        (4'd1),
        .str16        (TXT_FLT_LIVE),
        .pixel_on     (txt_flt_on)
    );

    //--------------------------------------------------------------------------
    // Final raster compositing
    //--------------------------------------------------------------------------
    always @* begin
        rgb_out = 24'h000000;

        if (in_panel) begin
            //------------------------------------------------------------------
            // Base panel
            //------------------------------------------------------------------
            rgb_out = 24'h0F1114;

            //------------------------------------------------------------------
            // Subtle texture
            //------------------------------------------------------------------
            if (in_body) begin
                if (((pix_y - PANEL_Y0) & 12'h0003) == 12'h0000)
                    rgb_out = 24'h12161A;

                if (((pix_x ^ pix_y) & 12'h0004) != 12'h0000)
                    rgb_out = 24'h14181C;
            end

            //------------------------------------------------------------------
            // Header band
            //------------------------------------------------------------------
            if (in_header)
                rgb_out = 24'h1A2028;

            //------------------------------------------------------------------
            // Moving header accent
            //------------------------------------------------------------------
            if (in_header &&
                (pix_x >= header_glow_x0) &&
                (pix_x <  (header_glow_x0 + 20)))
                rgb_out = clk_locked ? 24'h2A6A4A : 24'h5A2424;

            //------------------------------------------------------------------
            // Border
            //------------------------------------------------------------------
            if (in_border)
                rgb_out = border_color;

            //------------------------------------------------------------------
            // Divider under header
            //------------------------------------------------------------------
            if (in_panel &&
                (pix_y >= (PANEL_Y0 + HEADER_H - 1)) &&
                (pix_y <  (PANEL_Y0 + HEADER_H + 1)))
                rgb_out = 24'h303840;

            //------------------------------------------------------------------
            // CLK tile
            //------------------------------------------------------------------
            if ((pix_x >= CLK_X0) && (pix_x < CLK_X1) &&
                (pix_y >= TILE_Y0) && (pix_y < TILE_Y1))
                rgb_out = 24'h202830;

            if ((pix_x >= CLK_X0 + 2) && (pix_x < CLK_X1 - 2) &&
                (pix_y >= TILE_Y0 + 2) && (pix_y < TILE_Y1 - 2))
                rgb_out = clock_color;

            //------------------------------------------------------------------
            // VAL tile
            //------------------------------------------------------------------
            if ((pix_x >= VAL_X0) && (pix_x < VAL_X1) &&
                (pix_y >= TILE_Y0) && (pix_y < TILE_Y1))
                rgb_out = 24'h202830;

            if ((pix_x >= VAL_X0 + 2) && (pix_x < VAL_X1 - 2) &&
                (pix_y >= TILE_Y0 + 2) && (pix_y < TILE_Y1 - 2))
                rgb_out = val_color;

            //------------------------------------------------------------------
            // Activity tile
            //------------------------------------------------------------------
            if ((pix_x >= ACT_X0) && (pix_x < ACT_X1) &&
                (pix_y >= TILE_Y0) && (pix_y < TILE_Y1))
                rgb_out = 24'h202830;

            if ((pix_x >= ACT_X0 + 2) && (pix_x < ACT_X1 - 2) &&
                (pix_y >= TILE_Y0 + 2) && (pix_y < TILE_Y1 - 2)) begin
                if (((pix_x + pix_y + frame_ctr_lsb[2:0] +
                      (sonar_update_toggle ? 3'd3 : 3'd0)) & 12'h0004) != 12'h0000)
                    rgb_out = sonar_valid ? 24'h00D0C0 : 24'h505050;
                else
                    rgb_out = sonar_valid ? 24'h084040 : 24'h202020;
            end

            //------------------------------------------------------------------
            // RNG bar
            //------------------------------------------------------------------
            if ((pix_x >= DIST_X0) && (pix_x < DIST_X1) &&
                (pix_y >= DIST_Y0) && (pix_y < DIST_Y1))
                rgb_out = 24'h20252A;

            if (((pix_x == dist_marker_24)  ||
                 (pix_x == dist_marker_72)  ||
                 (pix_x == dist_marker_144)) &&
                (pix_y >= DIST_Y0) && (pix_y < DIST_Y1))
                rgb_out = 24'h505860;

            if ((pix_x >= DIST_X0) && (pix_x < (DIST_X0 + dist_fill_px)) &&
                (pix_y >= DIST_Y0 + 1) && (pix_y < DIST_Y1 - 1))
                rgb_out = dist_color;

            if (((pix_x == DIST_X0) || (pix_x == DIST_X1 - 1) ||
                 (pix_y == DIST_Y0) || (pix_y == DIST_Y1 - 1)) &&
                (pix_x >= DIST_X0) && (pix_x < DIST_X1) &&
                (pix_y >= DIST_Y0) && (pix_y < DIST_Y1))
                rgb_out = 24'h404850;

            //------------------------------------------------------------------
            // AGE bar
            //------------------------------------------------------------------
            if ((pix_x >= AGE_X0) && (pix_x < AGE_X1) &&
                (pix_y >= AGE_Y0) && (pix_y < AGE_Y1))
                rgb_out = 24'h20252A;

            if (((pix_x == age_marker_50)  ||
                 (pix_x == age_marker_100) ||
                 (pix_x == age_marker_150)) &&
                (pix_y >= AGE_Y0) && (pix_y < AGE_Y1))
                rgb_out = 24'h505860;

            if ((pix_x >= AGE_X0) && (pix_x < (AGE_X0 + age_fill_px)) &&
                (pix_y >= AGE_Y0 + 1) && (pix_y < AGE_Y1 - 1))
                rgb_out = age_color;

            if (((pix_x == AGE_X0) || (pix_x == AGE_X1 - 1) ||
                 (pix_y == AGE_Y0) || (pix_y == AGE_Y1 - 1)) &&
                (pix_x >= AGE_X0) && (pix_x < AGE_X1) &&
                (pix_y >= AGE_Y0) && (pix_y < AGE_Y1))
                rgb_out = 24'h404850;

            //------------------------------------------------------------------
            // Fault boxes
            //------------------------------------------------------------------
            if ((pix_y >= FAULT_Y0) && (pix_y < FAULT_Y1)) begin
                if ((pix_x >= FAULT0_X0) && (pix_x < (FAULT0_X0 + FAULT_W)))
                    rgb_out = fault_flags[0] ? 24'hC00000 : 24'h103018;

                if ((pix_x >= FAULT1_X0) && (pix_x < (FAULT1_X0 + FAULT_W)))
                    rgb_out = fault_flags[1] ? 24'hC04000 : 24'h103018;

                if ((pix_x >= FAULT2_X0) && (pix_x < (FAULT2_X0 + FAULT_W)))
                    rgb_out = fault_flags[2] ? 24'hC0A000 : 24'h103018;

                if ((pix_x >= FAULT3_X0) && (pix_x < (FAULT3_X0 + FAULT_W)))
                    rgb_out = fault_flags[3] ? 24'hFF00A0 : 24'h103018;
            end

            //------------------------------------------------------------------
            // Right-side range window
            //------------------------------------------------------------------
            if ((pix_x >= WIN_X0) && (pix_x < WIN_X1) &&
                (pix_y >= WIN_Y0) && (pix_y < WIN_Y1))
                rgb_out = 24'h10161C;

            if ((pix_x >= WIN_X0) && (pix_x < WIN_X1) &&
                (pix_y >= WIN_Y0) && (pix_y < WIN_Y1)) begin
                if (((pix_x - WIN_X0) & 12'h0007) == 12'h0000)
                    rgb_out = 24'h183040;

                if (((pix_y - WIN_Y0) & 12'h0007) == 12'h0000)
                    rgb_out = 24'h142634;
            end

            if ((pix_x >= WIN_X0) && (pix_x < WIN_X1) &&
                (pix_y == (WIN_Y0 + (WIN_H >> 1))))
                rgb_out = 24'h305060;

            if ((pix_x >= (win_cursor_x - 1)) && (pix_x <= (win_cursor_x + 1)) &&
                (pix_y >= WIN_Y0 + 4) && (pix_y < WIN_Y1 - 4))
                rgb_out = win_color;

            if ((pix_x >= (win_cursor_x - 3)) && (pix_x <= (win_cursor_x + 3)) &&
                (pix_y >= (WIN_Y0 + (WIN_H >> 1) - 3)) &&
                (pix_y <= (WIN_Y0 + (WIN_H >> 1) + 3)))
                rgb_out = win_color;

            if (((pix_x == WIN_X0) || (pix_x == WIN_X1 - 1) ||
                 (pix_y == WIN_Y0) || (pix_y == WIN_Y1 - 1)) &&
                (pix_x >= WIN_X0) && (pix_x < WIN_X1) &&
                (pix_y >= WIN_Y0) && (pix_y < WIN_Y1))
                rgb_out = 24'h405060;

            //------------------------------------------------------------------
            // Text overlay: highest priority
            //------------------------------------------------------------------
            if (txt_hdr_on) rgb_out = hdr_text_color;
            if (txt_clk_on) rgb_out = clk_text_color;
            if (txt_val_on) rgb_out = val_text_color;
            if (txt_rng_on) rgb_out = rng_text_color;
            if (txt_age_on) rgb_out = age_text_color;
            if (txt_flt_on) rgb_out = flt_text_color;
        end
    end

endmodule

`default_nettype wire