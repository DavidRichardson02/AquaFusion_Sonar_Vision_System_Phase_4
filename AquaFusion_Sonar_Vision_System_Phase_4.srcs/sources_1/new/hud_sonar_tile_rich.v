`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// hud_sonar_tile_rich.v 
//------------------------------------------------------------------------------
// PURPOSE
//   Pixel-domain “tile widget” for SONAR telemetry. The tile is rendered at a
//   fixed (TILE_X0, TILE_Y0) position and is fully deterministic with respect
//   to the pixel stream.
//
// WHAT THIS MODULE GUARANTEES (primary contracts)
//   (1) Tear-free telemetry presentation:
//       - Incoming snapshot words (snap_data_pix) may change at any time.
//       - This module captures snapshots into a *pending* register on snap_upd_pix.
//       - A pending snapshot is committed only on frame_tick (frame boundary).
//       - Therefore the visual appearance is frame-stable: no half-updated text.
//
//   (2) Pixel-exact rendering:
//       - rgb_out depends only on:
//           (pix_x, pix_y, active_video) and committed registers.
//       - No unbounded loops in sequential logic; bounded loops in combinational
//         blocks are fixed-iteration and synthesis-safe.
//
//   (3) Readability and debug intent:
//       - Text uses 3x5 glyphs via vga_char_glyph_3x5 instances.
//       - Numeric extraction is explicit and bounded (no div/mod by large values).
//
// INPUT SUMMARY (PIX domain)
//   pix_clk, rst_n:
//     Pixel clock and active-low reset for all internal state.
//
//   pix_x, pix_y, active_video:
//     Current pixel coordinate and whether pixel is within active video region.
//
//   frame_tick:
//     1-cycle pulse aligned to start-of-frame (or chosen frame boundary).
//     Used to commit pending telemetry to “displayed” registers.
//
//   snap_data_pix, snap_upd_pix:
//     Telemetry snapshot bus + update pulse (already CDC-safe/tear-free producer).
//
// OUTPUT
//   rgb_out:
//     Composite RGB pixel output. Outside active_video -> black.
//
// SNAPSHOT LAYOUT (subset used here, LSB..MSB)
//   [  7:  0] sonar_raw_u8
//   [ 15:  8] sonar_filt_u8
//   [ 31: 16] sonar_age_ms
//   [ 63: 32] sonar_update_count
//   [ 65: 64] sonar_src        (recommend: 0=UART, 1=PWM)
//   [ 66]     sonar_fresh
//   [ 67]     sonar_err_seen
//
// IMPLEMENTATION STRUCTURE
//   0) Reset polarity adaptation
//   1) Tear-free snapshot latch (pending -> committed on frame_tick)
//   2) Geometry helpers (pure combinational predicates)
//   3) Numeric helpers (digit extraction and formatting)
//   4) Update count mod-1000 (bounded subtract chain + digit extraction)
//   5) Bargraph math (clamp + scale to BAR_W)
//   6) Layout constants (glyph advance, line Y positions)
//   7) Derived digits / dynamic characters
//   8) Region detectors (tile, border, bar fill)
//   9) Character renderers (instances of vga_char_glyph_3x5)
//  10) Compositor (priority overlay selection)
//
//------------------------------------------------------------------------------
// DESIGN NOTES (why this approach is stable)
//   A pixel pipeline typically evaluates combinational logic per pixel at a high
//   rate. Any telemetry that changes “mid-frame” would otherwise cause tearing:
//   some pixels would show old values, later pixels show new values.
//   The pending/commit scheme enforces a temporal sampling theorem:
//
//     - sample telemetry at event time (snap_upd_pix) into snap_pending
//     - commit telemetry at frame boundary (frame_tick) into displayed regs
//     - render entire frame from displayed regs only
//
//   This is the same principle as double buffering: one buffer is being written,
//   another is being read; swap only at a safe boundary.
//==============================================================================

module hud_sonar_tile_rich #(
    // ------------------------------ Tile placement ----------------------------
    parameter integer TILE_X0 = 480,
    parameter integer TILE_Y0 = 16,
    parameter integer TILE_W  = 144,
    parameter integer TILE_H  = 112,

    // ------------------------------ Visual style ------------------------------
    parameter integer BORDER_T = 1,

    // ------------------------------ Text rendering ----------------------------
    // 3x5 glyphs, integer scaling (1..4)
    parameter integer GLYPH_SCALE = 2,
    parameter integer GLYPH_W     = 3,
    parameter integer GLYPH_H     = 5,
    parameter integer GLYPH_SPX   = 1,   // spacing between glyphs (scaled)

    // ------------------------------ Bargraph ----------------------------------
    parameter integer BAR_X_OFF = 8,
    parameter integer BAR_Y_OFF = 72,
    parameter integer BAR_W     = 128,
    parameter integer BAR_H     = 10,
    parameter [7:0]   MIN_INCH  = 8'd6,
    parameter [7:0]   MAX_INCH  = 8'd255
)(
    input  wire         pix_clk,
    input  wire         rst_n,

    input  wire [9:0]   pix_x,
    input  wire [9:0]   pix_y,
    input  wire         active_video,
    input  wire         frame_tick,

    input  wire [127:0] snap_data_pix,
    input  wire         snap_upd_pix,

    input  wire [11:0]  rgb_bg,
    output reg  [11:0]  rgb_out
);

    //==========================================================================
    // 0) Local reset adaptation
    //==========================================================================
    // CONTRACT
    //   - External reset is active-low (rst_n).
    //   - Internal submodules use active-high reset (rst_pix).
    //
    // WHY
    //   - Keeps the top-level interface consistent with the project’s
    //     “rst_n” convention while still supporting common active-high reset
    //     ports in pixel helpers.
    //
    // INVARIANT
    //   - rst_pix is asserted whenever rst_n is deasserted.
    wire rst_pix = ~rst_n;

    //==========================================================================
    // 1) Tear-free snapshot latch (pending -> committed on frame boundary)
    //==========================================================================
    // INTENT
    //   Provide a stable set of telemetry fields for the *entire* duration of a
    //   rendered frame. The producer may update telemetry at arbitrary times;
    //   this module ensures rendering uses a coherent snapshot.
    //
    // MECHANISM
    //   - snap_pending captures snap_data_pix at snap_upd_pix (event boundary).
    //   - snap_pending_vld indicates a new pending snapshot exists.
    //   - On frame_tick, if a pending snapshot exists, it is committed into
    //     “display” registers (sonar_*_r) and pending_vld is cleared.
    //
    // PROPERTIES
    //   - If multiple snap_upd_pix events occur between frame_tick pulses, the
    //     last one wins (latest snapshot is displayed).
    //   - If no pending snapshot exists at frame_tick, displayed registers hold.
    //
    // FAILURE MODES
    //   - If frame_tick never occurs, display will not update.
    //   - If snap_upd_pix never occurs, display remains at reset values.
    reg [127:0] snap_pending;
    reg         snap_pending_vld;

    reg [7:0]   sonar_raw_u8_r;
    reg [7:0]   sonar_filt_u8_r;
    reg [15:0]  sonar_age_ms_r;
    reg [31:0]  sonar_update_count_r;
    reg [1:0]   sonar_src_r;
    reg         sonar_fresh_r;
    reg         sonar_err_seen_r;

    always @(posedge pix_clk) begin
        if (!rst_n) begin
            // RESET BEHAVIOR
            //   - Clear pending snapshot state.
            //   - Initialize displayed telemetry to safe defaults.
            // RATIONALE
            //   - Ensures deterministic power-up visuals (no X-propagation).
            snap_pending           <= 128'd0;
            snap_pending_vld       <= 1'b0;

            sonar_raw_u8_r         <= 8'd0;
            sonar_filt_u8_r        <= 8'd0;
            sonar_age_ms_r         <= 16'd0;
            sonar_update_count_r   <= 32'd0;
            sonar_src_r            <= 2'd0;
            sonar_fresh_r          <= 1'b0;
            sonar_err_seen_r       <= 1'b0;
        end else begin
            // STEP 1: capture “pending” snapshot on update pulse
            //   - This is the earliest point at which new telemetry is known to
            //     be coherent at snap_data_pix.
            if (snap_upd_pix) begin
                snap_pending     <= snap_data_pix;
                snap_pending_vld <= 1'b1;
            end

            // STEP 2: commit pending snapshot at frame boundary
            //   - Guarantees that displayed telemetry does not change mid-frame.
            if (frame_tick) begin
                if (snap_pending_vld) begin
                    // FIELD EXTRACTION
                    //   Snap layout is fixed by producer contract; the tile
                    //   consumes a subset. (See header comment.)
                    sonar_raw_u8_r       <= snap_pending[7:0];
                    sonar_filt_u8_r      <= snap_pending[15:8];
                    sonar_age_ms_r       <= snap_pending[31:16];
                    sonar_update_count_r <= snap_pending[63:32];
                    sonar_src_r          <= snap_pending[65:64];
                    sonar_fresh_r        <= snap_pending[66];
                    sonar_err_seen_r     <= snap_pending[67];

                    // Mark pending as consumed
                    snap_pending_vld     <= 1'b0;
                end
            end
        end
    end

    //==========================================================================
    // 2) Geometry helpers
    //==========================================================================
    // These helper functions define purely geometric predicates.
    // They do not reference module state (other than inputs) and therefore are
    // deterministic, side-effect free, and safe for combinational use.
    //
    // -------------------------------------------------------------------------
    // function in_rect(x0,y0,w,h,x,y)
    //
    // Contract:
    //   Returns 1 if (x,y) lies within the axis-aligned rectangle:
    //     x in [x0, x0+w)
    //     y in [y0, y0+h)
    //
    // Notes:
    //   - Uses half-open intervals to avoid off-by-one ambiguity.
    //   - Assumes w,h > 0 in typical use.
    function in_rect;
        input integer x0, y0, w, h;
        input [9:0] x, y;
        begin
            in_rect = (x >= x0) && (x < (x0 + w)) && (y >= y0) && (y < (y0 + h));
        end
    endfunction

    // -------------------------------------------------------------------------
    // function in_border(x0,y0,w,h,t,x,y)
    //
    // Contract:
    //   Returns 1 if (x,y) lies inside the border region of a rectangle.
    //   Border thickness is t pixels and is measured inward from the rectangle.
    //
    // Edge policy:
    //   - If the rectangle is too small to contain an inner rectangle
    //     (w <= 2t or h <= 2t), the entire outer rectangle is treated as border.
    function in_border;
        input integer x0, y0, w, h, t;
        input [9:0] x, y;
        reg in_outer;
        reg in_inner;
        begin
            in_outer = in_rect(x0, y0, w, h, x, y);

            if ((w <= (2*t)) || (h <= (2*t))) begin
                // Degenerate case: “border consumes all”
                in_border = in_outer;
            end else begin
                in_inner  = in_rect(x0 + t, y0 + t, w - (2*t), h - (2*t), x, y);
                in_border = in_outer && !in_inner;
            end
        end
    endfunction

    //==========================================================================
    // 3) Numeric helpers
    //==========================================================================
    // Philosophy:
    //   - Keep digit extraction explicit and bounded.
    //   - Avoid large div/mod operators for small values when simple compare/
    //     subtract chains are sufficient and easier to reason about.
    //
    // The following helpers convert an 8-bit value (0..255) into decimal digits.
    // Because the domain is limited, hundreds digit can only be 0,1,2 and the
    // remaining digits are computed via bounded thresholds.
    function [3:0] d_hundreds_u8;
        input [7:0] v;
        begin
            d_hundreds_u8 = (v >= 8'd200) ? 4'd2 :
                            (v >= 8'd100) ? 4'd1 : 4'd0;
        end
    endfunction

    function [3:0] d_tens_u8;
        input [7:0] v;
        reg [7:0] r;
        begin
            // STEP 1: remove the hundreds contribution so r is 0..99.
            r = v;
            if (r >= 8'd200)      r = r - 8'd200;
            else if (r >= 8'd100) r = r - 8'd100;

            // STEP 2: threshold chain for tens digit.
            if      (r >= 8'd90) d_tens_u8 = 4'd9;
            else if (r >= 8'd80) d_tens_u8 = 4'd8;
            else if (r >= 8'd70) d_tens_u8 = 4'd7;
            else if (r >= 8'd60) d_tens_u8 = 4'd6;
            else if (r >= 8'd50) d_tens_u8 = 4'd5;
            else if (r >= 8'd40) d_tens_u8 = 4'd4;
            else if (r >= 8'd30) d_tens_u8 = 4'd3;
            else if (r >= 8'd20) d_tens_u8 = 4'd2;
            else if (r >= 8'd10) d_tens_u8 = 4'd1;
            else                 d_tens_u8 = 4'd0;
        end
    endfunction

    function [3:0] d_ones_u8;
        input [7:0] v;
        reg [7:0] r;
        reg [3:0] t;
        begin
            // STEP 1: remove hundreds as above.
            r = v;
            if (r >= 8'd200)      r = r - 8'd200;
            else if (r >= 8'd100) r = r - 8'd100;

            // STEP 2: compute tens digit (uses original v; safe due to bounded).
            t = d_tens_u8(v);

            // STEP 3: subtract tens*10 to leave ones.
            r = r - (t * 8'd10);
            d_ones_u8 = r[3:0];
        end
    endfunction

    // Convert 0..9 into ASCII '0'..'9'
    function [7:0] to_ascii_digit;
        input [3:0] d;
        begin
            to_ascii_digit = 8'h30 + {4'd0, d};
        end
    endfunction

    // Age display uses 10ms units to compress width.
    // Note: division by constant 10 is typically synthesized into a multiply/
    // shift network by modern tools; if ultra-tight area is needed, replacing
    // with an approximate scaling is possible.
    wire [15:0] age10 = sonar_age_ms_r / 16'd10;

    // 16-bit digit extraction (for age10).
    // Domain is assumed small enough to cap at 6k in this helper.
    function [3:0] d_thousands_u16;
        input [15:0] v;
        begin
            if      (v >= 16'd6000) d_thousands_u16 = 4'd6;
            else if (v >= 16'd5000) d_thousands_u16 = 4'd5;
            else if (v >= 16'd4000) d_thousands_u16 = 4'd4;
            else if (v >= 16'd3000) d_thousands_u16 = 4'd3;
            else if (v >= 16'd2000) d_thousands_u16 = 4'd2;
            else if (v >= 16'd1000) d_thousands_u16 = 4'd1;
            else                    d_thousands_u16 = 4'd0;
        end
    endfunction

    function [3:0] d_hundreds_u16;
        input [15:0] v;
        reg [15:0] r;
        reg [3:0] th;
        begin
            th = d_thousands_u16(v);
            r  = v - (th * 16'd1000);

            if      (r >= 16'd900) d_hundreds_u16 = 4'd9;
            else if (r >= 16'd800) d_hundreds_u16 = 4'd8;
            else if (r >= 16'd700) d_hundreds_u16 = 4'd7;
            else if (r >= 16'd600) d_hundreds_u16 = 4'd6;
            else if (r >= 16'd500) d_hundreds_u16 = 4'd5;
            else if (r >= 16'd400) d_hundreds_u16 = 4'd4;
            else if (r >= 16'd300) d_hundreds_u16 = 4'd3;
            else if (r >= 16'd200) d_hundreds_u16 = 4'd2;
            else if (r >= 16'd100) d_hundreds_u16 = 4'd1;
            else                   d_hundreds_u16 = 4'd0;
        end
    endfunction

    function [3:0] d_tens_u16;
        input [15:0] v;
        reg [15:0] r;
        reg [3:0] th, hu;
        begin
            th = d_thousands_u16(v);
            hu = d_hundreds_u16(v);
            r  = v - (th * 16'd1000) - (hu * 16'd100);

            if      (r >= 16'd90) d_tens_u16 = 4'd9;
            else if (r >= 16'd80) d_tens_u16 = 4'd8;
            else if (r >= 16'd70) d_tens_u16 = 4'd7;
            else if (r >= 16'd60) d_tens_u16 = 4'd6;
            else if (r >= 16'd50) d_tens_u16 = 4'd5;
            else if (r >= 16'd40) d_tens_u16 = 4'd4;
            else if (r >= 16'd30) d_tens_u16 = 4'd3;
            else if (r >= 16'd20) d_tens_u16 = 4'd2;
            else if (r >= 16'd10) d_tens_u16 = 4'd1;
            else                  d_tens_u16 = 4'd0;
        end
    endfunction

    function [3:0] d_ones_u16;
        input [15:0] v;
        reg [15:0] r;
        reg [3:0] th, hu, te;
        begin
            th = d_thousands_u16(v);
            hu = d_hundreds_u16(v);
            te = d_tens_u16(v);
            r  = v - (th * 16'd1000) - (hu * 16'd100) - (te * 16'd10);
            d_ones_u16 = r[3:0];
        end
    endfunction

    //==========================================================================
    // 4) Update count: mod 1000 (bounded subtract chain)
    //==========================================================================
    // INTENT
    //   Show “last 3 digits” of an update counter without requiring a true mod
    //   division by 1000 on a 32-bit number.
    //
    // OBSERVATION
    //   upd_cnt[13:0] is 0..16383, so repeated subtract-by-1000 converges in at
    //   most 16 iterations.
    //
    // SYNTHESIS NOTE
    //   The for-loop is static-bounded (16) => synthesizers unroll it into a
    //   fixed subtract/compare chain (no runtime loop).
    wire [31:0] upd_cnt  = sonar_update_count_r;
    wire [13:0] upd_lo14 = upd_cnt[13:0];

    reg [13:0] tmp14;
    reg [9:0]  upd_mod1000;
    integer i;

    always @(*) begin
        tmp14 = upd_lo14;

        // STEP-BY-STEP
        //   Repeatedly subtract 1000 until tmp14 < 1000.
        //   The loop body is guarded by a compare to prevent underflow.
        for (i = 0; i < 16; i = i + 1) begin
            if (tmp14 >= 14'd1000)
                tmp14 = tmp14 - 14'd1000;
        end

        // POSTCONDITION
        //   tmp14 is guaranteed 0..999.
        upd_mod1000 = tmp14[9:0];
    end

    // Convert 0..999 into hundreds/tens/ones.
    reg [3:0] upd_h;
    reg [3:0] upd_t;
    reg [3:0] upd_o;
    integer tmpi;

    always @(*) begin
        tmpi = upd_mod1000;

        // STEP 1: hundreds digit via threshold chain.
        upd_h = (tmpi >= 900) ? 4'd9 :
                (tmpi >= 800) ? 4'd8 :
                (tmpi >= 700) ? 4'd7 :
                (tmpi >= 600) ? 4'd6 :
                (tmpi >= 500) ? 4'd5 :
                (tmpi >= 400) ? 4'd4 :
                (tmpi >= 300) ? 4'd3 :
                (tmpi >= 200) ? 4'd2 :
                (tmpi >= 100) ? 4'd1 : 4'd0;
        tmpi = tmpi - (upd_h * 100);

        // STEP 2: tens digit on remainder.
        upd_t = (tmpi >= 90) ? 4'd9 :
                (tmpi >= 80) ? 4'd8 :
                (tmpi >= 70) ? 4'd7 :
                (tmpi >= 60) ? 4'd6 :
                (tmpi >= 50) ? 4'd5 :
                (tmpi >= 40) ? 4'd4 :
                (tmpi >= 30) ? 4'd3 :
                (tmpi >= 20) ? 4'd2 :
                (tmpi >= 10) ? 4'd1 : 4'd0;
        tmpi = tmpi - (upd_t * 10);

        // STEP 3: ones digit is remainder.
        upd_o = tmpi[3:0];
    end

    //==========================================================================
    // 5) Bargraph math (filtered inches)
    //==========================================================================
    // INTENT
    //   Render a horizontal bar whose fill amount corresponds to filtered inches
    //   mapped from [MIN_INCH..MAX_INCH] into [0..BAR_W].
    //
    // METHOD
    //   - Clamp filt into [MIN, MAX]
    //   - Compute offset from MIN and span (MAX-MIN)
    //   - Fill = (offset * BAR_W) / span
    //
    // SAFETY
    //   - If span=0, fill is forced to 0 to avoid division by zero.
    function [15:0] clamp_u16;
        input [15:0] v;
        input [15:0] lo;
        input [15:0] hi;
        begin
            if      (v < lo) clamp_u16 = lo;
            else if (v > hi) clamp_u16 = hi;
            else             clamp_u16 = v;
        end
    endfunction

    wire [15:0] filt_u16     = {8'd0, sonar_filt_u8_r};
    wire [15:0] min_u16      = {8'd0, MIN_INCH};
    wire [15:0] max_u16      = {8'd0, MAX_INCH};

    wire [15:0] filt_clamped = clamp_u16(filt_u16, min_u16, max_u16);
    wire [15:0] rng_span     = max_u16 - min_u16;
    wire [15:0] rng_off      = filt_clamped - min_u16;

    wire [15:0] bar_fill_u16 = (rng_span != 16'd0) ? ((rng_off * BAR_W) / rng_span) : 16'd0;

    // Clamp fill to BAR_W for safety against arithmetic rounding.
    wire [9:0]  bar_fill_px  = (bar_fill_u16[9:0] > BAR_W[9:0]) ? BAR_W[9:0] : bar_fill_u16[9:0];

    //==========================================================================
    // 6) Layout constants
    //==========================================================================
    // ADV is the “cell advance” in pixels between glyph origins.
    // It incorporates glyph width and spacing, both scaled.
    localparam integer ADV = (GLYPH_W * GLYPH_SCALE) + (GLYPH_SPX * GLYPH_SCALE);

    localparam integer TXT_X  = TILE_X0 + 8;
    localparam integer TXT_Y0 = TILE_Y0 + 8;

    localparam integer LINE0_Y = TXT_Y0 + 0;
    localparam integer LINE1_Y = TXT_Y0 + 16;
    localparam integer LINE2_Y = TXT_Y0 + 32;
    localparam integer LINE3_Y = TXT_Y0 + 48;

    // Scale must be at least 1. Also cap to 15 to match vga_char_glyph_3x5 port.
    wire [3:0] sc4 = (GLYPH_SCALE <= 0)  ? 4'd1  :
                     (GLYPH_SCALE >= 15) ? 4'd15 :
                     GLYPH_SCALE[3:0];

    //==========================================================================
    // 7) Derived digits / dynamic characters
    //==========================================================================
    // IN: filtered inches
    wire [3:0] in_h = d_hundreds_u8(sonar_filt_u8_r);
    wire [3:0] in_t = d_tens_u8(sonar_filt_u8_r);
    wire [3:0] in_o = d_ones_u8(sonar_filt_u8_r);

    // RW: raw inches
    wire [3:0] rw_h = d_hundreds_u8(sonar_raw_u8_r);
    wire [3:0] rw_t = d_tens_u8(sonar_raw_u8_r);
    wire [3:0] rw_o = d_ones_u8(sonar_raw_u8_r);

    // AGE: (age_ms / 10) in 4 digits (0..6999 in helper policy)
    wire [3:0] a_th = d_thousands_u16(age10);
    wire [3:0] a_hu = d_hundreds_u16(age10);
    wire [3:0] a_te = d_tens_u16(age10);
    wire [3:0] a_on = d_ones_u16(age10);

    // Source character policy:
    //   sonar_src_r==1 => PWM => 'P'
    //   else           => UART => 'U'
    wire [7:0] src_ch   = (sonar_src_r == 2'd1) ? 8'h50 : 8'h55;

    // Fresh indicator:
    //   sonar_fresh_r==1 => 'F'
    //   else             => ' ' (blank)
    wire [7:0] fresh_ch = sonar_fresh_r ? 8'h46 : 8'h20;

    //==========================================================================
    // 8) Regions
    //==========================================================================
    wire in_tile = in_rect(TILE_X0, TILE_Y0, TILE_W, TILE_H, pix_x, pix_y);
    wire in_bord = in_border(TILE_X0, TILE_Y0, TILE_W, TILE_H, BORDER_T, pix_x, pix_y);

    wire in_bar        = in_rect(TILE_X0 + BAR_X_OFF, TILE_Y0 + BAR_Y_OFF, BAR_W, BAR_H, pix_x, pix_y);
    wire in_bar_border = in_border(TILE_X0 + BAR_X_OFF, TILE_Y0 + BAR_Y_OFF, BAR_W, BAR_H, 1, pix_x, pix_y);

    wire in_bar_fill =
        in_bar &&
        (pix_x >= (TILE_X0 + BAR_X_OFF + 1)) &&
        (pix_x <  (TILE_X0 + BAR_X_OFF + 1 + bar_fill_px)) &&
        (pix_y >= (TILE_Y0 + BAR_Y_OFF + 1)) &&
        (pix_y <  (TILE_Y0 + BAR_Y_OFF + BAR_H - 1));

    //==========================================================================
    // 9) Character renderers (vga_char_glyph_3x5 instances)
    //==========================================================================
    // Methodology:
    //   Each character instance computes pixel_on for the *current pixel*.
    //   The outputs are OR-reduced to create word-level “text_on” wires.
    //
    // Timing:
    //   REGISTER_OUTPUT is set to 0 for purely combinational pixel_on. If timing
    //   closure requires it, set REGISTER_OUTPUT=1 and ensure consistent 1-cycle
    //   latency alignment across all text layers (typically by registering other
    //   overlay signals similarly).
    //
    // NOTE:
    //   vga_char_glyph_3x5 contains a minimal glyph subset. This tile uses only
    //   digits, A,C,D,E,F,G,I,L,N,O,P,R,S,T,U,W,':',' ' which are present in the
    //   provided subset.
    // -------------------------------------------------------------------------

    // ---------------- Title: "SONAR" ----------------
    wire t0_on, t1_on, t2_on, t3_on, t4_on;

    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_t0 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(TXT_X + 0*ADV), .y0(LINE0_Y),
        .char_code(8'h53), .scale(sc4),
        .pixel_on(t0_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_t1 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(TXT_X + 1*ADV), .y0(LINE0_Y),
        .char_code(8'h4F), .scale(sc4),
        .pixel_on(t1_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_t2 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(TXT_X + 2*ADV), .y0(LINE0_Y),
        .char_code(8'h4E), .scale(sc4),
        .pixel_on(t2_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_t3 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(TXT_X + 3*ADV), .y0(LINE0_Y),
        .char_code(8'h41), .scale(sc4),
        .pixel_on(t3_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_t4 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(TXT_X + 4*ADV), .y0(LINE0_Y),
        .char_code(8'h52), .scale(sc4),
        .pixel_on(t4_on)
    );
    wire title_on = t0_on | t1_on | t2_on | t3_on | t4_on;

    // ---------------- Line1: "IN:" + 3 digits ----------------
    wire inl0_on, inl1_on, inl2_on;
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_inl0 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(TXT_X + 0*ADV), .y0(LINE1_Y),
        .char_code(8'h49), .scale(sc4),
        .pixel_on(inl0_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_inl1 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(TXT_X + 1*ADV), .y0(LINE1_Y),
        .char_code(8'h4E), .scale(sc4),
        .pixel_on(inl1_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_inl2 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(TXT_X + 2*ADV), .y0(LINE1_Y),
        .char_code(8'h3A), .scale(sc4),
        .pixel_on(inl2_on)
    );
    wire inlbl_on = inl0_on | inl1_on | inl2_on;

    localparam integer IN_DIG_X = TXT_X + 4*ADV;
    wire ind0_on, ind1_on, ind2_on;

    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_ind0 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(IN_DIG_X + 0*ADV), .y0(LINE1_Y),
        .char_code(to_ascii_digit(in_h)), .scale(sc4),
        .pixel_on(ind0_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_ind1 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(IN_DIG_X + 1*ADV), .y0(LINE1_Y),
        .char_code(to_ascii_digit(in_t)), .scale(sc4),
        .pixel_on(ind1_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_ind2 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(IN_DIG_X + 2*ADV), .y0(LINE1_Y),
        .char_code(to_ascii_digit(in_o)), .scale(sc4),
        .pixel_on(ind2_on)
    );
    wire in_digits_on = ind0_on | ind1_on | ind2_on;

    // ---------------- Line1: "RW:" + 3 digits ----------------
    localparam integer RW_LBL_X = TXT_X + 8*ADV;
    wire rwl0_on, rwl1_on, rwl2_on;

    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_rwl0 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(RW_LBL_X + 0*ADV), .y0(LINE1_Y),
        .char_code(8'h52), .scale(sc4),
        .pixel_on(rwl0_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_rwl1 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(RW_LBL_X + 1*ADV), .y0(LINE1_Y),
        .char_code(8'h57), .scale(sc4),
        .pixel_on(rwl1_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_rwl2 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(RW_LBL_X + 2*ADV), .y0(LINE1_Y),
        .char_code(8'h3A), .scale(sc4),
        .pixel_on(rwl2_on)
    );
    wire rwlbl_on = rwl0_on | rwl1_on | rwl2_on;

    localparam integer RW_DIG_X = RW_LBL_X + 4*ADV;
    wire rwd0_on, rwd1_on, rwd2_on;

    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_rwd0 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(RW_DIG_X + 0*ADV), .y0(LINE1_Y),
        .char_code(to_ascii_digit(rw_h)), .scale(sc4),
        .pixel_on(rwd0_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_rwd1 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(RW_DIG_X + 1*ADV), .y0(LINE1_Y),
        .char_code(to_ascii_digit(rw_t)), .scale(sc4),
        .pixel_on(rwd1_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_rwd2 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(RW_DIG_X + 2*ADV), .y0(LINE1_Y),
        .char_code(to_ascii_digit(rw_o)), .scale(sc4),
        .pixel_on(rwd2_on)
    );
    wire rw_digits_on = rwd0_on | rwd1_on | rwd2_on;

    // ---------------- Line2: "AGE:" + 4 digits ----------------
    wire agl0_on, agl1_on, agl2_on, agl3_on;

    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_agl0 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(TXT_X + 0*ADV), .y0(LINE2_Y),
        .char_code(8'h41), .scale(sc4),
        .pixel_on(agl0_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_agl1 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(TXT_X + 1*ADV), .y0(LINE2_Y),
        .char_code(8'h47), .scale(sc4),
        .pixel_on(agl1_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_agl2 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(TXT_X + 2*ADV), .y0(LINE2_Y),
        .char_code(8'h45), .scale(sc4),
        .pixel_on(agl2_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_agl3 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(TXT_X + 3*ADV), .y0(LINE2_Y),
        .char_code(8'h3A), .scale(sc4),
        .pixel_on(agl3_on)
    );
    wire agelbl_on = agl0_on | agl1_on | agl2_on | agl3_on;

    localparam integer AGE_DIG_X = TXT_X + 5*ADV;
    wire agd0_on, agd1_on, agd2_on, agd3_on;

    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_agd0 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(AGE_DIG_X + 0*ADV), .y0(LINE2_Y),
        .char_code(to_ascii_digit(a_th)), .scale(sc4),
        .pixel_on(agd0_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_agd1 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(AGE_DIG_X + 1*ADV), .y0(LINE2_Y),
        .char_code(to_ascii_digit(a_hu)), .scale(sc4),
        .pixel_on(agd1_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_agd2 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(AGE_DIG_X + 2*ADV), .y0(LINE2_Y),
        .char_code(to_ascii_digit(a_te)), .scale(sc4),
        .pixel_on(agd2_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_agd3 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(AGE_DIG_X + 3*ADV), .y0(LINE2_Y),
        .char_code(to_ascii_digit(a_on)), .scale(sc4),
        .pixel_on(agd3_on)
    );
    wire age_digits_on = agd0_on | agd1_on | agd2_on | agd3_on;

    // ---------------- Line2: "UPD:" + 3 digits ----------------
    localparam integer UPD_LBL_X = TXT_X + 10*ADV;
    wire upl0_on, upl1_on, upl2_on, upl3_on;

    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_upl0 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(UPD_LBL_X + 0*ADV), .y0(LINE2_Y),
        .char_code(8'h55), .scale(sc4),
        .pixel_on(upl0_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_upl1 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(UPD_LBL_X + 1*ADV), .y0(LINE2_Y),
        .char_code(8'h50), .scale(sc4),
        .pixel_on(upl1_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_upl2 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(UPD_LBL_X + 2*ADV), .y0(LINE2_Y),
        .char_code(8'h44), .scale(sc4),
        .pixel_on(upl2_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_upl3 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(UPD_LBL_X + 3*ADV), .y0(LINE2_Y),
        .char_code(8'h3A), .scale(sc4),
        .pixel_on(upl3_on)
    );
    wire updlbl_on = upl0_on | upl1_on | upl2_on | upl3_on;

    localparam integer UPD_DIG_X = UPD_LBL_X + 5*ADV;
    wire upd0_on, upd1_on, upd2_on;

    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_upd0 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(UPD_DIG_X + 0*ADV), .y0(LINE2_Y),
        .char_code(to_ascii_digit(upd_h)), .scale(sc4),
        .pixel_on(upd0_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_upd1 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(UPD_DIG_X + 1*ADV), .y0(LINE2_Y),
        .char_code(to_ascii_digit(upd_t)), .scale(sc4),
        .pixel_on(upd1_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_upd2 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(UPD_DIG_X + 2*ADV), .y0(LINE2_Y),
        .char_code(to_ascii_digit(upd_o)), .scale(sc4),
        .pixel_on(upd2_on)
    );
    wire upd_digits_on = upd0_on | upd1_on | upd2_on;

    // ---------------- Line3: "SRC:" + src_ch, "F:" + fresh_ch, "ERR", "STL" ---
    wire srl0_on, srl1_on, srl2_on, srl3_on;
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_srl0 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(TXT_X + 0*ADV), .y0(LINE3_Y),
        .char_code(8'h53), .scale(sc4),
        .pixel_on(srl0_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_srl1 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(TXT_X + 1*ADV), .y0(LINE3_Y),
        .char_code(8'h52), .scale(sc4),
        .pixel_on(srl1_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_srl2 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(TXT_X + 2*ADV), .y0(LINE3_Y),
        .char_code(8'h43), .scale(sc4),
        .pixel_on(srl2_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_srl3 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(TXT_X + 3*ADV), .y0(LINE3_Y),
        .char_code(8'h3A), .scale(sc4),
        .pixel_on(srl3_on)
    );
    wire srclbl_on = srl0_on | srl1_on | srl2_on | srl3_on;

    localparam integer SRC_CH_X = TXT_X + 5*ADV;
    wire src_on;
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_srcch (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(SRC_CH_X), .y0(LINE3_Y),
        .char_code(src_ch), .scale(sc4),
        .pixel_on(src_on)
    );

    localparam integer F_LBL_X = TXT_X + 7*ADV;
    wire fl0_on, fl1_on;
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_fl0 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(F_LBL_X + 0*ADV), .y0(LINE3_Y),
        .char_code(8'h46), .scale(sc4),
        .pixel_on(fl0_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_fl1 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(F_LBL_X + 1*ADV), .y0(LINE3_Y),
        .char_code(8'h3A), .scale(sc4),
        .pixel_on(fl1_on)
    );
    wire flbl_on = fl0_on | fl1_on;

    localparam integer F_CH_X = F_LBL_X + 3*ADV;
    wire fresh_on;
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_fch (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(F_CH_X), .y0(LINE3_Y),
        .char_code(fresh_ch), .scale(sc4),
        .pixel_on(fresh_on)
    );

    localparam integer ERR_X = TXT_X + 11*ADV;
    wire err0_on, err1_on, err2_on;
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_err0 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(ERR_X + 0*ADV), .y0(LINE3_Y),
        .char_code(8'h45), .scale(sc4),
        .pixel_on(err0_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_err1 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(ERR_X + 1*ADV), .y0(LINE3_Y),
        .char_code(8'h52), .scale(sc4),
        .pixel_on(err1_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_err2 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(ERR_X + 2*ADV), .y0(LINE3_Y),
        .char_code(8'h52), .scale(sc4),
        .pixel_on(err2_on)
    );
    wire err_on = sonar_err_seen_r ? (err0_on | err1_on | err2_on) : 1'b0;

    localparam integer STL_X = TXT_X + 15*ADV;
    wire stl0_on, stl1_on, stl2_on;
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_stl0 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(STL_X + 0*ADV), .y0(LINE3_Y),
        .char_code(8'h53), .scale(sc4),
        .pixel_on(stl0_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_stl1 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(STL_X + 1*ADV), .y0(LINE3_Y),
        .char_code(8'h54), .scale(sc4),
        .pixel_on(stl1_on)
    );
    vga_char_glyph_3x5 #(.REGISTER_OUTPUT(0)) u_stl2 (
        .clk_pix(pix_clk), .rst_pix(rst_pix),
        .hcount(pix_x), .vcount(pix_y), .active_video(active_video),
        .x0(STL_X + 2*ADV), .y0(LINE3_Y),
        .char_code(8'h4C), .scale(sc4),
        .pixel_on(stl2_on)
    );
    wire stl_on = (!sonar_fresh_r) ? (stl0_on | stl1_on | stl2_on) : 1'b0;

    //==========================================================================
    // 10) Color selection + compositing
    //==========================================================================
    // COMPOSITING POLICY (priority order)
    //   1) Outside active_video -> black
    //   2) Outside tile         -> rgb_bg passthrough
    //   3) Inside tile:
    //        - base fill = dimmed background
    //        - border strokes
    //        - bargraph background/border/fill
    //        - text overlays
    //        - status overlays (STL, ERR) last, so they win if overlapping
    //
    // Note:
    //   Overlaps are intentionally resolved by sequential “if” statements. The
    //   later statements overwrite earlier assignments, making priority explicit.
    localparam [11:0] C_BORDER   = 12'hFFF;
    localparam [11:0] C_TEXT     = 12'hEEE;
    localparam [11:0] C_DIM      = 12'h666;

    localparam [11:0] C_BAR_OK   = 12'h0F0;
    localparam [11:0] C_BAR_ST   = 12'h080;

    localparam [11:0] C_ERR      = 12'hF00;
    localparam [11:0] C_STALE    = 12'hFA0; // amber

    // Background dimming: shift right by 1 (approx /2), keep MSB cleared.
    wire [11:0] bg_dim = {1'b0, rgb_bg[11:1]};

    always @(*) begin
        // DEFAULT: passthrough
        rgb_out = rgb_bg;

        // STEP 1: blanking outside active video
        if (!active_video) begin
            rgb_out = 12'h000;

        // STEP 2: tile area compositor
        end else if (in_tile) begin
            // Base tile fill
            rgb_out = bg_dim;

            // Border
            if (in_bord) rgb_out = C_BORDER;

            // Bargraph region
            if (in_bar_border)      rgb_out = C_BORDER;
            else if (in_bar_fill)   rgb_out = sonar_fresh_r ? C_BAR_OK : C_BAR_ST;
            else if (in_bar)        rgb_out = C_DIM;

            // Text overlays (labels and digits)
            if (title_on)       rgb_out = C_TEXT;

            if (inlbl_on)       rgb_out = C_TEXT;
            if (in_digits_on)   rgb_out = sonar_fresh_r ? C_TEXT : bg_dim;

            if (rwlbl_on)       rgb_out = C_TEXT;
            if (rw_digits_on)   rgb_out = sonar_fresh_r ? (C_TEXT >> 1) : bg_dim;

            if (agelbl_on)      rgb_out = C_TEXT;
            if (age_digits_on)  rgb_out = C_TEXT;

            if (updlbl_on)      rgb_out = C_TEXT;
            if (upd_digits_on)  rgb_out = C_TEXT;

            if (srclbl_on)      rgb_out = C_TEXT;
            if (src_on)         rgb_out = C_TEXT;

            if (flbl_on)        rgb_out = C_TEXT;
            if (fresh_on)       rgb_out = sonar_fresh_r ? C_TEXT : bg_dim;

            // Status overlays last (highest priority)
            if (stl_on)         rgb_out = C_STALE;
            if (err_on)         rgb_out = C_ERR;
        end
    end

endmodule

`default_nettype wire