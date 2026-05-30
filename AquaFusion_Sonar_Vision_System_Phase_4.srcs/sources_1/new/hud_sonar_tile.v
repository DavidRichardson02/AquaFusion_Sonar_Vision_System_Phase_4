`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// hud_sonar_tile
//------------------------------------------------------------------------------
// ROLE
//   Video-domain sonar status tile renderer.
//
// HIGH-LEVEL PURPOSE
//   This module renders a rectangular HUD tile summarizing one sonar channel.
//   The tile is drawn directly in raster space from the current pixel
//   coordinates and from a *committed* sonar snapshot.
//
//   The rendered tile contains:
//
//     1) a rectangular tile background
//     2) a white border
//     3) a horizontal range/status bar
//
//   The bar width is derived from the committed sonar distance value.
//   The bar color is derived from the committed status bits.
//
// RENDERING PHILOSOPHY
//   This module follows an important deterministic video rule:
//
//     "Only render from frame-stable committed state."
//
//   The incoming snapshot bus `sonar_snap_vid` is not used directly for live
//   rendering. Instead, the tile keeps its own video-domain register:
//
//       snap_commit_reg
//
//   and updates that register only when:
//
//       sonar_snap_commit_vid
//
//   is asserted.
//
//   This means the visible tile state changes only at explicit commit events,
//   rather than at arbitrary times during raster scanout.
//
// WHY THIS MODULE EXISTS
//   In a video HUD system, it is not enough merely to possess telemetry.
//   Telemetry must be converted into a visual representation that is:
//
//     - spatially deterministic
//     - temporally coherent
//     - easy to interpret
//
//   This tile converts a compact sonar snapshot into a simple geometric widget:
//   a bordered panel with a colored bar whose width encodes distance and whose
//   color encodes health/freshness.
//
// SIGNAL SEMANTICS
//   clk_vid
//     Video-domain clock.
//
//   rst_vid
//     Active-high synchronous reset for the video domain.
//
//   pix_x, pix_y
//     Current raster coordinate.
//
//   de
//     Data-enable / active-video qualifier.
//
//   sonar_snap_vid
//     Snapshot bus arriving from the CDC/commit pipeline.
//
//   sonar_snap_commit_vid
//     One-cycle pulse indicating that `sonar_snap_vid` is the newly committed
//     frame-stable snapshot and should be latched into the local tile register.
//
//   rgb_out
//     RGB contribution of this tile for the current pixel.
//
// SNAPSHOT FIELD DECODE
//   The module interprets the committed snapshot register as follows:
//
//     bits [9:0]   -> distance_in
//     bit  [10]    -> valid
//     bit  [11]    -> stale
//     bit  [12]    -> timeout_err
//
//   These fields are decoded only from the *committed* local register, not
//   directly from the live incoming bus.
//
// TILE GEOMETRY
//   The tile occupies:
//
//       x in [TILE_X0, TILE_X0 + 159]
//       y in [TILE_Y0, TILE_Y0 + 79]
//
//   since the implementation uses half-open comparisons:
//
//       pix_x >= TILE_X0           && pix_x < TILE_X0 + 160
//       pix_y >= TILE_Y0           && pix_y < TILE_Y0 + 80
//
//   Therefore:
//
//       tile width  = 160 pixels
//       tile height = 80 pixels
//
// BAR GEOMETRY
//   The horizontal range bar occupies:
//
//       x in [TILE_X0 + 12, TILE_X0 + 12 + bar_width - 1]
//       y in [TILE_Y0 + 48, TILE_Y0 + 59]
//
//   More precisely, because of the half-open upper bounds:
//
//       pix_x >= TILE_X0 + 12
//       pix_x <  TILE_X0 + 12 + bar_width
//
//       pix_y >= TILE_Y0 + 48
//       pix_y <  TILE_Y0 + 60
//
//   Thus:
//
//       bar maximum width  = 136 pixels
//       bar height         = 12 pixels
//
// DISTANCE-TO-BAR-WIDTH MAPPING
//   The bar width is computed as:
//
//       if distance_in > 255:
//           bar_width = 136
//       else:
//           bar_width = (distance_in * 136) / 255
//
//   This means:
//
//     - distance 0   -> width near 0
//     - distance 255 -> width 136
//
//   Values above 255 are saturated to full width.
//
// STATUS-TO-COLOR MAPPING
//   Inside the bar region, color priority is:
//
//     1) timeout_err -> red      (24'hFF0000)
//     2) stale       -> orange   (24'hFF8000)
//     3) valid       -> green    (24'h00FF80)
//     4) otherwise   -> gray     (24'h404040)
//
//   This priority is implemented procedurally, so the most severe condition
//   takes visual precedence.
//
// DRAW ORDER / PRIORITY
//   The always block that generates rgb_out is written in sequential override
//   style. The effective priority is:
//
//     1) default black
//     2) tile background
//     3) top/bottom border
//     4) left/right border
//     5) bar region color
//
//   Since later assignments overwrite earlier ones, the bar visually sits on
//   top of the background and can overwrite border color in overlapping pixels
//   if overlap were ever to occur geometrically.
//
// IMPORTANT COMPOSITING NOTE
//   This module outputs black (24'h000000) when it has nothing to draw at the
//   current pixel. In many simple compositors, black may effectively behave as
//   "transparent." That policy is external to this module, but this renderer is
//   compatible with that style.
//
// RESET BEHAVIOR
//   On rst_vid assertion:
//
//       snap_commit_reg <= 0
//
//   Therefore, after reset the tile renders from an all-zero committed
//   snapshot until the first explicit commit arrives.
//
// PEDAGOGICAL SUMMARY
//   The module can be understood in four conceptual stages:
//
//     Stage 1: latch the newly committed snapshot into a local register
//     Stage 2: decode fields from that local committed snapshot
//     Stage 3: compute geometric inclusion tests and bar width
//     Stage 4: choose pixel color according to geometry and status priority
//------------------------------------------------------------------------------
module hud_sonar_tile #(
    //--------------------------------------------------------------------------
    // Tile origin in raster coordinates.
    //--------------------------------------------------------------------------
    parameter integer TILE_X0 = 16,
    parameter integer TILE_Y0 = 16,

    //--------------------------------------------------------------------------
    // Width of the incoming snapshot bus.
    //--------------------------------------------------------------------------
    parameter integer SNAP_W  = 64
)(
    //--------------------------------------------------------------------------
    // Video-domain clock.
    //--------------------------------------------------------------------------
    input  wire              clk_vid,

    //--------------------------------------------------------------------------
    // Active-high synchronous video reset.
    //--------------------------------------------------------------------------
    input  wire              rst_vid,

    //--------------------------------------------------------------------------
    // Current raster coordinate.
    //--------------------------------------------------------------------------
    input  wire [11:0]       pix_x,
    input  wire [11:0]       pix_y,

    //--------------------------------------------------------------------------
    // Active-video qualifier.
    //--------------------------------------------------------------------------
    input  wire              de,

    //--------------------------------------------------------------------------
    // Incoming committed snapshot bus from the CDC/commit path.
    //
    // This bus is only latched when sonar_snap_commit_vid is asserted.
    //--------------------------------------------------------------------------
    input  wire [SNAP_W-1:0] sonar_snap_vid,

    //--------------------------------------------------------------------------
    // One-cycle pulse indicating that sonar_snap_vid should become the new
    // local committed snapshot for rendering.
    //--------------------------------------------------------------------------
    input  wire              sonar_snap_commit_vid,

    //--------------------------------------------------------------------------
    // RGB output for the current pixel.
    //--------------------------------------------------------------------------
    output reg  [23:0]       rgb_out
);

    //==========================================================================
    // Local committed snapshot register
    //--------------------------------------------------------------------------
    // Purpose:
    //   Hold the frame-stable sonar state that this tile actually renders from.
    //
    // Why local storage matters:
    //   The renderer must not depend on a live bus that could conceptually
    //   change outside the tile's own explicit commit event.
    //==========================================================================
    reg [SNAP_W-1:0] snap_commit_reg;

    //==========================================================================
    // Snapshot field decode
    //--------------------------------------------------------------------------
    // The committed snapshot register is decoded into semantically meaningful
    // local wires.
    //
    // distance_in
    //   10-bit sonar distance value
    //
    // valid
    //   Indicates that the underlying sonar sample is valid
    //
    // stale
    //   Indicates that the sonar data has aged beyond freshness threshold
    //
    // timeout_err
    //   Indicates timeout/fault condition
    //
    // Important:
    //   These are all derived from snap_commit_reg, not directly from the live
    //   input bus.
    //==========================================================================
    wire [9:0] distance_in = snap_commit_reg[9:0];
    wire       valid       = snap_commit_reg[10];
    wire       stale       = snap_commit_reg[11];
    wire       timeout_err = snap_commit_reg[12];

    //==========================================================================
    // Geometric inclusion signals
    //--------------------------------------------------------------------------
    // in_tile
    //   True when the current pixel lies within the overall tile rectangle.
    //
    // in_bar
    //   True when the current pixel lies within the horizontal bar rectangle.
    //
    // bar_width
    //   Width of the bar computed from the committed distance value.
    //==========================================================================
    wire in_tile;
    wire in_bar;
    reg [11:0] bar_width;

    //--------------------------------------------------------------------------
    // Main tile rectangle
    //
    // Geometry:
    //   x in [TILE_X0, TILE_X0 + 159]
    //   y in [TILE_Y0, TILE_Y0 + 79]
    //
    // de is included so the tile is only active during visible raster periods.
    //--------------------------------------------------------------------------
    assign in_tile = de &&
                     (pix_x >= TILE_X0) && (pix_x < TILE_X0 + 12'd160) &&
                     (pix_y >= TILE_Y0) && (pix_y < TILE_Y0 + 12'd80);

    //--------------------------------------------------------------------------
    // Bar rectangle
    //
    // Geometry:
    //   left   = TILE_X0 + 12
    //   right  = TILE_X0 + 12 + bar_width
    //   top    = TILE_Y0 + 48
    //   bottom = TILE_Y0 + 60
    //
    // So the bar has fixed height 12 and variable width controlled by
    // distance_in.
    //--------------------------------------------------------------------------
    assign in_bar  = de &&
                     (pix_x >= TILE_X0 + 12'd12) &&
                     (pix_x <  TILE_X0 + 12'd12 + bar_width) &&
                     (pix_y >= TILE_Y0 + 12'd48) &&
                     (pix_y <  TILE_Y0 + 12'd60);

    //==========================================================================
    // Snapshot commit register update
    //--------------------------------------------------------------------------
    // Step-by-step behavior:
    //
    //   Case 1: rst_vid asserted
    //     Clear the local committed snapshot to zero.
    //
    //   Case 2: sonar_snap_commit_vid asserted
    //     Latch sonar_snap_vid into snap_commit_reg.
    //
    //   Case 3: otherwise
    //     Hold previous committed state.
    //
    // Engineering meaning:
    //   This register is the visible-state contract for the tile.
    //   The tile's rendering only changes when this register changes.
    //==========================================================================
    always @(posedge clk_vid) begin
        if (rst_vid)
            snap_commit_reg <= {SNAP_W{1'b0}};
        else if (sonar_snap_commit_vid)
            snap_commit_reg <= sonar_snap_vid;
    end

    //==========================================================================
    // Distance-to-bar-width mapping
    //--------------------------------------------------------------------------
    // Step-by-step behavior:
    //
    //   1) If distance_in exceeds 255, saturate the bar to full width 136.
    //   2) Otherwise, scale distance_in linearly into the range [0, 136]
    //      using integer arithmetic:
    //
    //          bar_width = (distance_in * 136) / 255
    //
    // Why the saturation check exists:
    //   Because the visible bar geometry is only defined up to its maximum
    //   width. Distances larger than 255 are clamped rather than allowed to
    //   produce wider geometry.
    //
    // Why integer arithmetic is acceptable here:
    //   Bar width is a pixel quantity, so quantization to integer pixel counts
    //   is natural.
    //==========================================================================
    always @(*) begin
        if (distance_in > 10'd255)
            bar_width = 12'd136;
        else
            bar_width = (distance_in * 12'd136) / 10'd255;
    end

    //==========================================================================
    // Combinational pixel renderer
    //--------------------------------------------------------------------------
    // Rendering priority, in order of assignment:
    //
    //   1) default black
    //   2) tile background
    //   3) top/bottom border
    //   4) left/right border
    //   5) bar fill with status-dependent color
    //
    // Later assignments override earlier ones.
    //
    // This is a pure combinational raster renderer:
    //   current pixel + committed state -> output color
    //==========================================================================
    always @(*) begin
        //----------------------------------------------------------------------
        // Step 1: default output
        //
        // Begin from black so that only explicitly covered geometry contributes
        // visible color.
        //----------------------------------------------------------------------
        rgb_out = 24'h000000;

        //----------------------------------------------------------------------
        // Step 2: tile background
        //
        // If the current pixel lies anywhere inside the tile rectangle, assign
        // a dark bluish-gray background.
        //----------------------------------------------------------------------
        if (in_tile)
            rgb_out = 24'h101820;

        //----------------------------------------------------------------------
        // Step 3: top and bottom border lines
        //
        // Geometry:
        //   full tile width
        //   y == TILE_Y0           (top edge)
        //   y == TILE_Y0 + 79      (bottom edge)
        //
        // These lines overwrite the background where they apply.
        //----------------------------------------------------------------------
        if (de &&
            (pix_x >= TILE_X0) && (pix_x < TILE_X0 + 12'd160) &&
            ((pix_y == TILE_Y0) || (pix_y == TILE_Y0 + 12'd79)))
            rgb_out = 24'hFFFFFF;

        //----------------------------------------------------------------------
        // Step 4: left and right border lines
        //
        // Geometry:
        //   full tile height
        //   x == TILE_X0           (left edge)
        //   x == TILE_X0 + 159     (right edge)
        //
        // These lines overwrite the background where they apply.
        //----------------------------------------------------------------------
        if (de &&
            (pix_y >= TILE_Y0) && (pix_y < TILE_Y0 + 12'd80) &&
            ((pix_x == TILE_X0) || (pix_x == TILE_X0 + 12'd159)))
            rgb_out = 24'hFFFFFF;

        //----------------------------------------------------------------------
        // Step 5: range/status bar
        //
        // If the pixel lies inside the bar region, choose bar color according
        // to the committed status bits.
        //
        // Priority within the bar:
        //   timeout_err -> red
        //   stale       -> orange
        //   valid       -> green
        //   else        -> dark gray
        //
        // Interpretation:
        //   - red     : strongest fault indication
        //   - orange  : stale data
        //   - green   : valid/fresh-ish usable data
        //   - gray    : no valid status
        //----------------------------------------------------------------------
        if (in_bar) begin
            if (timeout_err)
                rgb_out = 24'hFF0000;
            else if (stale)
                rgb_out = 24'hFF8000;
            else if (valid)
                rgb_out = 24'h00FF80;
            else
                rgb_out = 24'h404040;
        end
    end

endmodule

`default_nettype wire