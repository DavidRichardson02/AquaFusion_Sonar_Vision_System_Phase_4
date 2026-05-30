`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// hud_debug_panel
//------------------------------------------------------------------------------
// ROLE
//   Small combinational HUD debug panel renderer.
//
// HIGH-LEVEL PURPOSE
//   This module renders a compact rectangular status panel directly in raster
//   space. Its output is a 24-bit RGB color for the *current* pixel
//   coordinate `(pix_x, pix_y)`.
//
//   The panel contains:
//
//     1) a dark background rectangle
//     2) a small horizontal status bar representing clock-lock state
//     3) a small horizontal status bar representing sonar-stale state
//
// RENDERING PHILOSOPHY
//   This module is purely combinational with respect to the current pixel
//   coordinate and status inputs. It does not:
//
//     - store any internal state
//     - contain counters
//     - depend on a local clock
//     - accumulate history
//
//   Instead, for each pixel being scanned by the video timing system, it
//   computes the appropriate RGB value immediately from geometry and status.
//
// WHY THIS STYLE MATTERS
//   In deterministic video systems, one of the cleanest renderer styles is:
//
//       current pixel coordinate + stable status inputs
//           ->
//       current pixel color
//
//   This avoids write-side framebuffers for simple overlays and makes behavior
//   easy to reason about. The renderer becomes a geometric function over the
//   raster.
//
// SIGNAL SEMANTICS
//   pix_x, pix_y
//     Current raster coordinate from the video timing system.
//
//   de
//     Data-enable / active-video qualifier.
//     Rendering is intended only when `de` is high, meaning the current raster
//     coordinate is inside the visible image region.
//
//   clk_locked
//     External status signal representing whether the relevant clocking path is
//     considered "locked" or ready.
//
//   sonar_stale
//     External status signal representing whether sonar data is stale.
//
//   rgb_out
//     24-bit RGB color generated for the current pixel.
//
// COLOR CONVENTION IN THIS MODULE
//   24'h000000 : black / transparent-from-this-module point of view
//   24'h101010 : dark gray panel background
//   24'h00FF00 : bright green, used for "clock locked"
//   24'h400000 : dark red, used for "clock not locked"
//   24'hFF8000 : orange, used for "sonar stale"
//   24'h0080FF : blue, used for "sonar fresh/not stale"
//
// IMPORTANT COMPOSITING NOTE
//   This module itself does not know how later compositing is performed.
//   However, in many simple overlay systems, black may effectively behave as
//   "draw nothing" if later compositors treat zero as transparent.
//   In this module, black is simply the default background color when the pixel
//   is outside the panel and status bars.
//
// PANEL GEOMETRY
//   Main panel rectangle:
//       x in [8, 151]
//       y in [8, 71]
//
//   Clock-lock indicator bar:
//       x in [16, 63]
//       y in [20, 27]
//
//   Sonar-stale indicator bar:
//       x in [16, 63]
//       y in [36, 43]
//
//   These intervals are half-open in Verilog form:
//
//       pix_x >= left  && pix_x < right
//       pix_y >= top   && pix_y < bottom
//
//   so the right and bottom limits are excluded.
//
// DRAW ORDER / PRIORITY
//   The always block assigns rgb_out multiple times in sequence.
//
//   The effective priority is:
//
//     1) default black
//     2) panel background if in_panel
//     3) clock-lock bar if inside clock-lock bar rectangle
//     4) sonar-stale bar if inside sonar-stale bar rectangle
//
//   Since later assignments overwrite earlier ones, the status bars visually
//   sit "on top of" the panel background.
//
// PEDAGOGICAL SUMMARY
//   The module can be understood as answering the following sequence:
//
//     Step 1: Is the current pixel outside everything?
//             -> draw black
//
//     Step 2: Is the current pixel inside the panel background?
//             -> draw dark gray
//
//     Step 3: Is the current pixel inside the clock status bar?
//             -> override with green or dark red
//
//     Step 4: Is the current pixel inside the sonar status bar?
//             -> override with orange or blue
//
//   Because the two status bars occupy different geometric regions, the final
//   visible result is a dark panel with two colored diagnostic bars.
//------------------------------------------------------------------------------
module hud_debug_panel (
    //--------------------------------------------------------------------------
    // Current horizontal raster coordinate.
    //--------------------------------------------------------------------------
    input  wire [11:0] pix_x,

    //--------------------------------------------------------------------------
    // Current vertical raster coordinate.
    //--------------------------------------------------------------------------
    input  wire [11:0] pix_y,

    //--------------------------------------------------------------------------
    // Active-video qualifier.
    //
    // Rendering is intended only when this is high.
    //--------------------------------------------------------------------------
    input  wire        de,

    //--------------------------------------------------------------------------
    // Clock-lock status input.
    //
    // Used to choose the color of the clock-status bar.
    //--------------------------------------------------------------------------
    input  wire        clk_locked,

    //--------------------------------------------------------------------------
    // Sonar freshness status input.
    //
    // Used to choose the color of the sonar-status bar.
    //--------------------------------------------------------------------------
    input  wire        sonar_stale,

    //--------------------------------------------------------------------------
    // RGB output color for the current pixel.
    //--------------------------------------------------------------------------
    output reg  [23:0] rgb_out
);

    //==========================================================================
    // Main panel inclusion test
    //--------------------------------------------------------------------------
    // Meaning:
    //   in_panel is high when the current pixel lies inside the rectangular
    //   debug panel region and also lies inside active video (`de = 1`).
    //
    // Geometric interpretation:
    //
    //   left   = 8
    //   right  = 152   (exclusive)
    //   top    = 8
    //   bottom = 72    (exclusive)
    //
    // So the panel occupies:
    //   width  = 152 - 8  = 144 pixels
    //   height = 72  - 8  = 64 pixels
    //
    // Requiring `de` ensures that the panel is not considered active outside
    // the visible raster region.
    //==========================================================================
    wire in_panel;
    assign in_panel = de &&
                      (pix_x >= 12'd8)  && (pix_x < 12'd152) &&
                      (pix_y >= 12'd8)  && (pix_y < 12'd72);

    //==========================================================================
    // Combinational pixel-color selection
    //--------------------------------------------------------------------------
    // This always block computes rgb_out for the current pixel coordinate.
    //
    // Step-by-step evaluation order:
    //
    //   1) Default to black.
    //      This means that pixels outside the panel and outside the bars produce
    //      no visible contribution from this module.
    //
    //   2) If the pixel lies inside the main panel rectangle, assign a dark
    //      gray background color.
    //
    //   3) If the pixel lies inside the first status bar rectangle, overwrite
    //      the previous color with:
    //         - green if clk_locked is true
    //         - dark red if clk_locked is false
    //
    //   4) If the pixel lies inside the second status bar rectangle, overwrite
    //      the previous color with:
    //         - orange if sonar_stale is true
    //         - blue if sonar_stale is false
    //
    // Priority explanation:
    //   Because these are ordinary procedural assignments in sequence, later
    //   assignments take precedence over earlier ones when multiple conditions
    //   are true.
    //
    // In practice:
    //   - panel background overrides the initial black
    //   - status bars override the panel background in their own regions
    //
    // Why @(*)?
    //   The output depends combinationally on the current input values. The
    //   sensitivity list therefore includes all right-hand-side dependencies via
    //   the wildcard form.
    //==========================================================================
    always @(*) begin
        //----------------------------------------------------------------------
        // Step 1: Default output
        //
        // Begin from black so that only explicitly covered geometry contributes
        // visible color.
        //----------------------------------------------------------------------
        rgb_out = 24'h000000;

        //----------------------------------------------------------------------
        // Step 2: Main panel background
        //
        // If the current pixel lies inside the panel rectangle, draw a dark gray
        // base region.
        //----------------------------------------------------------------------
        if (in_panel)
            rgb_out = 24'h101010;

        //----------------------------------------------------------------------
        // Step 3: Clock-lock status bar
        //
        // Geometry:
        //   x in [16, 63]
        //   y in [20, 27]
        //
        // Color selection:
        //   clk_locked = 1 -> bright green
        //   clk_locked = 0 -> dark red
        //
        // Meaning:
        //   This bar serves as a compact visual summary of clock readiness.
        //----------------------------------------------------------------------
        if (de &&
            (pix_x >= 12'd16) && (pix_x < 12'd64) &&
            (pix_y >= 12'd20) && (pix_y < 12'd28))
            rgb_out = clk_locked ? 24'h00FF00 : 24'h400000;

        //----------------------------------------------------------------------
        // Step 4: Sonar-stale status bar
        //
        // Geometry:
        //   x in [16, 63]
        //   y in [36, 43]
        //
        // Color selection:
        //   sonar_stale = 1 -> orange
        //   sonar_stale = 0 -> blue
        //
        // Meaning:
        //   This bar serves as a compact visual summary of sonar freshness.
        //----------------------------------------------------------------------
        if (de &&
            (pix_x >= 12'd16) && (pix_x < 12'd64) &&
            (pix_y >= 12'd36) && (pix_y < 12'd44))
            rgb_out = sonar_stale ? 24'hFF8000 : 24'h0080FF;
    end

endmodule

`default_nettype wire