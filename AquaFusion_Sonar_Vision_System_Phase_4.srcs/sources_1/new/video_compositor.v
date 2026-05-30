`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// video_compositor
//------------------------------------------------------------------------------
// ROLE
//   Fixed-priority combinational RGB compositor for one base layer plus four
//   overlay layers.
//
// HIGH-LEVEL PURPOSE
//   This module selects one final 24-bit RGB pixel from:
//
//       rgb_base
//       rgb_layer0
//       rgb_layer1
//       rgb_layer2
//       rgb_layer3
//
//   using a simple keyed-transparency overwrite rule.
//
// PRIORITY ORDER
//   Highest priority:
//       rgb_layer3
//       rgb_layer2
//       rgb_layer1
//       rgb_layer0
//       rgb_base
//   Lowest priority.
//
// TRANSPARENCY CONVENTION
//   For overlay layers only, the exact RGB value:
//
//       24'h000000
//
//   is interpreted as "transparent / no contribution".
//
//   Therefore:
//
//     - A non-black overlay pixel is visible.
//     - A black overlay pixel is treated as absent.
//     - The base layer is always taken as the starting pixel and is never
//       interpreted through the transparency key by this module.
//
// IMPORTANT LIMITATION
//   Because black is used as the transparency key, this compositor cannot
//   represent intentionally visible black pixels in overlay layers.
//
// DESIGN STYLE
//   - Fully combinational
//   - No clock
//   - No retained state
//   - Deterministic fixed-priority overwrite chain
//
// IMPLEMENTATION MODEL
//   Conceptually:
//
//       out := base
//       if layer0 is visible, out := layer0
//       if layer1 is visible, out := layer1
//       if layer2 is visible, out := layer2
//       if layer3 is visible, out := layer3
//
//   Since later assignments overwrite earlier ones, the last visible layer in
//   the chain is the final output pixel.
//
//------------------------------------------------------------------------------
module video_compositor (
    //--------------------------------------------------------------------------
    // Base/background pixel (always used as the starting value)
    //--------------------------------------------------------------------------
    input  wire [23:0] rgb_base,

    //--------------------------------------------------------------------------
    // Overlay layers in increasing priority order
    //--------------------------------------------------------------------------
    input  wire [23:0] rgb_layer0,
    input  wire [23:0] rgb_layer1,
    input  wire [23:0] rgb_layer2,
    input  wire [23:0] rgb_layer3,

    //--------------------------------------------------------------------------
    // Final composited output pixel
    //--------------------------------------------------------------------------
    output reg  [23:0] rgb_out
);

    //--------------------------------------------------------------------------
    // Transparency-key constant
    //--------------------------------------------------------------------------
    // The exact black RGB value is treated as "transparent" for overlay layers.
    // Keeping this as a named localparam improves readability and prevents
    // repeated literal use throughout the logic.
    //--------------------------------------------------------------------------
    localparam [23:0] TRANSPARENT_RGB = 24'h000000;

    //--------------------------------------------------------------------------
    // Per-layer visibility qualifiers
    //--------------------------------------------------------------------------
    // A layer is considered visible if its pixel is not equal to the
    // transparency key.
    //--------------------------------------------------------------------------
    wire layer0_visible;
    wire layer1_visible;
    wire layer2_visible;
    wire layer3_visible;

    assign layer0_visible = (rgb_layer0 != TRANSPARENT_RGB);
    assign layer1_visible = (rgb_layer1 != TRANSPARENT_RGB);
    assign layer2_visible = (rgb_layer2 != TRANSPARENT_RGB);
    assign layer3_visible = (rgb_layer3 != TRANSPARENT_RGB);

    //--------------------------------------------------------------------------
    // Fixed-priority combinational overwrite chain
    //--------------------------------------------------------------------------
    // Step-by-step:
    //
    //   1) Start from the base pixel.
    //   2) Overwrite with layer0 if visible.
    //   3) Overwrite with layer1 if visible.
    //   4) Overwrite with layer2 if visible.
    //   5) Overwrite with layer3 if visible.
    //
    // Effective priority:
    //
    //   layer3 > layer2 > layer1 > layer0 > base
    //--------------------------------------------------------------------------
    always @(*) begin
        //----------------------------------------------------------------------
        // Default to the background/base layer.
        //----------------------------------------------------------------------
        rgb_out = rgb_base;

        //----------------------------------------------------------------------
        // Apply overlays in ascending priority order.
        //----------------------------------------------------------------------
        if (layer0_visible)
            rgb_out = rgb_layer0;

        if (layer1_visible)
            rgb_out = rgb_layer1;

        if (layer2_visible)
            rgb_out = rgb_layer2;

        if (layer3_visible)
            rgb_out = rgb_layer3;
    end

endmodule

`default_nettype wire