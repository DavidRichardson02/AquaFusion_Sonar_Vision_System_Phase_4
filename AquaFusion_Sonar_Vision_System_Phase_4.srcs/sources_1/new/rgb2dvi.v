`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// rgb2dvi
//------------------------------------------------------------------------------
// ROLE
//   DVI-compatible TMDS transmitter front-end / integration wrapper for
//   Xilinx 7-series devices.
//
// HIGH-LEVEL PURPOSE
//   This module accepts a conventional parallel video interface in the pixel
//   clock domain:
//
//     - 24-bit RGB pixel data
//     - VDE (video data enable / active-video flag)
//     - HSYNC
//     - VSYNC
//
//   and converts it into the four differential TMDS pairs required by a
//   DVI/HDMI-style source path:
//
//     - one TMDS clock lane
//     - three TMDS data lanes
//
//   Internally, the work is divided into two conceptually separate stages:
//
//     1) TMDS encoding
//        Convert each 8-bit color/control stream into a 10-bit TMDS symbol.
//
//     2) TMDS serialization and differential output
//        Convert each 10-bit TMDS word into high-speed differential serial
//        signaling suitable for the physical output connector.
//
// WHY THIS MODULE EXISTS
//   The upstream video pipeline naturally produces one pixel per PixelClk
//   cycle. External DVI/HDMI-style physical links, however, do not transmit
//   raw RGB bytes directly in parallel. Instead, they transmit TMDS-coded
//   serial symbols over differential lanes.
//
//   Therefore a bridge is required between:
//
//       pixel-domain video representation
//           ->
//       TMDS-coded serial physical interface
//
//   This module is that bridge.
//
// ARCHITECTURAL DECOMPOSITION
//   The module can be understood as three major subfunctions:
//
//     A) Reset normalization
//        Convert configurable reset polarity into one internal reset signal.
//
//     B) Per-lane TMDS encoding
//        Encode red, green, and blue/control into 10-bit TMDS words.
//
//     C) TMDS output generation
//        - drive the TMDS clock lane
//        - serialize each 10-bit data-lane word
//
// TMDS LANE MAPPING
//   The three data lanes correspond to the three color channels:
//
//     Lane 2 -> red
//     Lane 1 -> green
//     Lane 0 -> blue
//
//   The blue lane is special during blanking intervals because it carries the
//   control information (HSYNC and VSYNC) when VDE is low.
//
// That is why:
//
//   - red encoder gets c0=0, c1=0
//   - green encoder gets c0=0, c1=0
//   - blue encoder gets c0=vid_pHSync, c1=vid_pVSync
//
// RESET POLICY
//   The module supports an interface-compatible reset-polarity selection:
//
//     if kRstActiveHigh != 0:
//         rst_i = aRst
//     else:
//         rst_i = ~aRst_n
//
//   So internally, the module always works with:
//
//       rst_i
//
//   as its active-high reset.
//
// IMPORTANT COMPATIBILITY NOTE
//   The following interface items are retained primarily for compatibility with
//   a previously frozen module header:
//
//     - kGenerateSerialClk
//     - kClkRange
//     - aRst_n
//
//   In the present implementation:
//
//     - SerialClk is externally supplied
//     - kGenerateSerialClk is not used to synthesize an internal clock
//     - kClkRange is not used for behavioral logic
//     - aRst_n is only used when kRstActiveHigh == 0, and otherwise is also
//       referenced in the unused-signal sink
//
// PIXEL-DOMAIN INPUT CONTRACT
//   vid_pData[23:16]
//     Red channel byte.
//
//   vid_pData[15:8]
//     Green channel byte.
//
//   vid_pData[7:0]
//     Blue channel byte.
//
//   vid_pVDE
//     Video data enable. When high, the color bytes are encoded as pixel data.
//     When low, TMDS control-symbol behavior applies instead.
//
//   vid_pHSync, vid_pVSync
//     Horizontal and vertical sync inputs.
//     These are only functionally routed into the blue TMDS encoder's control
//     inputs in the present implementation.
//
// CLOCKING CONTRACT
//   PixelClk
//     Pixel-rate clock. TMDS encoding occurs in this domain.
//
//   SerialClk
//     Higher-speed serializer clock used by the TMDS lane serializers.
//
// PHYSICAL OUTPUT CONTRACT
//   TMDS_Clk_p/n
//     Differential TMDS clock pair.
//
//   TMDS_Data_p/n[2:0]
//     Differential TMDS data pairs.
//
// IMPLEMENTATION TARGET
//   The module explicitly targets Xilinx 7-series primitives through the
//   downstream use of:
//
//     - OBUFDS
//     - OSERDESE2 (inside tmds_oserdes_lane)
//
// PEDAGOGICAL SUMMARY
//   The whole module can be read as:
//
//     Step 1: normalize reset polarity
//     Step 2: TMDS-encode red, green, and blue/control in the pixel domain
//     Step 3: drive the TMDS clock lane from PixelClk
//     Step 4: serialize each 10-bit TMDS word onto its differential lane
//------------------------------------------------------------------------------
module rgb2dvi #(
    //--------------------------------------------------------------------------
    // Interface-compatibility parameter.
    //
    // Present implementation note:
    //   The serial clock is externally supplied through SerialClk. This
    //   parameter is retained for header compatibility but is not used to
    //   generate a clock internally.
    //--------------------------------------------------------------------------
    parameter kGenerateSerialClk = 1'b0,

    //--------------------------------------------------------------------------
    // Interface-compatibility parameter.
    //
    // Present implementation note:
    //   Retained for compatibility; not used in functional logic here.
    //--------------------------------------------------------------------------
    parameter integer kClkRange  = 5,

    //--------------------------------------------------------------------------
    // Reset-polarity selector.
    //
    // If nonzero:
    //   aRst is treated as the active-high reset input.
    //
    // If zero:
    //   aRst_n is treated as the active-low reset input.
    //--------------------------------------------------------------------------
    parameter kRstActiveHigh     = 1'b1
)(
    //--------------------------------------------------------------------------
    // Differential TMDS outputs
    //--------------------------------------------------------------------------

    // TMDS clock differential pair.
    output wire        TMDS_Clk_p,
    output wire        TMDS_Clk_n,

    // TMDS data differential pairs.
    output wire [2:0]  TMDS_Data_p,
    output wire [2:0]  TMDS_Data_n,

    //--------------------------------------------------------------------------
    // Reset inputs
    //--------------------------------------------------------------------------

    // Active-high reset input, used when kRstActiveHigh != 0.
    input  wire        aRst,

    // Active-low reset input, used when kRstActiveHigh == 0.
    // Also retained for interface compatibility.
    input  wire        aRst_n,

    //--------------------------------------------------------------------------
    // Pixel-domain video inputs
    //--------------------------------------------------------------------------

    // 24-bit RGB888 pixel data.
    input  wire [23:0] vid_pData,

    // Video data enable (active video qualifier).
    input  wire        vid_pVDE,

    // Horizontal sync.
    input  wire        vid_pHSync,

    // Vertical sync.
    input  wire        vid_pVSync,

    //--------------------------------------------------------------------------
    // Clock inputs
    //--------------------------------------------------------------------------

    // Pixel-rate clock for TMDS encoding.
    input  wire        PixelClk,

    // Higher-speed clock for serialization.
    input  wire        SerialClk
);

    //==========================================================================
    // Internal normalized active-high reset
    //--------------------------------------------------------------------------
    // Purpose:
    //   Convert the configurable external reset interface into one internal
    //   active-high reset signal, `rst_i`, that downstream submodules can use
    //   uniformly.
    //
    // Step-by-step logic:
    //
    //   If kRstActiveHigh != 0:
    //       rst_i = aRst
    //
    //   Else:
    //       rst_i = ~aRst_n
    //
    // Engineering meaning:
    //   The rest of the implementation can assume an active-high reset without
    //   needing to reason about external reset polarity.
    //==========================================================================
    wire rst_i;
    assign rst_i = (kRstActiveHigh != 0) ? aRst : ~aRst_n;

    //==========================================================================
    // Per-lane 10-bit TMDS words
    //--------------------------------------------------------------------------
    // tmds_red
    //   TMDS-encoded symbol stream for the red channel.
    //
    // tmds_green
    //   TMDS-encoded symbol stream for the green channel.
    //
    // tmds_blue
    //   TMDS-encoded symbol stream for the blue channel and control symbols.
    //==========================================================================
    wire [9:0] tmds_red;
    wire [9:0] tmds_green;
    wire [9:0] tmds_blue;

    //==========================================================================
    // Red-channel TMDS encoder
    //--------------------------------------------------------------------------
    // Step-by-step meaning:
    //   1) Consume the red 8-bit pixel component in the PixelClk domain.
    //   2) Use vid_pVDE to decide whether the encoder is in active-video mode.
    //   3) Since red carries no sync/control information in blanking for this
    //      mapping, c0 and c1 are tied low.
    //   4) Produce a 10-bit TMDS word on tmds_red.
    //
    // Architectural meaning:
    //   This block is responsible only for symbol encoding, not serialization.
    //==========================================================================
    tmds_encoder u_tmds_enc_red (
        .clk        (PixelClk),
        .rst        (rst_i),
        .din        (vid_pData[23:16]),
        .c0         (1'b0),
        .c1         (1'b0),
        .de         (vid_pVDE),
        .dout       (tmds_red)
    );

    //==========================================================================
    // Green-channel TMDS encoder
    //--------------------------------------------------------------------------
    // Same interpretation as red channel:
    //   - encode the green component in active-video mode
    //   - use no control symbols on this lane in blanking
    //==========================================================================
    tmds_encoder u_tmds_enc_green (
        .clk        (PixelClk),
        .rst        (rst_i),
        .din        (vid_pData[15:8]),
        .c0         (1'b0),
        .c1         (1'b0),
        .de         (vid_pVDE),
        .dout       (tmds_green)
    );

    //==========================================================================
    // Blue/control TMDS encoder
    //--------------------------------------------------------------------------
    // This lane is special.
    //
    // Step-by-step meaning:
    //   1) During active video (vid_pVDE = 1), encode the blue pixel component.
    //   2) During blanking (vid_pVDE = 0), the TMDS encoder uses c0/c1 to emit
    //      control symbols rather than pixel data.
    //   3) Here:
    //        c0 = vid_pHSync
    //        c1 = vid_pVSync
    //
    // Architectural meaning:
    //   The blue lane carries the video control information during non-active
    //   periods, which is the standard mapping used here.
    //==========================================================================
    tmds_encoder u_tmds_enc_blue (
        .clk        (PixelClk),
        .rst        (rst_i),
        .din        (vid_pData[7:0]),
        .c0         (vid_pHSync),
        .c1         (vid_pVSync),
        .de         (vid_pVDE),
        .dout       (tmds_blue)
    );

    //==========================================================================
    // TMDS clock lane output buffer
    //--------------------------------------------------------------------------
    // Purpose:
    //   Drive the differential TMDS clock pair.
    //
    // Step-by-step meaning:
    //   1) Use PixelClk as the source for the TMDS clock lane.
    //   2) Convert that single-ended clock into a differential pair through
    //      OBUFDS configured for TMDS_33 signaling.
    //
    // Architectural meaning:
    //   The TMDS clock lane is not encoded like the data lanes. It is a direct
    //   forwarded clock reference for the receiver.
    //==========================================================================
    OBUFDS #(
        .IOSTANDARD("TMDS_33"),
        .SLEW("FAST")
    ) u_obufds_tmds_clk (
        .I  (PixelClk),
        .O  (TMDS_Clk_p),
        .OB (TMDS_Clk_n)
    );

    //==========================================================================
    // Blue TMDS data lane serializer/output
    //--------------------------------------------------------------------------
    // Step-by-step meaning:
    //   1) Accept the 10-bit TMDS word produced by the blue encoder.
    //   2) Use PixelClk for word loading / pixel-rate timing.
    //   3) Use SerialClk for high-speed bit serialization.
    //   4) Apply rst_i as the active-high reset.
    //   5) Emit the serialized differential TMDS lane on TMDS_Data_p/n[0].
    //==========================================================================
    tmds_oserdes_lane u_tmds_lane_blue (
        .PixelClk    (PixelClk),
        .SerialClk   (SerialClk),
        .rst         (rst_i),
        .tmds_word   (tmds_blue),
        .tmds_p      (TMDS_Data_p[0]),
        .tmds_n      (TMDS_Data_n[0])
    );

    //==========================================================================
    // Green TMDS data lane serializer/output
    //--------------------------------------------------------------------------
    // Same interpretation as blue lane serializer, but carrying the green
    // channel TMDS word.
    //==========================================================================
    tmds_oserdes_lane u_tmds_lane_green (
        .PixelClk    (PixelClk),
        .SerialClk   (SerialClk),
        .rst         (rst_i),
        .tmds_word   (tmds_green),
        .tmds_p      (TMDS_Data_p[1]),
        .tmds_n      (TMDS_Data_n[1])
    );

    //==========================================================================
    // Red TMDS data lane serializer/output
    //--------------------------------------------------------------------------
    // Same interpretation as the other lane serializers, but carrying the red
    // channel TMDS word.
    //==========================================================================
    tmds_oserdes_lane u_tmds_lane_red (
        .PixelClk    (PixelClk),
        .SerialClk   (SerialClk),
        .rst         (rst_i),
        .tmds_word   (tmds_red),
        .tmds_p      (TMDS_Data_p[2]),
        .tmds_n      (TMDS_Data_n[2])
    );

    //==========================================================================
    // Unused-signal sink
    //--------------------------------------------------------------------------
    // Purpose:
    //   Prevent synthesis/lint warnings for interface-compatibility artifacts
    //   that are retained by the frozen module header but not functionally used
    //   by the present implementation.
    //
    // Present items covered:
    //   - kGenerateSerialClk
    //   - kClkRange
    //   - aRst_n (when active-high reset mode is selected)
    //==========================================================================
    wire _unused;
    assign _unused = kGenerateSerialClk ^ kClkRange[0] ^ aRst_n;

endmodule

`default_nettype wire