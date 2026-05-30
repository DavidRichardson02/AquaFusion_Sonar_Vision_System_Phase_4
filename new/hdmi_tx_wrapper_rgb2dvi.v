`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// hdmi_tx_wrapper_rgb2dvi
//------------------------------------------------------------------------------
// ROLE
//   Nexys Video HDMI source wrapper.
//
// HIGH-LEVEL PURPOSE
//   This module forms the bridge between:
//
//     - the internal project video pipeline, which already provides:
//         * RGB pixel data
//         * HSYNC
//         * VSYNC
//         * DE (data enable)
//
//     and
//
//     - the physical HDMI source interface, which requires:
//         * appropriately generated pixel and serializer clocks
//         * domain-local synchronized resets
//         * TMDS encoding and serialization
//         * differential output pairs
//         * transmitter-enable policy tied to sink presence and internal
//           readiness
//
//   In short, this wrapper does not decide *what* to draw. It decides *how the
//   already prepared video stream is clocked, reset, and physically transmitted*
//   over the HDMI source port.
//
// ARCHITECTURAL FUNCTION
//   The wrapper performs four major functions:
//
//     1) Generate HDMI-related clocks from a system/reference clock
//     2) Convert asynchronous/raw reset conditions into per-domain synchronized
//        resets
//     3) Determine whether the transmitter should be considered enabled
//     4) Invoke the lower-level RGB-to-TMDS conversion/serialization block
//
// WHY THIS MODULE EXISTS
//   HDMI source output is not just "wire out RGB and sync signals."
//
//   A practical source path requires:
//
//     - a pixel-rate clock domain
//     - a higher-speed serializer clock domain
//     - reset sequencing consistent with those generated clocks
//     - a clean boundary around the TMDS encoder/serializer IP or wrapper
//
//   Without such a wrapper, the top-level design would become cluttered with
//   clock-generation, reset-domain, and transmitter-policy details.
//
// SYSTEM-LEVEL CONTRACT
//   Inputs:
//     clk_ref
//       Reference/system clock used to derive the HDMI-related clocks.
//
//     rst_ref
//       Reference-domain reset request.
//
//     rgb_in, hsync_in, vsync_in, de_in
//       Prepared video stream from the upstream raster/compositor pipeline.
//
//     hpd_in
//       HDMI hot-plug-detect input from the source connector, indicating that a
//       sink/display is present.
//
//   Outputs:
//     clk_vid
//       Pixel/video clock generated locally by the clock wizard.
//
//     rst_vid
//       Video-domain synchronized reset.
//
//     clk_tmds_5x
//       Higher-speed serializer clock generated locally.
//
//     rst_tmds_5x
//       Serializer-domain synchronized reset.
//
//     tx_en_o
//       Transmitter-enable indication derived from sink presence and local clock
//       readiness.
//
//     tmds_clk_p/n, tmds_data_p/n
//       Differential TMDS output pairs.
//
// CLOCKING MODEL
//   This wrapper uses two generated clocks:
//
//     clk_vid
//       Pixel-domain clock used by the video stream.
//
//     clk_tmds_5x
//       Higher-speed serializer-related clock used by the TMDS path.
//
//   The naming "5x" indicates that this clock is intended to run at a multiple
//   of the pixel clock appropriate for the serialization scheme being used.
//
// IMPORTANT DESIGN POINT
//   The module must create *separate synchronized resets* for these two clock
//   domains. Even if both clocks are derived from the same source, reset release
//   still needs to be aligned to each local domain clock.
//
// LOCK / RESET POLICY
//   The clock wizard produces:
//
//       hdmi_clk_locked
//
//   This is treated as a readiness indicator for the locally generated HDMI
//   clocking environment.
//
//   If the clock wizard is not locked, both video and serializer domains are
//   held in reset through the asynchronous reset expressions:
//
//       rst_vid_async  = rst_ref | ~hdmi_clk_locked
//       rst_tmds_async = rst_ref | ~hdmi_clk_locked
//
//   Those asynchronous reset requests are then synchronized locally by
//   reset_sync.
//
// TX ENABLE POLICY
//   The transmitter enable output is asserted only when:
//
//     - an HDMI sink is present (`hpd_in`)
//     - the generated HDMI clocks are locked
//     - the video domain is out of reset
//     - the serializer domain is out of reset
//
//   Concretely:
//
//       tx_en_o = hpd_in & hdmi_clk_locked & ~rst_vid & ~rst_tmds_5x;
//
//   This is a conservative and sensible policy for first-light HDMI bring-up.
//
// TMDS PATH
//   The wrapper delegates actual TMDS encoding and serialization to `rgb2dvi`.
//   Therefore this module is best interpreted as:
//
//       clock/reset/policy wrapper
//           around
//       rgb2dvi transport engine
//
// IMPORTANT RESET NOTE
//   The `rgb2dvi` instance is driven with:
//
//       .aRst (rst_vid_async)
//
//   That means the lower-level block sees the asynchronous reset condition based
//   on reference reset and lock state, while the wrapper also publishes
//   synchronized local resets for the rest of the design.
//
// INTENDED USE
//   The comments indicate that this wrapper is aimed at 640x480-class first
//   light via `clk_wiz_hdmi_640x480`. That means the wrapper should be viewed as
//   a bring-up-friendly HDMI source shell for a simple initial video mode.
//
// PEDAGOGICAL SUMMARY
//   The wrapper can be understood as:
//
//     Step 1: derive the local HDMI clocks from the reference clock
//     Step 2: derive asynchronous reset conditions from reset + lock status
//     Step 3: synchronize those resets into each generated clock domain
//     Step 4: decide whether the transmitter is considered enabled
//     Step 5: pass the prepared video stream into rgb2dvi for TMDS output
//------------------------------------------------------------------------------
module hdmi_tx_wrapper_rgb2dvi (
    //--------------------------------------------------------------------------
    // Reference/system-side inputs
    //--------------------------------------------------------------------------

    // Reference clock from the upstream system.
    input  wire        clk_ref,

    // Reference-domain reset request.
    input  wire        rst_ref,

    //--------------------------------------------------------------------------
    // Prepared video-stream inputs
    //--------------------------------------------------------------------------

    // 24-bit RGB888 pixel data from the upstream compositor or renderer.
    input  wire [23:0] rgb_in,

    // Horizontal synchronization input.
    input  wire        hsync_in,

    // Vertical synchronization input.
    input  wire        vsync_in,

    // Data-enable input indicating active video.
    input  wire        de_in,

    //--------------------------------------------------------------------------
    // HDMI sink-presence input
    //--------------------------------------------------------------------------

    // Hot-plug-detect from the HDMI source connector.
    input  wire        hpd_in,

    //--------------------------------------------------------------------------
    // Locally generated HDMI-related clocks and resets
    //--------------------------------------------------------------------------

    // Pixel/video clock derived from clk_ref.
    output wire        clk_vid,

    // Video-domain synchronized reset.
    output wire        rst_vid,

    // Higher-speed serializer clock derived from clk_ref.
    output wire        clk_tmds_5x,

    // Serializer-domain synchronized reset.
    output wire        rst_tmds_5x,

    //--------------------------------------------------------------------------
    // Transmitter policy output
    //--------------------------------------------------------------------------

    // Indicates that the HDMI source path is enabled/ready.
    output wire        tx_en_o,

    //--------------------------------------------------------------------------
    // Differential TMDS outputs
    //--------------------------------------------------------------------------

    // Differential TMDS clock pair.
    output wire        tmds_clk_p,
    output wire        tmds_clk_n,

    // Differential TMDS data pairs.
    output wire [2:0]  tmds_data_p,
    output wire [2:0]  tmds_data_n
);

    //==========================================================================
    // Internal readiness and reset-condition wires
    //--------------------------------------------------------------------------
    // hdmi_clk_locked
    //   Lock indication from the HDMI clock generator.
    //
    // rst_vid_async
    //   Asynchronous reset request for the video-domain reset synchronizer.
    //
    // rst_tmds_async
    //   Asynchronous reset request for the serializer-domain reset synchronizer.
    //==========================================================================
    wire hdmi_clk_locked;
    wire rst_vid_async;
    wire rst_tmds_async;

    //==========================================================================
    // 1) Video / serializer clock generation
    //--------------------------------------------------------------------------
    // Purpose:
    //   Generate the two clocks required by the HDMI source path:
    //
    //     - clk_vid
    //     - clk_tmds_5x
    //
    // Step-by-step meaning:
    //   1) Accept the upstream reference clock `clk_ref`.
    //   2) Accept the upstream reset `rst_ref`.
    //   3) Produce a pixel-domain clock for the video stream.
    //   4) Produce a higher-speed serializer clock for TMDS transmission.
    //   5) Publish a lock indication once the generated clocks are stable.
    //
    // Architectural meaning:
    //   This is the clock-root of the HDMI output subtree.
    //==========================================================================
    clk_wiz_hdmi_640x480 u_clk_wiz_hdmi_640x480 (
        .clk_in1  (clk_ref),
        .reset    (rst_ref),
        .clk_out1 (clk_vid),
        .clk_out2 (clk_tmds_5x),
        .locked   (hdmi_clk_locked)
    );

    //==========================================================================
    // 2) Asynchronous reset-condition generation
    //--------------------------------------------------------------------------
    // Purpose:
    //   Create raw asynchronous reset requests for the generated HDMI clock
    //   domains.
    //
    // Step-by-step logic:
    //   1) If rst_ref is asserted, the local HDMI domains must remain in reset.
    //   2) If the clock wizard is not locked, the local HDMI domains must also
    //      remain in reset.
    //   3) Therefore each asynchronous reset request is:
    //
    //         rst_ref OR (not locked)
    //
    // Engineering meaning:
    //   The local HDMI domains are only allowed to leave reset once the
    //   upstream reset is released and the generated clocks are stable.
    //==========================================================================
    assign rst_vid_async  = rst_ref | ~hdmi_clk_locked;
    assign rst_tmds_async = rst_ref | ~hdmi_clk_locked;

    //==========================================================================
    // 3) Domain-local synchronous resets
    //--------------------------------------------------------------------------
    // Purpose:
    //   Convert the asynchronous reset requests into resets that deassert in
    //   alignment with each local generated clock.
    //
    // Why separate reset_sync instances?
    //   Because clk_vid and clk_tmds_5x are distinct clock domains, and each
    //   domain must have its own synchronized reset release.
    //
    // Step-by-step:
    //   For video domain:
    //     arst = rst_vid_async
    //     clk  = clk_vid
    //     srst = rst_vid
    //
    //   For serializer domain:
    //     arst = rst_tmds_async
    //     clk  = clk_tmds_5x
    //     srst = rst_tmds_5x
    //==========================================================================
    reset_sync u_reset_sync_vid (
        .clk  (clk_vid),
        .arst (rst_vid_async),
        .srst (rst_vid)
    );

    reset_sync u_reset_sync_tmds (
        .clk  (clk_tmds_5x),
        .arst (rst_tmds_async),
        .srst (rst_tmds_5x)
    );

    //==========================================================================
    // 4) HDMI source enable policy
    //--------------------------------------------------------------------------
    // Purpose:
    //   Provide a conservative indication of whether the HDMI source path should
    //   be considered enabled.
    //
    // Step-by-step logic:
    //   1) Require hpd_in = 1, indicating a sink is present.
    //   2) Require hdmi_clk_locked = 1, indicating clock generation is stable.
    //   3) Require rst_vid = 0, meaning the video domain has left reset.
    //   4) Require rst_tmds_5x = 0, meaning the serializer domain has left
    //      reset.
    //
    // So:
    //
    //   tx_en_o = hpd_in & hdmi_clk_locked & ~rst_vid & ~rst_tmds_5x
    //
    // Engineering meaning:
    //   The transmitter is advertised as enabled only when both the external
    //   sink condition and the local internal readiness conditions are satisfied.
    //==========================================================================
    assign tx_en_o = hpd_in & hdmi_clk_locked & ~rst_vid & ~rst_tmds_5x;

    //==========================================================================
    // 5) TMDS encode + serialize + differential output
    //--------------------------------------------------------------------------
    // Purpose:
    //   Hand the prepared video stream to the lower-level TMDS engine.
    //
    // Step-by-step mapping:
    //   1) Pass the RGB888 pixel data into `vid_pData`.
    //   2) Pass DE into `vid_pVDE`.
    //   3) Pass HSYNC and VSYNC into their corresponding control inputs.
    //   4) Provide the locally generated pixel and serializer clocks.
    //   5) Provide the asynchronous reset condition used by the RGB-to-DVI/TMDS
    //      block.
    //   6) Receive differential TMDS clock/data outputs.
    //
    // Architectural meaning:
    //   This wrapper delegates the actual TMDS protocol conversion and
    //   serialization to rgb2dvi, while owning the surrounding clock/reset and
    //   policy infrastructure.
    //
    // Note on aRst / aRst_n:
    //   The instance is configured with active-high reset semantics
    //   (kRstActiveHigh = 1'b1), so `aRst` is the operative reset input here.
    //==========================================================================
    rgb2dvi #(
        .kGenerateSerialClk (1'b0),
        .kClkRange          (5),
        .kRstActiveHigh     (1'b1)
    ) u_rgb2dvi (
        .TMDS_Clk_p  (tmds_clk_p),
        .TMDS_Clk_n  (tmds_clk_n),
        .TMDS_Data_p (tmds_data_p),
        .TMDS_Data_n (tmds_data_n),

        .aRst        (rst_vid_async),
        .aRst_n      (1'b1),

        .vid_pData   (rgb_in),
        .vid_pVDE    (de_in),
        .vid_pHSync  (hsync_in),
        .vid_pVSync  (vsync_in),

        .PixelClk    (clk_vid),
        .SerialClk   (clk_tmds_5x)
    );

endmodule

`default_nettype wire