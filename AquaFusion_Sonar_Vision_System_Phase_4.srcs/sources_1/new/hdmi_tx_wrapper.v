`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// hdmi_tx_wrapper
//------------------------------------------------------------------------------
// ROLE
//   Real Nexys Video HDMI source wrapper for the J8 HDMI output path.
//
// CONTRACT
//   - Consumes pixel-domain RGB888 + HSYNC/VSYNC/DE
//   - Consumes TMDS serializer clock (5x pixel clock for the rgb2dvi path)
//   - Enables the HDMI source path only when HPD is asserted and resets are idle
//   - Drives the TMDS differential clock/data outputs used by the Nexys Video
//     HDMI source connector J8
//
// NOTES
//   - This wrapper assumes an external clocking path has already generated:
//         clk_vid      = pixel clock
//         clk_tmds     = serializer clock
//   - This wrapper does not generate clocks internally.
//   - The board-level XDC maps these outputs to the J8 TMDS pins and TXEN/HPD.
//==============================================================================

module hdmi_tx_wrapper (
    input  wire        clk_vid,
    input  wire        clk_tmds,
    input  wire        rst_vid,
    input  wire        rst_tmds,
    input  wire [23:0] rgb_in,
    input  wire        hsync_in,
    input  wire        vsync_in,
    input  wire        de_in,
    input  wire        hpd_in,
    output wire        tx_en_o,
    output wire        tmds_clk_p,
    output wire        tmds_clk_n,
    output wire [2:0]  tmds_data_p,
    output wire [2:0]  tmds_data_n
);

    //--------------------------------------------------------------------------
    // HDMI source enable policy
    //
    // HPD comes from the attached sink/display.
    // TX is enabled only when:
    //   - display is attached
    //   - pixel-domain reset is inactive
    //   - serializer-domain reset is inactive
    //--------------------------------------------------------------------------
    assign tx_en_o = hpd_in & ~rst_vid & ~rst_tmds;

    //--------------------------------------------------------------------------
    // TMDS/DVI transmitter
    //
    // Uses the existing rgb2dvi-compatible TMDS generation path.
    //--------------------------------------------------------------------------
    rgb2dvi #(
        .kGenerateSerialClk (1'b0),
        .kClkRange          (5),
        .kRstActiveHigh     (1'b1)
    ) u_rgb2dvi (
        .TMDS_Clk_p  (tmds_clk_p),
        .TMDS_Clk_n  (tmds_clk_n),
        .TMDS_Data_p (tmds_data_p),
        .TMDS_Data_n (tmds_data_n),

        .aRst        (rst_vid),
        .aRst_n      (1'b1),

        .vid_pData   (rgb_in),
        .vid_pVDE    (de_in),
        .vid_pHSync  (hsync_in),
        .vid_pVSync  (vsync_in),

        .PixelClk    (clk_vid),
        .SerialClk   (clk_tmds)
    );

endmodule

`default_nettype wire