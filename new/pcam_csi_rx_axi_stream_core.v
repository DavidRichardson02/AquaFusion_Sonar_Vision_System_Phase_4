`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// MODULE: pcam_csi_rx_axi_stream_core
//------------------------------------------------------------------------------
// ROLE
//   Real Pcam 5C CSI-2 receive backend for AquaFusion.
//
// IMPLEMENTATION
//   This wrapper instantiates the generated Xilinx MIPI CSI-2 RX Subsystem:
//
//       pcam_csi2_rx_ss
//
//   The generated IP is configured for:
//     - 7-series Artix-7 target
//     - two D-PHY data lanes
//     - RGB565 output
//     - one pixel per clock
//     - Video Format Bridge included
//     - CSI-2 controller AXI-Lite register interface disabled
//
// CLOCKING
//   clk_ref is the project 100 MHz board/system clock.
//   video_aclk and the AquaFusion AXI-stream-like output use clk_ref directly.
//   A small local MMCM derives the 200 MHz dphy_clk_200M required by the
//   Xilinx 7-series D-PHY receive logic.
//
// STATUS POLICY
//   locked is a readiness qualifier for downstream stream acceptance. It
//   asserts when the local D-PHY MMCM is locked, the backend is enabled, and
//   the generated CSI subsystem is not reporting its internal reset output.
//
// ERROR FLAGS
//   [0] CSI SoT sync failure
//   [1] CSI SoT error
//   [2] CRC error
//   [3] ECC double-bit error
//   [4] ECC corrected single-bit event
//   [5] line buffer full / output backpressure overflow
//   [6] generated subsystem reset or disable still in progress
//   [7] local 200 MHz D-PHY clock MMCM is not locked while enabled
//==============================================================================

module pcam_csi_rx_axi_stream_core #(
    parameter integer FRAME_W = 640,
    parameter integer FRAME_H = 480
)(
    input  wire        clk_ref,
    input  wire        rst,
    input  wire        enable,

    input  wire        cam_a_hs_clk_p,
    input  wire        cam_a_hs_clk_n,
    input  wire        cam_a_hs_lane0_p,
    input  wire        cam_a_hs_lane0_n,
    input  wire        cam_a_hs_lane1_p,
    input  wire        cam_a_hs_lane1_n,

    input  wire        cam_a_lp_clk_p,
    input  wire        cam_a_lp_clk_n,
    input  wire        cam_a_lp_lane0_p,
    input  wire        cam_a_lp_lane0_n,
    input  wire        cam_a_lp_lane1_p,
    input  wire        cam_a_lp_lane1_n,

    output wire        cam_a_bta_o,

    output wire        axis_clk,
    output wire        axis_tvalid,
    output wire [15:0] axis_tdata,
    output wire        axis_tuser,
    output wire        axis_tlast,
    input  wire        axis_tready,

    output wire        locked,
    output wire [7:0]  error_flags,
    output wire        overflow_pulse
);

    localparam [31:0] FRAME_W_CONST = FRAME_W;
    localparam [31:0] FRAME_H_CONST = FRAME_H;

    //--------------------------------------------------------------------------
    // Local 200 MHz D-PHY reference clock generation
    //--------------------------------------------------------------------------
    wire clk_ref_bufg;
    wire clkfb_mmcm;
    wire clkfb_bufg;
    wire dphy_clk_200m_mmcm;
    wire dphy_clk_200m;
    wire dphy_mmcm_locked;

    BUFG u_pcam_csi_ref_bufg (
        .I (clk_ref),
        .O (clk_ref_bufg)
    );

    MMCME2_BASE #(
        .BANDWIDTH          ("OPTIMIZED"),
        .CLKIN1_PERIOD      (10.000),
        .DIVCLK_DIVIDE      (1),
        .CLKFBOUT_MULT_F    (10.000),
        .CLKFBOUT_PHASE     (0.000),
        .CLKOUT0_DIVIDE_F   (5.000),
        .CLKOUT0_PHASE      (0.000),
        .CLKOUT0_DUTY_CYCLE (0.500),
        .CLKOUT1_DIVIDE     (1),
        .CLKOUT1_PHASE      (0.000),
        .CLKOUT1_DUTY_CYCLE (0.500),
        .CLKOUT2_DIVIDE     (1),
        .CLKOUT2_PHASE      (0.000),
        .CLKOUT2_DUTY_CYCLE (0.500),
        .CLKOUT3_DIVIDE     (1),
        .CLKOUT3_PHASE      (0.000),
        .CLKOUT3_DUTY_CYCLE (0.500),
        .CLKOUT4_DIVIDE     (1),
        .CLKOUT4_PHASE      (0.000),
        .CLKOUT4_DUTY_CYCLE (0.500),
        .CLKOUT5_DIVIDE     (1),
        .CLKOUT5_PHASE      (0.000),
        .CLKOUT5_DUTY_CYCLE (0.500),
        .CLKOUT6_DIVIDE     (1),
        .CLKOUT6_PHASE      (0.000),
        .CLKOUT6_DUTY_CYCLE (0.500),
        .STARTUP_WAIT       ("FALSE")
    ) u_pcam_csi_dphy_mmcm (
        .CLKIN1   (clk_ref_bufg),
        .RST      (rst),
        .PWRDWN   (1'b0),
        .CLKFBIN  (clkfb_bufg),
        .CLKFBOUT (clkfb_mmcm),
        .CLKOUT0  (dphy_clk_200m_mmcm),
        .CLKOUT1  (),
        .CLKOUT2  (),
        .CLKOUT3  (),
        .CLKOUT4  (),
        .CLKOUT5  (),
        .CLKOUT6  (),
        .LOCKED   (dphy_mmcm_locked)
    );

    BUFG u_pcam_csi_fb_bufg (
        .I (clkfb_mmcm),
        .O (clkfb_bufg)
    );

    BUFG u_pcam_csi_dphy_bufg (
        .I (dphy_clk_200m_mmcm),
        .O (dphy_clk_200m)
    );

    //--------------------------------------------------------------------------
    // Generated Xilinx CSI-2 RX subsystem wiring
    //--------------------------------------------------------------------------
    wire        subsystem_rst_out;
    wire        ctrl_dis_in_prgs;
    wire        errsotsynchs_intr;
    wire        errsoths_intr;
    wire        cl_stopstate_intr;
    wire        dl0_stopstate_intr;
    wire        dl1_stopstate_intr;
    wire        crc_status_intr;
    wire [1:0]  ecc_status_intr;
    wire        linebuffer_full;
    wire        frame_rcvd_pulse_out;
    wire        rxbyteclkhs;
    wire [9:0]  video_out_tdest;

    wire        core_enable;
    wire        video_resetn;

    assign axis_clk     = clk_ref;
    assign core_enable  = enable && dphy_mmcm_locked && !rst;
    assign video_resetn = core_enable;
    assign cam_a_bta_o  = 1'b0;

    pcam_csi2_rx_ss u_pcam_csi2_rx_ss (
        .dphy_clk_200M        (dphy_clk_200m),
        .rxbyteclkhs          (rxbyteclkhs),
        .system_rst_out       (subsystem_rst_out),
        .video_aclk           (clk_ref),
        .video_aresetn        (video_resetn),
        .ctrl_core_en         (core_enable),
        .active_lanes         (2'b01),
        .ctrl_dis_in_prgs     (ctrl_dis_in_prgs),
        .errsotsynchs_intr    (errsotsynchs_intr),
        .errsoths_intr        (errsoths_intr),
        .cl_stopstate_intr    (cl_stopstate_intr),
        .dl0_stopstate_intr   (dl0_stopstate_intr),
        .dl1_stopstate_intr   (dl1_stopstate_intr),
        .crc_status_intr      (crc_status_intr),
        .ecc_status_intr      (ecc_status_intr),
        .linebuffer_full      (linebuffer_full),
        .frame_rcvd_pulse_out (frame_rcvd_pulse_out),
        .video_out_tdata      (axis_tdata),
        .video_out_tdest      (video_out_tdest),
        .video_out_tlast      (axis_tlast),
        .video_out_tready     (axis_tready),
        .video_out_tuser      (axis_tuser),
        .video_out_tvalid     (axis_tvalid),
        .mipi_phy_if_clk_hs_n (cam_a_hs_clk_n),
        .mipi_phy_if_clk_hs_p (cam_a_hs_clk_p),
        .mipi_phy_if_clk_lp_n (cam_a_lp_clk_n),
        .mipi_phy_if_clk_lp_p (cam_a_lp_clk_p),
        .mipi_phy_if_data_hs_n({cam_a_hs_lane1_n, cam_a_hs_lane0_n}),
        .mipi_phy_if_data_hs_p({cam_a_hs_lane1_p, cam_a_hs_lane0_p}),
        .mipi_phy_if_data_lp_n({cam_a_lp_lane1_n, cam_a_lp_lane0_n}),
        .mipi_phy_if_data_lp_p({cam_a_lp_lane1_p, cam_a_lp_lane0_p})
    );

    assign locked =
        core_enable && !subsystem_rst_out;

    assign error_flags[0] = errsotsynchs_intr;
    assign error_flags[1] = errsoths_intr;
    assign error_flags[2] = crc_status_intr;
    assign error_flags[3] = ecc_status_intr[1];
    assign error_flags[4] = ecc_status_intr[0];
    assign error_flags[5] = linebuffer_full;
    assign error_flags[6] = subsystem_rst_out | ctrl_dis_in_prgs;
    assign error_flags[7] = enable && !dphy_mmcm_locked;

    assign overflow_pulse = linebuffer_full;

    wire _unused_status;
    assign _unused_status =
        rxbyteclkhs          ^
        cl_stopstate_intr    ^
        dl0_stopstate_intr   ^
        dl1_stopstate_intr   ^
        frame_rcvd_pulse_out ^
        video_out_tdest[0]   ^
        video_out_tdest[1]   ^
        video_out_tdest[2]   ^
        video_out_tdest[3]   ^
        video_out_tdest[4]   ^
        video_out_tdest[5]   ^
        video_out_tdest[6]   ^
        video_out_tdest[7]   ^
        video_out_tdest[8]   ^
        video_out_tdest[9]   ^
        FRAME_W_CONST[0]     ^
        FRAME_H_CONST[0];

endmodule

`default_nettype wire
