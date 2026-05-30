`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// MODULE: digilent_pcam_csi2_rx_axis_shim
//------------------------------------------------------------------------------
// ROLE
//   Project-local shim between the AquaFusion live-camera backend boundary and
//   the real Vivado/Digilent Pcam CSI receiver wrapper.
//
// SYSTEM CONTEXT
//   This module sits inside the true live-video path:
//
//       Pcam 5C OV5640
//         -> dual-lane MIPI CSI-2 / D-PHY
//         -> FMC Pcam Adapter translated FPGA signals
//         -> real Vivado/Digilent CSI receiver wrapper
//         -> digilent_pcam_csi2_rx_axis_shim
//         -> aquafusion_pcam_csi_backend_axis
//         -> pcam_csi_rx_external_ip
//         -> pcam_csi_rx_wrapper
//         -> camera_pixel_stream
//         -> camera_frame_sync
//         -> HDMI compositor
//
// IMPORTANT BOUNDARY
//   This module does not decode MIPI CSI-2 packets.
//   This module does not implement D-PHY lane recovery.
//   This module only adapts the real Vivado/Digilent backend wrapper into the
//   stable AquaFusion AXI-stream-like contract.
//
// REAL BACKEND INSTANCE
//   The current project backend contract expects a module named:
//
//       pcam_csi_rx_axi_stream_core
//
//   with these exact logical ports:
//
//       clk_ref
//       rst
//       enable
//
//       cam_a_hs_clk_p
//       cam_a_hs_clk_n
//       cam_a_hs_lane0_p
//       cam_a_hs_lane0_n
//       cam_a_hs_lane1_p
//       cam_a_hs_lane1_n
//
//       cam_a_lp_clk_p
//       cam_a_lp_clk_n
//       cam_a_lp_lane0_p
//       cam_a_lp_lane0_n
//       cam_a_lp_lane1_p
//       cam_a_lp_lane1_n
//
//       cam_a_bta_o
//
//       axis_clk
//       axis_tvalid
//       axis_tdata[15:0]
//       axis_tuser
//       axis_tlast
//       axis_tready
//
//       locked
//       error_flags[7:0]
//       overflow_pulse
//
//   If the generated Vivado block design exports a different module name or
//   different port names, edit only the instance in this shim. Do not propagate
//   vendor-specific names into the rest of AquaFusion.
//
// AXI-STREAM-LIKE OUTPUT CONTRACT
//   axis_clk:
//     Pixel stream clock from the real CSI backend.
//
//   axis_tvalid:
//     High when axis_tdata, axis_tuser, and axis_tlast are valid.
//
//   axis_tready:
//     Backpressure from the downstream adapter.
//
//   axis_tdata:
//     16-bit RGB565 pixel data.
//
//   axis_tuser:
//     Start-of-frame marker aligned with the first active pixel of a frame.
//
//   axis_tlast:
//     End-of-line marker aligned with the final active pixel of a line.
//
// HEALTH/STATUS CONTRACT
//   backend_locked:
//     High when the real backend reports lock.
//
//   backend_error_flags:
//     Backend-provided error/status flags.
//
//   backend_overflow_pulse:
//     One-cycle pulse when the backend reports dropped/overflowed pixel data.
//
// BTA CONTRACT
//   cam_a_bta_o is forwarded from the real backend.
//   For normal Pcam receive operation, this should normally remain low.
//
// CDC CONTRACT
//   No CDC is performed in this shim.
//   axis_clk owns all axis_* outputs and backend status outputs unless the real
//   backend explicitly documents otherwise.
//
// SYNTHESIS NOTES
//   - Verilog-2001 compatible.
//   - No SystemVerilog constructs.
//   - This file is intentionally thin.
//   - The instantiated backend must be added to the Vivado project.
//==============================================================================

module digilent_pcam_csi2_rx_axis_shim #(
    //--------------------------------------------------------------------------
    // FRAME_W / FRAME_H
    //--------------------------------------------------------------------------
    // Expected active output frame size.
    //
    // These parameters are forwarded to the real backend wrapper. The real
    // Vivado/Digilent configuration and OV5640 SCCB register table must agree
    // with these values.
    //--------------------------------------------------------------------------
    parameter integer FRAME_W = 640,
    parameter integer FRAME_H = 480
)(
    //--------------------------------------------------------------------------
    // Reference/control clock and reset
    //--------------------------------------------------------------------------
    input  wire        clk_ref,
    input  wire        rst,
    input  wire        enable,

    //--------------------------------------------------------------------------
    // FMC Pcam Adapter translated high-speed and low-power lane signals
    //--------------------------------------------------------------------------
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

    //--------------------------------------------------------------------------
    // Bus-turnaround control
    //--------------------------------------------------------------------------
    output wire        cam_a_bta_o,

    //--------------------------------------------------------------------------
    // AXI-stream-like pixel output to AquaFusion backend adapter
    //--------------------------------------------------------------------------
    output wire        axis_clk,
    output wire        axis_tvalid,
    input  wire        axis_tready,
    output wire [15:0] axis_tdata,
    output wire        axis_tuser,
    output wire        axis_tlast,

    //--------------------------------------------------------------------------
    // Backend health/status
    //--------------------------------------------------------------------------
    output wire        backend_locked,
    output wire [7:0]  backend_error_flags,
    output wire        backend_overflow_pulse
);

    //==========================================================================
    // REAL VIVADO / DIGILENT CSI BACKEND INSTANCE
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Instantiate the real CSI receive wrapper that is generated or supplied
    //   by the Vivado/Digilent backend flow.
    //
    // STEP-BY-STEP DATAFLOW
    //   1) The backend receives translated FMC Pcam Adapter high-speed and
    //      low-power signals.
    //   2) The backend performs D-PHY/CSI-2 reception internally.
    //   3) The backend emits one RGB565 pixel per accepted AXI-stream transfer.
    //   4) This shim forwards that stream unchanged to the AquaFusion backend
    //      adapter.
    //
    // EDITING RULE
    //   If the real Vivado-generated wrapper has different port names, modify
    //   only this instance mapping. Keep the shim's public ports unchanged.
    //==========================================================================

    pcam_csi_rx_axi_stream_core #(
        .FRAME_W (FRAME_W),
        .FRAME_H (FRAME_H)
    ) u_pcam_csi_rx_axi_stream_core (
        .clk_ref          (clk_ref),
        .rst              (rst),
        .enable           (enable),

        .cam_a_hs_clk_p   (cam_a_hs_clk_p),
        .cam_a_hs_clk_n   (cam_a_hs_clk_n),
        .cam_a_hs_lane0_p (cam_a_hs_lane0_p),
        .cam_a_hs_lane0_n (cam_a_hs_lane0_n),
        .cam_a_hs_lane1_p (cam_a_hs_lane1_p),
        .cam_a_hs_lane1_n (cam_a_hs_lane1_n),

        .cam_a_lp_clk_p   (cam_a_lp_clk_p),
        .cam_a_lp_clk_n   (cam_a_lp_clk_n),
        .cam_a_lp_lane0_p (cam_a_lp_lane0_p),
        .cam_a_lp_lane0_n (cam_a_lp_lane0_n),
        .cam_a_lp_lane1_p (cam_a_lp_lane1_p),
        .cam_a_lp_lane1_n (cam_a_lp_lane1_n),

        .cam_a_bta_o      (cam_a_bta_o),

        .axis_clk         (axis_clk),
        .axis_tvalid      (axis_tvalid),
        .axis_tdata       (axis_tdata),
        .axis_tuser       (axis_tuser),
        .axis_tlast       (axis_tlast),
        .axis_tready      (axis_tready),

        .locked           (backend_locked),
        .error_flags      (backend_error_flags),
        .overflow_pulse   (backend_overflow_pulse)
    );

endmodule

`default_nettype wire