`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// MODULE: aquafusion_pcam_csi_backend_axis
//------------------------------------------------------------------------------
// ROLE
//   Project-local wrapper around the real Pcam 5C CSI/D-PHY receive backend.
//
// SYSTEM CONTEXT
//   This module is the live-video backend boundary for AquaFusion:
//
//       Pcam 5C OV5640
//         -> dual-lane MIPI CSI-2 / D-PHY
//         -> FMC Pcam Adapter translated FPGA signals
//         -> Digilent/licensed CSI/D-PHY receiver
//         -> this backend wrapper
//         -> AXI-stream-like RGB565 pixel stream
//         -> pcam_csi_rx_external_ip
//         -> pcam_csi_rx_wrapper
//         -> camera_pixel_stream
//         -> camera_frame_sync
//         -> HDMI compositor
//
// IMPORTANT ENGINEERING BOUNDARY
//   This module does not implement MIPI CSI-2 or D-PHY decoding directly.
//   It wraps a real CSI receive core and adapts that core's output into the
//   AquaFusion AXI-stream-like pixel contract.
//
// EXPECTED OUTPUT CONTRACT
//   axis_clk:
//     Pixel-stream clock produced or forwarded by the real CSI backend.
//
//   axis_tvalid:
//     High when axis_tdata, axis_tuser, and axis_tlast are valid.
//
//   axis_tready:
//     Backpressure from downstream adapter.
//
//   axis_tdata:
//     16-bit RGB565 pixel data.
//
//   axis_tuser:
//     Start-of-frame marker aligned with the first pixel of a frame.
//
//   axis_tlast:
//     End-of-line marker aligned with the final pixel of a line.
//
//   backend_locked:
//     High when the real CSI backend is locked and producing valid stream timing.
//
//   backend_error_flags:
//     Backend-local diagnostic flags.
//
//   backend_overflow_pulse:
//     One-cycle pulse when the backend drops/overflows pixel data.
//
// BACKEND SELECTION
//   Define exactly one real backend macro when live video is being integrated:
//
//       AQUAFUSION_BACKEND_DIGILENT_AXIS
//
//   This module expects a project-local shim named:
//
//       digilent_pcam_csi2_rx_axis_shim
//
//   That shim should be the only file that knows the exact port names of the
//   Digilent Vivado IP / block design wrapper.
//
// FAIL-CLOSED POLICY
//   If no real backend macro is defined, this module produces no pixels and
//   asserts backend_error_flags[7] when enable is high.
//
// SYNTHESIS NOTES
//   - Verilog-2001 compatible.
//   - No SystemVerilog constructs.
//   - The real Digilent/licensed backend must be added to the Vivado project.
//   - The shim must be edited to match the generated Vivado IP or BD wrapper.
//==============================================================================

module aquafusion_pcam_csi_backend_axis #(
    //--------------------------------------------------------------------------
    // FRAME_W / FRAME_H
    //--------------------------------------------------------------------------
    // Expected active output frame size.
    //
    // These are forwarded to the backend shim for configuration/status
    // consistency. The real CSI backend may also require matching OV5640 SCCB
    // register configuration.
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
    // AXI-stream-like pixel output to pcam_csi_rx_external_ip
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
    // DIGILENT / VIVADO CSI BACKEND MODE
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Instantiate a project-local shim around the actual Digilent-generated
    //   CSI/D-PHY receive design.
    //
    // WHY A SHIM IS USED
    //   Digilent/Vivado IP port names and clocking structure may differ depending
    //   on the exact IP version and block design export. Keeping those details
    //   inside digilent_pcam_csi2_rx_axis_shim prevents the rest of AquaFusion
    //   from depending on vendor-specific naming.
    //
    // REQUIRED SHIM RESPONSIBILITY
    //   The shim must:
    //     1) connect to the real Digilent/licensed CSI receiver,
    //     2) expose RGB565 pixels as AXI-stream-like data,
    //     3) generate axis_tuser on the first pixel of each frame,
    //     4) generate axis_tlast on the last pixel of each line,
    //     5) report lock/error/overflow status.
    //==========================================================================
`ifdef AQUAFUSION_BACKEND_DIGILENT_AXIS
    
        digilent_pcam_csi2_rx_axis_shim #(
            .FRAME_W (FRAME_W),
            .FRAME_H (FRAME_H)
        ) u_digilent_pcam_csi2_rx_axis_shim (
            .clk_ref                (clk_ref),
            .rst                    (rst),
            .enable                 (enable),
    
            .cam_a_hs_clk_p         (cam_a_hs_clk_p),
            .cam_a_hs_clk_n         (cam_a_hs_clk_n),
            .cam_a_hs_lane0_p       (cam_a_hs_lane0_p),
            .cam_a_hs_lane0_n       (cam_a_hs_lane0_n),
            .cam_a_hs_lane1_p       (cam_a_hs_lane1_p),
            .cam_a_hs_lane1_n       (cam_a_hs_lane1_n),
    
            .cam_a_lp_clk_p         (cam_a_lp_clk_p),
            .cam_a_lp_clk_n         (cam_a_lp_clk_n),
            .cam_a_lp_lane0_p       (cam_a_lp_lane0_p),
            .cam_a_lp_lane0_n       (cam_a_lp_lane0_n),
            .cam_a_lp_lane1_p       (cam_a_lp_lane1_p),
            .cam_a_lp_lane1_n       (cam_a_lp_lane1_n),
    
            .cam_a_bta_o            (cam_a_bta_o),
    
            .axis_clk               (axis_clk),
            .axis_tvalid            (axis_tvalid),
            .axis_tready            (axis_tready),
            .axis_tdata             (axis_tdata),
            .axis_tuser             (axis_tuser),
            .axis_tlast             (axis_tlast),
    
            .backend_locked         (backend_locked),
            .backend_error_flags    (backend_error_flags),
            .backend_overflow_pulse (backend_overflow_pulse)
        );
    
`else

    //==========================================================================
    // FAIL-CLOSED MODE
    //--------------------------------------------------------------------------
    // Keep this backend boundary deterministic when the external CSI path is
    // enabled before a real Digilent/Vivado receive core has been supplied.
    //==========================================================================
    assign cam_a_bta_o            = 1'b0;
    assign axis_clk               = clk_ref;
    assign axis_tvalid            = 1'b0;
    assign axis_tdata             = 16'h0000;
    assign axis_tuser             = 1'b0;
    assign axis_tlast             = 1'b0;
    assign backend_locked         = 1'b0;
    assign backend_error_flags    = enable ? 8'h80 : 8'h00;
    assign backend_overflow_pulse = 1'b0;

    wire _unused_backend_inputs;
    assign _unused_backend_inputs =
        rst                    ^
        cam_a_hs_clk_p         ^ cam_a_hs_clk_n   ^
        cam_a_hs_lane0_p       ^ cam_a_hs_lane0_n ^
        cam_a_hs_lane1_p       ^ cam_a_hs_lane1_n ^
        cam_a_lp_clk_p         ^ cam_a_lp_clk_n   ^
        cam_a_lp_lane0_p       ^ cam_a_lp_lane0_n ^
        cam_a_lp_lane1_p       ^ cam_a_lp_lane1_n ^
        axis_tready;

`endif


endmodule

`default_nettype wire
