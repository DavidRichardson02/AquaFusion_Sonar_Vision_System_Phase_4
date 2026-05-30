`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// MODULE: pcam_csi_rx_external_ip
//------------------------------------------------------------------------------
// ROLE
//   AquaFusion external CSI backend adapter for true Pcam 5C live video.
//
// SYSTEM CONTEXT
//   This module is the integration point between a real CSI/D-PHY backend and
//   the project-local camera pipeline.
//
// LIVE VIDEO CHAIN
//   Pcam 5C OV5640
//     -> dual-lane MIPI CSI-2 / D-PHY
//     -> FMC Pcam Adapter translated FPGA signals
//     -> Digilent or licensed CSI/D-PHY receive IP
//     -> AXI-stream or equivalent pixel stream
//     -> pcam_csi_rx_external_ip
//     -> pcam_csi_rx_wrapper
//     -> camera_pixel_stream
//     -> camera_frame_sync
//     -> HDMI compositor
//
// IMPORTANT LIVE-VIDEO REQUIREMENT
//   Defining AQUAFUSION_USE_EXTERNAL_CSI_IP selects this external receive path,
//   but true Pcam 5C live video still requires a real CSI/D-PHY backend.
//
//   Without AQUAFUSION_CSI_BACKEND_AXI_STREAM, this module fails closed and
//   reports rx_error_flags[7].
//
// EXPECTED BACKEND MODULE
//   When AQUAFUSION_CSI_BACKEND_AXI_STREAM is defined, this module instantiates:
//
//       aquafusion_pcam_csi_backend_axis
//
//   That backend must convert the physical FMC Pcam Adapter signals into a
//   pixel AXI-stream.
//
// REQUIRED BACKEND INTERFACE
//   module aquafusion_pcam_csi_backend_axis #(
//       parameter integer FRAME_W = ...,
//       parameter integer FRAME_H = ...
//   )(
//       input  wire        clk_ref,
//       input  wire        rst,
//       input  wire        enable,
//
//       input  wire        cam_a_hs_clk_p,
//       input  wire        cam_a_hs_clk_n,
//       input  wire        cam_a_hs_lane0_p,
//       input  wire        cam_a_hs_lane0_n,
//       input  wire        cam_a_hs_lane1_p,
//       input  wire        cam_a_hs_lane1_n,
//
//       input  wire        cam_a_lp_clk_p,
//       input  wire        cam_a_lp_clk_n,
//       input  wire        cam_a_lp_lane0_p,
//       input  wire        cam_a_lp_lane0_n,
//       input  wire        cam_a_lp_lane1_p,
//       input  wire        cam_a_lp_lane1_n,
//
//       output wire        cam_a_bta_o,
//
//       output wire        axis_clk,
//       output wire        axis_tvalid,
//       input  wire        axis_tready,
//       output wire [15:0] axis_tdata,
//       output wire        axis_tuser,
//       output wire        axis_tlast,
//
//       output wire        backend_locked,
//       output wire [7:0]  backend_error_flags,
//       output wire        backend_overflow_pulse
//   );
//
// OWNERSHIP BOUNDARY
//   This module owns:
//     - backend-present vs fail-closed selection,
//     - AXI-stream adaptation into AquaFusion stream events,
//     - forwarding backend lock/error/overflow status.
//
//   This module does not own:
//     - MIPI CSI-2 packet decoding,
//     - D-PHY byte/lane recovery,
//     - OV5640 SCCB configuration,
//     - frame buffering,
//     - display-domain CDC.
//
// SYNTHESIS NOTES
//   - Verilog-2001 compatible.
//   - This file compiles safely without a real backend.
//   - Live video requires AQUAFUSION_CSI_BACKEND_AXI_STREAM plus the backend
//     module listed above.
//==============================================================================

module pcam_csi_rx_external_ip #(
    parameter integer FRAME_W = 640,
    parameter integer FRAME_H = 480
)(
    input  wire       clk_ref,
    input  wire       rst,
    input  wire       enable,

    input  wire       cam_a_hs_clk_p,
    input  wire       cam_a_hs_clk_n,
    input  wire       cam_a_hs_lane0_p,
    input  wire       cam_a_hs_lane0_n,
    input  wire       cam_a_hs_lane1_p,
    input  wire       cam_a_hs_lane1_n,

    input  wire       cam_a_lp_clk_p,
    input  wire       cam_a_lp_clk_n,
    input  wire       cam_a_lp_lane0_p,
    input  wire       cam_a_lp_lane0_n,
    input  wire       cam_a_lp_lane1_p,
    input  wire       cam_a_lp_lane1_n,

    output wire       cam_a_bta_o,

    output wire       rx_pixel_clk,
    output wire       rx_pixel_valid,
    output wire [15:0] rx_pixel_data,
    output wire       rx_frame_start,
    output wire       rx_line_start,
    output wire       rx_frame_done,
    output wire       rx_locked,
    output wire [7:0] rx_error_flags,
    output wire       overflow_pulse
);

`ifdef AQUAFUSION_CSI_BACKEND_AXI_STREAM

    //==========================================================================
    // REAL BACKEND MODE
    //==========================================================================

    wire        axis_clk;
    wire        axis_tvalid;
    wire        axis_tready;
    wire [15:0] axis_tdata;
    wire        axis_tuser;
    wire        axis_tlast;

    wire        backend_locked;
    wire [7:0]  backend_error_flags;
    wire        backend_overflow_pulse;

    assign rx_pixel_clk = axis_clk;

    aquafusion_pcam_csi_backend_axis #(
        .FRAME_W (FRAME_W),
        .FRAME_H (FRAME_H)
    ) u_aquafusion_pcam_csi_backend_axis (
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

    wire        adapt_rx_pixel_valid;
    wire [15:0] adapt_rx_pixel_data;
    wire        adapt_rx_frame_start;
    wire        adapt_rx_line_start;
    wire        adapt_rx_frame_done;
    wire        adapt_rx_locked;
    wire [7:0]  adapt_rx_error_flags;
    wire        adapt_overflow_pulse;

    csi_axis_to_pixel_stream #(
        .FRAME_W (FRAME_W),
        .FRAME_H (FRAME_H)
    ) u_csi_axis_to_pixel_stream (
        .axis_clk       (axis_clk),
        .rst            (rst),
        .enable         (enable && backend_locked),

        .axis_tvalid    (axis_tvalid),
        .axis_tready    (axis_tready),
        .axis_tdata     (axis_tdata),
        .axis_tuser     (axis_tuser),
        .axis_tlast     (axis_tlast),

        .rx_pixel_valid (adapt_rx_pixel_valid),
        .rx_pixel_data  (adapt_rx_pixel_data),
        .rx_frame_start (adapt_rx_frame_start),
        .rx_line_start  (adapt_rx_line_start),
        .rx_frame_done  (adapt_rx_frame_done),
        .rx_locked      (adapt_rx_locked),
        .rx_error_flags (adapt_rx_error_flags),
        .overflow_pulse (adapt_overflow_pulse)
    );

    assign rx_pixel_valid = adapt_rx_pixel_valid;
    assign rx_pixel_data  = adapt_rx_pixel_data;
    assign rx_frame_start = adapt_rx_frame_start;
    assign rx_line_start  = adapt_rx_line_start;
    assign rx_frame_done  = adapt_rx_frame_done;

    assign rx_locked =
        backend_locked && adapt_rx_locked;

    assign rx_error_flags =
        backend_error_flags | adapt_rx_error_flags;

    assign overflow_pulse =
        backend_overflow_pulse | adapt_overflow_pulse;

`else

    //==========================================================================
    // FAIL-CLOSED MODE
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Compile safely without a real CSI/D-PHY backend while making it obvious
    //   to status logic that live video is unavailable.
    //
    // ERROR POLICY
    //   rx_error_flags[7] is asserted whenever enable is high.
    //==========================================================================

    assign cam_a_bta_o      = 1'b0;
    assign rx_pixel_clk     = clk_ref;
    assign rx_pixel_valid   = 1'b0;
    assign rx_pixel_data    = 16'h0000;
    assign rx_frame_start   = 1'b0;
    assign rx_line_start    = 1'b0;
    assign rx_frame_done    = 1'b0;
    assign rx_locked        = 1'b0;
    assign rx_error_flags   = enable ? 8'h80 : 8'h00;
    assign overflow_pulse   = 1'b0;

    wire _unused_inputs;

    assign _unused_inputs =
        cam_a_hs_clk_p   ^ cam_a_hs_clk_n   ^
        cam_a_hs_lane0_p ^ cam_a_hs_lane0_n ^
        cam_a_hs_lane1_p ^ cam_a_hs_lane1_n ^
        cam_a_lp_clk_p   ^ cam_a_lp_clk_n   ^
        cam_a_lp_lane0_p ^ cam_a_lp_lane0_n ^
        cam_a_lp_lane1_p ^ cam_a_lp_lane1_n;

`endif

endmodule

`default_nettype wire