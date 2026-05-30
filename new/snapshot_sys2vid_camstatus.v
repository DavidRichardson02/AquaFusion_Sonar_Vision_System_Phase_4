`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// snapshot_sys2vid_camstatus
//------------------------------------------------------------------------------
// ROLE
//   Camera-status-specific wrapper for SYS-to-VID snapshot transfer and
//   frame-boundary commit.
//
// HIGH-LEVEL PURPOSE
//   This module transfers a camera status snapshot from the system/control clock
//   domain (`clk_sys`) into the video/render clock domain (`clk_vid`) and
//   ensures that the newly arrived status becomes publicly visible only on a
//   destination frame boundary.
//
//   Functionally, this module does not implement its own CDC algorithm.
//   Instead, it instantiates the generic `snapshot_sys2vid` module and binds its
//   generic source/destination interface to the semantic roles:
//
//       source      = camera control/status domain
//       destination = video/HUD rendering domain
//
// WHY THIS MODULE EXISTS
//   A generic CDC/commit block is useful, but higher-level camera code benefits
//   from a wrapper whose name reflects the meaning of the transported data.
//
//   This wrapper makes the architectural intent explicit:
//
//       camera status produced in SYS domain
//           ->
//       cross safely / stage in VID domain
//           ->
//       commit on frame boundary
//           ->
//       consume in camera debug HUD tile or other raster logic
//
//   In other words, this module exists to give the generic snapshot transfer a
//   camera-status-specific identity in the design hierarchy.
//
// FUNCTIONAL CONTRACT
//   Source-side meaning:
//     snap_sys
//       Camera status snapshot generated in the system/control domain.
//
//     snap_upd_sys
//       Event indicating that snap_sys should be published to the destination
//       domain.
//
//   Destination-side meaning:
//     snap_vid_committed
//       Frame-stable camera status snapshot visible in the video domain.
//
//     snap_commit_pulse_vid
//       One-cycle pulse in the video domain indicating that a new pending camera
//       status snapshot has just been committed at a frame boundary.
//
// FRAME-COHERENCE PURPOSE
//   This wrapper inherits the most important property of `snapshot_sys2vid`:
//
//     newly received destination-domain data is not exposed immediately;
//     instead, it is committed only when frame_tick_vid is asserted.
//
//   That is exactly the correct behavior for video/HUD logic, because it
//   prevents camera status indicators from changing halfway through a frame.
//
// SIGNAL SEMANTICS
//   clk_sys
//     Source/system clock domain in which camera control/status is produced.
//
//   rst_sys
//     Active-high reset for the source/system side of the snapshot-transfer
//     machinery.
//
//   snap_sys
//     Camera status snapshot bus in the source domain.
//
//   snap_upd_sys
//     Source-domain update event indicating that snap_sys should be captured and
//     transferred.
//
//   clk_vid
//     Destination/video clock domain in which committed camera status is
//     consumed by renderers or other frame-coherent logic.
//
//   rst_vid
//     Active-high reset for the destination/video side of the snapshot-transfer
//     machinery.
//
//   frame_tick_vid
//     Destination-domain frame boundary event used as the commit point.
//
//   snap_vid_committed
//     Public frame-stable camera status snapshot in the video domain.
//
//   snap_commit_pulse_vid
//     One-cycle pulse indicating that snap_vid_committed has just been updated
//     on a frame boundary.
//
// WHY A WRAPPER IS BETTER THAN DIRECT GENERIC INSTANTIATION EVERYWHERE
//   This wrapper improves readability and maintainability in several ways:
//
//     1) It provides a semantically meaningful name at instantiation sites.
//     2) It makes domain intent obvious without rereading port mapping.
//     3) It localizes the mapping between generic CDC terminology and
//        camera-status terminology.
//     4) It allows future camera-status-specific assertions, width checks, or
//        instrumentation to be added without modifying all higher-level users.
//
// BEHAVIORAL SUMMARY
//   This module is behaviorally equivalent to:
//
//       snapshot_sys2vid generic instance
//
//   with the following renaming:
//
//       clk_src        -> clk_sys
//       rst_src        -> rst_sys
//       snap_src       -> snap_sys
//       snap_upd_src   -> snap_upd_sys
//
//       clk_dst        -> clk_vid
//       rst_dst        -> rst_vid
//       frame_tick_dst -> frame_tick_vid
//
//       snap_dst_committed -> snap_vid_committed
//       commit_pulse_dst   -> snap_commit_pulse_vid
//
// PEDAGOGICAL SUMMARY
//   The module can be understood as a domain-specific alias:
//
//     "Use the generic snapshot transfer/commit mechanism, but interpret it as
//      the official camera-status publication path from SYS into VID."
//------------------------------------------------------------------------------
module snapshot_sys2vid_camstatus #(
    //--------------------------------------------------------------------------
    // Width of the camera status snapshot bus.
    //--------------------------------------------------------------------------
    parameter integer W = 64
)(
    //--------------------------------------------------------------------------
    // Source / system domain
    //--------------------------------------------------------------------------
    input  wire         clk_sys,
    input  wire         rst_sys,
    input  wire [W-1:0] snap_sys,
    input  wire         snap_upd_sys,

    //--------------------------------------------------------------------------
    // Destination / video domain
    //--------------------------------------------------------------------------
    input  wire         clk_vid,
    input  wire         rst_vid,
    input  wire         frame_tick_vid,

    //--------------------------------------------------------------------------
    // Destination committed outputs
    //--------------------------------------------------------------------------
    output wire [W-1:0] snap_vid_committed,
    output wire         snap_commit_pulse_vid
);

    //==========================================================================
    // Generic snapshot transfer / frame-commit instance
    //--------------------------------------------------------------------------
    // Step-by-step conceptual mapping:
    //
    //   1) Treat clk_sys/rst_sys as the source publication domain.
    //   2) Treat snap_sys + snap_upd_sys as the source snapshot interface.
    //   3) Treat clk_vid/rst_vid as the destination rendering domain.
    //   4) Use frame_tick_vid as the only legal commit boundary.
    //   5) Publish the committed result as snap_vid_committed.
    //   6) Publish the one-cycle commit event as snap_commit_pulse_vid.
    //
    // Engineering meaning:
    //   This wrapper adds no new logic. Its purpose is semantic specialization
    //   and clean hierarchical naming.
    //==========================================================================
    snapshot_sys2vid #(
        .W(W)
    ) u_snapshot_sys2vid (
        .clk_src            (clk_sys),
        .rst_src            (rst_sys),
        .snap_src           (snap_sys),
        .snap_upd_src       (snap_upd_sys),
        .clk_dst            (clk_vid),
        .rst_dst            (rst_vid),
        .frame_tick_dst     (frame_tick_vid),
        .snap_dst_committed (snap_vid_committed),
        .commit_pulse_dst   (snap_commit_pulse_vid)
    );

endmodule

`default_nettype wire