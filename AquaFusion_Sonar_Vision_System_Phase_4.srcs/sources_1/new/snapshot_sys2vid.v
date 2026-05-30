`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// snapshot_sys2vid
//------------------------------------------------------------------------------
// ROLE
//   Snapshot transfer and frame-boundary commit block from a source clock
//   domain into a destination/video clock domain.
//
// HIGH-LEVEL PURPOSE
//   This module transfers a wide snapshot bus, `snap_src`, from `clk_src` into
//   `clk_dst` using a request/acknowledge toggle handshake. However, it does
//   not immediately expose newly received data to the public destination output.
//
//   Instead, the destination side uses a two-stage policy:
//
//     1) newly arrived data is first captured into a pending register
//     2) that pending data is committed to the public output only when
//        `frame_tick_dst` occurs
//
//   This allows the destination domain, especially a raster/video system, to
//   update visible state only on explicit frame boundaries.
//
// WHY THIS MODULE EXISTS
//   In mixed-clock designs, especially video systems, two problems must be
//   solved simultaneously:
//
//   Problem A: clock-domain crossing
//     A snapshot created in one clock domain must be moved into another domain
//     without naive direct sampling of event signals.
//
//   Problem B: visual coherence
//     Even after new data has arrived in the video domain, it may be undesirable
//     to expose it immediately. If visible state changes in the middle of a
//     frame, the result can be tearing or inconsistent on-screen overlays.
//
//   This module solves both:
//
//     - request/ack toggles handle the event transfer
//     - pending/commit staging handles frame-coherent publication
//
// CONCEPTUAL MODEL
//   The source side behaves like a publisher:
//
//     "A new snapshot is ready. Hold it stable. Notify the destination."
//
//   The destination side behaves like a receiver plus staging buffer:
//
//     "A new snapshot arrived. Capture it as pending. Wait until the next frame
//      boundary. Then publish it."
//
//   Finally, the destination notifies the source:
//
//     "The pending copy has been captured. The source may now accept another
//      update."
//
// SOURCE-DOMAIN CONTRACT
//   Inputs:
//     clk_src
//       Source clock domain.
//
//     rst_src
//       Source-domain reset.
//
//     snap_src
//       Current source snapshot value.
//
//     snap_upd_src
//       One-cycle or event-like indication that `snap_src` should be published.
//
//   Source behavior:
//     - If not busy, a rising event on snap_upd_src causes the source to:
//         * copy snap_src into src_hold_reg
//         * toggle req_tgl_src
//         * mark itself busy
//     - While busy, new source updates are ignored
//     - Busy clears only after the destination acknowledges capture
//
// DESTINATION-DOMAIN CONTRACT
//   Inputs:
//     clk_dst
//       Destination/video clock domain.
//
//     rst_dst
//       Destination-domain reset.
//
//     frame_tick_dst
//       Frame-boundary event. This is the only time newly pending data becomes
//       visible on snap_dst_committed.
//
//   Outputs:
//     snap_dst_committed
//       Public destination-domain snapshot. This is the frame-stable output
//       intended for renderers or other frame-coherent consumers.
//
//     commit_pulse_dst
//       One-cycle pulse indicating that a pending snapshot has just been
//       committed to snap_dst_committed.
//
// HANDSHAKE PHILOSOPHY
//   The transfer uses toggles rather than single-cycle pulses across domains.
//
//   Why toggles?
//   Because a level transition can be synchronized and then compared against a
//   remembered prior value. This is a standard and robust way to communicate
//   discrete events across clock domains.
//
//   Request direction:
//     Source toggles req_tgl_src when new snapshot data is available.
//
//   Acknowledge direction:
//     Destination toggles ack_tgl_dst after it has captured the snapshot into
//     its pending register.
//
//   Each side detects "new event happened" by comparing the synchronized toggle
//   value against a locally remembered previous value.
//
// INTERNAL REGISTERS BY ROLE
//
//   Source side:
//     src_hold_reg
//       Stable source-side storage for the snapshot being transferred.
//
//     req_tgl_src
//       Source request toggle.
//
//     src_busy
//       Prevents overwriting src_hold_reg until the current transfer has been
//       acknowledged.
//
//     ack_sync1_src, ack_sync2_src
//       Two-stage synchronization of destination ack toggle back into the source
//       domain.
//
//     ack_seen_src
//       Source-side remembered copy of the last acknowledged toggle value.
//
//   Destination side:
//     req_sync1_dst, req_sync2_dst
//       Two-stage synchronization of source request toggle into the destination
//       domain.
//
//     req_seen_dst
//       Destination-side remembered copy of the last processed request toggle.
//
//     pending_dst
//       Destination-side staging register holding newly received snapshot data.
//
//     pending_valid_dst
//       Indicates that pending_dst contains a newly arrived snapshot that has
//       not yet been committed publicly.
//
//     ack_tgl_dst
//       Destination-generated acknowledge toggle.
//
// PUBLICATION POLICY
//   This module intentionally separates:
//
//     arrival in destination domain
//       from
//     public visibility in destination domain
//
//   Arrival happens when a new request toggle is detected.
//   Public visibility happens later, only when frame_tick_dst is asserted.
//
//   That separation is exactly what makes the module suitable for video/HUD
//   systems where frame-stable state is essential.
//
// IMPORTANT SOURCE-SIDE FLOW CONTROL
//   The source side accepts a new update only when `src_busy == 0`.
//
//   Therefore, if multiple snap_upd_src events occur before the destination has
//   acknowledged the previous transfer, the later updates are ignored.
//
//   This means the module implements a "single outstanding transfer" policy,
//   not an unbounded queue.
//
// IMPORTANT DATA-HOLD ASSUMPTION
//   The bus `src_hold_reg` is written in the source domain and then sampled by
//   the destination domain after the request toggle is synchronized.
//
//   The correctness of this style depends on the handshake discipline:
//
//     - source copies snapshot into src_hold_reg
//     - source toggles req_tgl_src
//     - source keeps src_hold_reg unchanged while src_busy is asserted
//     - destination captures src_hold_reg before acknowledging
//
//   In other words, the source data bus is held stable for the duration of the
//   handshake. This is the intended contract of the module.
//
// RESET BEHAVIOR
//   On rst_src:
//     - source holding register cleared
//     - request toggle cleared
//     - source busy cleared
//     - synchronized ack path cleared
//
//   On rst_dst:
//     - synchronized request path cleared
//     - pending register and pending-valid cleared
//     - committed output cleared
//     - commit pulse cleared
//     - ack toggle cleared
//
// PEDAGOGICAL SUMMARY
//   The module can be understood in five conceptual steps:
//
//     Step 1: Source sees a new snapshot and stores it in src_hold_reg
//     Step 2: Source toggles req_tgl_src to announce the new snapshot
//     Step 3: Destination synchronizes the request toggle and detects it
//     Step 4: Destination copies src_hold_reg into pending_dst and toggles ack
//     Step 5: On frame_tick_dst, destination commits pending_dst to the public
//             output and emits commit_pulse_dst
//
//   Meanwhile, the source observes the synchronized ack toggle and clears busy,
//   allowing the next snapshot transfer to begin.
//------------------------------------------------------------------------------
module snapshot_sys2vid #(
    //--------------------------------------------------------------------------
    // Snapshot bus width.
    //--------------------------------------------------------------------------
    parameter integer W = 64
)(
    //--------------------------------------------------------------------------
    // Source-domain interface
    //--------------------------------------------------------------------------
    input  wire         clk_src,
    input  wire         rst_src,
    input  wire [W-1:0] snap_src,
    input  wire         snap_upd_src,

    //--------------------------------------------------------------------------
    // Destination-domain interface
    //--------------------------------------------------------------------------
    input  wire         clk_dst,
    input  wire         rst_dst,
    input  wire         frame_tick_dst,
    output reg  [W-1:0] snap_dst_committed,
    output reg          commit_pulse_dst
);

    //==========================================================================
    // Source-domain hold register and request toggle
    //--------------------------------------------------------------------------
    // src_hold_reg
    //   Holds the source snapshot stable while the request is in flight.
    //
    // req_tgl_src
    //   Toggles each time a new snapshot transfer is launched.
    //
    // src_busy
    //   Indicates that a transfer is outstanding and the source must not accept
    //   another snapshot update yet.
    //==========================================================================
    reg [W-1:0] src_hold_reg;
    reg         req_tgl_src;
    reg         src_busy;

    //==========================================================================
    // Acknowledge synchronization back to source
    //--------------------------------------------------------------------------
    // ack_tgl_dst is generated in the destination domain, so it must be
    // synchronized before the source uses it.
    //
    // ack_sync1_src, ack_sync2_src
    //   Two-stage synchronizer for ack_tgl_dst into clk_src.
    //
    // ack_seen_src
    //   Remembers the last acknowledged toggle value already processed by the
    //   source side.
    //==========================================================================
    reg ack_sync1_src, ack_sync2_src;
    reg ack_seen_src;

    //==========================================================================
    // Request synchronization into destination
    //--------------------------------------------------------------------------
    // req_tgl_src is generated in the source domain, so it must be synchronized
    // into clk_dst.
    //
    // req_sync1_dst, req_sync2_dst
    //   Two-stage synchronizer for req_tgl_src into clk_dst.
    //
    // req_seen_dst
    //   Remembers the last request toggle value already processed by the
    //   destination side.
    //==========================================================================
    reg req_sync1_dst, req_sync2_dst;
    reg req_seen_dst;

    //==========================================================================
    // Destination-side pending register
    //--------------------------------------------------------------------------
    // pending_dst
    //   Holds newly received snapshot data after the handshake has delivered it
    //   into the destination domain, but before frame-boundary commit.
    //
    // pending_valid_dst
    //   Indicates that pending_dst contains uncommitted data waiting for the
    //   next frame tick.
    //==========================================================================
    reg [W-1:0] pending_dst;
    reg         pending_valid_dst;

    //==========================================================================
    // Destination-generated acknowledge toggle
    //--------------------------------------------------------------------------
    // Toggled when the destination has successfully captured the source-held
    // snapshot into its pending register.
    //==========================================================================
    reg ack_tgl_dst;

    //==========================================================================
    // Source-domain process
    //--------------------------------------------------------------------------
    // Responsibilities:
    //   1) synchronize ack toggle back from destination
    //   2) clear source busy when a new ack is observed
    //   3) accept a new snapshot only if not currently busy
    //   4) when accepting a new snapshot:
    //        - copy snap_src into src_hold_reg
    //        - toggle req_tgl_src
    //        - mark source busy
    //
    // Step-by-step interpretation:
    //
    //   A) First, always advance the two-stage ack synchronizer.
    //
    //   B) If the source is busy and the synchronized ack toggle differs from
    //      ack_seen_src, then the destination has completed capture of the
    //      outstanding request. Therefore:
    //        - remember the new ack value
    //        - clear src_busy
    //
    //   C) If a new source update event arrives and the source is not busy:
    //        - latch the source snapshot into src_hold_reg
    //        - toggle req_tgl_src to signal a new request event
    //        - set src_busy to block further launches until acknowledged
    //
    // Note:
    //   If snap_upd_src arrives while src_busy is already high, that update is
    //   ignored by the current implementation.
    //==========================================================================
    always @(posedge clk_src) begin
        if (rst_src) begin
            src_hold_reg  <= {W{1'b0}};
            req_tgl_src   <= 1'b0;
            src_busy      <= 1'b0;
            ack_sync1_src <= 1'b0;
            ack_sync2_src <= 1'b0;
            ack_seen_src  <= 1'b0;
        end else begin
            //------------------------------------------------------------------
            // Step 1: synchronize destination ack toggle back into clk_src
            //------------------------------------------------------------------
            ack_sync1_src <= ack_tgl_dst;
            ack_sync2_src <= ack_sync1_src;

            //------------------------------------------------------------------
            // Step 2: detect newly arrived ack event
            //
            // If source is busy and the synchronized ack toggle has changed
            // relative to the last seen value, then the in-flight transfer has
            // been accepted by the destination.
            //------------------------------------------------------------------
            if (src_busy && (ack_sync2_src != ack_seen_src)) begin
                ack_seen_src <= ack_sync2_src;
                src_busy     <= 1'b0;
            end

            //------------------------------------------------------------------
            // Step 3: launch a new transfer if a new snapshot update arrives and
            //         no transfer is currently outstanding
            //
            // Ordering note:
            //   src_hold_reg is updated in the same cycle as req_tgl_src is
            //   toggled. The protocol relies on src_hold_reg remaining stable
            //   thereafter while src_busy is asserted.
            //------------------------------------------------------------------
            if (snap_upd_src && !src_busy) begin
                src_hold_reg <= snap_src;
                req_tgl_src  <= ~req_tgl_src;
                src_busy     <= 1'b1;
            end
        end
    end

    //==========================================================================
    // Destination-domain process
    //--------------------------------------------------------------------------
    // Responsibilities:
    //   1) synchronize request toggle from source
    //   2) detect newly arrived request event
    //   3) capture source-held snapshot into pending_dst
    //   4) acknowledge the capture back to the source
    //   5) commit pending snapshot to public output on frame_tick_dst
    //
    // Step-by-step interpretation:
    //
    //   A) Clear commit_pulse_dst by default each destination clock cycle.
    //      It is a one-cycle pulse.
    //
    //   B) Advance the two-stage request synchronizer.
    //
    //   C) If a new synchronized request toggle is observed:
    //        - remember the new request value
    //        - copy src_hold_reg into pending_dst
    //        - mark pending_valid_dst
    //        - toggle ack_tgl_dst to tell the source capture has occurred
    //
    //   D) If a frame boundary occurs and pending_valid_dst is set:
    //        - move pending_dst into snap_dst_committed
    //        - clear pending_valid_dst
    //        - pulse commit_pulse_dst
    //
    // This means:
    //   arrival time and public-visibility time are intentionally decoupled.
    //==========================================================================
    always @(posedge clk_dst) begin
        if (rst_dst) begin
            req_sync1_dst      <= 1'b0;
            req_sync2_dst      <= 1'b0;
            req_seen_dst       <= 1'b0;
            pending_dst        <= {W{1'b0}};
            pending_valid_dst  <= 1'b0;
            snap_dst_committed <= {W{1'b0}};
            commit_pulse_dst   <= 1'b0;
            ack_tgl_dst        <= 1'b0;
        end else begin
            //------------------------------------------------------------------
            // Step 1: default pulse clearing
            //
            // commit_pulse_dst is a one-cycle event that marks the moment of
            // public commit.
            //------------------------------------------------------------------
            commit_pulse_dst <= 1'b0;

            //------------------------------------------------------------------
            // Step 2: synchronize source request toggle into clk_dst
            //------------------------------------------------------------------
            req_sync1_dst <= req_tgl_src;
            req_sync2_dst <= req_sync1_dst;

            //------------------------------------------------------------------
            // Step 3: detect newly arrived request event
            //
            // If the synchronized request toggle differs from the remembered
            // request state, a new transfer has arrived.
            //
            // Actions:
            //   - remember the new request toggle value
            //   - capture the stable source-held snapshot into pending_dst
            //   - mark pending_valid_dst
            //   - toggle ack_tgl_dst so the source may later clear busy
            //------------------------------------------------------------------
            if (req_sync2_dst != req_seen_dst) begin
                req_seen_dst      <= req_sync2_dst;
                pending_dst       <= src_hold_reg;
                pending_valid_dst <= 1'b1;
                ack_tgl_dst       <= ~ack_tgl_dst;
            end

            //------------------------------------------------------------------
            // Step 4: frame-boundary commit
            //
            // If a pending snapshot exists and the destination frame tick
            // occurs, make the pending value publicly visible.
            //
            // Actions:
            //   - copy pending_dst into snap_dst_committed
            //   - clear pending_valid_dst
            //   - pulse commit_pulse_dst
            //
            // This is the key frame-coherence step of the module.
            //------------------------------------------------------------------
            if (frame_tick_dst && pending_valid_dst) begin
                snap_dst_committed <= pending_dst;
                pending_valid_dst  <= 1'b0;
                commit_pulse_dst   <= 1'b1;
            end
        end
    end

endmodule

`default_nettype wire