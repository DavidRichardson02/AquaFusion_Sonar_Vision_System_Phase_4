`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// camera_status_snapshot_sys
//------------------------------------------------------------------------------
// ROLE
//   Pack camera control-plane status into one stable SYS-domain snapshot bus.
//
// SNAPSHOT LAYOUT
//   [63:32] ms_counter
//   [31:16] step_code
//   [15]    idle flag        (~busy)
//   [14]    nack_sticky
//   [13]    switch_err
//   [12]    switch_done
//   [11]    sensor_id_ok
//   [10]    init_fail
//   [9]     init_done
//   [8]     busy
//   [7:0]   last_err
//==============================================================================

module camera_status_snapshot_sys #(
    parameter integer SNAP_W = 64
)(
    input  wire              clk,
    input  wire              rst,
    input  wire              busy,
    input  wire              init_done,
    input  wire              init_fail,
    input  wire              sensor_id_ok,
    input  wire              switch_done,
    input  wire              switch_err,
    input  wire              nack_sticky,
    input  wire [7:0]        last_err,
    input  wire [15:0]       step_code,
    input  wire [31:0]       ms_counter,
    output reg  [SNAP_W-1:0] snap_data,
    output reg               snap_upd
);

    reg [SNAP_W-1:0] snap_prev;
    wire [SNAP_W-1:0] snap_next;

    assign snap_next = {
        ms_counter,
        step_code,
        (~busy),
        nack_sticky,
        switch_err,
        switch_done,
        sensor_id_ok,
        init_fail,
        init_done,
        busy,
        last_err
    };

    always @(posedge clk) begin
        if (rst) begin
            snap_data <= {SNAP_W{1'b0}};
            snap_prev <= {SNAP_W{1'b0}};
            snap_upd  <= 1'b0;
        end else begin
            snap_data <= snap_next;
            snap_upd  <= (snap_next != snap_prev);
            snap_prev <= snap_next;
        end
    end

endmodule

`default_nettype wire

//
/*
`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// camera_status_snapshot_sys
//------------------------------------------------------------------------------
// PURPOSE
//   SYS-domain status snapshot packer for the camera-control subsystem.
//
//   This module collects selected camera-initialization status signals and
//   packs them into a single snapshot word, `snap_data`, accompanied by a
//   one-clock update pulse, `snap_upd`.
//
//   The snapshot is intended to provide a compact, coherent observability
//   interface for downstream consumers such as:
//
//     - a SYS->PIX CDC snapshot bridge
//     - a telemetry exporter
//     - a debug register block
//     - a status HUD / overlay
//
// ARCHITECTURAL ROLE
//   The camera-control subsystem contains many fine-grained internal signals:
//   completion flags, failure flags, SCCB transaction events, and diagnostic
//   state such as error codes and table indices.
//
//   Rather than exposing all such signals individually across subsystem
//   boundaries, this module produces a compact packed representation:
//       "current summarized subsystem status"
//
//   Thus, this block is not part of the control loop itself.
//   It is an observability / status-export block.
//
// PUBLISHING POLICY
//   A new snapshot update pulse is emitted when one or more of the following
//   occurs:
//
//     1) A successful SCCB transaction completes (`sccb_done`)
//     2) An SCCB acknowledge error occurs (`sccb_ack_error`)
//     3) A general SCCB error occurs (`sccb_err`)
//     4) One of the tracked top-level state fields changes:
//           - init_done
//           - init_fail
//           - sensor_id_ok
//           - busy
//           - last_err
//
//   This means the snapshot is not merely transaction-driven; it is also
//   state-change-driven.
//
// STICKY DIAGNOSTICS
//   Two sticky status flags are maintained internally:
//
//     ack_err_sticky
//       Once any SCCB acknowledge error occurs, this flag remains set until
//       reset.
//
//     sccb_err_sticky
//       Once any SCCB general error occurs, this flag remains set until reset.
//
//   These sticky flags ensure that transient error pulses remain visible in
//   later snapshots even if the original pulse was brief.
//
// TRANSACTION COUNTERS
//   The module maintains two 16-bit counters:
//
//     tx_count
//       Incremented when `sccb_done` occurs and the completed command was a
//       write transaction.
//
//     rx_count
//       Incremented when `sccb_done` occurs and the completed command was a
//       read transaction.
//
//   The transaction type is determined by `sccb_cmd_type` at the time of
//   `sccb_done`.
//
// IMPORTANT SEMANTIC ASSUMPTION
//   This module assumes that `sccb_cmd_type` remains valid and corresponds to
//   the command that is completing when `sccb_done` is asserted.
//
//   In other words:
//       when sccb_done = 1,
//       sccb_cmd_type must still identify whether the completed transaction was
//       a read or a write.
//
//   If the producer of `sccb_cmd_type` changes it too early, the counters could
//   be misclassified.
//
// SNAPSHOT FIELD MAP
//   The packed word currently uses bits [61:0] as follows:
//
//     snap_data[0]      = init_done
//     snap_data[1]      = init_fail
//     snap_data[2]      = sensor_id_ok
//     snap_data[3]      = busy
//     snap_data[4]      = sticky-or-current SCCB acknowledge error
//     snap_data[5]      = sticky-or-current SCCB general error
//     snap_data[13:6]   = last_err
//     snap_data[29:14]  = tx_count
//     snap_data[45:30]  = rx_count
//     snap_data[61:46]  = table_index_last
//     snap_data[SNAP_W-1:62] = zero, when SNAP_W > 62
//
// WIDTH CONTRACT
//   This implementation assumes SNAP_W >= 62 for the full field map to exist.
//   If a smaller width is used, the fixed assignments to bits [61:46], [45:30],
//   etc. become invalid.
//
//   Therefore, the practical contract should be:
//
//       SNAP_W >= 62
//
// RESET BEHAVIOR
//   On reset:
//     - sticky error flags are cleared
//     - transaction counters are cleared
//     - delayed state-tracking registers are cleared
//     - snapshot data is cleared
//     - snapshot update pulse is deasserted
//
// OUTPUT CONTRACT
//   snap_data
//     Registered packed status image.
//
//   snap_upd
//     One-clock pulse indicating that snap_data has been refreshed this cycle.
//
// DESIGN STYLE
//   - Single synchronous always block
//   - No combinational feedback
//   - Bounded work per cycle
//   - Snapshot publishing separated from control logic
//
//==============================================================================
module camera_status_snapshot_sys #(
    parameter integer SNAP_W = 64
)(
    input  wire                  clk,
    input  wire                  rst,

    //--------------------------------------------------------------------------
    // High-level subsystem state inputs
    //--------------------------------------------------------------------------
    input  wire                  init_done,
    input  wire                  init_fail,
    input  wire                  sensor_id_ok,
    input  wire                  busy,
    input  wire [7:0]            last_err,

    //--------------------------------------------------------------------------
    // SCCB transaction/event inputs
    //--------------------------------------------------------------------------
    input  wire                  sccb_done,
    input  wire                  sccb_err,
    input  wire                  sccb_ack_error,
    input  wire                  sccb_cmd_type,

    //--------------------------------------------------------------------------
    // Most recent completed/failed table step index from upstream logic
    //--------------------------------------------------------------------------
    input  wire [15:0]           table_index_last,

    //--------------------------------------------------------------------------
    // Packed snapshot outputs
    //--------------------------------------------------------------------------
    output reg  [SNAP_W-1:0]     snap_data,
    output reg                   snap_upd
);

    //--------------------------------------------------------------------------
    // Sticky diagnostic flags
    //--------------------------------------------------------------------------
    // These remain set until reset once the corresponding error has been seen.
    //--------------------------------------------------------------------------
    reg ack_err_sticky;
    reg sccb_err_sticky;

    //--------------------------------------------------------------------------
    // Transaction counters
    //--------------------------------------------------------------------------
    // tx_count counts completed writes.
    // rx_count counts completed reads.
    //--------------------------------------------------------------------------
    reg [15:0] tx_count;
    reg [15:0] rx_count;

    //--------------------------------------------------------------------------
    // Delayed copies of selected state fields
    //--------------------------------------------------------------------------
    // These registers hold the previous-cycle values of selected inputs so that
    // state changes can be detected by comparison against the current values.
    //--------------------------------------------------------------------------
    reg        init_done_d;
    reg        init_fail_d;
    reg        sensor_id_ok_d;
    reg        busy_d;
    reg [7:0]  last_err_d;

    //--------------------------------------------------------------------------
    // Combinational state-change detector
    //--------------------------------------------------------------------------
    // This expression becomes 1 whenever one or more of the tracked top-level
    // state fields differs from its value as sampled on the previous cycle.
    //
    // That is, this is a "change since last clock" detector for the chosen
    // fields, not a general edge detector for every signal in the subsystem.
    //--------------------------------------------------------------------------
    wire state_changed;
    assign state_changed =
        (init_done    != init_done_d)    ||
        (init_fail    != init_fail_d)    ||
        (sensor_id_ok != sensor_id_ok_d) ||
        (busy         != busy_d)         ||
        (last_err     != last_err_d);

    //--------------------------------------------------------------------------
    // Main sequential process
    //--------------------------------------------------------------------------
    // Step-by-step behavior on each rising edge:
    //
    //   1) If reset is asserted:
    //        - clear all internal state
    //        - clear snapshot outputs
    //
    //   2) Otherwise:
    //        - default snap_upd low (pulse-style output)
    //        - latch sticky error flags if new errors occurred
    //        - increment transaction counters on sccb_done
    //        - request a snapshot update if:
    //             a) a transaction completed
    //             b) an error pulse occurred
    //             c) tracked subsystem state changed
    //        - update delayed comparison registers
    //        - repack the snapshot data fields
    //
    // Note:
    //   snap_data is refreshed every cycle, but snap_upd pulses only on event/
    //   change cycles. This means snap_data is always a live registered image,
    //   while snap_upd indicates that the image has meaningfully changed.
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            //------------------------------------------------------------------
            // Clear sticky diagnostics.
            //------------------------------------------------------------------
            ack_err_sticky  <= 1'b0;
            sccb_err_sticky <= 1'b0;

            //------------------------------------------------------------------
            // Clear transaction counters.
            //------------------------------------------------------------------
            tx_count        <= 16'd0;
            rx_count        <= 16'd0;

            //------------------------------------------------------------------
            // Clear delayed state trackers.
            //------------------------------------------------------------------
            init_done_d     <= 1'b0;
            init_fail_d     <= 1'b0;
            sensor_id_ok_d  <= 1'b0;
            busy_d          <= 1'b0;
            last_err_d      <= 8'd0;

            //------------------------------------------------------------------
            // Clear published snapshot outputs.
            //------------------------------------------------------------------
            snap_data       <= {SNAP_W{1'b0}};
            snap_upd        <= 1'b0;
        end else begin
            //------------------------------------------------------------------
            // Default pulse behavior:
            // snap_upd is asserted only on cycles where a meaningful publish
            // event occurs.
            //------------------------------------------------------------------
            snap_upd <= 1'b0;

            //------------------------------------------------------------------
            // Sticky error accumulation.
            // These latches preserve transient error events beyond their source
            // pulse duration.
            //------------------------------------------------------------------
            if (sccb_ack_error)
                ack_err_sticky <= 1'b1;

            if (sccb_err)
                sccb_err_sticky <= 1'b1;

            //------------------------------------------------------------------
            // Transaction accounting.
            //
            // On a successful SCCB completion pulse:
            //   - classify the completed command as read or write
            //   - increment the corresponding counter
            //   - request a snapshot update pulse
            //
            // Convention assumed here:
            //   sccb_cmd_type == 1 --> read transaction
            //   sccb_cmd_type == 0 --> write transaction
            //
            // If the system uses the opposite convention, this block must be
            // updated accordingly.
            //------------------------------------------------------------------
            if (sccb_done) begin
                if (sccb_cmd_type)
                    rx_count <= rx_count + 16'd1;
                else
                    tx_count <= tx_count + 16'd1;

                snap_upd <= 1'b1;
            end

            //------------------------------------------------------------------
            // Additional publish conditions.
            //
            // A snapshot update pulse is also requested if:
            //   - tracked high-level state changed
            //   - an SCCB acknowledge error occurred
            //   - a general SCCB error occurred
            //
            // Since nonblocking assignments are used, multiple assignments to
            // snap_upd within this clocked block resolve naturally: if any
            // branch assigns 1'b1, the final next-state value becomes 1.
            //------------------------------------------------------------------
            if (state_changed || sccb_ack_error || sccb_err)
                snap_upd <= 1'b1;

            //------------------------------------------------------------------
            // Update delayed copies of tracked state fields.
            //
            // These become the "previous" values used by state_changed on the
            // next clock cycle.
            //------------------------------------------------------------------
            init_done_d    <= init_done;
            init_fail_d    <= init_fail;
            sensor_id_ok_d <= sensor_id_ok;
            busy_d         <= busy;
            last_err_d     <= last_err;

            //------------------------------------------------------------------
            // Pack the registered snapshot image.
            //
            // Important detail:
            //   For the sticky error fields, the snapshot uses the OR of:
            //     - the already-stored sticky flag
            //     - the current-cycle error pulse
            //
            // This ensures that the snapshot reflects an error immediately on
            // the same cycle that the pulse first appears, rather than waiting
            // one extra cycle for the sticky register to update.
            //------------------------------------------------------------------
            snap_data[0]     <= init_done;
            snap_data[1]     <= init_fail;
            snap_data[2]     <= sensor_id_ok;
            snap_data[3]     <= busy;
            snap_data[4]     <= ack_err_sticky  | sccb_ack_error;
            snap_data[5]     <= sccb_err_sticky | sccb_err;
            snap_data[13:6]  <= last_err;
            snap_data[29:14] <= tx_count;
            snap_data[45:30] <= rx_count;
            snap_data[61:46] <= table_index_last;

            //------------------------------------------------------------------
            // Zero-fill unused upper bits when the snapshot width exceeds the
            // currently defined field map.
            //------------------------------------------------------------------
            if (SNAP_W > 62)
                snap_data[SNAP_W-1:62] <= {(SNAP_W-62){1'b0}};
        end
    end

endmodule

`default_nettype wire
*/