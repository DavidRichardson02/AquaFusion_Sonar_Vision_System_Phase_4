`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// sonar_status_snapshot_sys
//------------------------------------------------------------------------------
// ROLE
//   Pack sonar measurement and status information into a SYS-domain held
//   snapshot register and emit a one-cycle update pulse whenever the published
//   snapshot meaningfully changes.
//
// PURPOSE
//   This module is the publication source for the legacy/basic sonar HUD tile.
//   It converts event-style and status-style sonar information into an explicit
//   snapshot/update pair suitable for clean SYS->VID transfer.
//
// EXTERNAL INTERFACE
//   inputs
//     clk
//     rst
//     distance_in
//     distance_valid
//     stale
//     timeout_err
//     parse_err_sticky_set
//     age_ticks
//     update_count
//
//   outputs
//     snap_data
//     snap_upd
//
// PUBLICATION POLICY
//   A new snapshot is published when any of the following occurs:
//
//     1) distance_valid pulse
//     2) stale transition
//     3) timeout_err transition
//     4) parse_err_sticky becomes asserted
//     5) update_count changes
//     6) coarse age-bucket changes
//
// WHY AGE IS BUCKETED
//   Republishing on every increment of age_ticks would create an unnecessarily
//   high publication rate. Instead, age_ticks is bucketed by AGE_BUCKET_SHIFT
//   for republish decisions, while the full current age_ticks value is still
//   packed into the snapshot at publication time.
//
// SNAPSHOT PACKING MAP
//   snap_data[ 9: 0] = published_distance
//   snap_data[10]    = measurement_present
//   snap_data[11]    = stale
//   snap_data[12]    = timeout_err
//   snap_data[13]    = parse_err_sticky
//   snap_data[29:14] = update_count
//   snap_data[45:30] = age_ticks
//   snap_data[SNAP_W-1:46] = 0 when SNAP_W > 46
//
// IMPORTANT BIT[10] SEMANTICS
//   Bit[10] is not "distance_valid this cycle".
//   It means:
//     "a valid measurement has been published at least once since reset"
//   This allows status-only republishes to keep the stored distance field marked
//   as meaningful.
//
// RESET BEHAVIOR
//   On reset:
//     - published state is cleared
//     - parse_err_sticky is cleared
//     - measurement_present is cleared
//     - change-detection history is cleared
//     - snap_data is cleared
//     - snap_upd is deasserted
//
// SYNTHESIS NOTES
//   - Verilog-2001 compatible
//   - Single clocked process
//   - No dynamic constructs
//==============================================================================

module sonar_status_snapshot_sys #(
    parameter integer SNAP_W           = 64,
    parameter integer AGE_BUCKET_SHIFT = 4
)(
    input  wire              clk,
    input  wire              rst,
    input  wire [9:0]        distance_in,
    input  wire              distance_valid,
    input  wire              stale,
    input  wire              timeout_err,
    input  wire              parse_err_sticky_set,
    input  wire [15:0]       age_ticks,
    input  wire [15:0]       update_count,
    output reg  [SNAP_W-1:0] snap_data,
    output reg               snap_upd
);

    //--------------------------------------------------------------------------
    // Published-state registers
    //--------------------------------------------------------------------------
    reg        parse_err_sticky;
    reg        measurement_present;
    reg [9:0]  published_distance;

    //--------------------------------------------------------------------------
    // Change-detection history
    //--------------------------------------------------------------------------
    reg        stale_prev;
    reg        timeout_prev;
    reg        parse_prev;
    reg [15:0] update_count_prev;
    reg [15:0] age_bucket_prev;

    //--------------------------------------------------------------------------
    // Sequential-scope temporaries
    //--------------------------------------------------------------------------
    reg        parse_err_sticky_next;
    reg        measurement_present_next;
    reg [9:0]  published_distance_next;

    reg [15:0] age_bucket_now;

    reg        publish_due_to_distance;
    reg        publish_due_to_stale_change;
    reg        publish_due_to_timeout_change;
    reg        publish_due_to_parse_change;
    reg        publish_due_to_count_change;
    reg        publish_due_to_age_bucket_change;
    reg        publish_now;

    reg [45:0] snap_core_next;

    always @(posedge clk) begin
        if (rst) begin
            parse_err_sticky     <= 1'b0;
            measurement_present  <= 1'b0;
            published_distance   <= 10'd0;

            stale_prev           <= 1'b0;
            timeout_prev         <= 1'b0;
            parse_prev           <= 1'b0;
            update_count_prev    <= 16'd0;
            age_bucket_prev      <= 16'd0;

            snap_data            <= {SNAP_W{1'b0}};
            snap_upd             <= 1'b0;
        end else begin
            //------------------------------------------------------------------
            // Default one-cycle pulse behavior
            //------------------------------------------------------------------
            snap_upd <= 1'b0;

            //------------------------------------------------------------------
            // STEP 1: next-state computation for latched published state
            //------------------------------------------------------------------
            parse_err_sticky_next    = parse_err_sticky | parse_err_sticky_set;
            measurement_present_next = measurement_present | distance_valid;
            published_distance_next  = published_distance;

            if (distance_valid)
                published_distance_next = distance_in;

            //------------------------------------------------------------------
            // STEP 2: coarse age bucket for republish policy
            //------------------------------------------------------------------
            age_bucket_now = (age_ticks >> AGE_BUCKET_SHIFT);

            //------------------------------------------------------------------
            // STEP 3: publication-cause decode
            //------------------------------------------------------------------
            publish_due_to_distance          = distance_valid;
            publish_due_to_stale_change      = (stale != stale_prev);
            publish_due_to_timeout_change    = (timeout_err != timeout_prev);
            publish_due_to_parse_change      = (parse_err_sticky_next != parse_prev);
            publish_due_to_count_change      = (update_count != update_count_prev);
            publish_due_to_age_bucket_change = (age_bucket_now != age_bucket_prev);

            publish_now =
                publish_due_to_distance          |
                publish_due_to_stale_change      |
                publish_due_to_timeout_change    |
                publish_due_to_parse_change      |
                publish_due_to_count_change      |
                publish_due_to_age_bucket_change;

            //------------------------------------------------------------------
            // STEP 4: commit sticky/local publication state
            //------------------------------------------------------------------
            parse_err_sticky    <= parse_err_sticky_next;
            measurement_present <= measurement_present_next;
            published_distance  <= published_distance_next;

            //------------------------------------------------------------------
            // STEP 5: assemble packed core snapshot from next-state values
            //------------------------------------------------------------------
            snap_core_next[9:0]   = published_distance_next;
            snap_core_next[10]    = measurement_present_next;
            snap_core_next[11]    = stale;
            snap_core_next[12]    = timeout_err;
            snap_core_next[13]    = parse_err_sticky_next;
            snap_core_next[29:14] = update_count;
            snap_core_next[45:30] = age_ticks;

            //------------------------------------------------------------------
            // STEP 6: publish snapshot and pulse update
            //------------------------------------------------------------------
            if (publish_now) begin
                if (SNAP_W <= 46)
                    snap_data <= snap_core_next[SNAP_W-1:0];
                else
                    snap_data <= {{(SNAP_W-46){1'b0}}, snap_core_next};

                snap_upd <= 1'b1;
            end

            //------------------------------------------------------------------
            // STEP 7: update history after publish decision
            //------------------------------------------------------------------
            stale_prev        <= stale;
            timeout_prev      <= timeout_err;
            parse_prev        <= parse_err_sticky_next;
            update_count_prev <= update_count;
            age_bucket_prev   <= age_bucket_now;
        end
    end

endmodule

`default_nettype wire