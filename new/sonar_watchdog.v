`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// sonar_watchdog
//------------------------------------------------------------------------------
// ROLE
//   Freshness / aging / timeout tracker for sonar measurement updates.
//
// HIGH-LEVEL PURPOSE
//   This module observes a pulse stream:
//
//       distance_valid_pulse
//
//   where each pulse means:
//
//       "a new valid sonar measurement has just been accepted"
//
//   From that event stream, the module derives longer-lived status signals:
//
//       stale
//         Indicates that the time since the last valid update has become too
//         large.
//
//       age_ticks
//         Counts elapsed milliseconds since the most recent valid update.
//
//       timeout_err
//         Indicates that a stale/timeout-class condition has occurred.
//
//       update_count
//         Counts how many valid updates have been observed.
//
// WHY THIS MODULE EXISTS
//   A raw valid-data pulse is only a momentary event. Many downstream consumers
//   need a more persistent description of measurement freshness.
//
//   For example:
//
//     - a HUD may need to display "sample is stale"
//     - fusion logic may need to reject outdated measurements
//     - debug telemetry may need to show measurement age numerically
//     - health logic may need to know whether updates ever arrived at all
//
//   This module turns pulse events into explicit state.
//
// CONCEPTUAL MODEL
//   The module behaves like a stopwatch plus threshold detector:
//
//     1) Every time a valid sonar update arrives, the stopwatch is reset.
//     2) While no new update arrives, the stopwatch advances in milliseconds.
//     3) Once the elapsed time reaches the configured stale threshold,
//        the module marks the data as stale.
//     4) A timeout-class error flag is also asserted when that threshold is
//        reached.
//
// SIGNAL SEMANTICS
//   clk
//     Local synchronous clock used for timing and state updates.
//
//   rst
//     Active-high synchronous reset.
//
//   distance_valid_pulse
//     One-cycle pulse indicating that a new valid sonar sample has arrived.
//
//   stale
//     Freshness indicator.
//     Low means recent/fresh.
//     High means the elapsed time since the last valid sample has crossed the
//     stale threshold.
//
//   age_ticks
//     Millisecond-scale age of the current measurement stream.
//     Interpreted as the elapsed time, in ms, since the last valid update.
//
//   timeout_err
//     Timeout-class error indication.
//
//     IMPORTANT PRESENT-BEHAVIOR NOTE:
//       In the current implementation, once timeout_err becomes 1, it is not
//       cleared by a later valid sample. Therefore, it behaves as a sticky
//       fault latch after the first stale event.
//
//   update_count
//     Total number of valid updates observed since reset.
//
// PARAMETER SEMANTICS
//   CLK_HZ
//     Local clock frequency in hertz.
//
//   STALE_MS
//     Threshold, in milliseconds, at or above which the sample stream is
//     considered stale.
//
// DERIVED CONSTANT
//   TICKS_PER_MS = CLK_HZ / 1000
//
//   This gives the number of clk cycles corresponding to one millisecond.
//   A small divider counter, `ms_div`, uses this constant to convert the fast
//   FPGA clock into millisecond aging ticks.
//
// INTERNAL TIMING STRUCTURE
//   The module contains:
//
//     ms_div
//       A cycle counter that counts clk cycles until one millisecond has
//       elapsed.
//
//   Once `ms_div` reaches `TICKS_PER_MS - 1`, one millisecond is considered to
//   have passed, so:
//
//     - ms_div is reset to zero
//     - age_ticks is incremented, unless it is already saturated at 16'hFFFF
//
// RESET BEHAVIOR
//   On rst assertion:
//
//       ms_div       <= 0
//       stale        <= 1
//       age_ticks    <= 16'hFFFF
//       timeout_err  <= 0
//       update_count <= 0
//
//   This reset policy is intentionally conservative:
//
//     - stale starts asserted
//     - age_ticks starts saturated
//     - no successful updates are assumed yet
//
//   Engineering meaning:
//     Immediately after reset, the system should interpret the sonar stream as
//     "not yet fresh" until a real valid measurement pulse arrives.
//
// VALID-PULSE BEHAVIOR
//   When distance_valid_pulse is asserted:
//
//       ms_div       <= 0
//       age_ticks    <= 0
//       stale        <= 0
//       update_count <= update_count + 1
//
//   So a new valid sample:
//
//     - resets the elapsed-time measurement
//     - marks the data fresh
//     - increments the lifetime count of accepted updates
//
//   IMPORTANT NOTE:
//     timeout_err is *not* cleared in this branch.
//
//     Therefore, if timeout_err had previously been asserted, it remains
//     asserted even after new valid data arrives.
//
// NO-UPDATE BEHAVIOR
//   When distance_valid_pulse is not asserted:
//
//     1) ms_div counts clk cycles.
//     2) Each completed millisecond increments age_ticks, unless saturated.
//     3) If age_ticks is at or above STALE_MS, then:
//          stale       <= 1
//          timeout_err <= 1
//
// STALE-THRESHOLD COMPARISON
//   The stale condition is tested using:
//
//       if (age_ticks >= STALE_MS[15:0])
//
//   Therefore, once age_ticks reaches the stale threshold, the module asserts
//   stale and timeout_err.
//
//   Because the comparison is >= rather than ==, the stale condition remains
//   active for all later ages as well.
//
// SATURATION BEHAVIOR
//   age_ticks stops incrementing once it reaches 16'hFFFF.
//
//   This is a saturation policy rather than wraparound.
//   That is a good choice for age counters because wraparound would incorrectly
//   make very old data appear young again.
//
// DESIGN PHILOSOPHY
//   This module separates:
//
//     event-level information:
//       distance_valid_pulse
//
//   from:
//
//     stateful health information:
//       stale, age_ticks, timeout_err, update_count
//
//   That separation is valuable because most downstream logic reasons more
//   naturally about freshness state than about isolated pulses.
//
// IMPORTANT CURRENT SEMANTICS
//   The present implementation yields the following behavior:
//
//     - stale is a live freshness indicator
//     - age_ticks is a live age measure
//     - update_count is a cumulative live counter
//     - timeout_err is sticky after first timeout-class assertion
//
//   That behavior may be exactly what is desired, or it may motivate a later
//   refinement depending on system-level fault semantics.
//------------------------------------------------------------------------------
module sonar_watchdog #(
    //--------------------------------------------------------------------------
    // Local FPGA clock frequency in hertz.
    //--------------------------------------------------------------------------
    parameter integer CLK_HZ   = 100_000_000,

    //--------------------------------------------------------------------------
    // Freshness timeout threshold in milliseconds.
    //
    // Once age_ticks reaches or exceeds this value, the stream is considered
    // stale.
    //--------------------------------------------------------------------------
    parameter integer STALE_MS = 200
)(
    //--------------------------------------------------------------------------
    // Local synchronous clock.
    //--------------------------------------------------------------------------
    input  wire        clk,

    //--------------------------------------------------------------------------
    // Active-high synchronous reset.
    //--------------------------------------------------------------------------
    input  wire        rst,

    //--------------------------------------------------------------------------
    // One-cycle pulse indicating that a new valid sonar measurement has arrived.
    //--------------------------------------------------------------------------
    input  wire        distance_valid_pulse,

    //--------------------------------------------------------------------------
    // Live freshness indicator.
    //
    // 0 -> recent/fresh
    // 1 -> stale
    //--------------------------------------------------------------------------
    output reg         stale,

    //--------------------------------------------------------------------------
    // Elapsed age in milliseconds since the most recent valid update.
    //
    // Saturates at 16'hFFFF.
    //--------------------------------------------------------------------------
    output reg [15:0]  age_ticks,

    //--------------------------------------------------------------------------
    // Timeout-class error indicator.
    //
    // IMPORTANT:
    //   In the present implementation this behaves as a sticky flag after the
    //   first stale event, because no later branch clears it.
    //--------------------------------------------------------------------------
    output reg         timeout_err,

    //--------------------------------------------------------------------------
    // Count of valid updates observed since reset.
    //--------------------------------------------------------------------------
    output reg [15:0]  update_count
);

    //==========================================================================
    // Clock-to-millisecond conversion
    //--------------------------------------------------------------------------
    // Meaning:
    //   Number of local clk cycles corresponding to one millisecond.
    //
    // Example:
    //   If CLK_HZ = 100,000,000, then:
    //
    //       TICKS_PER_MS = 100,000,000 / 1000 = 100,000
    //
    //   So 100,000 clk cycles correspond to one millisecond.
    //==========================================================================
    localparam integer TICKS_PER_MS = CLK_HZ / 1000;

    //==========================================================================
    // Millisecond divider counter
    //--------------------------------------------------------------------------
    // Purpose:
    //   Count raw clk cycles until one millisecond has elapsed.
    //
    // Behavior:
    //   - increments while waiting
    //   - resets to zero when a full ms has elapsed
    //   - also resets to zero when a fresh valid sample arrives
    //==========================================================================
    reg [31:0] ms_div;

    //==========================================================================
    // Freshness / age / timeout sequential process
    //--------------------------------------------------------------------------
    // Top-level structure:
    //
    //   A) Reset handling
    //   B) Fresh valid-sample handling
    //   C) No-new-sample aging behavior
    //
    // The essential idea is:
    //   - a valid pulse refreshes the watchdog state
    //   - absence of valid pulses allows age to accumulate
    //   - excessive age causes stale/timeout indication
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            //------------------------------------------------------------------
            // Reset case
            //
            // Step-by-step:
            //   1) Clear the millisecond divider.
            //   2) Mark the data stale immediately.
            //   3) Set age_ticks to saturated all-ones.
            //   4) Clear timeout_err.
            //   5) Clear update_count.
            //
            // Engineering meaning:
            //   After reset, the system assumes that no trustworthy recent sonar
            //   sample exists yet. Freshness must be earned by an actual
            //   distance_valid_pulse.
            //------------------------------------------------------------------
            ms_div       <= 32'd0;
            stale        <= 1'b1;
            age_ticks    <= 16'hFFFF;
            timeout_err  <= 1'b0;
            update_count <= 16'd0;
        end else begin
            //------------------------------------------------------------------
            // Case 1: a new valid sample has arrived
            //
            // Step-by-step:
            //   1) Reset the millisecond divider.
            //   2) Reset age_ticks to zero because the sample is brand new.
            //   3) Clear stale because the stream is now fresh.
            //   4) Increment the cumulative update counter.
            //
            // Important present-behavior note:
            //   timeout_err is not assigned in this branch, so any previous
            //   asserted value remains asserted.
            //------------------------------------------------------------------
            if (distance_valid_pulse) begin
                ms_div       <= 32'd0;
                age_ticks    <= 16'd0;
                stale        <= 1'b0;
                update_count <= update_count + 16'd1;
            end else begin
                //----------------------------------------------------------------
                // Case 2: no new valid sample this cycle
                //
                // The watchdog must now age the stream.
                //
                // Two sub-operations happen here:
                //
                //   A) convert raw clk cycles into millisecond ticks
                //   B) compare age_ticks against the stale threshold
                //----------------------------------------------------------------

                //--------------------------------------------------------------
                // A) Millisecond divider / age increment logic
                //
                // Step-by-step:
                //   1) Check whether ms_div has reached the final cycle needed
                //      for one millisecond.
                //   2) If yes:
                //        - reset ms_div to zero
                //        - increment age_ticks unless already saturated
                //   3) If no:
                //        - increment ms_div only
                //
                // Saturation policy:
                //   age_ticks does not wrap. Once 16'hFFFF is reached, the value
                //   remains there permanently until a valid update resets it.
                //--------------------------------------------------------------
                if (ms_div == TICKS_PER_MS-1) begin
                    ms_div <= 32'd0;
                    if (age_ticks != 16'hFFFF)
                        age_ticks <= age_ticks + 16'd1;
                end else begin
                    ms_div <= ms_div + 32'd1;
                end

                //--------------------------------------------------------------
                // B) Stale / timeout threshold test
                //
                // Step-by-step:
                //   1) Compare the current age_ticks against STALE_MS.
                //   2) If age_ticks is at or above the threshold:
                //        - assert stale
                //        - assert timeout_err
                //
                // Important sequencing note:
                //   This comparison uses the current registered age_ticks value
                //   from the beginning of the clock edge, not the incremented
                //   value being scheduled in the same cycle via nonblocking
                //   assignment. Thus, the stale transition is aligned to the
                //   registered age value as observed cycle-by-cycle.
                //
                // Important present-behavior note:
                //   timeout_err is asserted here, but nowhere in the non-reset
                //   logic is it later cleared. Therefore it behaves as sticky
                //   fault state after first assertion.
                //--------------------------------------------------------------
                if (age_ticks >= STALE_MS[15:0]) begin
                    stale       <= 1'b1;
                    timeout_err <= 1'b1;
                end
            end
        end
    end

endmodule

`default_nettype wire