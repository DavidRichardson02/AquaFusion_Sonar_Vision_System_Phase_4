`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// heartbeat_gen
//------------------------------------------------------------------------------
// ROLE
//   Low-frequency heartbeat / liveness indicator generator.
//
// HIGH-LEVEL PURPOSE
//   This module converts a fast synchronous input clock into a much slower
//   output signal, `heartbeat_o`, that periodically toggles state.
//
//   The result is a visible or measurable "heartbeat" signal that can be used
//   for:
//
//     - board bring-up confirmation
//     - coarse liveness indication
//     - debug probing with LEDs, GPIO pins, or instruments
//     - verifying that the clocked system is still running
//
// WHY THIS MODULE EXISTS
//   In FPGA and digital systems, the primary system clock is often too fast to
//   observe directly with the naked eye or to use conveniently as a human-scale
//   status signal.
//
//   For example, a 100 MHz clock changes every 10 ns, which is far too fast
//   for a board LED or a casual probe-based sanity check.
//
//   A heartbeat generator solves this by dividing the clock down to a slow,
//   easily observable square wave.
//
// SIGNAL SEMANTICS
//   clk
//     Input clock from which the heartbeat timing is derived.
//
//   rst
//     Active-high synchronous reset.
//     Clears the internal divider counter and forces the heartbeat output low.
//
//   heartbeat_o
//     Output heartbeat signal.
//     This signal toggles at the programmed toggle rate and therefore forms a
//     square wave whose full-cycle frequency is lower than the input clock by a
//     large deterministic factor.
//
// PARAMETER SEMANTICS
//   CLK_HZ
//     Input clock frequency in hertz.
//
//   TOGGLE_HZ
//     Requested heartbeat toggle rate in toggles per second.
//
// IMPORTANT FREQUENCY INTERPRETATION
//   The parameter TOGGLE_HZ specifies how often `heartbeat_o` changes state,
//   not how often a full 0->1->0 cycle completes.
//
//   Because one complete square-wave period contains two toggles:
//
//       low -> high   (toggle 1)
//       high -> low   (toggle 2)
//
//   the full output waveform frequency is:
//
//       heartbeat frequency = TOGGLE_HZ / 2
//
//   Equivalently:
//
//       toggle interval = 1 / TOGGLE_HZ seconds
//
// WHY THE DIVISOR CONTAINS A FACTOR OF 2
//   The internal divisor is defined as:
//
//       DIVISOR = CLK_HZ / (2 * TOGGLE_HZ)
//
//   In the present code, the output toggles whenever the counter reaches
//   DIVISOR - 1.
//
//   Since a full square-wave cycle requires two toggles, the factor of 2 in the
//   divisor is intended to account for the fact that:
//
//       one toggle = half of one full output period
//
//   Therefore:
//
//       half-period in input-clock cycles = CLK_HZ / (2 * output_frequency)
//
// PEDAGOGICAL READING OF THE CURRENT IMPLEMENTATION
//   The internal behavior can be understood as:
//
//     1) Count input clock cycles using `ctr`
//     2) When enough cycles have elapsed to represent one heartbeat interval,
//        toggle the output
//     3) Reset the counter and repeat forever
//
// INTERNAL STATE
//   ctr
//     32-bit cycle counter that measures elapsed clk cycles since the previous
//     heartbeat toggle.
//
//   DIVISOR
//     Compile-time constant that determines how many clk cycles must elapse
//     before the heartbeat output changes state.
//
// RESET BEHAVIOR
//   On rst assertion:
//
//       ctr         <= 0
//       heartbeat_o <= 0
//
//   This means:
//     - the counter restarts from zero
//     - the heartbeat output restarts in a known low state
//
// TIMING BEHAVIOR
//   On each rising edge of clk:
//
//     Case 1: rst is asserted
//       Clear the counter and force the heartbeat low.
//
//     Case 2: ctr has reached DIVISOR - 1
//       - clear ctr back to zero
//       - invert heartbeat_o
//
//     Case 3: otherwise
//       - increment ctr by one
//       - leave heartbeat_o unchanged
//
// DETERMINISM
//   The output frequency is fully deterministic under the assumptions that:
//
//     - clk is stable at CLK_HZ
//     - the parameter values are valid
//
// LIMITATIONS / ASSUMPTIONS
//   1) Integer division truncation
//      DIVISOR is computed using integer arithmetic, so if CLK_HZ is not an
//      exact multiple of (2 * TOGGLE_HZ), the realized timing will be the
//      nearest lower integer-cycle approximation implied by truncation.
//
//   2) Parameter validity
//      TOGGLE_HZ must be nonzero.
//      Also, CLK_HZ must be large enough that DIVISOR is at least 1.
//
//   3) Counter width
//      The counter is fixed at 32 bits. This is ample for many practical FPGA
//      heartbeat uses, but extremely low toggle rates with very large clocks
//      should still be sanity-checked.
//
// DESIGN PHILOSOPHY
//   This module intentionally implements a very plain divider-based heartbeat.
//   It avoids unnecessary complexity so that liveness remains easy to verify,
//   reason about, and trust during bring-up.
//------------------------------------------------------------------------------
module heartbeat_gen #(
    //--------------------------------------------------------------------------
    // Input clock frequency in hertz.
    //
    // Example:
    //   100_000_000 for a 100 MHz system clock.
    //--------------------------------------------------------------------------
    parameter integer CLK_HZ    = 100_000_000,

    //--------------------------------------------------------------------------
    // Requested toggle rate in toggles per second.
    //
    // Important:
    //   This is a toggle rate, not a full square-wave frequency.
    //--------------------------------------------------------------------------
    parameter integer TOGGLE_HZ = 2
)(
    //--------------------------------------------------------------------------
    // Input clock.
    //
    // All timing inside this module is measured in cycles of this clock.
    //--------------------------------------------------------------------------
    input  wire clk,

    //--------------------------------------------------------------------------
    // Active-high synchronous reset.
    //
    // Resets the internal divider counter and forces heartbeat_o low.
    //--------------------------------------------------------------------------
    input  wire rst,

    //--------------------------------------------------------------------------
    // Heartbeat output.
    //
    // This signal toggles periodically and can be used as a coarse liveness
    // indicator.
    //--------------------------------------------------------------------------
    output reg  heartbeat_o
);

    //==========================================================================
    // Divider constant
    //--------------------------------------------------------------------------
    // Meaning:
    //   Number of input-clock cycles corresponding to one half-period interval
    //   of the heartbeat waveform, based on the chosen parameterization.
    //
    // Computation:
    //
    //   DIVISOR = CLK_HZ / (2 * TOGGLE_HZ)
    //
    // Interpretation:
    //   After DIVISOR input-clock cycles have elapsed, the heartbeat output is
    //   toggled once.
    //
    // Example:
    //   CLK_HZ    = 100_000_000
    //   TOGGLE_HZ = 2
    //
    //   DIVISOR = 100_000_000 / (2 * 2)
    //           = 25_000_000
    //
    //   So heartbeat_o changes state every 25,000,000 input clock cycles.
    //==========================================================================
    localparam integer DIVISOR = (CLK_HZ / (2*TOGGLE_HZ));

    //==========================================================================
    // Cycle counter
    //--------------------------------------------------------------------------
    // Purpose:
    //   Count how many clk edges have occurred since the most recent heartbeat
    //   toggle event.
    //
    // Width choice:
    //   32 bits is a practical fixed-width choice for many moderate-rate FPGA
    //   heartbeat dividers.
    //==========================================================================
    reg [31:0] ctr;

    //==========================================================================
    // Heartbeat divider sequential process
    //--------------------------------------------------------------------------
    // Step-by-step behavior on each rising edge of clk:
    //
    //   Case 1: rst is asserted
    //     1) Clear the counter to zero
    //     2) Force heartbeat_o low
    //
    //     Engineering meaning:
    //       Reset places the heartbeat generator into a known initial state.
    //
    //   Case 2: ctr has reached DIVISOR - 1
    //     1) Reset the counter to zero
    //     2) Invert heartbeat_o
    //
    //     Why DIVISOR - 1?
    //       Because counting begins at zero. Therefore, a count range of:
    //
    //           0, 1, 2, ..., DIVISOR-1
    //
    //       contains exactly DIVISOR clock cycles.
    //
    //     Why toggle instead of forcing a value?
    //       Because the output is intended to alternate between low and high,
    //       producing a square-wave-like heartbeat.
    //
    //   Case 3: normal counting
    //     1) Increment ctr by one
    //     2) Leave heartbeat_o unchanged
    //
    // Sequential interpretation:
    //   The counter measures elapsed time.
    //   Once the programmed interval has elapsed, the output flips state and
    //   the timing measurement restarts.
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            ctr         <= 32'd0;
            heartbeat_o <= 1'b0;
        end else if (ctr >= DIVISOR-1) begin
            ctr         <= 32'd0;
            heartbeat_o <= ~heartbeat_o;
        end else begin
            ctr <= ctr + 32'd1;
        end
    end

endmodule

`default_nettype wire