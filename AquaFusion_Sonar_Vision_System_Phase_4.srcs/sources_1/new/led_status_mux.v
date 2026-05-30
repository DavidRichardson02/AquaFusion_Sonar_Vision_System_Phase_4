`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// led_status_mux
//------------------------------------------------------------------------------
// ROLE
//   Registered board-LED status publisher.
//
// HIGH-LEVEL PURPOSE
//   This module takes a wider internal status bus, `status_word`, and publishes
//   an 8-bit subset of that status onto the board LEDs through `led_out`.
//
// WHY THIS MODULE EXISTS
//   Internal status in a digital system is often naturally represented as a
//   multi-bit diagnostic or telemetry word. Board LEDs, however, are a very
//   small and human-limited display surface. Therefore some narrowing or
//   selection policy is required between:
//
//       rich internal status representation
//           ->
//       small physical LED interface
//
//   This module is that narrowing/publishing boundary.
//
// IMPORTANT CURRENT BEHAVIOR
//   Despite the name "led_status_mux", the present implementation does *not*
//   perform dynamic multiplexing, bank switching, paging, or selection among
//   different status fields.
//
//   Instead, it performs a very simple and deterministic policy:
//
//       led_out <= status_word[7:0]
//
//   on each rising edge of clk_sys, except during reset when the LEDs are
//   cleared to zero.
//
//   So in its present form, this module is better understood as a registered
//   "LED status publisher" than as a true multiplexer.
//
// WHY THE MODULE NAME STILL MAKES SENSE
//   The name suggests the architectural direction of the block: a later revision
//   may genuinely multiplex among multiple diagnostic pages or status regions,
//   for example:
//
//     - lower status byte
//     - upper status byte
//     - selected error field
//     - activity counters
//     - switch-selected view pages
//
//   Keeping this module boundary now allows those future enhancements without
//   changing the rest of the top-level integration structure.
//
// SIGNAL SEMANTICS
//   clk_sys
//     System clock domain in which LED outputs are updated.
//
//   rst_sys
//     Active-high synchronous reset.
//     When asserted, forces led_out to 8'd0.
//
//   status_word
//     32-bit internal status vector produced elsewhere in the design.
//     In the current implementation, only bits [7:0] are used.
//
//   led_out
//     8-bit registered LED drive vector.
//
// REGISTERED OUTPUT POLICY
//   The LED outputs are updated synchronously rather than combinationally.
//   This has several advantages:
//
//     1) It makes the output timing explicit and deterministic.
//     2) It prevents direct combinational exposure of status_word changes to the
//        LED pins.
//     3) It makes the module behavior easier to reason about in simulation and
//        timing analysis.
//
// RESET POLICY
//   On rst_sys assertion:
//
//       led_out <= 8'd0
//
//   Therefore, during reset, all LEDs are driven low/off according to the
//   board's active-high LED convention assumed by the design.
//
// FUNCTIONAL SUMMARY
//   Present behavior can be summarized as:
//
//     if (rst_sys)
//         led_out = 8'h00;
//     else
//         led_out = status_word[7:0];
//
// TIMING INTERPRETATION
//   Since led_out is registered on clk_sys:
//
//     - a change in status_word[7:0] does not appear instantly at led_out
//     - it appears on the next rising edge of clk_sys
//
//   This means the LEDs show a sampled version of the chosen status byte.
//
// LIMITATIONS OF THE PRESENT REVISION
//   This module currently:
//
//     - ignores status_word[31:8]
//     - provides no page selection
//     - provides no blink policy
//     - provides no prioritization of alarms
//     - provides no sticky-latch behavior
//
//   Those limitations are acceptable for a small first-light or bring-up
//   system, especially when the low byte contains the most important immediate
//   health bits.
//
// PEDAGOGICAL INTERPRETATION
//   The core conceptual job of this module is:
//
//     "Take a large internal state description and project a small, stable,
//      human-visible subset of it onto the board LEDs."
//
//   The present projection is simply the least significant byte of the status
//   word.
//------------------------------------------------------------------------------
module led_status_mux (
    //--------------------------------------------------------------------------
    // System clock.
    //
    // LED outputs are updated synchronously on rising edges of this clock.
    //--------------------------------------------------------------------------
    input  wire        clk_sys,

    //--------------------------------------------------------------------------
    // Active-high synchronous system reset.
    //
    // When asserted, forces the LED output register to zero.
    //--------------------------------------------------------------------------
    input  wire        rst_sys,

    //--------------------------------------------------------------------------
    // Wide internal status vector.
    //
    // Present usage:
    //   Only status_word[7:0] is published to the LEDs.
    //--------------------------------------------------------------------------
    input  wire [31:0] status_word,

    //--------------------------------------------------------------------------
    // Registered LED output vector.
    //
    // This drives the physical board LEDs.
    //--------------------------------------------------------------------------
    output reg  [7:0]  led_out
);

    //==========================================================================
    // LED output register update process
    //--------------------------------------------------------------------------
    // Step-by-step behavior on each rising edge of clk_sys:
    //
    //   Case 1: rst_sys is asserted
    //     1) Clear led_out to 8'd0
    //     2) This guarantees that reset places the visible LED interface into a
    //        known, quiet state
    //
    //   Case 2: rst_sys is not asserted
    //     1) Observe the current 32-bit status_word input
    //     2) Select its least significant byte: status_word[7:0]
    //     3) Store that byte into the LED output register
    //
    // Sequential interpretation:
    //   The LEDs do not respond combinationally to status_word.
    //   Instead, they display the value sampled at the active clock edge.
    //
    // Engineering consequence:
    //   This creates a stable, synchronous board-facing output policy.
    //==========================================================================
    always @(posedge clk_sys) begin
        if (rst_sys)
            led_out <= 8'd0;
        else
            led_out <= status_word[7:0];
    end

endmodule

`default_nettype wire