`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// reset_sync
//------------------------------------------------------------------------------
// ROLE
//   Asynchronous-assert / synchronous-deassert reset synchronizer.
//
// HIGH-LEVEL PURPOSE
//   This module accepts a reset request that may be asserted independently of
//   the destination clock and converts it into a reset signal that:
//
//     1) asserts immediately when the asynchronous reset request is asserted
//     2) deasserts only in alignment with the rising edges of the destination
//        clock
//
// WHY THIS MODULE EXISTS
//   In synchronous digital systems, reset release must be treated carefully.
//
//   If reset were removed at an arbitrary time relative to the destination
//   clock, different flip-flops in the domain could observe the release on
//   different clock edges or near setup/hold boundaries. That can lead to
//   inconsistent startup behavior and, in the worst case, metastability-related
//   uncertainty during reset exit.
//
//   The standard remedy is:
//
//     - allow reset assertion to happen immediately
//     - synchronize reset deassertion to the target clock domain
//
//   That is exactly the contract implemented by this module.
//
// SIGNAL SEMANTICS
//   clk
//     Destination clock domain into which reset must be safely released.
//
//   arst
//     Asynchronous reset request, active high.
//     This signal may assert independently of clk.
//
//   srst
//     Synchronized reset output, active high.
//     This signal is safe to distribute to logic clocked by clk.
//
// INTERNAL STRUCTURE
//   A 3-bit shift register is used:
//
//       sync_ff[2:0]
//
//   On asynchronous reset assertion:
//       sync_ff <= 3'b111
//
//   On each subsequent clk rising edge after arst is released:
//       sync_ff <= {sync_ff[1:0], 1'b0}
//
//   Therefore the register contents evolve as:
//
//       111 -> 110 -> 100 -> 000
//
//   Since srst = sync_ff[2], the output reset remains high until the final
//   stage has shifted down to zero.
//
// DEASSERTION LATENCY
//   After arst is released, srst remains asserted for three rising edges of
//   clk before fully deasserting.
//
//   This multi-stage delay serves two purposes:
//
//     1) it provides a clean, clock-aligned reset release
//     2) it gives any metastability arising in the earliest synchronizer stage
//        additional time to settle before the final stage drives the public
//        reset output
//
// RESET STYLE
//   This module uses:
//     - asynchronous assertion
//     - synchronous release
//
//   This is one of the most common and recommended reset distribution patterns
//   for FPGA and synchronous RTL design.
//
// IMPORTANT LIMITATION
//   This module synchronizes *reset release* into exactly one clock domain.
//   A separate instance should be used for each independent clock domain.
//
//   In other words:
//     one reset_sync per destination clock domain
//
//   It must not be assumed that one synchronized reset can safely serve several
//   unrelated clocks.
//
// DESIGN NOTE ON POLARITY
//   Both arst and srst are active high.
//   If the surrounding system uses active-low reset inputs, polarity inversion
//   should happen outside this module, as seen in wrappers such as
//   clk_reset_mgr.
//
// PEDAGOGICAL SUMMARY
//   The module can be understood in two phases:
//
//   Phase A: asynchronous assertion
//     As soon as arst rises, all three synchronizer bits are forced to 1,
//     making srst immediately high.
//
//   Phase B: synchronous release
//     Once arst falls, zeros are shifted in on successive clk edges until the
//     final stage goes low. Only then is srst deasserted.
//
//   This means the system enters reset immediately but leaves reset in a
//   controlled, clock-aligned way.
//------------------------------------------------------------------------------
module reset_sync (
    //--------------------------------------------------------------------------
    // Destination clock for the reset domain.
    //
    // All downstream logic driven by srst is assumed to use this same clk.
    //--------------------------------------------------------------------------
    input  wire clk,

    //--------------------------------------------------------------------------
    // Asynchronous reset request, active high.
    //
    // Assertion semantics:
    //   arst = 1 immediately forces the synchronizer register to all ones,
    //   regardless of the clock edge timing.
    //--------------------------------------------------------------------------
    input  wire arst,

    //--------------------------------------------------------------------------
    // Synchronized reset output, active high.
    //
    // Behavioral contract:
    //   - asserts asynchronously with arst
    //   - deasserts synchronously to clk after the synchronizer pipeline clears
    //--------------------------------------------------------------------------
    output wire srst
);

    //==========================================================================
    // Internal synchronizer shift register
    //--------------------------------------------------------------------------
    // sync_ff[2] is the publicly observed reset state.
    // sync_ff[1:0] provide intermediate stages that support safe deassertion.
    //
    // Initial asynchronous assertion state:
    //   3'b111
    //
    // Release sequence after arst goes low:
    //   111 -> 110 -> 100 -> 000
    //==========================================================================
    reg [2:0] sync_ff;

    //==========================================================================
    // Reset synchronizer sequential process
    //--------------------------------------------------------------------------
    // Sensitivity list:
    //   posedge clk or posedge arst
    //
    // This means:
    //   - the process reacts immediately when arst is asserted
    //   - otherwise it advances on rising edges of clk
    //
    // Step-by-step behavior:
    //
    //   Case 1: arst is asserted
    //     The synchronizer register is forced to 3'b111 immediately.
    //
    //     Why?
    //       Because reset assertion should not wait for the next clock edge.
    //       The system should enter reset as soon as the reset request occurs.
    //
    //   Case 2: arst is not asserted
    //     The register shifts left toward zero:
    //
    //         sync_ff <= {sync_ff[1:0], 1'b0};
    //
    //     This operation performs the following:
    //       - previous bit [1] moves into bit [2]
    //       - previous bit [0] moves into bit [1]
    //       - a zero is inserted into bit [0]
    //
    //     Therefore, once reset is released, the "reset-high" state drains
    //     through the pipeline over multiple clock cycles.
    //
    // Timing interpretation:
    //   Suppose arst is released just before some rising edge of clk.
    //   If sync_ff is currently 111, then after successive rising edges:
    //
    //       edge 1 -> 110
    //       edge 2 -> 100
    //       edge 3 -> 000
    //
    //   Since srst = sync_ff[2], srst stays high through the first two release
    //   edges and finally falls low after the third stage clears.
    //==========================================================================
    always @(posedge clk or posedge arst) begin
        if (arst)
            sync_ff <= 3'b111;
        else
            sync_ff <= {sync_ff[1:0], 1'b0};
    end

    //==========================================================================
    // Public synchronized reset output
    //--------------------------------------------------------------------------
    // The most delayed stage of the synchronizer is used as the exported reset.
    //
    // Why the final stage?
    //   Because it is the stage farthest removed from any metastability risk in
    //   the earlier stages during reset release, and because it guarantees that
    //   reset remains asserted until the entire synchronizer pipeline has
    //   cleared.
    //==========================================================================
    assign srst = sync_ff[2];

endmodule

`default_nettype wire