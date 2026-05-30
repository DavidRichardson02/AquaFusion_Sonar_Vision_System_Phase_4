`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// board_io_ctrl
//------------------------------------------------------------------------------
// ROLE
//   Board-level human-input conditioning block for the clk_sys domain.
//
// HIGH-LEVEL PURPOSE
//   This module accepts raw switch and pushbutton inputs coming from board I/O
//   pins and republishes them as clk_sys-domain signals:
//
//     - sw_sys   : synchronized switch image
//     - btn_sys  : synchronized button image
//
//   It also derives a simple internal reset request:
//
//     - reset_req_sys = btn_sys[0]
//
// WHY THIS MODULE EXISTS
//   External board inputs such as slide switches and pushbuttons are not
//   inherently synchronous to the internal system clock. Even though a human
//   presses a button slowly in real time, the transition at the FPGA pin can
//   still occur at an arbitrary moment relative to clk_sys.
//
//   If such a signal were used directly by synchronous logic, then at the
//   instant of sampling a receiving flip-flop could see a transition too close
//   to the active clock edge. That creates risk of uncertain sampling and
//   metastability propagation.
//
//   This module reduces that risk by passing the raw inputs through a two-stage
//   register pipeline in the clk_sys domain.
//
// FUNCTIONAL SCOPE
//   The present module does:
//
//     1) sample raw switches into a first-stage register
//     2) sample that first-stage switch image into a second-stage register
//     3) sample raw buttons into a first-stage register
//     4) sample that first-stage button image into a second-stage register
//     5) expose btn_sys[0] as a simple reset request signal
//
//   The present module does *not*:
//
//     - debounce mechanical contacts
//     - detect edges or generate one-cycle pulses
//     - distinguish short press vs long press
//     - latch events
//     - apply any priority or policy beyond direct publication
//
// PEDAGOGICAL INTERPRETATION
//   The internal logic should be understood as two sequential layers:
//
//   Layer 1: metastability-filtering / first observation
//     sw_meta  captures sw_in
//     btn_meta captures btn_in
//
//   Layer 2: public synchronized image
//     sw_sys  captures sw_meta
//     btn_sys captures btn_meta
//
//   This pattern does not remove all practical issues with human inputs.
//   In particular, it does not address mechanical bounce. It does, however,
//   create a much cleaner domain boundary for synchronous logic.
//
// RESET POLICY
//   On rst_sys assertion, all internal and output registers are cleared to zero.
//   Therefore:
//
//     sw_sys       -> 0
//     btn_sys      -> 0
//     reset_req_sys-> 0
//
//   This means no user command is considered active during reset.
//
// OUTPUT MEANING
//   sw_sys
//     Stable clk_sys-domain view of the raw slide-switch inputs, delayed by the
//     two-stage synchronization pipeline.
//
//   btn_sys
//     Stable clk_sys-domain view of the raw pushbutton inputs, delayed by the
//     two-stage synchronization pipeline.
//
//   reset_req_sys
//     Direct level alias of btn_sys[0].
//
// IMPORTANT LIMITATION ON reset_req_sys
//   Since reset_req_sys is assigned directly from btn_sys[0], it is:
//
//     - level-sensitive, not pulse-based
//     - dependent on the synchronized button level
//     - still vulnerable to mechanical bounce effects
//
//   Therefore reset_req_sys is appropriate as a simple bring-up soft-reset
//   request, but not yet a fully conditioned operator-command interface.
//
// TIMING INTERPRETATION
//   Because there are two sequential register stages:
//
//       raw input -> meta stage -> published stage
//
//   a stable change at the input will generally appear at sw_sys / btn_sys
//   after two rising edges of clk_sys, assuming no reset condition intervenes.
//
// DESIGN PHILOSOPHY
//   This module intentionally keeps board-facing conditioning separate from
//   downstream control logic. That separation improves reviewability and allows
//   future upgrades such as:
//
//     - per-button debouncing
//     - rising-edge pulse generation
//     - long-press qualification
//     - sticky event flags
//
//   without changing the rest of the design hierarchy.
//------------------------------------------------------------------------------
module board_io_ctrl (
    //--------------------------------------------------------------------------
    // System clock domain into which board inputs are synchronized.
    //--------------------------------------------------------------------------
    input  wire       clk_sys,

    //--------------------------------------------------------------------------
    // Active-high synchronized system reset.
    //
    // When asserted, all internal and published button/switch states are
    // cleared to zero.
    //--------------------------------------------------------------------------
    input  wire       rst_sys,

    //--------------------------------------------------------------------------
    // Raw board switch inputs.
    //
    // These arrive from board pins and are not assumed to be synchronous to
    // clk_sys.
    //--------------------------------------------------------------------------
    input  wire [7:0] sw_in,

    //--------------------------------------------------------------------------
    // Raw board pushbutton inputs.
    //
    // These also arrive from board pins and are not assumed to be synchronous
    // to clk_sys.
    //--------------------------------------------------------------------------
    input  wire [4:0] btn_in,

    //--------------------------------------------------------------------------
    // Synchronized switch image in the clk_sys domain.
    //
    // This is the second-stage registered form of sw_in.
    //--------------------------------------------------------------------------
    output reg  [7:0] sw_sys,

    //--------------------------------------------------------------------------
    // Synchronized button image in the clk_sys domain.
    //
    // This is the second-stage registered form of btn_in.
    //--------------------------------------------------------------------------
    output reg  [4:0] btn_sys,

    //--------------------------------------------------------------------------
    // Simple soft-reset request.
    //
    // Present policy:
    //   directly mirror synchronized button 0 as a level-sensitive reset
    //   request.
    //--------------------------------------------------------------------------
    output wire       reset_req_sys
);

    //==========================================================================
    // First-stage synchronization registers
    //--------------------------------------------------------------------------
    // These registers capture the raw board inputs directly.
    //
    // Conceptual purpose:
    //   They form the first sampling boundary between asynchronous external
    //   inputs and the synchronous clk_sys domain.
    //
    // The second stage (sw_sys / btn_sys) then samples these intermediate
    // values.
    //==========================================================================
    reg [7:0] sw_meta;
    reg [4:0] btn_meta;

    //==========================================================================
    // Input synchronization sequential process
    //--------------------------------------------------------------------------
    // Step-by-step behavior on each rising edge of clk_sys:
    //
    //   Case 1: rst_sys is asserted
    //     1) Clear the first-stage switch register    : sw_meta  <= 0
    //     2) Clear the published switch register      : sw_sys   <= 0
    //     3) Clear the first-stage button register    : btn_meta <= 0
    //     4) Clear the published button register      : btn_sys  <= 0
    //
    //     Engineering meaning:
    //       The internal system starts from a known "no input asserted" state.
    //
    //   Case 2: rst_sys is not asserted
    //     1) Capture the current raw switch input into sw_meta
    //     2) Publish the previous sw_meta into sw_sys
    //     3) Capture the current raw button input into btn_meta
    //     4) Publish the previous btn_meta into btn_sys
    //
    //     Pipeline interpretation:
    //       sw_in  -> sw_meta -> sw_sys
    //       btn_in -> btn_meta -> btn_sys
    //
    // Why this is written with nonblocking assignments:
    //   All updates occur concurrently with respect to the current clock edge.
    //   Thus sw_sys receives the *previous* value of sw_meta, not the just-
    //   sampled raw switch value from the same edge. This is exactly what is
    //   desired for a two-stage synchronizer structure.
    //
    // Important note on bounce:
    //   This logic synchronizes the board inputs into clk_sys, but it does not
    //   suppress mechanical bounce. A bouncing pushbutton may still appear as
    //   multiple level transitions across successive clock cycles.
    //==========================================================================
    always @(posedge clk_sys) begin
        if (rst_sys) begin
            sw_meta  <= 8'd0;
            sw_sys   <= 8'd0;
            btn_meta <= 5'd0;
            btn_sys  <= 5'd0;
        end else begin
            sw_meta  <= sw_in;
            sw_sys   <= sw_meta;
            btn_meta <= btn_in;
            btn_sys  <= btn_meta;
        end
    end

    //==========================================================================
    // Soft-reset request derivation
    //--------------------------------------------------------------------------
    // Present policy:
    //   Use synchronized button 0 directly as a system-local reset request.
    //
    // Step-by-step meaning:
    //   1) Observe the already synchronized button vector btn_sys.
    //   2) Select bit 0.
    //   3) Publish that level directly as reset_req_sys.
    //
    // Consequences:
    //   - reset_req_sys is synchronous to clk_sys
    //   - reset_req_sys is level-sensitive
    //   - reset_req_sys remains asserted as long as btn_sys[0] remains high
    //
    // Design caution:
    //   Because this is not debounced and not edge-qualified, downstream logic
    //   should treat it as a simple operator request rather than a fully
    //   hardened command pulse.
    //==========================================================================
    assign reset_req_sys = btn_sys[0];

endmodule

`default_nettype wire