`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// camera_pwup_fsm
//------------------------------------------------------------------------------
// ROLE
//   Camera power-up timing sequencer.
//
// HIGH-LEVEL PURPOSE
//   This module enforces the required timing relationship between:
//
//     1) asserting/deasserting the camera power-up control signal `cam_pwup`
//     2) declaring when the camera control path may safely begin SCCB traffic
//
//   In other words, this is a *time-sequencing FSM* for the camera bring-up
//   control plane.
//
// FUNCTIONAL INTENT
//   The module implements a simple four-state sequence:
//
//     ST_IDLE
//       Wait for a start request.
//
//     ST_LOW
//       Hold `cam_pwup` low for PWUP_LOW_MS milliseconds.
//
//     ST_HIGHWT
//       Drive `cam_pwup` high, then wait PWUP_TO_SCCB_MS milliseconds before
//       declaring SCCB access safe.
//
//     ST_DONE
//       Hold the camera in the powered-up state and advertise completion.
//
// WHY THIS MODULE EXISTS
//   Image sensors and camera modules often require control pins to obey timing
//   contracts from their reference documentation. A camera is not always ready
//   for bus transactions immediately when a control pin changes state.
//
//   Therefore, a safe bring-up flow often looks like:
//
//     - hold control pin in one state for a required interval
//     - transition the control pin
//     - wait an additional settling interval
//     - only then allow configuration traffic
//
//   This module embodies exactly that structure.
//
// SIGNAL SEMANTICS
//   clk
//     Local synchronous clock used to measure all timing intervals.
//
//   rst
//     Active-high synchronous reset.
//
//   start
//     Request to begin the power-up sequence.
//
//     IMPORTANT PRESENT-BEHAVIOR NOTE:
//       This input is treated as a level-sensitive request, not as an edge-only
//       pulse. In particular, the FSM may restart from ST_DONE if `start` is
//       high in that state.
//
//   cam_pwup
//     Camera power-up control output.
//
//     Present sequence:
//       - low in ST_IDLE and ST_LOW
//       - high in ST_HIGHWT and ST_DONE
//
//   busy
//     Indicates that the FSM is actively performing the power-up sequence.
//
//   done
//     Indicates that the sequence has completed successfully and the FSM is in
//     its completed state.
//
//   ready_for_sccb
//     Indicates that the post-power-up wait interval has completed and the next
//     stage may begin SCCB communication.
//
//   err_flag
//     Error indicator.
//
//     IMPORTANT PRESENT-BEHAVIOR NOTE:
//       In the current implementation, err_flag is always cleared and never
//       asserted. It exists as an interface placeholder for future expansion.
//
// PARAMETER SEMANTICS
//   CLK_HZ
//     Local clock frequency in hertz.
//
//   PWUP_LOW_MS
//     Time, in milliseconds, for which cam_pwup must be held low after start.
//
//   PWUP_TO_SCCB_MS
//     Additional wait time, in milliseconds, after cam_pwup is driven high
//     before SCCB traffic is considered safe.
//
// DERIVED TIMING CONSTANTS
//   TICKS_PER_MS
//     Number of clk cycles corresponding to one millisecond.
//
//   LOW_TICKS_TARGET
//     Total number of clk cycles required for the low-hold interval.
//
//   HIGH_TICKS_TARGET
//     Total number of clk cycles required for the post-high wait interval.
//
// CONCEPTUAL TIMELINE
//   After start:
//
//     Phase A: low hold
//       cam_pwup = 0
//       wait PWUP_LOW_MS
//
//     Phase B: high settle
//       cam_pwup = 1
//       wait PWUP_TO_SCCB_MS
//
//     Phase C: complete
//       ready_for_sccb = 1
//       done           = 1
//       busy           = 0
//
// FSM DESIGN PHILOSOPHY
//   Each state corresponds to a semantically meaningful stage of the power-up
//   sequence, not merely to a counter condition:
//
//     ST_IDLE   -> no active sequence
//     ST_LOW    -> enforcing low hold interval
//     ST_HIGHWT -> enforcing post-high wait interval
//     ST_DONE   -> completed/ready state
//
//   This makes the control behavior easy to read, simulate, and audit against
//   the camera timing requirements.
//
// RESET BEHAVIOR
//   On rst assertion:
//
//     state          <= ST_IDLE
//     ctr            <= 0
//     cam_pwup       <= 0
//     busy           <= 0
//     done           <= 0
//     ready_for_sccb <= 0
//     err_flag       <= 0
//
//   Engineering meaning:
//     Reset returns the FSM to a conservative not-yet-powered-up state.
//
// IMPORTANT CURRENT SEMANTICS
//   1) `ready_for_sccb` is explicitly cleared in ST_IDLE and ST_LOW.
//   2) `ready_for_sccb` is asserted when the HIGH wait finishes.
//   3) In ST_DONE, `ready_for_sccb` is not reassigned, so it remains at its
//      previously established value.
//   4) `err_flag` is presently a structural placeholder only.
//   5) A new `start` in ST_DONE restarts the sequence.
//
// DESIGN LIMITATIONS / ASSUMPTIONS
//   1) No explicit start-edge qualification
//      The current interface assumes higher-level logic either provides a pulse
//      or tolerates the level-sensitive restart behavior.
//
//   2) No timeout/failure detection
//      The FSM assumes that time elapsing is sufficient to reach readiness.
//      It does not observe camera feedback pins.
//
//   3) No explicit illegal-parameter checks
//      The arithmetic assumes sensible values for CLK_HZ, PWUP_LOW_MS, and
//      PWUP_TO_SCCB_MS.
//
// PEDAGOGICAL SUMMARY
//   The module can be understood as a three-phase stopwatch:
//
//     Step 1: wait in IDLE for permission to begin
//     Step 2: hold the camera control low for the required interval
//     Step 3: drive the control high and wait the required settle interval
//     Step 4: announce completion and SCCB readiness
//------------------------------------------------------------------------------
module camera_pwup_fsm #(
    //--------------------------------------------------------------------------
    // Local clock frequency in hertz.
    //--------------------------------------------------------------------------
    parameter integer CLK_HZ          = 100_000_000,

    //--------------------------------------------------------------------------
    // Required duration, in milliseconds, to hold cam_pwup low after start.
    //--------------------------------------------------------------------------
    parameter integer PWUP_LOW_MS     = 100,

    //--------------------------------------------------------------------------
    // Required duration, in milliseconds, to wait after driving cam_pwup high
    // before SCCB is considered safe.
    //--------------------------------------------------------------------------
    parameter integer PWUP_TO_SCCB_MS = 50
)(
    //--------------------------------------------------------------------------
    // Local synchronous clock.
    //--------------------------------------------------------------------------
    input  wire clk,

    //--------------------------------------------------------------------------
    // Active-high synchronous reset.
    //--------------------------------------------------------------------------
    input  wire rst,

    //--------------------------------------------------------------------------
    // Start request for the power-up sequence.
    //
    // Present semantics:
    //   level-sensitive request, not edge-qualified.
    //--------------------------------------------------------------------------
    input  wire start,

    //--------------------------------------------------------------------------
    // Camera power-up control output.
    //--------------------------------------------------------------------------
    output reg  cam_pwup,

    //--------------------------------------------------------------------------
    // Indicates that the FSM is currently executing the sequence.
    //--------------------------------------------------------------------------
    output reg  busy,

    //--------------------------------------------------------------------------
    // Indicates that the sequence has completed.
    //--------------------------------------------------------------------------
    output reg  done,

    //--------------------------------------------------------------------------
    // Indicates that the post-power-up wait interval has completed and SCCB
    // access may begin.
    //--------------------------------------------------------------------------
    output reg  ready_for_sccb,

    //--------------------------------------------------------------------------
    // Error indicator.
    //
    // Present implementation note:
    //   This remains deasserted in all implemented state paths.
    //--------------------------------------------------------------------------
    output reg  err_flag
);

    //==========================================================================
    // State encoding
    //--------------------------------------------------------------------------
    // ST_IDLE
    //   Waiting for start.
    //
    // ST_LOW
    //   Hold cam_pwup low for the required low interval.
    //
    // ST_HIGHWT
    //   Hold cam_pwup high while waiting for SCCB-safe delay to expire.
    //
    // ST_DONE
    //   Sequence complete; camera remains powered-up.
    //==========================================================================
    localparam [1:0] ST_IDLE    = 2'd0;
    localparam [1:0] ST_LOW     = 2'd1;
    localparam [1:0] ST_HIGHWT  = 2'd2;
    localparam [1:0] ST_DONE    = 2'd3;

    //==========================================================================
    // Timing conversion constants
    //--------------------------------------------------------------------------
    // TICKS_PER_MS:
    //   Number of clk cycles corresponding to one millisecond.
    //
    // LOW_TICKS_TARGET:
    //   Number of clk cycles required to satisfy PWUP_LOW_MS.
    //
    // HIGH_TICKS_TARGET:
    //   Number of clk cycles required to satisfy PWUP_TO_SCCB_MS.
    //
    // Example:
    //   If CLK_HZ = 100_000_000, then:
    //
    //       TICKS_PER_MS = 100_000
    //
    //   If PWUP_LOW_MS = 100:
    //
    //       LOW_TICKS_TARGET = 10_000_000
    //
    //   If PWUP_TO_SCCB_MS = 50:
    //
    //       HIGH_TICKS_TARGET = 5_000_000
    //==========================================================================
    localparam integer TICKS_PER_MS      = CLK_HZ / 1000;
    localparam integer LOW_TICKS_TARGET  = PWUP_LOW_MS     * TICKS_PER_MS;
    localparam integer HIGH_TICKS_TARGET = PWUP_TO_SCCB_MS * TICKS_PER_MS;

    //==========================================================================
    // Internal FSM state and interval counter
    //--------------------------------------------------------------------------
    // state
    //   Current FSM state.
    //
    // ctr
    //   Counts clk cycles inside the active timing interval of ST_LOW or
    //   ST_HIGHWT.
    //==========================================================================
    reg [1:0]  state;
    reg [31:0] ctr;

    //==========================================================================
    // Power-up sequence state machine
    //--------------------------------------------------------------------------
    // Overall structure:
    //
    //   A) Reset handling
    //   B) State-dependent timing and output control
    //
    // This FSM is Moore-like in the sense that each state strongly determines
    // the control outputs, while transitions are driven by `start` or by the
    // interval counter reaching its target.
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            //------------------------------------------------------------------
            // Reset case
            //
            // Step-by-step:
            //   1) Return to the IDLE state.
            //   2) Clear the interval counter.
            //   3) Force cam_pwup low.
            //   4) Mark the FSM not busy, not done, and not SCCB-ready.
            //   5) Clear err_flag.
            //
            // Engineering meaning:
            //   Reset returns the power-up controller to its most conservative
            //   initial condition.
            //------------------------------------------------------------------
            state          <= ST_IDLE;
            ctr            <= 32'd0;
            cam_pwup       <= 1'b0;
            busy           <= 1'b0;
            done           <= 1'b0;
            ready_for_sccb <= 1'b0;
            err_flag       <= 1'b0;
        end else begin
            case (state)

                //==============================================================
                // ST_IDLE
                //--------------------------------------------------------------
                // Purpose:
                //   Hold the interface in its inactive baseline state while
                //   waiting for a start request.
                //
                // Step-by-step behavior:
                //   1) Clear the interval counter.
                //   2) Hold cam_pwup low.
                //   3) Mark the FSM not busy.
                //   4) Mark done low.
                //   5) Mark ready_for_sccb low.
                //   6) Clear err_flag.
                //   7) If start is asserted:
                //        - move to ST_LOW
                //        - assert busy
                //        - keep cam_pwup low
                //
                // Design note:
                //   Because start is level-sensitive, remaining high at the
                //   wrong time can retrigger the sequence later from ST_DONE.
                //==============================================================
                ST_IDLE: begin
                    ctr            <= 32'd0;
                    cam_pwup       <= 1'b0;
                    busy           <= 1'b0;
                    done           <= 1'b0;
                    ready_for_sccb <= 1'b0;
                    err_flag       <= 1'b0;

                    if (start) begin
                        state    <= ST_LOW;
                        busy     <= 1'b1;
                        cam_pwup <= 1'b0;
                    end
                end

                //==============================================================
                // ST_LOW
                //--------------------------------------------------------------
                // Purpose:
                //   Enforce the required low interval on cam_pwup.
                //
                // Step-by-step behavior:
                //   1) Hold cam_pwup low.
                //   2) Mark the FSM busy.
                //   3) Keep done low.
                //   4) Keep ready_for_sccb low.
                //   5) Compare ctr against LOW_TICKS_TARGET - 1.
                //   6) If the interval has completed:
                //        - clear ctr
                //        - drive cam_pwup high
                //        - transition to ST_HIGHWT
                //      Else:
                //        - increment ctr
                //
                // Why compare against TARGET - 1?
                //   Because ctr starts at zero, so counting from 0 up through
                //   TARGET-1 spans exactly TARGET clock cycles.
                //==============================================================
                ST_LOW: begin
                    cam_pwup       <= 1'b0;
                    busy           <= 1'b1;
                    done           <= 1'b0;
                    ready_for_sccb <= 1'b0;

                    if (ctr >= LOW_TICKS_TARGET - 1) begin
                        ctr      <= 32'd0;
                        cam_pwup <= 1'b1;
                        state    <= ST_HIGHWT;
                    end else begin
                        ctr <= ctr + 32'd1;
                    end
                end

                //==============================================================
                // ST_HIGHWT
                //--------------------------------------------------------------
                // Purpose:
                //   Hold cam_pwup high and wait the required additional settling
                //   interval before allowing SCCB transactions.
                //
                // Step-by-step behavior:
                //   1) Drive cam_pwup high.
                //   2) Mark the FSM busy.
                //   3) Keep done low during the wait.
                //   4) Compare ctr against HIGH_TICKS_TARGET - 1.
                //   5) If the interval has completed:
                //        - clear ctr
                //        - assert ready_for_sccb
                //        - assert done
                //        - clear busy
                //        - transition to ST_DONE
                //      Else:
                //        - increment ctr
                //
                // Engineering meaning:
                //   This state models the post-power-up stabilization time
                //   before control-bus traffic may safely begin.
                //==============================================================
                ST_HIGHWT: begin
                    cam_pwup <= 1'b1;
                    busy     <= 1'b1;
                    done     <= 1'b0;

                    if (ctr >= HIGH_TICKS_TARGET - 1) begin
                        ctr            <= 32'd0;
                        ready_for_sccb <= 1'b1;
                        done           <= 1'b1;
                        busy           <= 1'b0;
                        state          <= ST_DONE;
                    end else begin
                        ctr <= ctr + 32'd1;
                    end
                end

                //==============================================================
                // ST_DONE
                //--------------------------------------------------------------
                // Purpose:
                //   Hold the "sequence complete" condition.
                //
                // Step-by-step behavior:
                //   1) Keep cam_pwup high.
                //   2) Keep busy low.
                //   3) Keep done high.
                //   4) Leave ready_for_sccb unchanged from its previously
                //      asserted value.
                //   5) If start is asserted again:
                //        - clear ctr
                //        - drive cam_pwup low
                //        - assert busy
                //        - clear done
                //        - clear ready_for_sccb
                //        - transition back to ST_LOW
                //
                // Present semantics:
                //   This state is both the completed state and the restart point
                //   for subsequent power-up attempts.
                //==============================================================
                ST_DONE: begin
                    cam_pwup <= 1'b1;
                    busy     <= 1'b0;
                    done     <= 1'b1;

                    if (start) begin
                        ctr            <= 32'd0;
                        cam_pwup       <= 1'b0;
                        busy           <= 1'b1;
                        done           <= 1'b0;
                        ready_for_sccb <= 1'b0;
                        state          <= ST_LOW;
                    end
                end

                //==============================================================
                // Default recovery
                //--------------------------------------------------------------
                // Purpose:
                //   Recover from any illegal or unknown state encoding by
                //   returning to ST_IDLE.
                //==============================================================
                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire