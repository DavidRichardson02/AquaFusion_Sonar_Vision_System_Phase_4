`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// sonar_filter
//------------------------------------------------------------------------------
// ROLE
//   Canonical sonar-distance conditioning and estimation stage for the
//   AquaFusion sonar lane.
//
// PURPOSE
//   This module accepts whole-inch sonar samples, rejects physically invalid or
//   weakly-supported candidates, smooths accepted measurements with small
//   deterministic front-end filters, and finally performs a full scalar Kalman
//   covariance update with runtime division.
//
//   The resulting architecture preserves several desirable properties:
//
//     1) It is physically constrained.
//        The accepted input range is explicitly limited to the sensor's valid
//        inch-domain operating range.
//
//     2) It is deterministic.
//        Every accepted sample follows a bounded, reviewable path.
//
//     3) It is robust to spurious spikes.
//        Large jumps must persist before they are admitted.
//
//     4) It is reviewable.
//        Each stage has a narrow, well-documented purpose.
//
// EXTERNAL UNIT CONTRACT
//   distance_in_raw      : unsigned whole-inch sonar distance
//   distance_valid_pulse : one-cycle pulse indicating a newly decoded sample
//   distance_out_filt    : unsigned whole-inch filtered distance
//   distance_valid_out   : one-cycle pulse indicating publication of a new
//                          filtered estimate
//
// FILTER PIPELINE
//   Incoming samples conceptually pass through the following stages:
//
//     Stage 1  Plausibility gate
//       Reject distances outside the physical inch-domain window.
//
//     Stage 2  Outlier persistence gate
//       If a plausible sample differs too much from the current accepted trend,
//       do not trust it immediately. Require repeated nearby confirmations.
//
//     Stage 3  Debounce / consensus gate
//       Require repeated agreement around a candidate before promoting it into
//       the accepted stream.
//
//     Stage 4  Median-of-3 smoother
//       Remove single-sample residual excursions from the accepted stream.
//
//     Stage 5  4-sample moving average
//       Reduce small measurement jitter while remaining inexpensive.
//
//     Stage 6  Scalar Kalman update with covariance recursion
//       Compute the gain from P/(P+R) at runtime and update both state and
//       covariance.
//
// DESIGN PHILOSOPHY
//   - Verilog-2001 only.
//   - Bounded work only; no unbounded loops.
//   - All state is explicit and synchronous to clk.
//   - No combinational feedback.
//   - The external interface remains compatible with the present top-level
//     integration path.
//
// TIMING MODEL
//   - Input samples are considered only on cycles where distance_valid_pulse=1.
//   - A newly accepted front-end measurement may either initialize the filter
//     immediately or be queued into the one-entry pending slot.
//   - After lock, a Kalman update requires a bounded divider/apply sequence.
//   - distance_valid_out pulses only when the final filtered distance register
//     is updated.
//
// RESET MODEL
//   - rst is synchronous and active-high.
//   - Reset clears the entire conditioning history and Kalman state.
//
// IMPLEMENTATION NOTE
//   This module intentionally emphasizes engineering clarity over absolute LUT
//   minimization. The code is structured so that future timing/resource tuning
//   can be done stage-by-stage without changing the external contract.
//==============================================================================
module sonar_filter #(
    //--------------------------------------------------------------------------
    // Physical plausibility window (inch domain)
    //--------------------------------------------------------------------------
    parameter [9:0] MIN_PLAUSIBLE           = 10'd6,
    parameter [9:0] MAX_PLAUSIBLE           = 10'd255,

    //--------------------------------------------------------------------------
    // Outlier persistence controls
    //--------------------------------------------------------------------------
    parameter [9:0] OUTLIER_DELTA           = 10'd24,
    parameter integer OUTLIER_CONFIRM_COUNT = 3,
    parameter [9:0] OUTLIER_GROUP_DELTA     = 10'd6,

    //--------------------------------------------------------------------------
    // Debounce / consensus controls
    //--------------------------------------------------------------------------
    parameter integer DEBOUNCE_COUNT        = 2,
    parameter [9:0] DEBOUNCE_DELTA          = 10'd4,

    //--------------------------------------------------------------------------
    // Fixed-point formats
    //--------------------------------------------------------------------------
    parameter integer X_FRAC_BITS           = 8,
    parameter integer K_FRAC_BITS           = 12,

    //--------------------------------------------------------------------------
    // Kalman covariance parameters in Q(X_FRAC_BITS) domain
    //--------------------------------------------------------------------------
    parameter [23:0] Q_COV_QX               = 24'd64,
    parameter [23:0] R_COV_QX               = 24'd512,
    parameter [23:0] P0_COV_QX              = 24'd1024
)(
    input  wire       clk,
    input  wire       rst,
    input  wire [9:0] distance_in_raw,
    input  wire       distance_valid_pulse,
    output reg  [9:0] distance_out_filt,
    output reg        distance_valid_out
);

    //=========================================================================
    // LOCAL WIDTHS AND FSM ENCODING
    //=========================================================================
    // X_W
    //   Width of the fixed-point state x. The integer portion holds the same
    //   10-bit whole-inch range as the input, while the fractional portion is
    //   used by the Kalman update for sub-inch internal precision.
    //
    // P_W
    //   Width of the covariance registers.
    //
    // NUM_W / DEN_W
    //   Widths used by the restoring divider that computes the Kalman gain.
    //
    // DIVC_W
    //   Width of the divider iteration counter.
    //=========================================================================
    localparam integer X_W    = 10 + X_FRAC_BITS;
    localparam integer P_W    = 24;
    localparam integer NUM_W  = P_W + K_FRAC_BITS;
    localparam integer DEN_W  = P_W + 1;
    localparam integer DIVC_W = 8;

    localparam [1:0] ST_IDLE  = 2'd0;
    localparam [1:0] ST_DIV   = 2'd1;
    localparam [1:0] ST_APPLY = 2'd2;

    //=========================================================================
    // UTILITY FUNCTIONS
    //=========================================================================

    //--------------------------------------------------------------------------
    // abs_diff10
    //-------------------------------------------------------------------------
    // Mathematical role
    //   Compute |a-b| for unsigned 10-bit values.
    //
    // Why this matters
    //   Nearly every front-end acceptance decision is based on whether two
    //   samples are "close enough." Since the values are unsigned, the simplest
    //   safe way to measure closeness is absolute difference.
    //
    // Step-by-step behavior
    //   1) Compare a and b.
    //   2) Subtract the smaller from the larger.
    //   3) Return the non-negative magnitude.
    //--------------------------------------------------------------------------
    function [9:0] abs_diff10;
        input [9:0] a;
        input [9:0] b;
        begin
            if (a >= b)
                abs_diff10 = a - b;
            else
                abs_diff10 = b - a;
        end
    endfunction

    //--------------------------------------------------------------------------
    // min3_10
    //-------------------------------------------------------------------------
    // Mathematical role
    //   Return the minimum of three unsigned 10-bit values.
    //
    // Why this matters
    //   The median of three can be computed efficiently as:
    //     a + b + c - min(a,b,c) - max(a,b,c)
    //   so explicit min/max helpers keep the median function simple.
    //
    // Step-by-step behavior
    //   1) Compare a and b to form a provisional minimum m.
    //   2) Compare m with c.
    //   3) Return the smaller result.
    //--------------------------------------------------------------------------
    function [9:0] min3_10;
        input [9:0] a;
        input [9:0] b;
        input [9:0] c;
        reg [9:0] m;
        begin
            m = (a < b) ? a : b;
            min3_10 = (m < c) ? m : c;
        end
    endfunction

    //--------------------------------------------------------------------------
    // max3_10
    //-------------------------------------------------------------------------
    // Mathematical role
    //   Return the maximum of three unsigned 10-bit values.
    //
    // Why this matters
    //   Together with min3_10, this allows an inexpensive median-of-3
    //   implementation without a full sorting network.
    //
    // Step-by-step behavior
    //   1) Compare a and b to form a provisional maximum m.
    //   2) Compare m with c.
    //   3) Return the larger result.
    //--------------------------------------------------------------------------
    function [9:0] max3_10;
        input [9:0] a;
        input [9:0] b;
        input [9:0] c;
        reg [9:0] m;
        begin
            m = (a > b) ? a : b;
            max3_10 = (m > c) ? m : c;
        end
    endfunction

    //--------------------------------------------------------------------------
    // median3_10
    //-------------------------------------------------------------------------
    // Mathematical role
    //   Return the middle value of three unsigned 10-bit inputs.
    //
    // Why this matters
    //   The median stage is specifically intended to suppress single-sample
    //   spikes that may still slip past the earlier acceptance logic.
    //
    // Step-by-step behavior
    //   1) Form the arithmetic sum a+b+c using a widened accumulator.
    //   2) Subtract the smallest member.
    //   3) Subtract the largest member.
    //   4) The remaining value is the median.
    //--------------------------------------------------------------------------
    function [9:0] median3_10;
        input [9:0] a;
        input [9:0] b;
        input [9:0] c;
        reg [11:0] s;
        begin
            s = a + b + c;
            median3_10 = s - min3_10(a,b,c) - max3_10(a,b,c);
        end
    endfunction

    //=========================================================================
    // FRONT-END CONDITIONING STATE
    //=========================================================================
    // These registers implement the deterministic pre-Kalman acceptance and
    // smoothing pipeline.
    //
    // accepted_sample / have_accepted
    //   The most recent sample that successfully passed the acceptance logic.
    //   This is the anchor against which future large steps are judged.
    //
    // outlier_candidate / outlier_confirm_count / outlier_pending
    //   State used to decide whether a large jump is a transient excursion or a
    //   persistent regime change.
    //
    // debounce_candidate / debounce_count / debounce_pending
    //   State used to require repeated nearby agreement before a candidate is
    //   admitted into the accepted stream.
    //
    // med_s0..med_s2 / med_count
    //   Short history for the median-of-3 stage.
    //
    // avg_s0..avg_s3 / avg_count
    //   Short history for the 4-sample moving average stage.
    //=========================================================================
    reg [9:0] accepted_sample;
    reg       have_accepted;

    reg [9:0] outlier_candidate;
    reg [7:0] outlier_confirm_count;
    reg       outlier_pending;

    reg [9:0] debounce_candidate;
    reg [7:0] debounce_count;
    reg       debounce_pending;

    reg [9:0] med_s0;
    reg [9:0] med_s1;
    reg [9:0] med_s2;
    reg [1:0] med_count;

    reg [9:0] avg_s0;
    reg [9:0] avg_s1;
    reg [9:0] avg_s2;
    reg [9:0] avg_s3;
    reg [2:0] avg_count;

    //=========================================================================
    // KALMAN ESTIMATOR STATE
    //=========================================================================
    // x_state_qx
    //   Current filtered state in fixed-point inches.
    //
    // p_state_qx
    //   Current covariance.
    //
    // kalman_locked
    //   Indicates that the estimator has already been initialized with at least
    //   one accepted front-end measurement.
    //=========================================================================
    reg [X_W-1:0] x_state_qx;
    reg [P_W-1:0] p_state_qx;
    reg           kalman_locked;

    //=========================================================================
    // PENDING-MEASUREMENT HANDOFF
    //=========================================================================
    // The front-end conditioner can accept a measurement while the Kalman FSM is
    // idle. That measurement is stored here until the divider/apply sequence is
    // launched. A one-entry slot is sufficient for the low-rate sonar stream.
    //=========================================================================
    reg [9:0] pending_meas;
    reg       pending_valid;

    //=========================================================================
    // DIVIDER / APPLY FSM STATE
    //=========================================================================
    reg [1:0] state;

    reg [P_W-1:0] p_pred_qx;
    reg [DEN_W-1:0] den_qx;

    reg [NUM_W-1:0] div_num_shift;
    reg [DEN_W-1:0] div_den;
    reg [DEN_W:0]   div_rem;
    reg [K_FRAC_BITS-1:0] div_quot;
    reg [DIVC_W-1:0] div_count;

    reg [K_FRAC_BITS-1:0] k_gain_qk;
    reg [9:0]             z_meas_reg;

    //=========================================================================
    // SEQUENTIAL-SCOPE TEMPORARIES
    //=========================================================================
    // These temporaries support stepwise arithmetic inside the main sequential
    // process. They are declared as registers because Verilog-2001 does not
    // permit local block declarations inside procedural statements.
    //=========================================================================
    reg        plausible_now;
    reg        sample_is_large_step;
    reg        outlier_accept_now;
    reg        debounce_accept_now;
    reg        accepted_now;
    reg [9:0]  accepted_value_now;
    reg [9:0]  median_now;
    reg [11:0] avg_sum12;
    reg [9:0]  avg_now;

    reg signed [X_W:0] innovation_qx;
    reg signed [X_W+K_FRAC_BITS:0] correction_full;
    reg signed [X_W+K_FRAC_BITS:0] correction_rounded;
    reg signed [X_W:0] x_next_signed;
    reg [P_W+K_FRAC_BITS-1:0] p_mult_full;
    reg [K_FRAC_BITS:0] one_minus_k_qk;
    reg [P_W+K_FRAC_BITS-1:0] p_next_full;
    reg [P_W-1:0] p_next_qx;
    reg [X_W-1:0] x_next_qx;
    reg [X_W-1:0] z_meas_qx;

    integer avg_div;

    //=========================================================================
    // MAIN SEQUENTIAL PROCESS
    //=========================================================================
    always @(posedge clk) begin
        if (rst) begin
            distance_out_filt      <= 10'd0;
            distance_valid_out     <= 1'b0;

            accepted_sample        <= 10'd0;
            have_accepted          <= 1'b0;

            outlier_candidate      <= 10'd0;
            outlier_confirm_count  <= 8'd0;
            outlier_pending        <= 1'b0;

            debounce_candidate     <= 10'd0;
            debounce_count         <= 8'd0;
            debounce_pending       <= 1'b0;

            med_s0                 <= 10'd0;
            med_s1                 <= 10'd0;
            med_s2                 <= 10'd0;
            med_count              <= 2'd0;

            avg_s0                 <= 10'd0;
            avg_s1                 <= 10'd0;
            avg_s2                 <= 10'd0;
            avg_s3                 <= 10'd0;
            avg_count              <= 3'd0;

            x_state_qx             <= {X_W{1'b0}};
            p_state_qx             <= P0_COV_QX;
            kalman_locked          <= 1'b0;

            pending_meas           <= 10'd0;
            pending_valid          <= 1'b0;

            state                  <= ST_IDLE;
            p_pred_qx              <= {P_W{1'b0}};
            den_qx                 <= {DEN_W{1'b0}};
            div_num_shift          <= {NUM_W{1'b0}};
            div_den                <= {DEN_W{1'b0}};
            div_rem                <= {(DEN_W+1){1'b0}};
            div_quot               <= {K_FRAC_BITS{1'b0}};
            div_count              <= {DIVC_W{1'b0}};
            k_gain_qk              <= {K_FRAC_BITS{1'b0}};
            z_meas_reg             <= 10'd0;
        end else begin
            //------------------------------------------------------------------
            // Default output pulse behavior
            //
            // Publication is edge-like: only the cycle in which a new filtered
            // estimate is committed should assert distance_valid_out.
            //------------------------------------------------------------------
            distance_valid_out <= 1'b0;

            //------------------------------------------------------------------
            // Default temporary values for this cycle
            //------------------------------------------------------------------
            plausible_now        = 1'b0;
            sample_is_large_step = 1'b0;
            outlier_accept_now   = 1'b0;
            debounce_accept_now  = 1'b0;
            accepted_now         = 1'b0;
            accepted_value_now   = 10'd0;
            median_now           = 10'd0;
            avg_sum12            = 12'd0;
            avg_now              = 10'd0;

            //------------------------------------------------------------------
            // INPUT CONDITIONING PIPELINE
            //
            // This block executes only on a new decoded sonar sample. It is the
            // place where physically implausible, isolated, or weakly-supported
            // measurements are denied entry into the estimator.
            //------------------------------------------------------------------
            if (distance_valid_pulse) begin
                //--------------------------------------------------------------
                // Stage 1: Plausibility gate
                //
                // Only measurements inside the explicitly declared physical
                // window are allowed to participate in later decisions.
                //--------------------------------------------------------------
                plausible_now =
                    (distance_in_raw >= MIN_PLAUSIBLE) &&
                    (distance_in_raw <= MAX_PLAUSIBLE);

                if (plausible_now) begin
                    //----------------------------------------------------------
                    // Stage 2: Outlier persistence gate
                    //
                    // Once a trend has been accepted, a new sample that differs
                    // too much from that trend is not trusted immediately. It is
                    // instead treated as a candidate step and must reappear in a
                    // locally consistent cluster before being admitted.
                    //----------------------------------------------------------
                    if (!have_accepted) begin
                        // No prior accepted trend exists yet, so the first
                        // plausible sample is allowed to proceed into the
                        // debounce stage immediately.
                        outlier_accept_now  = 1'b1;
                        accepted_value_now  = distance_in_raw;
                        outlier_pending     <= 1'b0;
                        outlier_confirm_count <= 8'd0;
                    end else begin
                        sample_is_large_step =
                            (abs_diff10(distance_in_raw, accepted_sample) > OUTLIER_DELTA);

                        if (!sample_is_large_step) begin
                            // Sample is close to the currently accepted trend,
                            // so it may proceed.
                            outlier_accept_now    = 1'b1;
                            accepted_value_now    = distance_in_raw;
                            outlier_pending       <= 1'b0;
                            outlier_confirm_count <= 8'd0;
                        end else begin
                            // Large step detected. Require repeated nearby
                            // confirmations before allowing it through.
                            if (!outlier_pending) begin
                                outlier_pending       <= 1'b1;
                                outlier_candidate     <= distance_in_raw;
                                outlier_confirm_count <= 8'd1;
                            end else if (abs_diff10(distance_in_raw, outlier_candidate)
                                         <= OUTLIER_GROUP_DELTA) begin
                                if (outlier_confirm_count + 1 >= OUTLIER_CONFIRM_COUNT) begin
                                    outlier_accept_now    = 1'b1;
                                    accepted_value_now    = distance_in_raw;
                                    outlier_pending       <= 1'b0;
                                    outlier_confirm_count <= 8'd0;
                                end else begin
                                    outlier_confirm_count <= outlier_confirm_count + 8'd1;
                                end
                            end else begin
                                // A different large step arrived before the
                                // prior candidate was confirmed. Restart the
                                // persistence check around the new candidate.
                                outlier_candidate     <= distance_in_raw;
                                outlier_confirm_count <= 8'd1;
                            end
                        end
                    end

                    //----------------------------------------------------------
                    // Stage 3: Debounce / consensus acceptance
                    //
                    // Even after the outlier stage permits a sample, it is not
                    // yet promoted into the accepted stream until repeated
                    // nearby agreement has occurred around a candidate.
                    //----------------------------------------------------------
                    if (outlier_accept_now) begin
                        if (!debounce_pending) begin
                            debounce_pending   <= 1'b1;
                            debounce_candidate <= accepted_value_now;
                            debounce_count     <= 8'd1;

                            if (DEBOUNCE_COUNT <= 1) begin
                                debounce_accept_now = 1'b1;
                            end
                        end else if (abs_diff10(accepted_value_now, debounce_candidate)
                                     <= DEBOUNCE_DELTA) begin
                            if (debounce_count + 1 >= DEBOUNCE_COUNT) begin
                                debounce_accept_now = 1'b1;
                            end else begin
                                debounce_count <= debounce_count + 8'd1;
                            end
                        end else begin
                            // Consensus shifted; restart the debounce cluster.
                            debounce_candidate <= accepted_value_now;
                            debounce_count     <= 8'd1;

                            if (DEBOUNCE_COUNT <= 1) begin
                                debounce_accept_now = 1'b1;
                            end
                        end
                    end

                    //----------------------------------------------------------
                    // Stages 4 and 5: accepted stream -> median -> moving avg
                    //
                    // Once a sample is fully accepted, it becomes the new trend
                    // anchor, is inserted into the median history, and then into
                    // the moving-average history.
                    //----------------------------------------------------------
                    if (debounce_accept_now) begin
                        accepted_now      = 1'b1;
                        accepted_sample   <= accepted_value_now;
                        have_accepted     <= 1'b1;
                        debounce_pending  <= 1'b0;
                        debounce_count    <= 8'd0;

                        // Median warm-up behavior:
                        //   With fewer than three accepted samples, the newest
                        //   accepted value is forwarded directly.
                        case (med_count)
                            2'd0: begin
                                med_s0    <= accepted_value_now;
                                med_count <= 2'd1;
                                median_now = accepted_value_now;
                            end
                            2'd1: begin
                                med_s1    <= accepted_value_now;
                                med_count <= 2'd2;
                                median_now = accepted_value_now;
                            end
                            default: begin
                                med_s2    <= accepted_value_now;
                                med_count <= 2'd3;
                                median_now = median3_10(med_s0, med_s1, accepted_value_now);
                                med_s0    <= med_s1;
                                med_s1    <= accepted_value_now;
                            end
                        endcase

                        // Moving-average warm-up behavior:
                        //   Before four values exist, divide by the number of
                        //   available samples rather than by a fixed four.
                        case (avg_count)
                            3'd0: begin
                                avg_s0    <= median_now;
                                avg_count <= 3'd1;
                                avg_now    = median_now;
                            end
                            3'd1: begin
                                avg_sum12 = avg_s0 + median_now;
                                avg_s1    <= median_now;
                                avg_count <= 3'd2;
                                avg_div   = avg_sum12 / 2;
                                avg_now   = avg_div[9:0];
                            end
                            3'd2: begin
                                avg_sum12 = avg_s0 + avg_s1 + median_now;
                                avg_s2    <= median_now;
                                avg_count <= 3'd3;
                                avg_div   = avg_sum12 / 3;
                                avg_now   = avg_div[9:0];
                            end
                            default: begin
                                avg_sum12 = avg_s0 + avg_s1 + avg_s2 + median_now;
                                avg_s3    <= median_now;
                                avg_count <= 3'd4;
                                avg_now   = avg_sum12[11:2];
                                avg_s0    <= avg_s1;
                                avg_s1    <= avg_s2;
                                avg_s2    <= median_now;
                            end
                        endcase

                        //------------------------------------------------------
                        // Stage 6 handoff: queue the smoothed measurement for
                        // the Kalman core.
                        //------------------------------------------------------
                        pending_meas  <= avg_now;
                        pending_valid <= 1'b1;
                    end
                end
            end

            //------------------------------------------------------------------
            // KALMAN FSM
            //
            // The front-end creates a smoothed measurement stream. The Kalman
            // core consumes one pending measurement at a time and either:
            //   - initializes the estimator on first lock, or
            //   - performs a full covariance/gain/state update.
            //------------------------------------------------------------------
            case (state)
                ST_IDLE: begin
                    if (pending_valid) begin
                        if (!kalman_locked) begin
                            //--------------------------------------------------
                            // Initial lock
                            //
                            // The first accepted front-end measurement becomes
                            // the initial state directly. Covariance is seeded
                            // from P0_COV_QX. Publication occurs immediately.
                            //--------------------------------------------------
                            x_state_qx         <= {pending_meas, {X_FRAC_BITS{1'b0}}};
                            p_state_qx         <= P0_COV_QX;
                            kalman_locked      <= 1'b1;
                            distance_out_filt  <= pending_meas;
                            distance_valid_out <= 1'b1;
                            pending_valid      <= 1'b0;
                        end else begin
                            //--------------------------------------------------
                            // Launch a full update
                            //
                            // 1) Predict covariance: P_pred = P + Q
                            // 2) Form denominator: P_pred + R
                            // 3) Initialize restoring divider to compute:
                            //      K = (P_pred << K_FRAC_BITS) / (P_pred + R)
                            //--------------------------------------------------
                            p_pred_qx     <= p_state_qx + Q_COV_QX;
                            den_qx        <= (p_state_qx + Q_COV_QX) + R_COV_QX;
                            div_num_shift <= {p_state_qx + Q_COV_QX, {K_FRAC_BITS{1'b0}}};
                            div_den       <= ((p_state_qx + Q_COV_QX) + R_COV_QX);
                            div_rem       <= {(DEN_W+1){1'b0}};
                            div_quot      <= {K_FRAC_BITS{1'b0}};
                            div_count     <= K_FRAC_BITS[DIVC_W-1:0];
                            z_meas_reg    <= pending_meas;
                            pending_valid <= 1'b0;
                            state         <= ST_DIV;
                        end
                    end
                end

                ST_DIV: begin
                    //----------------------------------------------------------
                    // Restoring divider iteration
                    //
                    // At each cycle:
                    //   1) Shift one numerator bit into the remainder.
                    //   2) Compare remainder against denominator.
                    //   3) If remainder is large enough, subtract denominator
                    //      and emit a quotient bit of 1. Otherwise emit 0.
                    //
                    // After K_FRAC_BITS iterations, the quotient contains the
                    // fixed-point Kalman gain K in Q(K_FRAC_BITS).
                    //----------------------------------------------------------
                    div_rem       <= {div_rem[DEN_W-1:0], div_num_shift[NUM_W-1]};
                    div_num_shift <= {div_num_shift[NUM_W-2:0], 1'b0};

                    if ({div_rem[DEN_W-1:0], div_num_shift[NUM_W-1]} >= {1'b0, div_den}) begin
                        div_rem  <= {div_rem[DEN_W-1:0], div_num_shift[NUM_W-1]} - {1'b0, div_den};
                        div_quot <= {div_quot[K_FRAC_BITS-2:0], 1'b1};
                    end else begin
                        div_quot <= {div_quot[K_FRAC_BITS-2:0], 1'b0};
                    end

                    if (div_count == {{(DIVC_W-1){1'b0}},1'b1}) begin
                        k_gain_qk <= {div_quot[K_FRAC_BITS-2:0],
                                      (({div_rem[DEN_W-1:0], div_num_shift[NUM_W-1]} >= {1'b0, div_den}) ? 1'b1 : 1'b0)};
                        state     <= ST_APPLY;
                    end

                    div_count <= div_count - {{(DIVC_W-1){1'b0}},1'b1};
                end

                ST_APPLY: begin
                    //----------------------------------------------------------
                    // Kalman state/covariance update
                    //
                    // Detailed arithmetic sequence:
                    //   1) Convert measurement z to QX fixed-point.
                    //   2) Compute innovation e = z - x.
                    //   3) Compute correction K*e in widened precision.
                    //   4) Round and shift the correction into QX domain.
                    //   5) Apply correction to x.
                    //   6) Update covariance with (1-K)P_pred.
                    //   7) Publish the rounded whole-inch state.
                    //----------------------------------------------------------
                    z_meas_qx       = {z_meas_reg, {X_FRAC_BITS{1'b0}}};
                    innovation_qx   = $signed({1'b0, z_meas_qx}) - $signed({1'b0, x_state_qx});
                    correction_full = innovation_qx * $signed({1'b0, k_gain_qk});

                    if (correction_full >= 0)
                        correction_rounded = correction_full + ({{(X_W+K_FRAC_BITS+1-K_FRAC_BITS){1'b0}}, 1'b1} << (K_FRAC_BITS-1));
                    else
                        correction_rounded = correction_full - ({{(X_W+K_FRAC_BITS+1-K_FRAC_BITS){1'b0}}, 1'b1} << (K_FRAC_BITS-1));

                    x_next_signed =
                        $signed({1'b0, x_state_qx}) +
                        ($signed(correction_rounded) >>> K_FRAC_BITS);

                    if (x_next_signed < 0)
                        x_next_qx = {X_W{1'b0}};
                    else
                        x_next_qx = x_next_signed[X_W-1:0];

                    one_minus_k_qk = (1 << K_FRAC_BITS) - k_gain_qk;
                    p_mult_full    = p_pred_qx * one_minus_k_qk;
                    p_next_full    = p_mult_full + (1 << (K_FRAC_BITS-1));
                    p_next_qx      = p_next_full >> K_FRAC_BITS;

                    x_state_qx <= x_next_qx;
                    p_state_qx <= p_next_qx;

                    // Publish whole-inch result with rounding from QX.
                    if (x_next_qx[X_FRAC_BITS-1])
                        distance_out_filt <= x_next_qx[X_W-1:X_FRAC_BITS] + 10'd1;
                    else
                        distance_out_filt <= x_next_qx[X_W-1:X_FRAC_BITS];

                    distance_valid_out <= 1'b1;
                    state              <= ST_IDLE;
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
`default_nettype wire
