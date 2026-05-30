`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// sweep_glow_depositor_pix
//------------------------------------------------------------------------------
// ROLE
//   Generate a bounded burst of phosphor "sweep deposit" coordinates once per
//   video frame in the PIX domain.
//
// SYSTEM CONTEXT
//   A radar-like phosphor plane often benefits from two distinct write sources:
//
//     1) target-hit deposits
//        - sparse, event-driven, tied to actual detections
//
//     2) sweep-glow deposits
//        - low-energy deposits distributed along the current sweep direction
//
//   This module produces the second class of deposits. It does not write BRAM
//   directly. Instead, it publishes a short sequence of UV coordinates:
//
//       deposit_pulse
//       deposit_u
//       deposit_v
//
//   for downstream phosphor-plane write logic.
//
// TEMPORAL CONTRACT
//   - All state is owned by clk_pix / rst_pix.
//   - A new burst is armed only on frame_tick.
//   - During an active burst, one deposit_pulse may be emitted per clk_pix.
//   - deposit_u / deposit_v are combinationally derived from the current r_px
//     and current direction vector, and are intended to be sampled when
//     deposit_pulse is asserted.
//
// GEOMETRIC CONTRACT
//   - dir_x_q15 / dir_y_q15 are signed Q1.15 direction components.
//   - r_px is an integer radial distance in widget pixels.
//   - The deposit point is formed by:
//         center + r * direction
//     then mapped from widget-local pixel coordinates into phosphor-plane UV.
//
// NUMERICAL CONTRACT
//   - Intermediate products use widened signed registers.
//   - Division-by-zero protection is included for degenerate W/H cases.
//   - Final UV outputs are clamped into valid phosphor address space.
//
// WHY THIS MODULE EXISTS
//   Separating sweep-coordinate generation from the phosphor writer keeps the
//   responsibilities clean:
//
//     coordinate generation
//         !=
//     read-modify-write arbitration / saturation / decay
//
//   This improves reviewability and makes timing easier to reason about.
//==============================================================================
module sweep_glow_depositor_pix #(
    parameter integer X0 = 16,
    parameter integer Y0 = 16,
    parameter integer W  = 256,
    parameter integer H  = 256,

    parameter integer R_MAX_PX = 120,

    parameter integer PHOS_W = 128,
    parameter integer PHOS_H = 128,

    parameter integer DEPOSITS_PER_FRAME = 8,
    parameter integer R_STEP_PX          = 4,
    parameter integer ENABLE             = 1
) (
    input  wire        clk_pix,
    input  wire        rst_pix,
    input  wire        frame_tick,

    input  wire signed [15:0] dir_x_q15,
    input  wire signed [15:0] dir_y_q15,

    output reg         deposit_pulse,
    output reg  [7:0]  deposit_u,
    output reg  [7:0]  deposit_v
);

    //==========================================================================
    // Sized local constants
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Convert parameter-space maxima into explicit vector constants so later
    //   comparisons and assignments remain width-safe and synthesis-friendly.
    //
    // NOTE
    //   The "(> 0) ? ... : 0" guard avoids illegal negative constant formation
    //   if a degenerate parameter were ever supplied.
    //==========================================================================
    localparam [7:0] PHOS_WM1_U8 = (PHOS_W > 0) ? (PHOS_W-1) : 0;
    localparam [7:0] PHOS_HM1_U8 = (PHOS_H > 0) ? (PHOS_H-1) : 0;

    //==========================================================================
    // Burst-control state
    //--------------------------------------------------------------------------
    // burst_active
    //   Indicates that the module is currently emitting a finite sequence of
    //   sweep deposits.
    //
    // burst_left
    //   Remaining deposits in the current burst.
    //
    // r_px
    //   Current radial distance in widget pixels used to place the next glow
    //   deposit along the sweep direction.
    //==========================================================================
    reg        burst_active;
    reg [7:0]  burst_left;
    reg [15:0] r_px;

    //--------------------------------------------------------------------------
    // FUNCTION: clamp_u10
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Clamp a signed coordinate into the inclusive unsigned range:
    //
    //       [0, max_v]
    //
    //   and return the result as a 10-bit unsigned value.
    //
    // WHY THIS FUNCTION EXISTS
    //   The endpoint computation is naturally signed:
    //
    //       center +/- projected_offset
    //
    //   so the raw result may be negative or may exceed the widget bounds.
    //   Before mapping into phosphor UV coordinates, the point must be limited
    //   to the drawable widget rectangle.
    //
    // STEP-BY-STEP LOGIC
    //   1) If v < 0:
    //        the point lies left/up of the widget-local origin,
    //        so clamp to 0.
    //
    //   2) Else if v > max_v:
    //        the point lies beyond the right/bottom edge,
    //        so clamp to max_v.
    //
    //   3) Else:
    //        the point already lies inside the legal range,
    //        so pass the lower 10 bits through unchanged.
    //
    // WIDTH / RANGE NOTE
    //   The returned width is 10 bits because the widget-local pixel logic in
    //   this design family commonly uses 10-bit unsigned coordinates.
    //--------------------------------------------------------------------------
    function [9:0] clamp_u10;
        input signed [15:0] v;
        input integer max_v;
        begin
            if (v < 0)
                clamp_u10 = 10'd0;
            else if (v > max_v)
                clamp_u10 = max_v[9:0];
            else
                clamp_u10 = v[9:0];
        end
    endfunction

    //==========================================================================
    // Combinational geometry intermediates
    //--------------------------------------------------------------------------
    // dx_q15 / dy_q15
    //   Signed Q1.15 projected offsets before de-scaling.
    //
    // dx_i / dy_i
    //   Integer pixel offsets after arithmetic right shift by 15.
    //
    // ex_l_s / ey_l_s
    //   Local widget-space signed endpoint.
    //
    // ex_l_u10 / ey_l_u10
    //   Widget-space endpoint after clamp into legal local pixel bounds.
    //
    // u_tmp / v_tmp
    //   Temporary phosphor-plane coordinates before final clamp to 8 bits.
    //==========================================================================
    reg signed [31:0] dx_q15;
    reg signed [31:0] dy_q15;
    reg signed [15:0] dx_i;
    reg signed [15:0] dy_i;

    reg signed [15:0] ex_l_s;
    reg signed [15:0] ey_l_s;

    reg [9:0] ex_l_u10;
    reg [9:0] ey_l_u10;

    reg [15:0] u_tmp;
    reg [15:0] v_tmp;

    //--------------------------------------------------------------------------
    // COMBINATIONAL CONTRACT
    //
    // PURPOSE
    //   Convert the current burst radius and current sweep direction into one
    //   phosphor-plane deposit coordinate.
    //
    // STEP-BY-STEP
    //   1) Form signed projected offsets:
    //          dx_q15 = r_px * dir_x_q15
    //          dy_q15 = r_px * dir_y_q15
    //
    //   2) Convert from Q1.15-scaled displacement into integer pixels by
    //      arithmetic right shift.
    //
    //   3) Translate from center-relative coordinates into widget-local
    //      coordinates. Positive Y direction in screen space points downward,
    //      so the vertical term uses subtraction.
    //
    //   4) Clamp the widget-local endpoint into:
    //          [0 .. W-1], [0 .. H-1]
    //
    //   5) Map widget-local pixel coordinates into phosphor UV coordinates by
    //      proportional rescaling:
    //          u = ex_local * PHOS_W / W
    //          v = ey_local * PHOS_H / H
    //
    //   6) Clamp the mapped UV results into:
    //          [0 .. PHOS_W-1], [0 .. PHOS_H-1]
    //
    // WHY CLAMP TWICE
    //   The first clamp protects widget geometry.
    //   The second clamp protects phosphor-plane address space after scaling.
    //--------------------------------------------------------------------------
    always @* begin
        dx_q15 = $signed({1'b0, r_px}) * $signed(dir_x_q15);
        dy_q15 = $signed({1'b0, r_px}) * $signed(dir_y_q15);

        dx_i   = dx_q15 >>> 15;
        dy_i   = dy_q15 >>> 15;

        ex_l_s = $signed(W/2) + dx_i;
        ey_l_s = $signed(H/2) - dy_i;

        ex_l_u10 = clamp_u10(ex_l_s, W-1);
        ey_l_u10 = clamp_u10(ey_l_s, H-1);

        u_tmp = (W != 0) ? ((ex_l_u10 * PHOS_W) / W) : 16'd0;
        v_tmp = (H != 0) ? ((ey_l_u10 * PHOS_H) / H) : 16'd0;

        if (u_tmp[15:8] != 8'd0)
            deposit_u = PHOS_WM1_U8;
        else if (u_tmp[7:0] > PHOS_WM1_U8)
            deposit_u = PHOS_WM1_U8;
        else
            deposit_u = u_tmp[7:0];

        if (v_tmp[15:8] != 8'd0)
            deposit_v = PHOS_HM1_U8;
        else if (v_tmp[7:0] > PHOS_HM1_U8)
            deposit_v = PHOS_HM1_U8;
        else
            deposit_v = v_tmp[7:0];
    end

    //--------------------------------------------------------------------------
    // SEQUENTIAL CONTRACT
    //
    // STATE OWNER
    //   clk_pix / rst_pix
    //
    // UPDATE RULE
    //   1) Reset clears the burst FSM and suppresses deposits.
    //   2) On frame_tick, if enabled, a new burst is armed:
    //        - burst_active asserted
    //        - burst_left loaded
    //        - r_px reset to zero
    //   3) While burst_active and burst_left != 0:
    //        - emit one deposit_pulse
    //        - decrement burst_left
    //        - advance r_px by R_STEP_PX
    //        - wrap r_px to zero once R_MAX_PX would be crossed
    //   4) Once burst_left reaches zero, burst_active is cleared.
    //
    // PULSE SEMANTICS
    //   deposit_pulse is explicitly cleared every cycle before any emission
    //   condition is checked, guaranteeing a one-cycle pulse.
    //--------------------------------------------------------------------------
    always @(posedge clk_pix) begin
        if (rst_pix) begin
            burst_active  <= 1'b0;
            burst_left    <= 8'd0;
            r_px          <= 16'd0;
            deposit_pulse <= 1'b0;
        end else begin
            deposit_pulse <= 1'b0;

            if ((ENABLE != 0) && frame_tick) begin
                burst_active <= 1'b1;
                burst_left   <= DEPOSITS_PER_FRAME[7:0];
                r_px         <= 16'd0;
            end else if (burst_active) begin
                if (burst_left != 8'd0) begin
                    deposit_pulse <= 1'b1;
                    burst_left    <= burst_left - 8'd1;

                    if (r_px + R_STEP_PX[15:0] >= R_MAX_PX[15:0])
                        r_px <= 16'd0;
                    else
                        r_px <= r_px + R_STEP_PX[15:0];
                end else begin
                    burst_active <= 1'b0;
                end
            end
        end
    end

endmodule

`default_nettype wire