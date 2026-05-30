`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// phos_plane_writer
//------------------------------------------------------------------------------
// ROLE
//   Own the phosphor-plane read-modify-write policy.
//
// PURPOSE
//   Maintain a phosphor intensity plane by combining three update classes:
//
//     1) hit deposits
//        - triggered by commit_pulse
//        - stronger intensity increment
//
//     2) sweep deposits
//        - triggered by sweep_deposit_en
//        - weaker intensity increment
//
//     3) maintenance decay
//        - applied gradually each frame to a bounded number of cells
//
// DESIGN DOCTRINE
//   This module is the sole owner of phosphor-plane write semantics. It decides
//   when to read a cell, how to transform the cell's intensity, and when to
//   write the new value back.
//
// TEMPORAL CONTRACT
//   - All state is owned by clk / rst.
//   - BRAM is assumed synchronous.
//   - Each read-modify-write operation uses:
//
//         S_IDLE  -> choose operation, drive read address
//         S_READ  -> allow read data to return
//         S_WRITE -> emit write enable and write data
//
// READ-PORT OWNERSHIP EXPORT
//   rd_busy and wr_raddr are exported so an enclosing wrapper may grant the
//   shared BRAM read port to this writer while an operation is in flight.
//
// SINGLE-DRIVER SAFETY
//   Pending-request latches are written in only one always block.
//   The FSM requests consumption through one-cycle clear strobes. This avoids
//   accidental multi-driver behavior on pending flags.
//
// PRIORITY POLICY
//   In S_IDLE, operation selection priority is:
//
//     1) pending hit deposit
//     2) pending sweep deposit
//     3) maintenance decay
//
//   This policy ensures externally visible event deposits are serviced ahead of
//   background maintenance.
//==============================================================================
module phos_plane_writer #(
    parameter integer PHOS_W = 128,
    parameter integer PHOS_H = 128,
    parameter integer AW     = 14,
    parameter [7:0]  DECAY      = 8'd2,
    parameter [7:0]  HIT_ADD    = 8'd80,
    parameter [7:0]  SWEEP_ADD  = 8'd6,
    parameter integer MAINT_K_PER_FRAME = 256
) (
    input  wire          clk,
    input  wire          rst,

    input  wire          frame_tick,

    input  wire          commit_pulse,
    input  wire [7:0]    hit_u,
    input  wire [7:0]    hit_v,

    input  wire          sweep_deposit_en,
    input  wire [7:0]    sweep_u,
    input  wire [7:0]    sweep_v,

    output reg           bram_we,
    output reg  [AW-1:0] bram_waddr,
    output reg  [7:0]    bram_wdata,
    output reg  [AW-1:0] bram_raddr,
    input  wire [7:0]    bram_rdata,

    output reg           rd_busy,
    output reg  [AW-1:0] wr_raddr
);

    //--------------------------------------------------------------------------
    // Total cell count in the phosphor plane.
    //--------------------------------------------------------------------------
    localparam integer DEPTH = PHOS_W * PHOS_H;

    //==========================================================================
    // Maintenance state
    //--------------------------------------------------------------------------
    // maint_idx
    //   Current cell index for background decay scanning.
    //
    // maint_left
    //   Remaining number of maintenance operations allowed in the current frame.
    //==========================================================================
    reg [AW-1:0] maint_idx;
    reg [15:0]   maint_left;

    //==========================================================================
    // Request latches
    //--------------------------------------------------------------------------
    // hit_pending / sweep_pending
    //   Remember externally requested deposits until they are consumed by the
    //   FSM in S_IDLE.
    //
    // *_u_q / *_v_q
    //   Coordinate snapshots associated with the pending request.
    //==========================================================================
    reg       hit_pending;
    reg [7:0] hit_u_q,   hit_v_q;

    reg       sweep_pending;
    reg [7:0] sweep_u_q, sweep_v_q;

    //==========================================================================
    // FSM state
    //--------------------------------------------------------------------------
    // S_IDLE
    //   No active RMW operation. Choose next operation if one exists.
    //
    // S_READ
    //   Read address has been issued; wait for BRAM data return timing.
    //
    // S_WRITE
    //   Compute updated intensity and perform the write-back.
    //==========================================================================
    localparam [1:0] S_IDLE  = 2'd0,
                     S_READ  = 2'd1,
                     S_WRITE = 2'd2;

    reg [1:0]    st;
    reg [AW-1:0] op_addr;
    reg [7:0]    op_add;
    reg          op_is_decay;

    //--------------------------------------------------------------------------
    // One-cycle clear strobes
    //--------------------------------------------------------------------------
    // These are asserted by the FSM and consumed by the latch block, ensuring
    // the pending flags remain single-driver-safe.
    //--------------------------------------------------------------------------
    reg clr_hit_pending;
    reg clr_sweep_pending;

    //--------------------------------------------------------------------------
    // FUNCTION: addr_uv
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Convert 2-D phosphor coordinates (u, v) into a 1-D linear BRAM address.
    //
    // ADDRESSING RULE
    //   addr = v * PHOS_W + u
    //
    // STEP-BY-STEP
    //   1) Cast u and v to integers so the multiply/add is expressed clearly.
    //   2) Multiply the row index v by the row stride PHOS_W.
    //   3) Add the column index u.
    //   4) Return the result truncated to AW bits.
    //
    // ASSUMPTION
    //   Incoming u and v are already bounded to legal phosphor-plane range by
    //   upstream logic.
    //--------------------------------------------------------------------------
    function automatic [AW-1:0] addr_uv(input [7:0] u, input [7:0] v);
        integer iu, iv;
        begin
            iu = u;
            iv = v;
            addr_uv = (iv * PHOS_W) + iu;
        end
    endfunction

    //--------------------------------------------------------------------------
    // FUNCTION: sat_add
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Perform saturating 8-bit addition.
    //
    // STEP-BY-STEP
    //   1) Extend the operands to 9 bits by storing the sum in s.
    //   2) If the carry-out bit s[8] is set, overflow occurred.
    //   3) On overflow, clamp to 8'hFF.
    //   4) Otherwise return the lower 8 bits of the exact sum.
    //
    // WHY SATURATION IS USED
    //   Phosphor intensity should accumulate energy up to a visible maximum but
    //   should never wrap around through zero.
    //--------------------------------------------------------------------------
    function automatic [7:0] sat_add(input [7:0] a, input [7:0] b);
        reg [8:0] s;
        begin
            s = a + b;
            sat_add = s[8] ? 8'hFF : s[7:0];
        end
    endfunction

    //--------------------------------------------------------------------------
    // FUNCTION: sat_sub
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Perform saturating 8-bit subtraction.
    //
    // STEP-BY-STEP
    //   1) Compare a and b.
    //   2) If a > b, return the exact difference.
    //   3) Otherwise, clamp to zero rather than underflowing.
    //
    // WHY SATURATION IS USED
    //   Decay should fade intensity toward black, never wrap an unsigned value
    //   back upward.
    //--------------------------------------------------------------------------
    function automatic [7:0] sat_sub(input [7:0] a, input [7:0] b);
        begin
            sat_sub = (a > b) ? (a - b) : 8'd0;
        end
    endfunction

    //--------------------------------------------------------------------------
    // Request latches
    //--------------------------------------------------------------------------
    // ROLE
    //   Capture incoming deposit requests and hold them until the FSM consumes
    //   them.
    //
    // STEP-BY-STEP
    //   1) Reset clears all pending flags and stored coordinates.
    //   2) Clear strobes are processed first, allowing the FSM to consume a
    //      request cleanly.
    //   3) New incoming requests are then captured.
    //
    // IMPORTANT CONSEQUENCE
    //   If a new request and a clear strobe occur in the same cycle for the
    //   same request class, the new request wins and remains pending. This is
    //   often desirable because it preserves the most recent event.
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            hit_pending   <= 1'b0;
            sweep_pending <= 1'b0;
            hit_u_q       <= 8'd0;
            hit_v_q       <= 8'd0;
            sweep_u_q     <= 8'd0;
            sweep_v_q     <= 8'd0;
        end else begin
            if (clr_hit_pending)
                hit_pending <= 1'b0;

            if (clr_sweep_pending)
                sweep_pending <= 1'b0;

            if (commit_pulse) begin
                hit_pending <= 1'b1;
                hit_u_q     <= hit_u;
                hit_v_q     <= hit_v;
            end

            if (sweep_deposit_en) begin
                sweep_pending <= 1'b1;
                sweep_u_q     <= sweep_u;
                sweep_v_q     <= sweep_v;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Decay budget per frame
    //--------------------------------------------------------------------------
    // ROLE
    //   Limit the amount of maintenance work performed each frame.
    //
    // STEP-BY-STEP
    //   1) Reset clears the budget.
    //   2) On frame_tick, reload the maintenance budget from the parameter.
    //   3) While idle, and only when no hit or sweep request is pending,
    //      decrement the budget after each maintenance selection.
    //
    // WHY THIS MATTERS
    //   Full-plane decay in one frame could be too expensive. A bounded budget
    //   spreads maintenance over time and keeps per-frame work controlled.
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            maint_left <= 16'd0;
        end else if (frame_tick) begin
            maint_left <= MAINT_K_PER_FRAME[15:0];
        end else if ((st == S_IDLE) && (maint_left != 16'd0) &&
                     !hit_pending && !sweep_pending) begin
            maint_left <= maint_left - 16'd1;
        end
    end

    //--------------------------------------------------------------------------
    // Read-port ownership export
    //--------------------------------------------------------------------------
    // ROLE
    //   Tell the enclosing wrapper whether the writer currently owns the shared
    //   BRAM read port, and if so, which address it is reading.
    //
    // STEP-BY-STEP
    //   1) rd_busy is asserted whenever the FSM is not in S_IDLE.
    //   2) wr_raddr mirrors the current bram_raddr so external muxing can use
    //      the same address.
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            rd_busy  <= 1'b0;
            wr_raddr <= {AW{1'b0}};
        end else begin
            rd_busy  <= (st != S_IDLE);
            wr_raddr <= bram_raddr;
        end
    end

    //--------------------------------------------------------------------------
    // Read-modify-write FSM
    //--------------------------------------------------------------------------
    // ROLE
    //   Select operations and perform the three-stage synchronous BRAM RMW flow.
    //
    // PER-CYCLE DEFAULTS
    //   bram_we and both clear strobes are cleared every cycle unless
    //   explicitly asserted in the current state.
    //
    // OPERATION SELECTION IN S_IDLE
    //   Priority order:
    //     1) hit deposit
    //     2) sweep deposit
    //     3) maintenance decay
    //
    // STATE TRANSITIONS
    //   S_IDLE:
    //     choose next operation, drive bram_raddr, move to S_READ
    //
    //   S_READ:
    //     wait one cycle for synchronous BRAM read data to stabilize,
    //     then move to S_WRITE
    //
    //   S_WRITE:
    //     compute new intensity from bram_rdata and write it back,
    //     then return to S_IDLE
    //
    // MAINTENANCE INDEX POLICY
    //   maint_idx increments on each chosen maintenance operation and wraps to
    //   zero at the end of the plane.
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            st                <= S_IDLE;
            maint_idx         <= {AW{1'b0}};
            bram_we           <= 1'b0;
            bram_waddr        <= {AW{1'b0}};
            bram_wdata        <= 8'd0;
            bram_raddr        <= {AW{1'b0}};
            op_addr           <= {AW{1'b0}};
            op_add            <= 8'd0;
            op_is_decay       <= 1'b0;
            clr_hit_pending   <= 1'b0;
            clr_sweep_pending <= 1'b0;
        end else begin
            bram_we           <= 1'b0;
            clr_hit_pending   <= 1'b0;
            clr_sweep_pending <= 1'b0;

            case (st)
                S_IDLE: begin
                    if (hit_pending) begin
                        op_addr     <= addr_uv(hit_u_q, hit_v_q);
                        op_add      <= HIT_ADD;
                        op_is_decay <= 1'b0;

                        bram_raddr  <= addr_uv(hit_u_q, hit_v_q);

                        clr_hit_pending <= 1'b1;
                        st <= S_READ;

                    end else if (sweep_pending) begin
                        op_addr     <= addr_uv(sweep_u_q, sweep_v_q);
                        op_add      <= SWEEP_ADD;
                        op_is_decay <= 1'b0;

                        bram_raddr  <= addr_uv(sweep_u_q, sweep_v_q);

                        clr_sweep_pending <= 1'b1;
                        st <= S_READ;

                    end else if (maint_left != 16'd0) begin
                        op_addr     <= maint_idx;
                        op_add      <= DECAY;
                        op_is_decay <= 1'b1;

                        bram_raddr  <= maint_idx;

                        if (maint_idx == (DEPTH-1))
                            maint_idx <= {AW{1'b0}};
                        else
                            maint_idx <= maint_idx + {{(AW-1){1'b0}},1'b1};

                        st <= S_READ;
                    end
                end

                S_READ: begin
                    st <= S_WRITE;
                end

                S_WRITE: begin
                    bram_waddr <= op_addr;
                    bram_wdata <= op_is_decay ? sat_sub(bram_rdata, op_add)
                                              : sat_add(bram_rdata, op_add);
                    bram_we    <= 1'b1;
                    st         <= S_IDLE;
                end

                default: begin
                    st <= S_IDLE;
                end
            endcase
        end
    end

endmodule

`default_nettype wire