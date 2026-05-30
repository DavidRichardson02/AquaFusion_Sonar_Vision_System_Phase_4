`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// radar_phosphor_plane_pix
//------------------------------------------------------------------------------
// ROLE
//   Maintain and render a phosphor-intensity plane in the PIX domain using a
//   single-port read path and a read-modify-write maintenance / deposit writer.
//
// HIGH-LEVEL PURPOSE
//   This module combines three responsibilities:
//
//     1) map the current scan pixel into a phosphor-plane read address
//     2) multiplex BRAM read ownership between:
//          - the renderer
//          - the phosphor writer
//     3) return the phosphor intensity aligned to the pixel stream by one cycle
//
// WHY A SEPARATE PHOSPHOR PLANE EXISTS
//   The main radar overlay can render instantaneous geometry directly from
//   current telemetry, but a phosphor plane adds temporal persistence:
//
//     recent hits remain visible
//     recent sweep traces glow briefly
//     stale energy decays gradually
//
// PIX-DOMAIN CONTRACT
//   - All logic is owned by clk_pix / rst_pix.
//   - commit_pulse is already a 1-cycle PIX-domain event.
//   - BRAM read data is synchronous, so the exported intensity corresponds to
//     the previous cycle's read address.
//   - phos_I_d1 / in_phos_d1 are therefore intentionally delayed by one cycle.
//
// ADDRESS OWNERSHIP CONTRACT
//   The BRAM read port is shared between:
//
//     renderer path:
//       phos_raddr_render
//
//     writer path:
//       wr_raddr, exported by phos_plane_writer
//
//   When wr_rd_busy is asserted, writer ownership takes priority.
//==============================================================================

module radar_phosphor_plane_pix #(
    parameter integer X0 = 16,
    parameter integer Y0 = 16,
    parameter integer W  = 256,
    parameter integer H  = 256,

    parameter integer PHOS_W = 128,
    parameter integer PHOS_H = 128,
    parameter integer PHOS_AW = 14,

    parameter [7:0] PHOS_DECAY     = 8'd2,
    parameter [7:0] PHOS_HIT_ADD   = 8'd80,
    parameter [7:0] PHOS_SWEEP_ADD = 8'd6,
    parameter integer PHOS_MAINT_K = 256
) (
    input  wire        clk_pix,
    input  wire        rst_pix,

    input  wire [9:0]  pix_x,
    input  wire [9:0]  pix_y,
    input  wire        active_video,

    input  wire        frame_tick,

    input  wire        commit_pulse,

    input  wire signed [15:0] ex_s,
    input  wire signed [15:0] ey_s,

    input  wire        sample_en_mask,

    input  wire        sweep_deposit_en,
    input  wire [7:0]  sweep_u,
    input  wire [7:0]  sweep_v,

    output reg  [7:0]  phos_I_d1,
    output reg         in_phos_d1
);

    //==========================================================================
    // Widget-local coordinates
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Convert absolute screen coordinates into widget-local coordinates.
    //
    // NOTE
    //   These are simple unsigned subtractions. The enclosing logic is expected
    //   to use sample_en_mask so only valid in-widget positions are sampled.
    //==========================================================================
    wire [9:0] lx = pix_x - X0[9:0];
    wire [9:0] ly = pix_y - Y0[9:0];

    //==========================================================================
    // Pixel -> phosphor UV mapping
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Map the currently scanned widget-local pixel into phosphor-plane UV.
    //
    // STEP-BY-STEP
    //   1) Use widget-local pixel coordinate lx / ly.
    //   2) Rescale into phosphor-plane coordinates by proportional mapping.
    //   3) If W or H were ever zero, force the result to zero to avoid an
    //      illegal divide-by-zero.
    //==========================================================================
    wire [7:0] phos_u_pix = (W != 0) ? ((lx * PHOS_W) / W) : 8'd0;
    wire [7:0] phos_v_pix = (H != 0) ? ((ly * PHOS_H) / H) : 8'd0;

    //==========================================================================
    // Endpoint -> phosphor UV mapping
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Convert a signed screen-space endpoint into a phosphor-plane deposit
    //   location for target-hit persistence writes.
    //
    // STEP-BY-STEP
    //   1) Translate the endpoint from global screen coordinates into
    //      widget-local coordinates.
    //   2) Clamp the local coordinate into legal widget bounds.
    //   3) Rescale from widget-local pixels into phosphor-plane UV.
    //==========================================================================
    wire signed [15:0] ex_l = ex_s - $signed(X0);
    wire signed [15:0] ey_l = ey_s - $signed(Y0);

    wire [9:0] ex_l_u10 = (ex_l < 0) ? 10'd0 : (ex_l > (W-1)) ? (W-1) : ex_l[9:0];
    wire [9:0] ey_l_u10 = (ey_l < 0) ? 10'd0 : (ey_l > (H-1)) ? (H-1) : ey_l[9:0];

    wire [7:0] phos_u_hit = (W != 0) ? ((ex_l_u10 * PHOS_W) / W) : 8'd0;
    wire [7:0] phos_v_hit = (H != 0) ? ((ey_l_u10 * PHOS_H) / H) : 8'd0;

    //==========================================================================
    // Renderer read address
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Generate the BRAM read address for the renderer path.
    //
    // ADDRESS FORM
    //   linear_addr = v * PHOS_W + u
    //
    // GATING
    //   If either:
    //     - active_video is low, or
    //     - sample_en_mask is low
    //
    //   then the read address falls back to zero. The returned read data is
    //   irrelevant in those masked regions because in_phos_d1 will also be low.
    //==========================================================================
    reg  [PHOS_AW-1:0] phos_raddr_render;

    always @* begin
        phos_raddr_render = {PHOS_AW{1'b0}};
        if (active_video && sample_en_mask) begin
            phos_raddr_render = (phos_v_pix * PHOS_W) + phos_u_pix;
        end
    end

    //==========================================================================
    // BRAM interface signals
    //--------------------------------------------------------------------------
    // phos_we / phos_waddr / phos_wdata
    //   Write port owned by phos_plane_writer.
    //
    // phos_raddr_mux
    //   Shared read-address input to BRAM.
    //
    // phos_rdata
    //   One-cycle-latent synchronous BRAM read result.
    //
    // wr_rd_busy / wr_raddr
    //   Writer-exported read-port ownership signals.
    //==========================================================================
    wire                 phos_we;
    wire [PHOS_AW-1:0]   phos_waddr;
    wire [7:0]           phos_wdata;

    wire [PHOS_AW-1:0]   phos_raddr_mux;
    wire [7:0]           phos_rdata;

    wire                 wr_rd_busy;
    wire [PHOS_AW-1:0]   wr_raddr;

    //--------------------------------------------------------------------------
    // Read-port ownership mux
    //--------------------------------------------------------------------------
    // POLICY
    //   When the writer is actively performing a read-modify-write sequence,
    //   the BRAM read address must come from the writer.
    //
    //   Otherwise, the renderer owns the read address.
    //--------------------------------------------------------------------------
    assign phos_raddr_mux = (wr_rd_busy) ? wr_raddr : phos_raddr_render;

    //==========================================================================
    // Phosphor BRAM
    //==========================================================================
    bram_1r1w_u8 #(
        .W (PHOS_W),
        .H (PHOS_H),
        .AW(PHOS_AW)
    ) u_phos_bram (
        .clk   (clk_pix),
        .we    (phos_we),
        .waddr (phos_waddr),
        .wdata (phos_wdata),
        .raddr (phos_raddr_mux),
        .rdata (phos_rdata)
    );

    //==========================================================================
    // Phosphor writer
    //--------------------------------------------------------------------------
    // ROLE
    //   Own the read-modify-write behavior for:
    //     - target-hit deposits
    //     - sweep deposits
    //     - background decay / maintenance
    //
    // IMPORTANT INTEGRATION NOTE
    //   bram_raddr is intentionally left unconnected because this wrapper owns
    //   the external read-address mux and instead consumes wr_raddr / rd_busy.
    //==========================================================================
    phos_plane_writer #(
        .PHOS_W           (PHOS_W),
        .PHOS_H           (PHOS_H),
        .AW               (PHOS_AW),
        .DECAY            (PHOS_DECAY),
        .HIT_ADD          (PHOS_HIT_ADD),
        .SWEEP_ADD        (PHOS_SWEEP_ADD),
        .MAINT_K_PER_FRAME(PHOS_MAINT_K)
    ) u_phos_wr (
        .clk              (clk_pix),
        .rst              (rst_pix),

        .frame_tick       (frame_tick),

        .commit_pulse     (commit_pulse),
        .hit_u            (phos_u_hit),
        .hit_v            (phos_v_hit),

        .sweep_deposit_en (sweep_deposit_en),
        .sweep_u          (sweep_u),
        .sweep_v          (sweep_v),

        .bram_we          (phos_we),
        .bram_waddr       (phos_waddr),
        .bram_wdata       (phos_wdata),
        .bram_raddr       (),
        .bram_rdata       (phos_rdata),

        .rd_busy          (wr_rd_busy),
        .wr_raddr         (wr_raddr)
    );

    //--------------------------------------------------------------------------
    // Output alignment register
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Align the synchronous BRAM read data to the pixel stream.
    //
    // STEP-BY-STEP
    //   1) On each clk_pix, capture phos_rdata into phos_I_d1.
    //   2) In parallel, capture whether the previous pixel belonged to the
    //      phosphor sample region.
    //
    // RESULT
    //   phos_I_d1 and in_phos_d1 are time-aligned.
    //--------------------------------------------------------------------------
    always @(posedge clk_pix) begin
        if (rst_pix) begin
            phos_I_d1  <= 8'd0;
            in_phos_d1 <= 1'b0;
        end else begin
            phos_I_d1  <= phos_rdata;
            in_phos_d1 <= (active_video && sample_en_mask);
        end
    end

endmodule

`default_nettype wire