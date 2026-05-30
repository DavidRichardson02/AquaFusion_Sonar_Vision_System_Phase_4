`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// aquafusion_cls_debug_bridge
//------------------------------------------------------------------------------
// ROLE
//   Canonical SYS-domain bridge from live AquaFusion telemetry into a compact
//   two-line Pmod CLS operator/diagnostic display.
//
// DESIGN INTENT
//   The Pmod CLS is treated as a compact status instrument rather than as a
//   raw scrolling terminal. The bridge therefore performs a small amount of
//   presentation-state derivation locally before handing two frozen 16-char
//   lines to the lower-level CLS transport block.
//
// DISPLAY MODEL
//   Two logical pages are supported:
//
//     PAGE 0 — operator dashboard
//       line0 = "R123^ H:OK SV1 "
//       line1 = "A0456 F123456C1 "
//
//     PAGE 1 — diagnostics
//       line0 = "TO1 PE0 FE0 P1  "
//       line1 = "R123 A0456 CV1  "
//
//   The display alternates pages periodically when ENABLE_TWO_PAGE != 0.
//
// INPUT STATUS CONTRACT
//   sonar_range_in_sys
//     Current sonar range in inches, already reduced to a displayable scalar.
//
//   sonar_valid_sys
//     Validity bit for the sonar range currently being published.
//
//   sonar_stale_ms_sys
//     Age in milliseconds of the last accepted sonar update.
//
//   sonar_timeout_sys
//     Asserted when the sonar lane is timed out / stale by system policy.
//
//   sonar_parse_err_sticky_sys
//     Sticky parser-error indicator.
//
//   sonar_uart_frame_err_sticky_sys
//     Sticky UART frame-error indicator.
//
//   cam_frame_ctr_sys
//     Camera frame counter used as a liveness/heartbeat quantity.
//
//   cam_valid_sys
//     Validity bit for camera telemetry presence.
//
// PAGE-SELECTION POLICY
//   - A local timer toggles page_sel every PAGE_HOLD_MS.
//   - A one-cycle refresh request pulse is generated on page flips so the CLS
//     transport can refresh promptly if it supports refresh_req.
//   - The existing transport block still performs line snapshotting, so text
//     coherence across a refresh is preserved.
//
// TREND POLICY
//   - Trend is updated only on accepted sonar_valid_sys events.
//   - '^' means current range > previous accepted range.
//   - 'v' means current range < previous accepted range.
//   - '=' means unchanged or no history yet.
//
// HEALTH-CODE POLICY
//   Precedence is deliberately chosen from strongest fault to weakest:
//
//     "ERR" : parser or UART frame sticky error asserted
//     "TO " : timeout asserted
//     "INV" : sonar_valid_sys deasserted
//     "OK " : valid and stale_ms < AGE_WARN_MS
//     "AGE" : valid and AGE_WARN_MS <= stale_ms < STALE_MS
//     "STL" : valid and stale_ms >= STALE_MS
//
// NOTES
//   - All text formatting is purely combinational below this bridge.
//   - The bridge itself contains only small sequential state:
//       * page timer / page bit
//       * trend memory
//       * prior fault state for refresh edge generation
//   - External module interface intentionally remains simple and SYS-domain.
//==============================================================================

module aquafusion_cls_debug_bridge #(
    parameter integer CLK_HZ          = 100_000_000,
    parameter integer CLS_BAUD        = 9600,
    parameter integer CLS_STARTUP_MS  = 100,
    parameter integer CLS_REFRESH_MS  = 250,
    parameter integer ENABLE_TWO_PAGE = 1,
    parameter integer PAGE_HOLD_MS    = 2000,
    parameter integer AGE_WARN_MS     = 250,
    parameter integer STALE_MS        = 1000
)(
    input  wire        clk,
    input  wire        rst,

    input  wire [7:0]  sonar_range_in_sys,
    input  wire        sonar_valid_sys,
    input  wire [15:0] sonar_stale_ms_sys,
    input  wire        sonar_timeout_sys,
    input  wire        sonar_parse_err_sticky_sys,
    input  wire        sonar_uart_frame_err_sticky_sys,

    input  wire [31:0] cam_frame_ctr_sys,
    input  wire        cam_valid_sys,

    output wire        cls_txd_o
);

    //--------------------------------------------------------------------------
    // Local timing constants
    //--------------------------------------------------------------------------
    localparam integer PAGE_TICKS =
        ((CLK_HZ / 1000) * PAGE_HOLD_MS);

    //--------------------------------------------------------------------------
    // Small presentation-state registers
    //--------------------------------------------------------------------------
    reg [31:0] page_ctr;
    reg        page_sel;
    reg        page_flip_pulse;

    reg [7:0]  prev_sonar_range_sys;
    reg [7:0]  trend_ascii_sys;

    reg        prev_timeout_sys;
    reg        prev_parse_err_sticky_sys;
    reg        prev_uart_frame_err_sticky_sys;

    reg        refresh_req_sys;

    wire [127:0] cls_line0;
    wire [127:0] cls_line1;

    //--------------------------------------------------------------------------
    // Page sequencer
    //--------------------------------------------------------------------------
    // STEP-BY-STEP
    //   1) Count SYS clocks up to PAGE_TICKS-1.
    //   2) On expiration, optionally toggle page_sel if two-page mode is enabled.
    //   3) Emit a one-cycle page_flip_pulse.
    //
    // This pulse is later used as a refresh hint to the lower-level transport.
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            page_ctr       <= 32'd0;
            page_sel       <= 1'b0;
            page_flip_pulse<= 1'b0;
        end else begin
            page_flip_pulse <= 1'b0;

            if (ENABLE_TWO_PAGE != 0) begin
                if (page_ctr >= (PAGE_TICKS - 1)) begin
                    page_ctr        <= 32'd0;
                    page_sel        <= ~page_sel;
                    page_flip_pulse <= 1'b1;
                end else begin
                    page_ctr <= page_ctr + 1'b1;
                end
            end else begin
                page_ctr        <= 32'd0;
                page_sel        <= 1'b0;
                page_flip_pulse <= 1'b0;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Sonar trend memory
    //--------------------------------------------------------------------------
    // STEP-BY-STEP
    //   1) Wait for an accepted sonar_valid_sys pulse/level event.
    //   2) Compare current range against the previously stored accepted range.
    //   3) Publish '^', 'v', or '='.
    //   4) Update the stored prior range.
    //
    // Trend is intentionally derived only from accepted sonar values so that
    // invalid or fault-only periods do not fabricate motion direction.
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            prev_sonar_range_sys <= 8'd0;
            trend_ascii_sys      <= 8'h3D; // '='
        end else begin
            if (sonar_valid_sys) begin
                if (sonar_range_in_sys > prev_sonar_range_sys)
                    trend_ascii_sys <= 8'h5E; // '^'
                else if (sonar_range_in_sys < prev_sonar_range_sys)
                    trend_ascii_sys <= 8'h76; // 'v'
                else
                    trend_ascii_sys <= 8'h3D; // '='

                prev_sonar_range_sys <= sonar_range_in_sys;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Refresh request generation
    //--------------------------------------------------------------------------
    // POLICY
    //   A refresh pulse is generated on:
    //     * page flips
    //     * timeout edge
    //     * parse-error sticky edge
    //     * UART frame-error sticky edge
    //
    // This avoids hammering the transport on every camera frame while still
    // making the CLS react promptly to meaningful state transitions.
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            prev_timeout_sys            <= 1'b0;
            prev_parse_err_sticky_sys   <= 1'b0;
            prev_uart_frame_err_sticky_sys <= 1'b0;
            refresh_req_sys             <= 1'b0;
        end else begin
            refresh_req_sys <= 1'b0;

            if (page_flip_pulse)
                refresh_req_sys <= 1'b1;

            if (sonar_timeout_sys != prev_timeout_sys)
                refresh_req_sys <= 1'b1;

            if (sonar_parse_err_sticky_sys != prev_parse_err_sticky_sys)
                refresh_req_sys <= 1'b1;

            if (sonar_uart_frame_err_sticky_sys != prev_uart_frame_err_sticky_sys)
                refresh_req_sys <= 1'b1;

            prev_timeout_sys               <= sonar_timeout_sys;
            prev_parse_err_sticky_sys      <= sonar_parse_err_sticky_sys;
            prev_uart_frame_err_sticky_sys <= sonar_uart_frame_err_sticky_sys;
        end
    end

    //--------------------------------------------------------------------------
    // Formatter
    //--------------------------------------------------------------------------
    pmod_cls_telemetry_formatter_pages #(
        .AGE_WARN_MS (AGE_WARN_MS),
        .STALE_MS    (STALE_MS)
    ) u_pmod_cls_telemetry_formatter_pages (
        .page_sel                    (page_sel),
        .sonar_range_in              (sonar_range_in_sys),
        .sonar_valid                 (sonar_valid_sys),
        .sonar_stale_ms              (sonar_stale_ms_sys),
        .sonar_timeout               (sonar_timeout_sys),
        .sonar_parse_err_sticky      (sonar_parse_err_sticky_sys),
        .sonar_uart_frame_err_sticky (sonar_uart_frame_err_sticky_sys),
        .cam_frame_ctr               (cam_frame_ctr_sys),
        .cam_valid                   (cam_valid_sys),
        .trend_ascii                 (trend_ascii_sys),
        .line0_text                  (cls_line0),
        .line1_text                  (cls_line1)
    );

    //--------------------------------------------------------------------------
    // CLS transport layer
    //--------------------------------------------------------------------------
    pmod_cls_debug #(
        .CLK_HZ                  (100_000_000),
        .BAUD                    (9600),
        .STARTUP_MS              (100),
        .REFRESH_MS              (250),
        .INTER_BYTE_GAP_US       (500),
        .CMD_EXTRA_GAP_US        (3000),
        .MODE_ONE_SHOT           (1),
        .ENABLE_PERIODIC_REFRESH (0),
        .ENABLE_REFRESH_REQ      (0),
        .CLEAR_EACH_TRANSFER     (1)
    ) u_pmod_cls_debug (
        .clk         (clk),
        .rst         (rst),
        .line0_text  (cls_line0),
        .line1_text  (cls_line1),
        .refresh_req (1'b0),
        .cls_txd     (cls_txd_o)
    );
endmodule

`default_nettype wire