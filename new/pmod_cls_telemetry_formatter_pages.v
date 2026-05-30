`timescale 1ns / 1ps
`default_nettype none


//==============================================================================
// pmod_cls_telemetry_formatter_pages
//------------------------------------------------------------------------------
// ROLE
//   Convert live sonar/camera telemetry and fault bits into exactly two fixed
//   16-character CLS lines, with support for two logical pages.
//
// PAGE 0 — operator dashboard
//   line0 = "R123^ H:OK SV1 "
//   line1 = "A0456 F123456C1 "
//
// PAGE 1 — diagnostics
//   line0 = "TO1 PE0 FE0 P1  "
//   line1 = "R123 A0456 CV1  "
//
// HEALTH-CODE PRIORITY
//   ERR > TO > INV > OK > AGE > STL
//
// NOTES
//   - Output is purely combinational.
//   - Widths are fixed at 16 ASCII bytes per line.
//==============================================================================

module pmod_cls_telemetry_formatter_pages #(
    parameter integer AGE_WARN_MS = 250,
    parameter integer STALE_MS    = 1000
)(
    input  wire        page_sel,

    input  wire [7:0]  sonar_range_in,
    input  wire        sonar_valid,
    input  wire [15:0] sonar_stale_ms,
    input  wire        sonar_timeout,
    input  wire        sonar_parse_err_sticky,
    input  wire        sonar_uart_frame_err_sticky,

    input  wire [31:0] cam_frame_ctr,
    input  wire        cam_valid,

    input  wire [7:0]  trend_ascii,

    output reg  [127:0] line0_text,
    output reg  [127:0] line1_text
);

    wire [23:0] sonar_digits_ascii;
    wire [31:0] stale_digits_ascii;
    wire [47:0] frame_digits_ascii;

    wire [7:0] sonar_valid_ascii;
    wire [7:0] cam_valid_ascii;
    wire [7:0] timeout_ascii;
    wire [7:0] parse_err_ascii;
    wire [7:0] frame_err_ascii;
    wire [7:0] page_ascii;

    reg  [23:0] health_ascii;

    cls_ascii_u8_3d u_cls_ascii_u8_3d (
        .value        (sonar_range_in),
        .ascii_digits (sonar_digits_ascii)
    );

    cls_ascii_u16_4d_sat u_cls_ascii_u16_4d_sat (
        .value        (sonar_stale_ms),
        .ascii_digits (stale_digits_ascii)
    );

    cls_ascii_u32_6d_mod u_cls_ascii_u32_6d_mod (
        .value        (cam_frame_ctr),
        .ascii_digits (frame_digits_ascii)
    );

    assign sonar_valid_ascii = sonar_valid                 ? 8'h31 : 8'h30; // '1'/'0'
    assign cam_valid_ascii   = cam_valid                   ? 8'h31 : 8'h30; // '1'/'0'
    assign timeout_ascii     = sonar_timeout               ? 8'h31 : 8'h30; // '1'/'0'
    assign parse_err_ascii   = sonar_parse_err_sticky      ? 8'h31 : 8'h30; // '1'/'0'
    assign frame_err_ascii   = sonar_uart_frame_err_sticky ? 8'h31 : 8'h30; // '1'/'0'
    assign page_ascii        = page_sel                    ? 8'h31 : 8'h30; // '1'/'0'

    //--------------------------------------------------------------------------
    // Health-code encoder
    //--------------------------------------------------------------------------
    // STEP-BY-STEP
    //   1) Check strong sticky error conditions first.
    //   2) Then check timeout.
    //   3) Then check data validity.
    //   4) If data is valid and not faulted, classify freshness age.
    //
    // This ordering ensures the operator line always shows the most important
    // health interpretation, not merely the first passing condition.
    //--------------------------------------------------------------------------
    always @* begin
        if (sonar_parse_err_sticky || sonar_uart_frame_err_sticky) begin
            health_ascii = {8'h45, 8'h52, 8'h52}; // "ERR"
        end else if (sonar_timeout) begin
            health_ascii = {8'h54, 8'h4F, 8'h20}; // "TO "
        end else if (!sonar_valid) begin
            health_ascii = {8'h49, 8'h4E, 8'h56}; // "INV"
        end else if (sonar_stale_ms < AGE_WARN_MS[15:0]) begin
            health_ascii = {8'h4F, 8'h4B, 8'h20}; // "OK "
        end else if (sonar_stale_ms < STALE_MS[15:0]) begin
            health_ascii = {8'h41, 8'h47, 8'h45}; // "AGE"
        end else begin
            health_ascii = {8'h53, 8'h54, 8'h4C}; // "STL"
        end
    end

    //--------------------------------------------------------------------------
    // Page formatter
    //--------------------------------------------------------------------------
    always @* begin
        if (!page_sel) begin
            //------------------------------------------------------------------
            // PAGE 0 — operator dashboard
            //   line0 = "R123^ H:OK SV1 "
            //   line1 = "A0456 F123456C1 "
            //------------------------------------------------------------------
            line0_text = {
                8'h52,                 // 'R'
                sonar_digits_ascii,    // 3 chars
                trend_ascii,           // '^' or 'v' or '='
                8'h20,                 // ' '
                8'h48,                 // 'H'
                8'h3A,                 // ':'
                health_ascii,          // 3 chars
                8'h20,                 // ' '
                8'h53,                 // 'S'
                8'h56,                 // 'V'
                sonar_valid_ascii,     // '0'/'1'
                8'h20                  // trailing space
            };

            line1_text = {
                8'h41,                 // 'A'
                stale_digits_ascii,    // 4 chars
                8'h20,                 // ' '
                8'h46,                 // 'F'
                frame_digits_ascii,    // 6 chars
                8'h43,                 // 'C'
                cam_valid_ascii,       // '0'/'1'
                8'h20                  // trailing space
            };
        end else begin
            //------------------------------------------------------------------
            // PAGE 1 — diagnostics
            //   line0 = "TO1 PE0 FE0 P1  "
            //   line1 = "R123 A0456 CV1  "
            //------------------------------------------------------------------
            line0_text = {
                8'h54,                 // 'T'
                8'h4F,                 // 'O'
                timeout_ascii,         // '0'/'1'
                8'h20,                 // ' '
                8'h50,                 // 'P'
                8'h45,                 // 'E'
                parse_err_ascii,       // '0'/'1'
                8'h20,                 // ' '
                8'h46,                 // 'F'
                8'h45,                 // 'E'
                frame_err_ascii,       // '0'/'1'
                8'h20,                 // ' '
                8'h50,                 // 'P'
                page_ascii,            // '1'
                8'h20,                 // trailing space
                8'h20                  // trailing space
            };

            line1_text = {
                8'h52,                 // 'R'
                sonar_digits_ascii,    // 3 chars
                8'h20,                 // ' '
                8'h41,                 // 'A'
                stale_digits_ascii,    // 4 chars
                8'h20,                 // ' '
                8'h43,                 // 'C'
                8'h56,                 // 'V'
                cam_valid_ascii,       // '0'/'1'
                8'h20,                 // trailing space
                8'h20                  // trailing space
            };
        end
    end

endmodule

`default_nettype wire