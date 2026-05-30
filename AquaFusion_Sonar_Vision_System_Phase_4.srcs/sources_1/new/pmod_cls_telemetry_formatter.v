`timescale 1ns/1ps
`default_nettype none


//==============================================================================
// pmod_cls_telemetry_formatter
//------------------------------------------------------------------------------
// ROLE
//   Format live sonar and camera telemetry into two fixed 16-character lines
//   for the Pmod CLS.
//
// LINE LAYOUT
//   line0 = "R123in AGE0456ms"
//   line1 = "F012345 SV1 CV1 "
//
// FIELD DEFINITIONS
//   R123      sonar range in inches, zero-padded to 3 digits
//   AGE0456   stale age in milliseconds, zero-padded to 4 digits
//   F012345   camera frame counter modulo 1,000,000
//   SV1       sonar-valid flag
//   CV1       camera-valid flag
//
// NOTES
//   - Output text is purely combinational.
//   - The existing pmod_cls_debug block snapshots line text at refresh start,
//     so display tearing is avoided by the transport block’s snapshot behavior.
//==============================================================================

module pmod_cls_telemetry_formatter (
    input  wire [7:0]   sonar_range_in,
    input  wire         sonar_valid,
    input  wire [15:0]  sonar_stale_ms,
    input  wire [31:0]  cam_frame_ctr,
    input  wire         cam_valid,

    output wire [127:0] line0_text,
    output wire [127:0] line1_text
);

    wire [23:0] sonar_digits_ascii;
    wire [31:0] stale_digits_ascii;
    wire [47:0] frame_digits_ascii;

    wire [7:0] sonar_valid_ascii;
    wire [7:0] cam_valid_ascii;

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

    assign sonar_valid_ascii = sonar_valid ? 8'h31 : 8'h30; // '1' : '0'
    assign cam_valid_ascii   = cam_valid   ? 8'h31 : 8'h30; // '1' : '0'

    // "R123in AGE0456ms"
    assign line0_text = {
        8'h52,                 // 'R'
        sonar_digits_ascii,    // 3 chars
        8'h69,                 // 'i'
        8'h6E,                 // 'n'
        8'h20,                 // ' '
        8'h41,                 // 'A'
        8'h47,                 // 'G'
        8'h45,                 // 'E'
        stale_digits_ascii,    // 4 chars
        8'h6D,                 // 'm'
        8'h73                  // 's'
    };

    // "F012345 SV1 CV1 "
    assign line1_text = {
        8'h46,                 // 'F'
        frame_digits_ascii,    // 6 chars
        8'h20,                 // ' '
        8'h53,                 // 'S'
        8'h56,                 // 'V'
        sonar_valid_ascii,     // '0' or '1'
        8'h20,                 // ' '
        8'h43,                 // 'C'
        8'h56,                 // 'V'
        cam_valid_ascii,       // '0' or '1'
        8'h20                  // trailing space
    };

endmodule

`default_nettype wire