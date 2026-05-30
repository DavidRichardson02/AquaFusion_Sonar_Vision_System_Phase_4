`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// MODULE: hud_camera_debug_tile
//------------------------------------------------------------------------------
// ROLE
//   Render a compact camera-status debug tile into the HDMI/VGA pixel stream.
//
// SYSTEM CONTEXT
//   This module is a VID-domain HUD renderer. It consumes a frame-committed
//   camera-status snapshot and converts it into a small textual/status panel.
//
//   Typical upstream path:
//
//       camera_status_regs
//           -> SYS-domain packed status snapshot
//           -> explicit SYS-to-VID snapshot CDC
//           -> cam_snap_vid / cam_snap_commit_vid
//           -> hud_camera_debug_tile
//
//   Typical downstream path:
//
//       rgb_out
//           -> video compositor
//           -> HDMI/TMDS output
//
// OWNERSHIP BOUNDARY
//   This module owns:
//     - VID-domain copy of the committed camera status snapshot.
//     - Decoding of the packed camera-status bit fields.
//     - ASCII formatting of short status lines.
//     - Tile background, border, text, and right-side status indicators.
//
//   This module does not own:
//     - SYS-to-VID CDC.
//     - Camera initialization state generation.
//     - CSI/D-PHY reception.
//     - Final HDMI composition.
//     - Font glyph implementation.
//
// CLOCK / RESET CONTRACT
//   clk_vid:
//     Pixel/video clock domain. All state in this module is owned by clk_vid.
//
//   rst_vid:
//     Active-high reset synchronous to clk_vid.
//
// INPUT CONTRACT
//   pix_x / pix_y:
//     Current raster coordinate in clk_vid domain.
//
//   de:
//     Active-video qualifier. Rendering is suppressed outside active video.
//
//   cam_snap_vid:
//     Packed camera-status snapshot already committed into clk_vid domain.
//
//   cam_snap_commit_vid:
//     One-cycle clk_vid pulse. When asserted, cam_snap_vid is sampled into this
//     module's local snapshot register.
//
// OUTPUT CONTRACT
//   rgb_out:
//     Combinational RGB888 color for this tile only.
//     Pixels outside the tile render black.
//     A downstream compositor decides whether/how to overlay this RGB surface.
//
// CDC CONTRACT
//   This module performs no CDC.
//   The caller must provide cam_snap_vid and cam_snap_commit_vid already safely
//   synchronized into clk_vid.
//
// SNAPSHOT WIDTH CONTRACT
//   SNAP_W must be at least 256 because this renderer decodes fields down from
//   bit 255. The default value is 256.
//
// RENDERING DISCIPLINE
//   The visible pixel is a pure function of:
//     - current pixel coordinate,
//     - active-video qualifier,
//     - frame-stable committed snapshot state,
//     - text glyph outputs.
//
//   The tile does not directly sample moving SYS-domain state.
//==============================================================================

module hud_camera_debug_tile #(
    //--------------------------------------------------------------------------
    // Tile placement
    //--------------------------------------------------------------------------
    // TILE_X0 / TILE_Y0:
    //   Upper-left corner of the camera debug tile in screen coordinates.
    //
    // TILE_W / TILE_H:
    //   Width and height of the tile in pixels.
    //--------------------------------------------------------------------------
    parameter integer TILE_X0 = 192,
    parameter integer TILE_Y0 = 16,
    parameter integer TILE_W  = 208,
    parameter integer TILE_H  = 112,

    //--------------------------------------------------------------------------
    // Snapshot width
    //--------------------------------------------------------------------------
    // SNAP_W:
    //   Width of cam_snap_vid. Must be >= 256 for the bit layout decoded below.
    //--------------------------------------------------------------------------
    parameter integer SNAP_W  = 256
)(
    //--------------------------------------------------------------------------
    // VID-domain raster inputs
    //--------------------------------------------------------------------------
    input  wire              clk_vid,
    input  wire              rst_vid,
    input  wire [11:0]       pix_x,
    input  wire [11:0]       pix_y,
    input  wire              de,

    //--------------------------------------------------------------------------
    // VID-domain committed camera snapshot
    //--------------------------------------------------------------------------
    input  wire [SNAP_W-1:0] cam_snap_vid,
    input  wire              cam_snap_commit_vid,

    //--------------------------------------------------------------------------
    // Tile-local RGB output
    //--------------------------------------------------------------------------
    output reg  [23:0]       rgb_out
);

    //==========================================================================
    // Local committed snapshot register
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Hold the last frame-committed camera-status snapshot locally.
    //
    // WHY THIS REGISTER EXISTS
    //   Rendering should be stable for a whole frame. Rather than decoding a
    //   potentially changing upstream bus every pixel, this tile only updates its
    //   local copy when cam_snap_commit_vid asserts.
    //
    // OWNERSHIP
    //   snap_commit_reg is owned by clk_vid.
    //==========================================================================
    reg [SNAP_W-1:0] snap_commit_reg;

    //==========================================================================
    // Camera snapshot bit-field contract
    //--------------------------------------------------------------------------
    // The status packer upstream must preserve this bit layout.
    //
    //   [255:248] mode_code
    //   [247:240] format_code
    //   [239:232] camera_port
    //   [231:224] init_state_dbg
    //
    //   [223:192] currently unused/reserved by this tile
    //
    //   [191:160] freshness_ms
    //   [159:128] frame_count
    //   [127:112] dropped_frames
    //   [111:096] overflow_count
    //   [095:080] step_index
    //   [079:064] retry_count
    //   [063:048] frame_w
    //   [047:032] frame_h
    //   [031:024] csi_error_flags
    //   [023:016] last_err
    //
    //   [15]      camera_ready
    //   [14]      frame_store_valid
    //   [13]      csi_locked
    //   [12]      init_fail
    //   [11]      sensor_id_ok
    //   [10]      sccb_init_done
    //   [9]       sccb_busy
    //   [8]       power_good
    //   [7]       overflow_sticky
    //   [6]       drop_sticky
    //   [5]       csi_error_sticky
    //   [4]       sccb_error_sticky
    //   [3:0]     reserved
    //==========================================================================
    wire [7:0]  mode_code;
    wire [7:0]  format_code;
    wire [7:0]  camera_port;
    wire [7:0]  init_state_dbg;
    wire [31:0] freshness_ms;
    wire [31:0] frame_count;
    wire [15:0] dropped_frames;
    wire [15:0] overflow_count;
    wire [15:0] step_index;
    wire [15:0] retry_count;
    wire [15:0] frame_w;
    wire [15:0] frame_h;
    wire [7:0]  csi_error_flags;
    wire [7:0]  last_err;

    wire camera_ready;
    wire frame_store_valid;
    wire csi_locked;
    wire init_fail;
    wire sensor_id_ok;
    wire sccb_init_done;
    wire sccb_busy;
    wire power_good;
    wire overflow_sticky;
    wire drop_sticky;
    wire csi_error_sticky;
    wire sccb_error_sticky;

    assign mode_code         = snap_commit_reg[255:248];
    assign format_code       = snap_commit_reg[247:240];
    assign camera_port       = snap_commit_reg[239:232];
    assign init_state_dbg    = snap_commit_reg[231:224];
    assign freshness_ms      = snap_commit_reg[191:160];
    assign frame_count       = snap_commit_reg[159:128];
    assign dropped_frames    = snap_commit_reg[127:112];
    assign overflow_count    = snap_commit_reg[111:96];
    assign step_index        = snap_commit_reg[95:80];
    assign retry_count       = snap_commit_reg[79:64];
    assign frame_w           = snap_commit_reg[63:48];
    assign frame_h           = snap_commit_reg[47:32];
    assign csi_error_flags   = snap_commit_reg[31:24];
    assign last_err          = snap_commit_reg[23:16];

    assign camera_ready      = snap_commit_reg[15];
    assign frame_store_valid = snap_commit_reg[14];
    assign csi_locked        = snap_commit_reg[13];
    assign init_fail         = snap_commit_reg[12];
    assign sensor_id_ok      = snap_commit_reg[11];
    assign sccb_init_done    = snap_commit_reg[10];
    assign sccb_busy         = snap_commit_reg[9];
    assign power_good        = snap_commit_reg[8];
    assign overflow_sticky   = snap_commit_reg[7];
    assign drop_sticky       = snap_commit_reg[6];
    assign csi_error_sticky  = snap_commit_reg[5];
    assign sccb_error_sticky = snap_commit_reg[4];

    //==========================================================================
    // FUNCTION: ascii_bool
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Convert a one-bit Boolean status into an ASCII character suitable for
    //   the text renderer.
    //
    // INPUT CONTRACT
    //   bit_in:
    //     1-bit status value.
    //
    // OUTPUT CONTRACT
    //   Returns:
    //     8'h31, ASCII "1", when bit_in is true.
    //     8'h30, ASCII "0", when bit_in is false.
    //
    // WHY ASCII IS USED
    //   vga_textline_3x5 accepts packed ASCII strings. Status bits therefore
    //   need to be converted into printable character codes before concatenation
    //   into line strings.
    //
    // STEP-BY-STEP
    //   1) Test the input bit.
    //   2) If the bit is 1, return ASCII code 0x31.
    //   3) Otherwise, return ASCII code 0x30.
    //
    // WORKED EXAMPLE
    //   camera_ready = 1'b1
    //     ascii_bool(camera_ready) = 8'h31 = "1"
    //
    //   csi_locked = 1'b0
    //     ascii_bool(csi_locked) = 8'h30 = "0"
    //
    // SYNTHESIS NOTE
    //   This function synthesizes to a simple 2:1 mux.
    //==========================================================================
    function [7:0] ascii_bool;
        input bit_in;
        begin
            ascii_bool = bit_in ? 8'h31 : 8'h30;
        end
    endfunction

    //==========================================================================
    // FUNCTION: hex_ascii
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Convert a 4-bit nibble into its uppercase hexadecimal ASCII character.
    //
    // INPUT CONTRACT
    //   nib:
    //     4-bit unsigned value in the range 0..15.
    //
    // OUTPUT CONTRACT
    //   Returns one ASCII byte:
    //     0..9  -> "0".."9", ASCII 0x30..0x39
    //     10..15 -> "A".."F", ASCII 0x41..0x46
    //
    // MATHEMATICAL BASIS
    //   Decimal digit range:
    //     ascii = "0" + nib
    //
    //   Hex letter range:
    //     ascii = "A" + (nib - 10)
    //
    // STEP-BY-STEP
    //   1) Compare nib against decimal 10.
    //   2) If nib is less than 10, create a numeric digit:
    //        8'h30 + nib
    //   3) Otherwise, create an uppercase hex letter:
    //        8'h41 + (nib - 10)
    //
    // WORKED EXAMPLES
    //   nib = 4'h3
    //     nib < 10, so output = 8'h30 + 3 = 8'h33 = "3"
    //
    //   nib = 4'hA
    //     nib >= 10, so output = 8'h41 + (10 - 10)
    //                        = 8'h41 = "A"
    //
    //   nib = 4'hF
    //     output = 8'h41 + (15 - 10)
    //            = 8'h46 = "F"
    //
    // SYNTHESIS NOTE
    //   This function synthesizes to a small comparator, subtractor, and adder.
    //==========================================================================
    function [7:0] hex_ascii;
        input [3:0] nib;
        begin
            if (nib < 4'd10)
                hex_ascii = 8'h30 + {4'd0, nib};
            else
                hex_ascii = 8'h41 + ({4'd0, nib} - 8'd10);
        end
    endfunction

    //==========================================================================
    // Numeric ASCII formatter outputs
    //--------------------------------------------------------------------------
    // These helper modules perform decimal formatting for fields that are too
    // wide or too frequently used to hand-format with local functions.
    //
    // FORMAT CONTRACT
    //   frame_ascii6:
    //     Six decimal digits from frame_count.
    //
    //   *_ascii4:
    //     Four decimal digits, usually saturating at 9999 inside the formatter.
    //==========================================================================
    wire [47:0] frame_ascii6;
    wire [31:0] age_ascii4;
    wire [31:0] drop_ascii4;
    wire [31:0] ovf_ascii4;
    wire [31:0] retry_ascii4;
    wire [31:0] step_ascii4;
    wire [31:0] width_ascii4;
    wire [31:0] height_ascii4;

    cls_ascii_u32_6d_mod u_fmt_frame (
        .value        (frame_count),
        .ascii_digits (frame_ascii6)
    );

    cls_ascii_u16_4d_sat u_fmt_age (
        .value        (freshness_ms[15:0]),
        .ascii_digits (age_ascii4)
    );

    cls_ascii_u16_4d_sat u_fmt_drop (
        .value        (dropped_frames),
        .ascii_digits (drop_ascii4)
    );

    cls_ascii_u16_4d_sat u_fmt_ovf (
        .value        (overflow_count),
        .ascii_digits (ovf_ascii4)
    );

    cls_ascii_u16_4d_sat u_fmt_retry (
        .value        (retry_count),
        .ascii_digits (retry_ascii4)
    );

    cls_ascii_u16_4d_sat u_fmt_step (
        .value        (step_index),
        .ascii_digits (step_ascii4)
    );

    cls_ascii_u16_4d_sat u_fmt_w (
        .value        (frame_w),
        .ascii_digits (width_ascii4)
    );

    cls_ascii_u16_4d_sat u_fmt_h (
        .value        (frame_h),
        .ascii_digits (height_ascii4)
    );

    //==========================================================================
    // Text line contracts
    //--------------------------------------------------------------------------
    // Each line is exactly 16 ASCII characters = 128 bits.
    //
    // The vga_textline_3x5 module receives str16 and draws one line of text.
    // The leftmost byte in the concatenation is the first displayed character.
    //
    // LINE SUMMARY
    //   line0: CAM frame counter
    //   line1: freshness age and dropped frames
    //   line2: overflow count and SCCB retry count
    //   line3: current init step and last error byte
    //   line4: frame size, mode, and format
    //   line5: port, init-state debug byte, lock/ready bits
    //   line6: SCCB done, sensor ID, CSI error sticky, overflow sticky
    //
    // WIDTH-SAFETY NOTE
    //   These concatenations are intentionally kept at exactly 16 bytes.
    //   Wider concatenations would silently truncate when assigned to [127:0];
    //   narrower concatenations would silently zero-pad.
    //==========================================================================

    // "CAM:" + 6 frame digits + 6 spaces
    wire [127:0] line0 = {
        8'h43, 8'h41, 8'h4D, 8'h3A,
        frame_ascii6,
        48'h20_20_20_20_20_20
    };

    // "AGE:" + age4 + " DR:" + drop4
    wire [127:0] line1 = {
        8'h41, 8'h47, 8'h45, 8'h3A,
        age_ascii4,
        8'h20,
        8'h44, 8'h52, 8'h3A,
        drop_ascii4
    };

    // "OVF:" + ovf4 + " RT:" + retry4
    wire [127:0] line2 = {
        8'h4F, 8'h56, 8'h46, 8'h3A,
        ovf_ascii4,
        8'h20,
        8'h52, 8'h54, 8'h3A,
        retry_ascii4
    };

    // "STP:" + step4 + " ER:" + err_hex2 + two spaces
    wire [127:0] line3 = {
        8'h53, 8'h54, 8'h50, 8'h3A,
        step_ascii4,
        8'h20,
        8'h45, 8'h52, 8'h3A,
        hex_ascii(last_err[7:4]),
        hex_ascii(last_err[3:0]),
        8'h20,
        8'h20
    };

    // "SZ:" + width4 + "X" + height4 + "M" + mode_hex + "F" + format_hex
    // Example: "SZ:0640X0480M0F0"
    wire [127:0] line4 = {
        8'h53, 8'h5A, 8'h3A,
        width_ascii4,
        8'h58,
        height_ascii4,
        8'h4D,
        hex_ascii(mode_code[3:0]),
        8'h46,
        hex_ascii(format_code[3:0])
    };

    // "P" + port_hex + " I" + init_state_hex2 + " CK" + lock + " RD" + ready + two spaces
    // Example: "P0 I04 CK1 RD1  "
    wire [127:0] line5 = {
        8'h50,
        hex_ascii(camera_port[3:0]),
        8'h20,
        8'h49,
        hex_ascii(init_state_dbg[7:4]),
        hex_ascii(init_state_dbg[3:0]),
        8'h20,
        8'h43, 8'h4B,
        ascii_bool(csi_locked),
        8'h20,
        8'h52, 8'h44,
        ascii_bool(camera_ready),
        8'h20,
        8'h20
    };

    // "SC" + done + " ID" + id_ok + " CE" + csi_err + " OF" + overflow
    // Example: "SC1 ID1 CE0 OF0 "
    wire [127:0] line6 = {
        8'h53, 8'h43,
        ascii_bool(sccb_init_done),
        8'h20,
        8'h49, 8'h44,
        ascii_bool(sensor_id_ok),
        8'h20,
        8'h43, 8'h45,
        ascii_bool(csi_error_sticky),
        8'h20,
        8'h4F, 8'h46,
        ascii_bool(overflow_sticky),
        8'h20
    };

    //==========================================================================
    // Coordinate narrowing for text renderer
    //--------------------------------------------------------------------------
    // vga_textline_3x5 uses 10-bit hcount/vcount. This tile is intended for
    // standard 640x480 baseline video, where pix_x[9:0] and pix_y[9:0] cover
    // the active coordinate range.
    //==========================================================================
    wire [9:0] hcount10 = pix_x[9:0];
    wire [9:0] vcount10 = pix_y[9:0];

    //==========================================================================
    // Text-line pixel-on outputs
    //--------------------------------------------------------------------------
    // Each text renderer produces a one-bit mask. The final color block treats
    // any asserted mask as foreground text.
    //==========================================================================
    wire txt0_on;
    wire txt1_on;
    wire txt2_on;
    wire txt3_on;
    wire txt4_on;
    wire txt5_on;
    wire txt6_on;

    vga_textline_3x5 u_txt0 (
        .clk_pix      (clk_vid),
        .rst_pix      (rst_vid),
        .hcount       (hcount10),
        .vcount       (vcount10),
        .active_video (de),
        .x0           (TILE_X0 + 8),
        .y0           (TILE_Y0 + 8),
        .scale        (4'd2),
        .str16        (line0),
        .pixel_on     (txt0_on)
    );

    vga_textline_3x5 u_txt1 (
        .clk_pix      (clk_vid),
        .rst_pix      (rst_vid),
        .hcount       (hcount10),
        .vcount       (vcount10),
        .active_video (de),
        .x0           (TILE_X0 + 8),
        .y0           (TILE_Y0 + 22),
        .scale        (4'd2),
        .str16        (line1),
        .pixel_on     (txt1_on)
    );

    vga_textline_3x5 u_txt2 (
        .clk_pix      (clk_vid),
        .rst_pix      (rst_vid),
        .hcount       (hcount10),
        .vcount       (vcount10),
        .active_video (de),
        .x0           (TILE_X0 + 8),
        .y0           (TILE_Y0 + 36),
        .scale        (4'd2),
        .str16        (line2),
        .pixel_on     (txt2_on)
    );

    vga_textline_3x5 u_txt3 (
        .clk_pix      (clk_vid),
        .rst_pix      (rst_vid),
        .hcount       (hcount10),
        .vcount       (vcount10),
        .active_video (de),
        .x0           (TILE_X0 + 8),
        .y0           (TILE_Y0 + 50),
        .scale        (4'd2),
        .str16        (line3),
        .pixel_on     (txt3_on)
    );

    vga_textline_3x5 u_txt4 (
        .clk_pix      (clk_vid),
        .rst_pix      (rst_vid),
        .hcount       (hcount10),
        .vcount       (vcount10),
        .active_video (de),
        .x0           (TILE_X0 + 8),
        .y0           (TILE_Y0 + 64),
        .scale        (4'd2),
        .str16        (line4),
        .pixel_on     (txt4_on)
    );

    vga_textline_3x5 u_txt5 (
        .clk_pix      (clk_vid),
        .rst_pix      (rst_vid),
        .hcount       (hcount10),
        .vcount       (vcount10),
        .active_video (de),
        .x0           (TILE_X0 + 8),
        .y0           (TILE_Y0 + 78),
        .scale        (4'd2),
        .str16        (line5),
        .pixel_on     (txt5_on)
    );

    vga_textline_3x5 u_txt6 (
        .clk_pix      (clk_vid),
        .rst_pix      (rst_vid),
        .hcount       (hcount10),
        .vcount       (vcount10),
        .active_video (de),
        .x0           (TILE_X0 + 8),
        .y0           (TILE_Y0 + 92),
        .scale        (4'd2),
        .str16        (line6),
        .pixel_on     (txt6_on)
    );

    //==========================================================================
    // Tile geometry masks
    //--------------------------------------------------------------------------
    // in_tile:
    //   Active only inside the rectangular tile and active video.
    //
    // border:
    //   Active only on the tile boundary pixels.
    //==========================================================================
    wire in_tile = de &&
                   (pix_x >= TILE_X0) && (pix_x < TILE_X0 + TILE_W) &&
                   (pix_y >= TILE_Y0) && (pix_y < TILE_Y0 + TILE_H);

    wire border = in_tile &&
                  ((pix_x == TILE_X0) || (pix_x == TILE_X0 + TILE_W - 1) ||
                   (pix_y == TILE_Y0) || (pix_y == TILE_Y0 + TILE_H - 1));

    //==========================================================================
    // Color palette
    //--------------------------------------------------------------------------
    // Colors are grouped as named constants so the visual policy is centralized.
    //==========================================================================
    localparam [23:0] RGB_BLACK       = 24'h000000;
    localparam [23:0] RGB_BG_OK       = 24'h081018;
    localparam [23:0] RGB_BG_FAIL     = 24'h180808;
    localparam [23:0] RGB_BORDER_OFF  = 24'h404040;
    localparam [23:0] RGB_BORDER_ON   = 24'h80E0FF;
    localparam [23:0] RGB_TEXT_LOCKED = 24'hD8F4FF;
    localparam [23:0] RGB_TEXT_WARN   = 24'hFFC080;
    localparam [23:0] RGB_SCCB_BUSY   = 24'hFFE040;
    localparam [23:0] RGB_SCCB_ERROR  = 24'hFF5050;
    localparam [23:0] RGB_READY       = 24'h50FF80;

    //==========================================================================
    // Right-side status indicator geometry
    //--------------------------------------------------------------------------
    // These narrow bars provide glanceable status independent of text parsing.
    //
    // BUSY_BAR:
    //   Yellow bar when SCCB is currently active.
    //
    // ERR_BAR:
    //   Red bar when SCCB error sticky is set.
    //
    // READY_BAR:
    //   Green bar when camera_ready is set.
    //==========================================================================
    wire busy_bar_region;
    wire err_bar_region;
    wire ready_bar_region;

    assign busy_bar_region =
        in_tile &&
        (pix_x >= TILE_X0 + TILE_W - 10) &&
        (pix_x <  TILE_X0 + TILE_W - 4) &&
        (pix_y >= TILE_Y0 + 8) &&
        (pix_y <  TILE_Y0 + TILE_H - 8);

    assign err_bar_region =
        in_tile &&
        (pix_x >= TILE_X0 + TILE_W - 20) &&
        (pix_x <  TILE_X0 + TILE_W - 14) &&
        (pix_y >= TILE_Y0 + 8) &&
        (pix_y <  TILE_Y0 + TILE_H - 8);

    assign ready_bar_region =
        in_tile &&
        (pix_x >= TILE_X0 + TILE_W - 30) &&
        (pix_x <  TILE_X0 + TILE_W - 24) &&
        (pix_y >= TILE_Y0 + 8) &&
        (pix_y <  TILE_Y0 + TILE_H - 8);

    //==========================================================================
    // SEQUENTIAL BLOCK: snapshot commit register
    //--------------------------------------------------------------------------
    // STATE OWNER
    //   clk_vid owns snap_commit_reg.
    //
    // RESET BEHAVIOR
    //   Reset clears the committed snapshot to zero. This renders the tile as
    //   inactive/default until the first real status commit arrives.
    //
    // UPDATE RULE
    //   1) If rst_vid is asserted, clear the local snapshot.
    //   2) Else, if cam_snap_commit_vid is asserted, capture cam_snap_vid.
    //   3) Otherwise, hold the last committed snapshot.
    //
    // WHY THIS IS FRAME-SAFE
    //   The upstream CDC should generate cam_snap_commit_vid only at a safe
    //   frame-commit point. This register then keeps the text fields stable
    //   between commits.
    //==========================================================================
    always @(posedge clk_vid) begin
        if (rst_vid)
            snap_commit_reg <= {SNAP_W{1'b0}};
        else if (cam_snap_commit_vid)
            snap_commit_reg <= cam_snap_vid;
    end

    //==========================================================================
    // COMBINATIONAL BLOCK: tile color resolver
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Resolve the RGB color for the current pixel.
    //
    // LAYERING ORDER
    //   The assignments below intentionally behave like a painter's algorithm:
    //
    //     1) Start with black.
    //     2) Paint tile background if in_tile.
    //     3) Paint border if on border.
    //     4) Paint text foreground if any text pixel is active.
    //     5) Paint narrow status bars over the tile.
    //
    // PRIORITY POLICY
    //   Later assignments have higher priority.
    //
    // STATUS COLOR POLICY
    //   Background:
    //     dark blue/black when init is healthy,
    //     dark red when init_fail is set.
    //
    //   Border:
    //     cyan when power_good is set,
    //     gray when power is not yet good.
    //
    //   Text:
    //     light cyan when csi_locked is set,
    //     amber when CSI is not locked.
    //
    //   Bars:
    //     yellow = SCCB busy,
    //     red    = SCCB sticky error,
    //     green  = camera ready.
    //==========================================================================
    always @(*) begin
        rgb_out = RGB_BLACK;

        if (in_tile)
            rgb_out = init_fail ? RGB_BG_FAIL : RGB_BG_OK;

        if (border)
            rgb_out = power_good ? RGB_BORDER_ON : RGB_BORDER_OFF;

        if (txt0_on || txt1_on || txt2_on || txt3_on ||
            txt4_on || txt5_on || txt6_on)
            rgb_out = csi_locked ? RGB_TEXT_LOCKED : RGB_TEXT_WARN;

        if (sccb_busy && busy_bar_region)
            rgb_out = RGB_SCCB_BUSY;

        if (sccb_error_sticky && err_bar_region)
            rgb_out = RGB_SCCB_ERROR;

        if (camera_ready && ready_bar_region)
            rgb_out = RGB_READY;
    end

    //==========================================================================
    // Reserved/unused signal sink
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Acknowledge decoded fields that are intentionally not yet rendered by
    //   this tile revision.
    //
    // WHY THIS EXISTS
    //   Strict lint settings often warn about unused decoded wires. This XOR
    //   reduction gives them a benign consumer without changing functionality.
    //
    // NOTE
    //   drop_sticky and frame_store_valid are decoded for future tile expansion
    //   and are included here to document that they are intentionally unused.
    //==========================================================================
    wire _unused_flags;
    assign _unused_flags =
        csi_error_flags[0] ^ csi_error_flags[1] ^
        csi_error_flags[2] ^ csi_error_flags[3] ^
        csi_error_flags[4] ^ csi_error_flags[5] ^
        csi_error_flags[6] ^ csi_error_flags[7] ^
        drop_sticky        ^ frame_store_valid;

endmodule

`default_nettype wire