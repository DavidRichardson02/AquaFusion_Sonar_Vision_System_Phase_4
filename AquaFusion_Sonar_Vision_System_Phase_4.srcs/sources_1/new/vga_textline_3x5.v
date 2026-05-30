`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// vga_textline_3x5.v
//------------------------------------------------------------------------------
// Draws a single 16-character text line using 3x5 glyphs + 1-column spacing.
//
// IMPORTANT PACKING CONTRACT:
//   str16 is 16 packed *bytes* in MSB-first order:
//
//     str16[127:120] = cell 0 (leftmost)
//     str16[119:112] = cell 1
//     ...
//     str16[  7:  0] = cell 15 (rightmost)
//
// Geometry:
//   glyph size   = 3 x 5
//   spacing      = 1 empty column between glyphs
//   pitch        = (3 + 1) * scale = 4*scale
//   total width  = (16*3 + 15*1) * scale = 63*scale
//   total height = 5*scale
//
// Notes:
//   - No division operators are required (bounded subtract used).
//   - Spacing column is explicitly blanked (even if glyph module would be off).
//==============================================================================

module vga_textline_3x5 #(
    parameter integer REGISTER_GLYPH   = 0, // passed into glyph module
    parameter integer REGISTER_SELECT  = 0  // optional reg stage for select path
)(
    input  wire         clk_pix,
    input  wire         rst_pix,

    input  wire [9:0]   hcount,
    input  wire [9:0]   vcount,
    input  wire         active_video,

    input  wire [9:0]   x0,
    input  wire [9:0]   y0,
    input  wire [3:0]   scale,

    input  wire [127:0] str16,
    output wire         pixel_on
);

    //--------------------------------------------------------------------------
    // 0) Constants
    //--------------------------------------------------------------------------
    localparam integer GLYPH_W  = 3;
    localparam integer GLYPH_H  = 5;
    localparam integer SP_COLS  = 1;

    //--------------------------------------------------------------------------
    // 1) Scale sanitize: treat scale=0 as 1
    //--------------------------------------------------------------------------
    wire [3:0] sc = (scale == 4'd0) ? 4'd1 : scale;

    //--------------------------------------------------------------------------
    // 2) Pitch + bounding box
    //--------------------------------------------------------------------------
    wire [11:0] pitch  = {8'd0, sc} << 2;           // 4*sc
    wire [11:0] line_w = 12'd63 * {8'd0, sc};       // 63*sc
    wire [11:0] line_h = 12'd5  * {8'd0, sc};       //  5*sc

    wire [11:0] hx  = {2'd0, hcount};
    wire [11:0] vx  = {2'd0, vcount};
    wire [11:0] x0x = {2'd0, x0};
    wire [11:0] y0x = {2'd0, y0};

    wire in_x = (hx >= x0x) && (hx < (x0x + line_w));
    wire in_y = (vx >= y0x) && (vx < (y0x + line_h));
    wire in_line_box = active_video && in_x && in_y;

    //--------------------------------------------------------------------------
    // 3) Character index (0..15) and remainder-within-cell (0..pitch-1)
    //    Bounded subtract: avoids division.
    //--------------------------------------------------------------------------
    wire [11:0] dx = hx - x0x;

    reg  [3:0]  char_idx_c;
    reg  [11:0] rem_x_c;
    integer     i;

    always @* begin
        char_idx_c = 4'd0;
        rem_x_c    = dx;

        for (i = 0; i < 16; i = i + 1) begin
            if ((char_idx_c != 4'd15) && (rem_x_c >= pitch)) begin
                rem_x_c    = rem_x_c - pitch;
                char_idx_c = char_idx_c + 4'd1;
            end
        end
    end

    //--------------------------------------------------------------------------
    // 4) Extract 8-bit ASCII char from packed str16 (MSB-first bytes)
    //--------------------------------------------------------------------------
    reg [7:0] ch8_c;

    always @* begin
        case (char_idx_c)
            4'd0:   ch8_c = str16[127:120];
            4'd1:   ch8_c = str16[119:112];
            4'd2:   ch8_c = str16[111:104];
            4'd3:   ch8_c = str16[103: 96];
            4'd4:   ch8_c = str16[ 95: 88];
            4'd5:   ch8_c = str16[ 87: 80];
            4'd6:   ch8_c = str16[ 79: 72];
            4'd7:   ch8_c = str16[ 71: 64];
            4'd8:   ch8_c = str16[ 63: 56];
            4'd9:   ch8_c = str16[ 55: 48];
            4'd10:  ch8_c = str16[ 47: 40];
            4'd11:  ch8_c = str16[ 39: 32];
            4'd12:  ch8_c = str16[ 31: 24];
            4'd13:  ch8_c = str16[ 23: 16];
            4'd14:  ch8_c = str16[ 15:  8];
            default: ch8_c = str16[  7:  0]; // cell 15
        endcase
    end

    //--------------------------------------------------------------------------
    // 5) Selected character origin x_char0 = x0 + pitch*char_idx
    //--------------------------------------------------------------------------
    wire [11:0] x_char_off_c = pitch * {8'd0, char_idx_c};
    wire [9:0]  x_char0_c    = x0 + x_char_off_c[9:0];

    //--------------------------------------------------------------------------
    // 6) Explicit spacing blanking
    //    rem_x_c is within [0, pitch-1]. Glyph columns occupy [0, 3*sc-1].
    //--------------------------------------------------------------------------
    wire [11:0] glyph_w_scaled = 12'd3 * {8'd0, sc};
    wire        in_glyph_cols_c = (rem_x_c < glyph_w_scaled);

    //--------------------------------------------------------------------------
    // 7) Optional register stage (timing help)
    //--------------------------------------------------------------------------
    reg [7:0] ch8_r;
    reg [9:0] x_char0_r;
    reg       box_r;
    reg       in_glyph_cols_r;

    always @(posedge clk_pix) begin
        if (rst_pix) begin
            ch8_r          <= 8'h20; // space
            x_char0_r      <= 10'd0;
            box_r          <= 1'b0;
            in_glyph_cols_r <= 1'b0;
        end else if (REGISTER_SELECT != 0) begin
            ch8_r          <= ch8_c;
            x_char0_r      <= x_char0_c;
            box_r          <= in_line_box;
            in_glyph_cols_r <= in_glyph_cols_c;
        end
    end

    wire [7:0] ch8_eff           = (REGISTER_SELECT != 0) ? ch8_r           : ch8_c;
    wire [9:0] x_char0_eff       = (REGISTER_SELECT != 0) ? x_char0_r       : x_char0_c;
    wire       box_eff           = (REGISTER_SELECT != 0) ? box_r           : in_line_box;
    wire       in_glyph_cols_eff = (REGISTER_SELECT != 0) ? in_glyph_cols_r : in_glyph_cols_c;

    //--------------------------------------------------------------------------
    // 8) Glyph renderer (3x5, scaled)
    //--------------------------------------------------------------------------
    wire pixel_on_glyph;

    vga_char_glyph_3x5 #(
        .REGISTER_OUTPUT(REGISTER_GLYPH)
    ) u_glyph (
        .clk_pix      (clk_pix),
        .rst_pix      (rst_pix),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (x_char0_eff),
        .y0           (y0),
        .char_code    (ch8_eff),
        .scale        (sc),
        .pixel_on     (pixel_on_glyph)
    );

    assign pixel_on = box_eff && in_glyph_cols_eff && pixel_on_glyph;

endmodule
`default_nettype wire
