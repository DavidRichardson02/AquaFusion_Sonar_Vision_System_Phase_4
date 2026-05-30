`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// sonar_map_overlay_telem
//------------------------------------------------------------------------------
// ROLE
//   PIX-domain telemetry overlay for a sonar map panel.
//
// PURPOSE
//   Consume committed painter telemetry, renderer telemetry, and sonar bus48
//   telemetry, then render compact observability text on top of an existing map
//   panel region.
//
// RENDERING POLICY
//   - Purely PIX-domain
//   - No CDC inside this block
//   - Telemetry is latched only on update pulses, then held stable
//   - Overlay is deterministic function of pixel position + committed registers
//
// NOTE
//   - This version includes an internal 3x5 glyph renderer so the overlay is
//     self-contained and immediately synthesizable.
//==============================================================================

module sonar_map_overlay_telem #(
    parameter integer PANEL_X0      = 8,
    parameter integer PANEL_Y0      = 344,
    parameter integer PANEL_W       = 128,
    parameter integer PANEL_H       = 128,
    parameter integer GLYPH_SCALE   = 2,
    parameter integer LINE_CHARS    = 12
)(
    input  wire        pix_clk,
    input  wire        pix_rst,
    input  wire [9:0]  hcount,
    input  wire [9:0]  vcount,
    input  wire        active_video,

    input  wire [47:0] sonar_telem_pix,
    input  wire        sonar_telem_upd_pix,

    input  wire [63:0] painter_telem_pix,
    input  wire        painter_telem_upd_pix,

    input  wire [63:0] renderer_telem_pix,
    input  wire        renderer_telem_upd_pix,

    input  wire [11:0] rgb_bg,
    output reg  [11:0] rgb_out
);

    localparam integer CHAR_W   = 3 * GLYPH_SCALE;
    localparam integer CHAR_H   = 5 * GLYPH_SCALE;
    localparam integer CHAR_ADV = 4 * GLYPH_SCALE;

    localparam integer TXT_X    = PANEL_X0 + 4;
    localparam integer TXT_Y0   = PANEL_Y0 + 4;
    localparam integer LINE_DY  = 6 * GLYPH_SCALE;

    localparam integer LINE0_Y  = TXT_Y0 + (0 * LINE_DY);
    localparam integer LINE1_Y  = TXT_Y0 + (1 * LINE_DY);
    localparam integer LINE2_Y  = TXT_Y0 + (2 * LINE_DY);
    localparam integer LINE3_Y  = TXT_Y0 + (3 * LINE_DY);
    localparam integer LINE4_Y  = TXT_Y0 + (4 * LINE_DY);
    localparam integer LINE5_Y  = TXT_Y0 + (5 * LINE_DY);

    //--------------------------------------------------------------------------
    // Latched telemetry
    //--------------------------------------------------------------------------
    reg [15:0] dist_mm_r;
    reg [9:0]  age_ms_r;

    reg [7:0]  p_seq_r;
    reg        p_nt_r;
    reg        p_clamp_r;
    reg        p_oob_r;
    reg [15:0] p_ray_r;
    reg [15:0] p_free_r;
    reg [7:0]  p_hit_r;
    reg [7:0]  p_brush_r;
    reg [4:0]  p_cost_r;

    reg [3:0]  r_max_r;
    reg [15:0] r_addr_r;
    reg [11:0] r_cx_r;
    reg [11:0] r_cy_r;
    reg [5:0]  r_clamp_r;
    reg [5:0]  r_nonzero_r;
    reg [3:0]  r_req_r;
    reg [3:0]  r_inside_r;

    always @(posedge pix_clk) begin
        if (pix_rst) begin
            dist_mm_r   <= 16'd0;
            age_ms_r    <= 10'd0;

            p_seq_r     <= 8'd0;
            p_nt_r      <= 1'b0;
            p_clamp_r   <= 1'b0;
            p_oob_r     <= 1'b0;
            p_ray_r     <= 16'd0;
            p_free_r    <= 16'd0;
            p_hit_r     <= 8'd0;
            p_brush_r   <= 8'd0;
            p_cost_r    <= 5'd0;

            r_max_r     <= 4'd0;
            r_addr_r    <= 16'd0;
            r_cx_r      <= 12'd0;
            r_cy_r      <= 12'd0;
            r_clamp_r   <= 6'd0;
            r_nonzero_r <= 6'd0;
            r_req_r     <= 4'd0;
            r_inside_r  <= 4'd0;
        end else begin
            if (sonar_telem_upd_pix) begin
                dist_mm_r <= sonar_telem_pix[47:32];
                age_ms_r  <= sonar_telem_pix[31:22];
            end

            if (painter_telem_upd_pix) begin
                p_seq_r   <= painter_telem_pix[63:56];
                p_nt_r    <= painter_telem_pix[55];
                p_clamp_r <= painter_telem_pix[54];
                p_oob_r   <= painter_telem_pix[53];
                p_ray_r   <= painter_telem_pix[52:37];
                p_free_r  <= painter_telem_pix[36:21];
                p_hit_r   <= painter_telem_pix[20:13];
                p_brush_r <= painter_telem_pix[12:5];
                p_cost_r  <= painter_telem_pix[4:0];
            end

            if (renderer_telem_upd_pix) begin
                r_max_r     <= renderer_telem_pix[63:60];
                r_addr_r    <= renderer_telem_pix[59:44];
                r_cx_r      <= renderer_telem_pix[43:32];
                r_cy_r      <= renderer_telem_pix[31:20];
                r_clamp_r   <= renderer_telem_pix[19:14];
                r_nonzero_r <= renderer_telem_pix[13:8];
                r_req_r     <= renderer_telem_pix[7:4];
                r_inside_r  <= renderer_telem_pix[3:0];
            end
        end
    end

    //--------------------------------------------------------------------------
    // Decimal helpers
    //--------------------------------------------------------------------------
    function [7:0] dig;
        input [3:0] d;
        begin
            dig = 8'h30 + {4'd0, d};
        end
    endfunction

    function [3:0] d2_tens;
        input [7:0] v;
        begin
            d2_tens = (v >= 8'd90) ? 4'd9 :
                      (v >= 8'd80) ? 4'd8 :
                      (v >= 8'd70) ? 4'd7 :
                      (v >= 8'd60) ? 4'd6 :
                      (v >= 8'd50) ? 4'd5 :
                      (v >= 8'd40) ? 4'd4 :
                      (v >= 8'd30) ? 4'd3 :
                      (v >= 8'd20) ? 4'd2 :
                      (v >= 8'd10) ? 4'd1 : 4'd0;
        end
    endfunction

    function [3:0] d2_ones;
        input [7:0] v;
        reg [7:0] t;
        begin
            t = d2_tens(v);
            d2_ones = v - (t * 8'd10);
        end
    endfunction

    function [3:0] d3_h;
        input [15:0] v;
        begin
            d3_h = (v >= 16'd900) ? 4'd9 :
                   (v >= 16'd800) ? 4'd8 :
                   (v >= 16'd700) ? 4'd7 :
                   (v >= 16'd600) ? 4'd6 :
                   (v >= 16'd500) ? 4'd5 :
                   (v >= 16'd400) ? 4'd4 :
                   (v >= 16'd300) ? 4'd3 :
                   (v >= 16'd200) ? 4'd2 :
                   (v >= 16'd100) ? 4'd1 : 4'd0;
        end
    endfunction

    function [3:0] d3_t;
        input [15:0] v;
        reg [15:0] r;
        reg [3:0] h;
        begin
            h = d3_h(v);
            r = v - (h * 16'd100);
            d3_t = (r >= 16'd90) ? 4'd9 :
                   (r >= 16'd80) ? 4'd8 :
                   (r >= 16'd70) ? 4'd7 :
                   (r >= 16'd60) ? 4'd6 :
                   (r >= 16'd50) ? 4'd5 :
                   (r >= 16'd40) ? 4'd4 :
                   (r >= 16'd30) ? 4'd3 :
                   (r >= 16'd20) ? 4'd2 :
                   (r >= 16'd10) ? 4'd1 : 4'd0;
        end
    endfunction

    function [3:0] d3_o;
        input [15:0] v;
        reg [15:0] r;
        reg [3:0] h;
        reg [3:0] t;
        begin
            h = d3_h(v);
            r = v - (h * 16'd100);
            t = d3_t(v);
            d3_o = r - (t * 16'd10);
        end
    endfunction

    //--------------------------------------------------------------------------
    // 3x5 glyph ROM
    // Each glyph is packed row-major into 15 bits:
    //   bit 14..12 = row0 col0..2
    //   bit 11..9  = row1
    //   ...
    //   bit 2..0   = row4
    //--------------------------------------------------------------------------
    function [14:0] glyph3x5;
        input [7:0] ch;
        begin
            case (ch)
                "0": glyph3x5 = 15'b111_101_101_101_111;
                "1": glyph3x5 = 15'b010_110_010_010_111;
                "2": glyph3x5 = 15'b111_001_111_100_111;
                "3": glyph3x5 = 15'b111_001_111_001_111;
                "4": glyph3x5 = 15'b101_101_111_001_001;
                "5": glyph3x5 = 15'b111_100_111_001_111;
                "6": glyph3x5 = 15'b111_100_111_101_111;
                "7": glyph3x5 = 15'b111_001_001_001_001;
                "8": glyph3x5 = 15'b111_101_111_101_111;
                "9": glyph3x5 = 15'b111_101_111_001_111;

                "A": glyph3x5 = 15'b111_101_111_101_101;
                "B": glyph3x5 = 15'b110_101_110_101_110;
                "C": glyph3x5 = 15'b111_100_100_100_111;
                "F": glyph3x5 = 15'b111_100_111_100_100;
                "H": glyph3x5 = 15'b101_101_111_101_101;
                "M": glyph3x5 = 15'b101_111_111_101_101;
                "N": glyph3x5 = 15'b101_111_111_111_101;
                "O": glyph3x5 = 15'b111_101_101_101_111;
                "Q": glyph3x5 = 15'b111_101_101_111_001;
                "R": glyph3x5 = 15'b110_101_110_101_101;
                "S": glyph3x5 = 15'b111_100_111_001_111;
                "X": glyph3x5 = 15'b101_101_010_101_101;
                "Y": glyph3x5 = 15'b101_101_010_010_010;
                "Z": glyph3x5 = 15'b111_001_010_100_111;

                ":": glyph3x5 = 15'b000_010_000_010_000;
                "-": glyph3x5 = 15'b000_000_111_000_000;
                " ": glyph3x5 = 15'b000_000_000_000_000;

                default: glyph3x5 = 15'b000_000_000_000_000;
            endcase
        end
    endfunction

    function glyph_pixel_on;
        input [7:0] ch;
        input [1:0] gx;
        input [2:0] gy;
        reg [14:0] g;
        integer bit_index;
        begin
            g = glyph3x5(ch);
            bit_index = 14 - ((gy * 3) + gx);
            glyph_pixel_on = g[bit_index];
        end
    endfunction

    function line_text_ink;
        input [9:0] px;
        input [9:0] py;
        input integer x0;
        input integer y0;
        input [8*LINE_CHARS-1:0] text_bus;
        integer k;
        integer rel_x;
        integer rel_y;
        integer ch_x0;
        reg hit;
        reg [7:0] ch;
        begin
            hit = 1'b0;
            if ((py >= y0) && (py < (y0 + CHAR_H))) begin
                rel_y = (py - y0) / GLYPH_SCALE;
                for (k = 0; k < LINE_CHARS; k = k + 1) begin
                    ch_x0 = x0 + (k * CHAR_ADV);
                    if ((px >= ch_x0) && (px < (ch_x0 + CHAR_W))) begin
                        rel_x = (px - ch_x0) / GLYPH_SCALE;
                        ch = text_bus[(8*(LINE_CHARS-k))-1 -: 8];
                        if (glyph_pixel_on(ch, rel_x[1:0], rel_y[2:0]))
                            hit = 1'b1;
                    end
                end
            end
            line_text_ink = hit;
        end
    endfunction

    //--------------------------------------------------------------------------
    // Fixed-width text lines
    //--------------------------------------------------------------------------
    wire [8*LINE_CHARS-1:0] line0 = {
        "M",":", dig(d3_h(dist_mm_r)), dig(d3_t(dist_mm_r)), dig(d3_o(dist_mm_r)),
        " ",
        "A",":", dig(d3_h(age_ms_r)), dig(d3_t(age_ms_r)), dig(d3_o(age_ms_r)),
        " "
    };

    wire [8*LINE_CHARS-1:0] line1 = {
        "R",":", dig(d3_h(p_ray_r)), dig(d3_t(p_ray_r)), dig(d3_o(p_ray_r)),
        " ",
        "F",":", dig(d3_h(p_free_r)), dig(d3_t(p_free_r)), dig(d3_o(p_free_r)),
        " "
    };

    wire [8*LINE_CHARS-1:0] line2 = {
        "H",":", dig(d2_tens(p_hit_r)), dig(d2_ones(p_hit_r)),
        " ",
        "B",":", dig(d2_tens(p_brush_r)), dig(d2_ones(p_brush_r)),
        " ",
        "C",":", dig(d2_tens(p_cost_r)), dig(d2_ones(p_cost_r))
    };

    wire [8*LINE_CHARS-1:0] line3 = {
        (p_nt_r    ? "N" : "-"),
        (p_clamp_r ? "C" : "-"),
        (p_oob_r   ? "O" : "-"),
        " ",
        "S",":", dig(d3_h(p_seq_r)), dig(d3_t(p_seq_r)), dig(d3_o(p_seq_r)),
        " "," "," "
    };

    wire [8*LINE_CHARS-1:0] line4 = {
        "X",":", dig(d3_h(r_cx_r)), dig(d3_t(r_cx_r)), dig(d3_o(r_cx_r)),
        " ",
        "Y",":", dig(d3_h(r_cy_r)), dig(d3_t(r_cy_r)), dig(d3_o(r_cy_r)),
        " "
    };

    wire [8*LINE_CHARS-1:0] line5 = {
        "Z",":", dig(r_max_r),
        " ",
        "Q",":", dig(r_req_r),
        " ",
        "N",":", dig(d2_tens(r_nonzero_r)), dig(d2_ones(r_nonzero_r)),
        " "," "
    };

    wire [9:0] temp_x = PANEL_X0 + PANEL_W;
    wire [9:0] temp_y = PANEL_Y0 + PANEL_H;    

    wire in_panel =
        active_video &&
        (hcount >= PANEL_X0[9:0]) &&
        (hcount <  temp_x[9:0]) &&
        (vcount >= PANEL_Y0[9:0]) &&
        (vcount <  temp_y[9:0]);

    wire ink0 = line_text_ink(hcount, vcount, TXT_X, LINE0_Y, line0);
    wire ink1 = line_text_ink(hcount, vcount, TXT_X, LINE1_Y, line1);
    wire ink2 = line_text_ink(hcount, vcount, TXT_X, LINE2_Y, line2);
    wire ink3 = line_text_ink(hcount, vcount, TXT_X, LINE3_Y, line3);
    wire ink4 = line_text_ink(hcount, vcount, TXT_X, LINE4_Y, line4);
    wire ink5 = line_text_ink(hcount, vcount, TXT_X, LINE5_Y, line5);

    localparam [11:0] C_TEXT = 12'hEEE;
    localparam [11:0] C_WARN = 12'hFA0;
    localparam [11:0] C_BAD  = 12'hF33;
    localparam [11:0] C_INFO = 12'h0AF;
    localparam [11:0] C_BG0  = 12'h011;
    localparam [11:0] C_BG1  = 12'h012;
    localparam [11:0] C_BORD = 12'h0A8;

    wire panel_border =
        in_panel &&
        ((hcount == PANEL_X0[9:0]) ||
         (hcount == (temp_x[9:0] - 10'd1)) ||
         (vcount == PANEL_Y0[9:0]) ||
         (vcount == (temp_y[9:0] - 10'd1)));

    always @* begin
        rgb_out = rgb_bg;

        if (in_panel) begin
            if (rgb_bg == 12'h000)
                rgb_out = ((hcount[3] ^ vcount[3]) != 1'b0) ? C_BG1 : C_BG0;

            if (panel_border)
                rgb_out = C_BORD;

            if (ink0 || ink1 || ink2 || ink4 || ink5)
                rgb_out = C_TEXT;

            if (ink3)
                rgb_out = p_oob_r   ? C_BAD  :
                          p_clamp_r ? C_WARN :
                          p_nt_r    ? C_INFO :
                                      C_TEXT;
        end
    end

endmodule

`default_nettype wire
