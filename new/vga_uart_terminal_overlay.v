`timescale 1ns/1ps
`default_nettype none

// ============================================================================
// vga_uart_terminal_overlay.v
//------------------------------------------------------------------------------
// Purpose:
//   VGA compositor overlay with two runtime modes selected by sw_mode:
//
//     sw_mode=0: HEX byte scroller (RX/TX rings)
//     sw_mode=1: ASCII terminal (printable logging, CR/LF/TAB/BS)
//
// Resource strategy:
//   - ASCII mode uses lazy-clear via per-line generation bits + per-cell tags.
//   - HEX mode uses validity counters (no ring init loops).
//   - Glyph rendering uses vga_char_glyph_3x5 + font3x5 (small font).
//   - Glyph is centered inside each CHAR_W x CHAR_H cell.
//
// Notes:
//   - rx_vld / tx_vld assumed synchronous to pix_clk.
// ============================================================================

module vga_uart_terminal_overlay #(
    // ---------------- Placement ----------------
    parameter integer BOX_X0 = 16,
    parameter integer BOX_Y0 = 16,

    // Cell metrics (overlay grid cell size)
    parameter integer CHAR_W = 8,
    parameter integer CHAR_H = 8,

    // Character grid size (applies to both modes)
    parameter integer BOX_COLS = 40,
    parameter integer BOX_ROWS = 14,

    // Border thickness (pixels)
    parameter integer BORDER_T = 1,

    // ------------------------------------------------------------------------
    // 3x5 glyph backend parameters
    // ------------------------------------------------------------------------
    parameter integer GLYPH_SCALE = 2,          // runtime scale forwarded to vga_char_glyph_3x5
    parameter integer GLYPH_REG   = 0,          // 1 enables registered pixel_on in vga_char_glyph_3x5
    parameter integer GLYPH_RANGE_GATING = 1,   // forwarded to font3x5
    parameter integer GLYPH_MAP_LOWERCASE = 1,  // forwarded to font3x5
    parameter [14:0]  GLYPH_FALLBACK = 15'b111_111_111_111_111,

    // ------------------------------------------------------------------------
    // HEX scroller parameters (sw_mode=0)
    // ------------------------------------------------------------------------
    parameter integer HEX_RX_LOG_N = 32,
    parameter integer HEX_TX_LOG_N = 32,

    parameter integer HEX_RX_ROWS  = 6,
    parameter integer HEX_TX_ROWS  = 6,

    // Optional typewriter reveal (sw_mode=0)
    parameter integer HEX_TYPE_EN        = 0, // 0=off, 1=on
    parameter integer HEX_TYPE_FRAME_DIV = 2, // reveal rate in frames

    // Title row enable (sw_mode=0)
    parameter integer HEX_TITLE_EN = 1,

    // ------------------------------------------------------------------------
    // ASCII terminal parameters (sw_mode=1)
    // ------------------------------------------------------------------------
    // 0: merged terminal (RX and TX write into same ring)
    // 1: split terminal  (top pane RX, separator row, bottom pane TX)
    parameter integer ASCII_SPLIT_MODE = 0,

    // If ASCII_SPLIT_MODE=1:
    parameter integer ASCII_RX_ROWS = 7,   // must satisfy 1 <= ASCII_RX_ROWS <= BOX_ROWS-2

    // If ASCII_SPLIT_MODE=0 and ASCII_SHOW_DIR_PREFIX=1:
    parameter integer ASCII_SHOW_DIR_PREFIX = 1,

    // Tab stop (columns)
    parameter integer ASCII_TAB_W = 4,

    // ------------------------------------------------------------------------
    // Colors (RGB444)
    // ------------------------------------------------------------------------
    parameter [11:0] BOX_FILL_RGB   = 12'h111,
    parameter [11:0] BOX_BORDER_RGB = 12'hAAA,

    // HEX colors
    parameter [11:0] HEX_TEXT_RGB   = 12'hFFF,
    parameter [11:0] HEX_TITLE_RGB  = 12'hCCF,
    parameter [11:0] HEX_SEP_RGB    = 12'hAAA,

    // ASCII colors
    parameter [11:0] ASCII_TEXT_RGB    = 12'hFFF,
    parameter [11:0] ASCII_RX_TEXT_RGB = 12'hCFF,
    parameter [11:0] ASCII_TX_TEXT_RGB = 12'hFFC,
    parameter [11:0] ASCII_SEP_RGB     = 12'h555
)(
    input  wire        pix_clk,
    input  wire        rst,

    // VGA timing
    input  wire [9:0]  hcount,
    input  wire [9:0]  vcount,
    input  wire        active_video,

    // Frame tick (HEX typewriter pacing; ASCII ignores)
    input  wire        frame_tick,

    // UART events (pix_clk domain)
    input  wire [7:0]  rx_byte,
    input  wire        rx_vld,
    input  wire [7:0]  tx_byte,
    input  wire        tx_vld,

    // Runtime mode select
    // 0: HEX
    // 1: ASCII
    input  wire        sw_mode,

    // Background stream
    input  wire [11:0] rgb_bg,
    output reg  [11:0] rgb_out
);

    // =========================================================================
    // Helpers
    // =========================================================================
    function integer clog2_int;
        input integer v;
        integer t;
        integer x;
    begin
        t = 0;
        x = v - 1;
        while (x > 0) begin
            t = t + 1;
            x = x >> 1;
        end
        clog2_int = (t < 1) ? 1 : t;
    end
    endfunction

    // =========================================================================
    // Shared geometry / hit-test
    // =========================================================================
    localparam integer BOX_W = BOX_COLS * CHAR_W;
    localparam integer BOX_H = BOX_ROWS * CHAR_H;

    localparam integer BOX_X1 = BOX_X0 + BOX_W;
    localparam integer BOX_Y1 = BOX_Y0 + BOX_H;

    localparam [7:0] BOX_COLS_U8 = BOX_COLS[7:0];
    localparam [7:0] BOX_ROWS_U8 = BOX_ROWS[7:0];

    wire in_box =
        (hcount >= BOX_X0) && (hcount < BOX_X1) &&
        (vcount >= BOX_Y0) && (vcount < BOX_Y1);

    wire [9:0] x_local = hcount - BOX_X0;
    wire [9:0] y_local = vcount - BOX_Y0;

    wire in_border =
        in_box &&
        (x_local < BORDER_T || x_local >= (BOX_W - BORDER_T) ||
         y_local < BORDER_T || y_local >= (BOX_H - BORDER_T));

    // Cell decode: shift for 8x8, else divide.
    reg [7:0] cell_x;
    reg [7:0] cell_y;

    always @* begin
        if ((CHAR_W == 8) && (CHAR_H == 8)) begin
            cell_x = x_local[9:3];
            cell_y = y_local[9:3];
        end else begin
            cell_x = x_local / CHAR_W;
            cell_y = y_local / CHAR_H;
        end
    end

    // Cell origin
    reg [9:0] cell_x0;
    reg [9:0] cell_y0;

    always @* begin
        if ((CHAR_W == 8) && (CHAR_H == 8)) begin
            cell_x0 = BOX_X0 + {cell_x, 3'b000};
            cell_y0 = BOX_Y0 + {cell_y, 3'b000};
        end else begin
            cell_x0 = BOX_X0 + (cell_x * CHAR_W);
            cell_y0 = BOX_Y0 + (cell_y * CHAR_H);
        end
    end

    // =========================================================================
    // 3x5 glyph placement inside cell (centered)
    // =========================================================================
    wire [3:0] glyph_scale_w = (GLYPH_SCALE <= 0) ? 4'd1 : GLYPH_SCALE[3:0];

    wire [11:0] gs12     = {8'd0, glyph_scale_w};
    wire [11:0] glyph_w  = 12'd3 * gs12;
    wire [11:0] glyph_h  = 12'd5 * gs12;

    wire [11:0] cell_w12 = {2'b00, CHAR_W[9:0]};
    wire [11:0] cell_h12 = {2'b00, CHAR_H[9:0]};

    wire [11:0] off_x12 = (cell_w12 > glyph_w) ? ((cell_w12 - glyph_w) >> 1) : 12'd0;
    wire [11:0] off_y12 = (cell_h12 > glyph_h) ? ((cell_h12 - glyph_h) >> 1) : 12'd0;

    wire [9:0] glyph_x0 = cell_x0 + off_x12[9:0];
    wire [9:0] glyph_y0 = cell_y0 + off_y12[9:0];

    // =========================================================================
    // MODE outputs (computed in parallel, selected by sw_mode)
    // =========================================================================
    reg  [7:0]  hex_char_code;
    reg  [11:0] hex_text_rgb;
    reg         hex_glyph_en;

    reg  [7:0]  asc_char_code;
    reg  [11:0] asc_text_rgb;
    reg         asc_glyph_en;

    // Selected
    reg  [7:0]  char_code_sel;
    reg  [11:0] text_rgb_sel;
    reg         glyph_region_en;

    always @* begin
        if (sw_mode) begin
            char_code_sel   = asc_char_code;
            text_rgb_sel    = asc_text_rgb;
            glyph_region_en = asc_glyph_en;
        end else begin
            char_code_sel   = hex_char_code;
            text_rgb_sel    = hex_text_rgb;
            glyph_region_en = hex_glyph_en;
        end
    end

    // =========================================================================
    // sw_mode=0: HEX scroller (validity counters, no init loops)
    // =========================================================================
    function [7:0] hex_ascii;
        input [3:0] nib;
    begin
        if (nib < 4'd10) hex_ascii = 8'd48 + nib;
        else             hex_ascii = 8'd65 + (nib - 4'd10);
    end
    endfunction

    localparam integer RX_LOG_N = (HEX_RX_LOG_N < 1) ? 1 : HEX_RX_LOG_N;
    localparam integer TX_LOG_N = (HEX_TX_LOG_N < 1) ? 1 : HEX_TX_LOG_N;

    localparam integer RX_ROWS  = (HEX_RX_ROWS < 1) ? 1 : HEX_RX_ROWS;
    localparam integer TX_ROWS  = (HEX_TX_ROWS < 1) ? 1 : HEX_TX_ROWS;

    localparam integer RX_AW = clog2_int(RX_LOG_N);
    localparam integer TX_AW = clog2_int(TX_LOG_N);

    localparam [RX_AW:0] RX_LOG_N_U = RX_LOG_N;
    localparam [TX_AW:0] TX_LOG_N_U = TX_LOG_N;

    reg [7:0] rx_ring [0:RX_LOG_N-1];
    reg [7:0] tx_ring [0:TX_LOG_N-1];
    reg [RX_AW-1:0] rx_head;
    reg [TX_AW-1:0] tx_head;

    reg [RX_AW:0] rx_count;
    reg [TX_AW:0] tx_count;

    always @(posedge pix_clk) begin
        if (rst) begin
            rx_head  <= {RX_AW{1'b0}};
            tx_head  <= {TX_AW{1'b0}};
            rx_count <= {(RX_AW+1){1'b0}};
            tx_count <= {(TX_AW+1){1'b0}};
        end else begin
            if (rx_vld) begin
                rx_ring[rx_head] <= rx_byte;

                if (rx_head == (RX_LOG_N-1)) rx_head <= {RX_AW{1'b0}};
                else                         rx_head <= rx_head + {{(RX_AW-1){1'b0}},1'b1};

                if (rx_count < RX_LOG_N_U) rx_count <= rx_count + {{RX_AW{1'b0}},1'b1};
            end
            if (tx_vld) begin
                tx_ring[tx_head] <= tx_byte;

                if (tx_head == (TX_LOG_N-1)) tx_head <= {TX_AW{1'b0}};
                else                         tx_head <= tx_head + {{(TX_AW-1){1'b0}},1'b1};

                if (tx_count < TX_LOG_N_U) tx_count <= tx_count + {{TX_AW{1'b0}},1'b1};
            end
        end
    end

    // Layout in character cells
    localparam [7:0] TITLE_ROW = 8'd0;
    localparam [7:0] RX_ROW0   = (HEX_TITLE_EN != 0) ? 8'd1 : 8'd0;
    localparam [7:0] SEP_ROW   = RX_ROW0 + RX_ROWS[7:0];
    localparam [7:0] TX_ROW0   = SEP_ROW + 8'd1;

    localparam [7:0] PREFIX_X0 = 8'd1;
    localparam [7:0] TOK_X0    = 8'd4;

    localparam integer TOK_CHARS = 3; // "HH "
    localparam integer TOK_COLS_AVAIL = (BOX_COLS - 4);
    localparam integer TOKS_PER_ROW   = (TOK_COLS_AVAIL / TOK_CHARS);

    localparam integer RX_VIS_TOKS = RX_ROWS * TOKS_PER_ROW;
    localparam integer TX_VIS_TOKS = TX_ROWS * TOKS_PER_ROW;

    localparam integer RX_REVEAL_MAX = RX_VIS_TOKS * TOK_CHARS;
    localparam integer TX_REVEAL_MAX = TX_VIS_TOKS * TOK_CHARS;

    reg [15:0] rx_reveal_chars;
    reg [15:0] tx_reveal_chars;
    reg [15:0] type_div;

    always @(posedge pix_clk) begin
        if (rst) begin
            rx_reveal_chars <= 16'd0;
            tx_reveal_chars <= 16'd0;
            type_div        <= 16'd0;
        end else begin
            if (HEX_TYPE_EN != 0) begin
                if (rx_vld) rx_reveal_chars <= 16'd0;
                if (tx_vld) tx_reveal_chars <= 16'd0;

                if (frame_tick) begin
                    if (HEX_TYPE_FRAME_DIV <= 1) begin
                        type_div <= 16'd0;
                    end else begin
                        if (type_div == (HEX_TYPE_FRAME_DIV-1))
                            type_div <= 16'd0;
                        else
                            type_div <= type_div + 16'd1;
                    end

                    if ((HEX_TYPE_FRAME_DIV <= 1) || (type_div == (HEX_TYPE_FRAME_DIV-1))) begin
                        if (rx_reveal_chars < RX_REVEAL_MAX[15:0]) rx_reveal_chars <= rx_reveal_chars + 16'd1;
                        if (tx_reveal_chars < TX_REVEAL_MAX[15:0]) tx_reveal_chars <= tx_reveal_chars + 16'd1;
                    end
                end
            end else begin
                rx_reveal_chars <= RX_REVEAL_MAX[15:0];
                tx_reveal_chars <= TX_REVEAL_MAX[15:0];
                type_div        <= 16'd0;
            end
        end
    end

    function [7:0] title_char_at;
        input [7:0] cx;
    begin
        title_char_at = 8'd32;
        case (cx)
            8'd1: title_char_at = "U";
            8'd2: title_char_at = "A";
            8'd3: title_char_at = "R";
            8'd4: title_char_at = "T";
            8'd5: title_char_at = " ";
            8'd6: title_char_at = "L";
            8'd7: title_char_at = "O";
            8'd8: title_char_at = "G";
            default: title_char_at = 8'd32;
        endcase
    end
    endfunction

    function [7:0] prefix_char_at;
        input is_rx;
        input [7:0] cx;
    begin
        prefix_char_at = 8'd32;
        if (cx == PREFIX_X0)        prefix_char_at = is_rx ? "R" : "T";
        else if (cx == PREFIX_X0+1) prefix_char_at = "X";
        else if (cx == PREFIX_X0+2) prefix_char_at = " ";
        else                        prefix_char_at = 8'd32;
    end
    endfunction

    function [7:0] sep_char_at;
        input [7:0] cx;
    begin
        if (cx >= 8'd1 && cx < (BOX_COLS-1)) sep_char_at = "-";
        else                                 sep_char_at = 8'd32;
    end
    endfunction

    function [RX_AW-1:0] rx_ring_index_of;
        input integer token_idx;
        integer tmp;
    begin
        tmp = rx_head - 1 - token_idx;
        if (tmp < 0)              tmp = tmp + RX_LOG_N;
        else if (tmp >= RX_LOG_N) tmp = tmp - RX_LOG_N;
        rx_ring_index_of = tmp; // implicit truncation is width-safe
    end
    endfunction

    function [TX_AW-1:0] tx_ring_index_of;
        input integer token_idx;
        integer tmp;
    begin
        tmp = tx_head - 1 - token_idx;
        if (tmp < 0)              tmp = tmp + TX_LOG_N;
        else if (tmp >= TX_LOG_N) tmp = tmp - TX_LOG_N;
        tx_ring_index_of = tmp; // implicit truncation is width-safe
    end
    endfunction

    wire hex_in_rx_rows = (cell_y >= RX_ROW0) && (cell_y < (RX_ROW0 + RX_ROWS[7:0]));
    wire hex_in_tx_rows = (cell_y >= TX_ROW0) && (cell_y < (TX_ROW0 + TX_ROWS[7:0]));
    wire hex_in_sep_row = (cell_y == SEP_ROW);

    wire hex_in_tok_cols = (cell_x >= TOK_X0) && (cell_x < BOX_COLS_U8);

    wire [7:0] token_col_offset = cell_x - TOK_X0;
    wire [7:0] tok_slot_u       = token_col_offset / TOK_CHARS;
    wire [1:0] tok_char_u       = token_col_offset % TOK_CHARS;

    wire [7:0] rx_row_off = cell_y - RX_ROW0;
    wire [7:0] tx_row_off = cell_y - TX_ROW0;

    integer rx_token_idx;
    integer tx_token_idx;
    integer rx_char_idx;
    integer tx_char_idx;

    reg [7:0] b_sel;

    always @* begin
        hex_char_code = 8'd32;
        hex_text_rgb  = HEX_TEXT_RGB;
        hex_glyph_en  = 1'b0;
        b_sel         = 8'h00;

        if ((HEX_TITLE_EN != 0) && (cell_y == TITLE_ROW)) begin
            hex_glyph_en  = 1'b1;
            hex_char_code = title_char_at(cell_x);
            hex_text_rgb  = HEX_TITLE_RGB;

        end else if (hex_in_sep_row) begin
            hex_glyph_en  = 1'b1;
            hex_char_code = sep_char_at(cell_x);
            hex_text_rgb  = HEX_SEP_RGB;

        end else if (hex_in_rx_rows) begin
            if (cell_x >= PREFIX_X0 && cell_x <= (PREFIX_X0+2)) begin
                hex_glyph_en  = 1'b1;
                hex_char_code = prefix_char_at(1'b1, cell_x);
                hex_text_rgb  = HEX_TEXT_RGB;

            end else if (hex_in_tok_cols) begin
                rx_token_idx = (rx_row_off * TOKS_PER_ROW) + tok_slot_u;
                rx_char_idx  = (rx_token_idx * TOK_CHARS) + tok_char_u;

                hex_glyph_en = 1'b1;
                if ((rx_token_idx < RX_VIS_TOKS) && (rx_token_idx < rx_count) && (rx_char_idx < rx_reveal_chars)) begin
                    b_sel = rx_ring[rx_ring_index_of(rx_token_idx)];
                    case (tok_char_u)
                        2'd0: hex_char_code = hex_ascii(b_sel[7:4]);
                        2'd1: hex_char_code = hex_ascii(b_sel[3:0]);
                        default: hex_char_code = 8'd32;
                    endcase
                end else begin
                    hex_char_code = 8'd32;
                end
            end

        end else if (hex_in_tx_rows) begin
            if (cell_x >= PREFIX_X0 && cell_x <= (PREFIX_X0+2)) begin
                hex_glyph_en  = 1'b1;
                hex_char_code = prefix_char_at(1'b0, cell_x);
                hex_text_rgb  = HEX_TEXT_RGB;

            end else if (hex_in_tok_cols) begin
                tx_token_idx = (tx_row_off * TOKS_PER_ROW) + tok_slot_u;
                tx_char_idx  = (tx_token_idx * TOK_CHARS) + tok_char_u;

                hex_glyph_en = 1'b1;
                if ((tx_token_idx < TX_VIS_TOKS) && (tx_token_idx < tx_count) && (tx_char_idx < tx_reveal_chars)) begin
                    b_sel = tx_ring[tx_ring_index_of(tx_token_idx)];
                    case (tok_char_u)
                        2'd0: hex_char_code = hex_ascii(b_sel[7:4]);
                        2'd1: hex_char_code = hex_ascii(b_sel[3:0]);
                        default: hex_char_code = 8'd32;
                    endcase
                end else begin
                    hex_char_code = 8'd32;
                end
            end
        end
    end

    // =========================================================================
    // sw_mode=1: ASCII terminal (lazy-clear via per-line generation bits)
    // =========================================================================
    function is_printable;
        input [7:0] b;
    begin
        is_printable = (b >= 8'h20) && (b <= 8'h7E);
    end
    endfunction

    localparam integer RX_ROWS_E =
        (ASCII_SPLIT_MODE != 0) ? ((ASCII_RX_ROWS < 1) ? 1 : ASCII_RX_ROWS) : 1;

    localparam integer TX_ROWS_RAW =
        (ASCII_SPLIT_MODE != 0) ? (BOX_ROWS - RX_ROWS_E - 1) : 1;

    localparam integer TX_ROWS_E =
        (ASCII_SPLIT_MODE != 0) ? ((TX_ROWS_RAW < 1) ? 1 : TX_ROWS_RAW) : 1;

    localparam integer M_LINES = (BOX_ROWS < 1) ? 1 : BOX_ROWS;

    localparam integer M_SIZE  = M_LINES  * BOX_COLS;
    localparam integer RX_SIZE = RX_ROWS_E * BOX_COLS;
    localparam integer TX_SIZE = TX_ROWS_E * BOX_COLS;

    localparam integer M_HEAD_W  = clog2_int(M_LINES);
    localparam integer RX_HEAD_W = clog2_int(RX_ROWS_E);
    localparam integer TX_HEAD_W = clog2_int(TX_ROWS_E);

    localparam [7:0] ASCII_RX_ROWS_U8 = ASCII_RX_ROWS[7:0];

    reg [7:0] m_ram  [0:M_SIZE-1];
    reg [7:0] rx_ram [0:RX_SIZE-1];
    reg [7:0] tx_ram [0:TX_SIZE-1];

    reg       m_tag  [0:M_SIZE-1];
    reg       rx_tag [0:RX_SIZE-1];
    reg       tx_tag [0:TX_SIZE-1];

    reg m_line_gen  [0:M_LINES-1];
    reg rx_line_gen [0:RX_ROWS_E-1];
    reg tx_line_gen [0:TX_ROWS_E-1];

    reg [M_HEAD_W-1:0]  m_head;
    reg [7:0]           m_cur_col;

    reg [RX_HEAD_W-1:0] rx_head_a;
    reg [TX_HEAD_W-1:0] tx_head_a;

    reg [7:0] rx_cur_col;
    reg [7:0] tx_cur_col;

    reg m_line_start_rx;
    reg m_line_start_tx;

    integer ii;

    function integer idx1;
        input integer line;
        input integer col;
        input integer cols;
    begin
        idx1 = (line * cols) + col;
    end
    endfunction

    function integer ring_line_index;
        input integer head;
        input integer rows;
        input integer vis_r;
        integer tmp;
    begin
        if (rows <= 1) begin
            ring_line_index = 0;
        end else begin
            tmp = head + vis_r - rows;
            if (tmp < 0) tmp = tmp + rows;
            ring_line_index = tmp;
        end
    end
    endfunction

    task toggle_m_line_gen;
        input integer line_idx;
    begin
        m_line_gen[line_idx] <= ~m_line_gen[line_idx];
    end
    endtask

    task toggle_rx_line_gen;
        input integer line_idx;
    begin
        rx_line_gen[line_idx] <= ~rx_line_gen[line_idx];
    end
    endtask

    task toggle_tx_line_gen;
        input integer line_idx;
    begin
        tx_line_gen[line_idx] <= ~tx_line_gen[line_idx];
    end
    endtask

    task merged_newline;
        integer next_head;
    begin
        next_head = (m_head == (M_LINES-1)) ? 0 : (m_head + 1);
        m_head    <= next_head[M_HEAD_W-1:0];
        m_cur_col <= 8'd0;
        toggle_m_line_gen(next_head);
    end
    endtask

    task rx_newline;
        integer next_head;
    begin
        next_head  = (rx_head_a == (RX_ROWS_E-1)) ? 0 : (rx_head_a + 1);
        rx_head_a  <= next_head[RX_HEAD_W-1:0];
        rx_cur_col <= 8'd0;
        toggle_rx_line_gen(next_head);
    end
    endtask

    task tx_newline;
        integer next_head;
    begin
        next_head  = (tx_head_a == (TX_ROWS_E-1)) ? 0 : (tx_head_a + 1);
        tx_head_a  <= next_head[TX_HEAD_W-1:0];
        tx_cur_col <= 8'd0;
        toggle_tx_line_gen(next_head);
    end
    endtask

    task merged_insert_prefix;
        input is_rx;
        integer base;
        reg genb;
    begin
        if (ASCII_SHOW_DIR_PREFIX != 0) begin
            if (m_cur_col == 8'd0) begin
                base = idx1(m_head, 0, BOX_COLS);
                genb = m_line_gen[m_head];

                m_ram[base + 0] <= is_rx ? "R" : "T";
                m_tag[base + 0] <= genb;

                if (BOX_COLS > 1) begin
                    m_ram[base + 1] <= ":";
                    m_tag[base + 1] <= genb;
                end
                if (BOX_COLS > 2) begin
                    m_ram[base + 2] <= " ";
                    m_tag[base + 2] <= genb;
                end

                m_cur_col <= (BOX_COLS > 3) ? 8'd3 : 8'd0;
            end
        end
    end
    endtask

    task merged_consume_byte;
        input is_rx;
        input [7:0] b;
        integer next_tab;
        integer base;
        integer col_i;
        reg genb;
    begin
        genb = m_line_gen[m_head];

        if (b == 8'h0D) begin
            m_cur_col <= 8'd0;

        end else if (b == 8'h0A) begin
            merged_newline();
            if (is_rx) m_line_start_rx <= 1'b1;
            else       m_line_start_tx <= 1'b1;

        end else if (b == 8'h08) begin
            if (m_cur_col != 0) begin
                col_i     = m_cur_col - 1;
                m_cur_col <= col_i[7:0];
                base      = idx1(m_head, col_i, BOX_COLS);

                m_ram[base] <= 8'h20;
                m_tag[base] <= genb;
            end

        end else if (b == 8'h09) begin
            next_tab = ((m_cur_col + ASCII_TAB_W) / ASCII_TAB_W) * ASCII_TAB_W;
            if (next_tab >= BOX_COLS) begin
                merged_newline();
                if (is_rx) m_line_start_rx <= 1'b1;
                else       m_line_start_tx <= 1'b1;
            end else begin
                m_cur_col <= next_tab[7:0];
            end

        end else if (is_printable(b)) begin
            if (ASCII_SHOW_DIR_PREFIX != 0) begin
                if (is_rx && m_line_start_rx) begin
                    merged_insert_prefix(1'b1);
                    m_line_start_rx <= 1'b0;
                    genb = m_line_gen[m_head];
                end
                if (!is_rx && m_line_start_tx) begin
                    merged_insert_prefix(1'b0);
                    m_line_start_tx <= 1'b0;
                    genb = m_line_gen[m_head];
                end
            end

            if (m_cur_col < BOX_COLS) begin
                base = idx1(m_head, m_cur_col, BOX_COLS);
                m_ram[base] <= b;
                m_tag[base] <= genb;

                if ((m_cur_col + 1) >= BOX_COLS) begin
                    merged_newline();
                    if (is_rx) m_line_start_rx <= 1'b1;
                    else       m_line_start_tx <= 1'b1;
                end else begin
                    m_cur_col <= m_cur_col + 8'd1;
                end
            end else begin
                merged_newline();
                if (is_rx) m_line_start_rx <= 1'b1;
                else       m_line_start_tx <= 1'b1;
            end
        end
    end
    endtask

    task split_consume_byte;
        input is_rx;
        input [7:0] b;
        integer next_tab;
        integer base;
        integer col_i;
        reg genb;
    begin
        if (is_rx) begin
            genb = rx_line_gen[rx_head_a];

            if (b == 8'h0D) begin
                rx_cur_col <= 8'd0;
            end else if (b == 8'h0A) begin
                rx_newline();
            end else if (b == 8'h08) begin
                if (rx_cur_col != 0) begin
                    col_i      = rx_cur_col - 1;
                    rx_cur_col <= col_i[7:0];
                    base       = idx1(rx_head_a, col_i, BOX_COLS);

                    rx_ram[base] <= 8'h20;
                    rx_tag[base] <= genb;
                end
            end else if (b == 8'h09) begin
                next_tab = ((rx_cur_col + ASCII_TAB_W) / ASCII_TAB_W) * ASCII_TAB_W;
                if (next_tab >= BOX_COLS) rx_newline();
                else rx_cur_col <= next_tab[7:0];
            end else if (is_printable(b)) begin
                if (rx_cur_col < BOX_COLS) begin
                    base = idx1(rx_head_a, rx_cur_col, BOX_COLS);
                    rx_ram[base] <= b;
                    rx_tag[base] <= genb;

                    if ((rx_cur_col + 1) >= BOX_COLS) rx_newline();
                    else rx_cur_col <= rx_cur_col + 8'd1;
                end else begin
                    rx_newline();
                end
            end
        end else begin
            genb = tx_line_gen[tx_head_a];

            if (b == 8'h0D) begin
                tx_cur_col <= 8'd0;
            end else if (b == 8'h0A) begin
                tx_newline();
            end else if (b == 8'h08) begin
                if (tx_cur_col != 0) begin
                    col_i      = tx_cur_col - 1;
                    tx_cur_col <= col_i[7:0];
                    base       = idx1(tx_head_a, col_i, BOX_COLS);

                    tx_ram[base] <= 8'h20;
                    tx_tag[base] <= genb;
                end
            end else if (b == 8'h09) begin
                next_tab = ((tx_cur_col + ASCII_TAB_W) / ASCII_TAB_W) * ASCII_TAB_W;
                if (next_tab >= BOX_COLS) tx_newline();
                else tx_cur_col <= next_tab[7:0];
            end else if (is_printable(b)) begin
                if (tx_cur_col < BOX_COLS) begin
                    base = idx1(tx_head_a, tx_cur_col, BOX_COLS);
                    tx_ram[base] <= b;
                    tx_tag[base] <= genb;

                    if ((tx_cur_col + 1) >= BOX_COLS) tx_newline();
                    else tx_cur_col <= tx_cur_col + 8'd1;
                end else begin
                    tx_newline();
                end
            end
        end
    end
    endtask

    always @(posedge pix_clk) begin
        if (rst) begin
            m_head <= {M_HEAD_W{1'b0}};
            m_cur_col <= 8'd0;
            m_line_start_rx <= 1'b1;
            m_line_start_tx <= 1'b1;

            rx_head_a <= {RX_HEAD_W{1'b0}};
            tx_head_a <= {TX_HEAD_W{1'b0}};
            rx_cur_col <= 8'd0;
            tx_cur_col <= 8'd0;

            for (ii = 0; ii < M_LINES; ii = ii + 1)   m_line_gen[ii]  <= 1'b0;
            for (ii = 0; ii < RX_ROWS_E; ii = ii + 1) rx_line_gen[ii] <= 1'b0;
            for (ii = 0; ii < TX_ROWS_E; ii = ii + 1) tx_line_gen[ii] <= 1'b0;

            for (ii = 0; ii < M_SIZE; ii = ii + 1)   m_tag[ii]  <= 1'b0;
            for (ii = 0; ii < RX_SIZE; ii = ii + 1)  rx_tag[ii] <= 1'b0;
            for (ii = 0; ii < TX_SIZE; ii = ii + 1)  tx_tag[ii] <= 1'b0;

        end else begin
            if (ASCII_SPLIT_MODE == 0) begin
                if (rx_vld) merged_consume_byte(1'b1, rx_byte);
                if (tx_vld) merged_consume_byte(1'b0, tx_byte);
            end else begin
                if (rx_vld) split_consume_byte(1'b1, rx_byte);
                if (tx_vld) split_consume_byte(1'b0, tx_byte);
            end
        end
    end

    integer line_idx;
    integer col_idx;
    integer vis_row;
    integer pane_row;
    integer base;

    reg genb_r;

    wire asc_sep_row_hit = (ASCII_SPLIT_MODE != 0) && (cell_y == ASCII_RX_ROWS_U8);

    always @* begin
        asc_char_code = 8'h20;
        asc_text_rgb  = ASCII_TEXT_RGB;
        asc_glyph_en  = 1'b0;

        if (in_box && !in_border) begin
            asc_glyph_en = 1'b1;
            col_idx = cell_x;

            if (ASCII_SPLIT_MODE == 0) begin
                vis_row  = cell_y;
                line_idx = ring_line_index(m_head, M_LINES, vis_row);
                base     = idx1(line_idx, col_idx, BOX_COLS);

                genb_r = m_line_gen[line_idx];
                if (m_tag[base] == genb_r) asc_char_code = m_ram[base];
                else                       asc_char_code = 8'h20;

                asc_text_rgb = ASCII_TEXT_RGB;

            end else begin
                if (asc_sep_row_hit) begin
                    asc_char_code = 8'h2D;
                    asc_text_rgb  = ASCII_SEP_RGB;

                end else if (cell_y < ASCII_RX_ROWS_U8) begin
                    pane_row = cell_y;
                    line_idx = ring_line_index(rx_head_a, RX_ROWS_E, pane_row);
                    base     = idx1(line_idx, col_idx, BOX_COLS);

                    genb_r = rx_line_gen[line_idx];
                    if (rx_tag[base] == genb_r) asc_char_code = rx_ram[base];
                    else                         asc_char_code = 8'h20;

                    asc_text_rgb = ASCII_RX_TEXT_RGB;

                end else begin
                    pane_row = cell_y - (ASCII_RX_ROWS_U8 + 8'd1);
                    line_idx = ring_line_index(tx_head_a, TX_ROWS_E, pane_row);
                    base     = idx1(line_idx, col_idx, BOX_COLS);

                    genb_r = tx_line_gen[line_idx];
                    if (tx_tag[base] == genb_r) asc_char_code = tx_ram[base];
                    else                         asc_char_code = 8'h20;

                    asc_text_rgb = ASCII_TX_TEXT_RGB;
                end
            end
        end
    end

    // =========================================================================
    // Glyph pixel backend: 3x5 font renderer
    // =========================================================================
    wire glyph_on;

    vga_char_glyph_3x5 #(
        .REGISTER_OUTPUT (GLYPH_REG),
        .ASCII_MIN       (7'h20),
        .ASCII_MAX       (7'h5F),
        .EN_RANGE_GATING (GLYPH_RANGE_GATING),
        .MAP_LOWERCASE   (GLYPH_MAP_LOWERCASE),
        .FALLBACK_GLYPH  (GLYPH_FALLBACK)
    ) u_glyph_3x5 (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (glyph_x0),
        .y0           (glyph_y0),
        .char_code    (char_code_sel[6:0]),
        .scale        (glyph_scale_w),
        .pixel_on     (glyph_on)
    );

    // =========================================================================
    // Final compositing
    // =========================================================================
    always @* begin
        if (!active_video) begin
            rgb_out = 12'h000;
        end else begin
            rgb_out = rgb_bg;

            if (in_box) begin
                rgb_out = in_border ? BOX_BORDER_RGB : BOX_FILL_RGB;
            end

            if (in_box && !in_border) begin
                if (glyph_region_en && glyph_on) begin
                    rgb_out = text_rgb_sel;
                end
            end
        end
    end

endmodule

`default_nettype wire
