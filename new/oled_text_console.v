`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// oled_text_console
//------------------------------------------------------------------------------
// PURPOSE
//   Convert four packed ASCII text lines into monochrome page-byte data for a
//   128x32 SSD1306-style OLED text console.
//
// HIGH-LEVEL ROLE
//   The SSD1306 page-addressed display model stores display content as bytes,
//   where each byte typically corresponds to one vertical column of 8 pixels
//   within one page.
//
//   This module does not store a framebuffer. Instead, it computes the display
//   byte corresponding to a requested byte address `byte_addr` by:
//
//     1) determining which text row (page) is being addressed,
//     2) determining which column within that row is being addressed,
//     3) converting the column into a character index and an intra-character
//        column offset,
//     4) extracting the corresponding ASCII character from the selected line,
//     5) querying a 5x7 font ROM for the requested glyph column,
//     6) inserting a spacer column after each glyph,
//     7) outputting the final 8-bit page byte.
//
// DISPLAY GEOMETRY
//   OLED geometry:
//     - 128 columns
//     - 32 rows total
//     - organized as 4 pages of 8 vertical pixels each
//
//   Therefore:
//     - page_sel = 0..3 selects one text row
//     - col_sel  = 0..127 selects one byte-column within that row
//
// TEXT CELL GEOMETRY
//   Each displayed text character occupies a 6x8 cell:
//
//     width  = 6 columns total
//            = 5 glyph columns + 1 blank spacer column
//
//     height = 8 pixels
//
//   Thus, each page can display:
//
//       floor(128 / 6) = 21
//
//   full visible character cells, consuming:
//
//       21 * 6 = 126 columns
//
//   leaving columns 126 and 127 unused/blank.
//
// ASCII BUS CONVENTION
//   Each line is presented as a packed ASCII bus:
//
//     line*_ascii[7:0]      = character 0
//     line*_ascii[15:8]     = character 1
//     ...
//     line*_ascii[167:160]  = character 20     (for LINE_CHARS = 21)
//
//   Therefore, character index i is stored in bits:
//
//       [(8*i)+7 : 8*i]
//
// FONT ASSUMPTION
//   The instantiated font ROM is assumed to map:
//
//       (char_code, col_idx) -> col_bits[7:0]
//
//   where:
//
//     - char_code is the ASCII code of the character,
//     - col_idx is the glyph column number,
//     - col_bits is the 8-pixel vertical bit pattern for that glyph column.
//
//   Only glyph columns 0..4 are intended for visible font data.
//   Column 5 is generated locally by this module as a blank spacer column.
//   Column values beyond that are not requested from the font in the visible
//   path of this design.
//
// COMBINATIONAL NATURE
//   This module is fully combinational:
//     - no clock
//     - no internal state registers that retain history
//     - output byte_data is an immediate function of:
//         * line0_ascii..line3_ascii
//         * byte_addr
//
// SYSTEM USE
//   This style of module is well suited to a display scan engine or OLED driver
//   FSM that sequentially walks byte addresses and transmits the resulting
//   bytes over SPI or I2C.
//
//==============================================================================
module oled_text_console #(
    parameter integer LINE_CHARS = 21
)(
    //--------------------------------------------------------------------------
    // Packed ASCII input lines
    //--------------------------------------------------------------------------
    // Each line contains LINE_CHARS packed ASCII bytes.
    // Character 0 resides in the least-significant byte.
    //--------------------------------------------------------------------------
    input  wire [(LINE_CHARS*8)-1:0] line0_ascii,
    input  wire [(LINE_CHARS*8)-1:0] line1_ascii,
    input  wire [(LINE_CHARS*8)-1:0] line2_ascii,
    input  wire [(LINE_CHARS*8)-1:0] line3_ascii,

    //--------------------------------------------------------------------------
    // Byte address into the logical 128x32 page memory image
    //--------------------------------------------------------------------------
    // Address mapping:
    //
    //   byte_addr[8:7] = page index 0..3
    //   byte_addr[6:0] = column index 0..127
    //
    // Since 4 pages * 128 columns = 512 bytes, 9 bits are sufficient.
    //--------------------------------------------------------------------------
    input  wire [8:0]               byte_addr,

    //--------------------------------------------------------------------------
    // Output page byte for the requested address
    //--------------------------------------------------------------------------
    // This is the column byte to be sent to the OLED for the selected page and
    // column.
    //--------------------------------------------------------------------------
    output reg  [7:0]               byte_data
);

    //--------------------------------------------------------------------------
    // Address decomposition fields
    //--------------------------------------------------------------------------
    // page_sel
    //   Selects which of the four text lines is active.
    //
    // col_sel
    //   Selects the column within that page.
    //
    // char_idx
    //   Character-cell index within the line.
    //
    // col_in_char
    //   Column within the 6-column character cell:
    //     0..4 -> font glyph columns
    //     5    -> blank spacer column
    //--------------------------------------------------------------------------
    reg [1:0] page_sel;
    reg [6:0] col_sel;
    reg [4:0] char_idx;
    reg [2:0] col_in_char;

    //--------------------------------------------------------------------------
    // ASCII code of the currently selected character cell
    //--------------------------------------------------------------------------
    // Defaults to space when the computed character index is out of range or
    // the page decode falls through unexpectedly.
    //--------------------------------------------------------------------------
    reg [7:0] active_line_char;

    //--------------------------------------------------------------------------
    // Font-ROM output for the selected ASCII character and glyph column
    //--------------------------------------------------------------------------
    wire [7:0] glyph_col_bits;

    //--------------------------------------------------------------------------
    // Font ROM instance
    //--------------------------------------------------------------------------
    // The font ROM converts:
    //   (ASCII code, glyph column index) -> 8 vertical pixel bits
    //
    // This module supplies:
    //   char_code = active_line_char
    //   col_idx   = col_in_char
    //
    // In normal visible-glyph operation, only col_in_char values 0..4 are
    // used for actual glyph data. The spacer column at col_in_char==5 is
    // handled locally and returned as 8'h00.
    //--------------------------------------------------------------------------
    oled_font_rom_5x7 u_oled_font_rom_5x7 (
        .char_code (active_line_char),
        .col_idx   (col_in_char),
        .col_bits  (glyph_col_bits)
    );

    //--------------------------------------------------------------------------
    // Function: get_line_char
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Extract one ASCII character from a packed line bus.
    //
    // INPUTS
    //   line_bus
    //     Packed ASCII bus of width LINE_CHARS*8.
    //
    //   idx
    //     Character index to extract.
    //
    // METHOD
    //   The packed bus is shifted right by idx*8 bits so that the requested
    //   character lands in bits [7:0]. The least-significant byte is then
    //   returned.
    //
    // STEP-BY-STEP
    //   Suppose:
    //
    //     line_bus = {char20, ..., char2, char1, char0}
    //
    //   and idx = 3.
    //
    //   Then:
    //     tmp = line_bus >> 24
    //
    //   which moves char3 into tmp[7:0].
    //
    //   Finally:
    //     get_line_char = tmp[7:0]
    //
    // IMPORTANT NOTE
    //   This function assumes idx is valid for the line width. The enclosing
    //   module protects its use by checking:
    //
    //       if (char_idx < LINE_CHARS)
    //
    //   before selecting a line character.
    //--------------------------------------------------------------------------
    function [7:0] get_line_char;
        input [(LINE_CHARS*8)-1:0] line_bus;
        input integer idx;
        reg   [(LINE_CHARS*8)-1:0] tmp;
        begin
            //------------------------------------------------------------------
            // Shift the requested character down into the least-significant
            // byte position.
            //------------------------------------------------------------------
            tmp = line_bus >> (idx * 8);

            //------------------------------------------------------------------
            // Return the extracted ASCII byte.
            //------------------------------------------------------------------
            get_line_char = tmp[7:0];
        end
    endfunction

    //--------------------------------------------------------------------------
    // Main combinational mapping logic
    //--------------------------------------------------------------------------
    // This block performs the full address-to-byte conversion.
    //
    // Step-by-step:
    //
    //   1) Split byte_addr into page and column.
    //   2) Convert the page column into:
    //        - a character index
    //        - a column within that character cell
    //   3) Select the appropriate line bus based on page_sel.
    //   4) Extract the active ASCII character for that line/position.
    //   5) If outside the visible text range, output 0.
    //   6) If the intra-character column is the spacer column, output 0.
    //   7) Otherwise output the glyph ROM column bits.
    //
    // Because the block is fully combinational, every output and intermediate
    // selection is a direct function of current inputs.
    //--------------------------------------------------------------------------
    always @(*) begin
        //----------------------------------------------------------------------
        // Decode the OLED page and column from the flat byte address.
        //
        // byte_addr spans 0..511:
        //
        //   page 0 : addresses   0..127
        //   page 1 : addresses 128..255
        //   page 2 : addresses 256..383
        //   page 3 : addresses 384..511
        //
        // So:
        //   page_sel = upper 2 bits
        //   col_sel  = lower 7 bits
        //----------------------------------------------------------------------
        page_sel = byte_addr[8:7];   // 0..3
        col_sel  = byte_addr[6:0];   // 0..127

        //----------------------------------------------------------------------
        // Convert column index into text-cell coordinates.
        //
        // Since each character cell is 6 columns wide:
        //
        //   char_idx    = col_sel / 6
        //   col_in_char = col_sel % 6
        //
        // Example:
        //   col_sel = 0  -> char 0, column 0
        //   col_sel = 5  -> char 0, spacer column
        //   col_sel = 6  -> char 1, column 0
        //   col_sel = 11 -> char 1, spacer column
        //----------------------------------------------------------------------
        char_idx    = col_sel / 7'd6;
        col_in_char = col_sel % 7'd6;

        //----------------------------------------------------------------------
        // Default character is ASCII space (0x20).
        //
        // This ensures that if the selected address falls outside the intended
        // visible character range, or if the page decode is otherwise invalid,
        // the font lookup defaults to a benign blank-like glyph.
        //----------------------------------------------------------------------
        active_line_char = 8'h20;

        //----------------------------------------------------------------------
        // Select the active line and extract the corresponding ASCII character,
        // but only when the character index is within the declared line width.
        //
        // If char_idx >= LINE_CHARS, the character remains space and the final
        // byte_data logic will force output to zero anyway.
        //----------------------------------------------------------------------
        if (char_idx < LINE_CHARS) begin
            case (page_sel)
                2'd0: active_line_char = get_line_char(line0_ascii, char_idx);
                2'd1: active_line_char = get_line_char(line1_ascii, char_idx);
                2'd2: active_line_char = get_line_char(line2_ascii, char_idx);
                2'd3: active_line_char = get_line_char(line3_ascii, char_idx);
                default: active_line_char = 8'h20;
            endcase
        end

        //----------------------------------------------------------------------
        // Final byte generation
        //
        // There are three cases:
        //
        //   Case A: char_idx outside visible line range
        //       Output blank byte 0x00.
        //
        //   Case B: spacer column inside a valid character cell
        //       Output blank byte 0x00.
        //
        //   Case C: visible glyph column 0..4
        //       Output the font-ROM column bits.
        //
        // This creates a 6x8 text cell made from:
        //   - 5 visible glyph columns
        //   - 1 blank spacer column
        //----------------------------------------------------------------------
        if (char_idx >= LINE_CHARS) begin
            //------------------------------------------------------------------
            // Outside the visible text region for this line.
            //------------------------------------------------------------------
            byte_data = 8'h00;
        end else if (col_in_char == 3'd5) begin
            //------------------------------------------------------------------
            // Sixth column of the 6-column cell: blank spacer.
            //------------------------------------------------------------------
            byte_data = 8'h00;
        end else begin
            //------------------------------------------------------------------
            // Visible glyph column. Use the font ROM result directly.
            //------------------------------------------------------------------
            byte_data = glyph_col_bits;
        end
    end

endmodule

`default_nettype wire