`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// oled_font_rom_5x7
//------------------------------------------------------------------------------
// ROLE
//   5x7 ASCII font column ROM.
//
// HIGH-LEVEL PURPOSE
//   This module stores a small read-only font for a telemetry-oriented subset
//   of ASCII characters. It returns one *vertical column* of one glyph at a
//   time.
//
//   Conceptually, this module answers:
//
//       "For character C and glyph-column index K, what vertical 8-bit pixel
//        pattern should be emitted?"
//
// COLUMN-ORIENTED REPRESENTATION
//   This font ROM is not organized as rows of pixels. Instead, it is organized
//   as columns.
//
//   For each supported character:
//
//     - the glyph is 5 columns wide
//     - each column is returned as an 8-bit vertical pattern
//
//   Therefore, the caller is expected to scan through glyph columns rather than
//   asking for entire characters at once.
//
// BIT MAPPING
//   The output column uses the following convention:
//
//       bit[0] = top pixel
//       bit[1] = next pixel downward
//       ...
//       bit[6] = bottom glyph pixel
//       bit[7] = blank spacer row
//
//   So each returned byte represents one vertical slice of the glyph.
//
// WHY THIS MODULE EXISTS
//   In a small OLED text system, it is useful to separate:
//
//     - text layout logic
//     - glyph storage
//
//   This module provides the glyph storage only.
//
//   A higher-level text renderer is responsible for:
//
//     - choosing which character to draw
//     - choosing which glyph column is being requested
//     - placing the returned bits into the correct display-memory location
//
//   This separation makes the font definition compact, reviewable, and easy to
//   extend independently of the text compositor.
//
// SUPPORTED CHARACTER SET
//   The module supports a deliberately limited subset of ASCII sufficient for
//   telemetry-style text fields:
//
//     - space
//     - dash '-'
//     - colon ':'
//     - digits '0' through '9'
//     - selected uppercase letters used by status words and telemetry labels
//
//   Unsupported characters fall back to blank output.
//
// PORT SEMANTICS
//   char_code
//     ASCII code of the requested character.
//
//   col_idx
//     Column index within the glyph.
//
//     Intended interpretation:
//       0..4 -> real glyph columns
//
//     The comment notes that column 5 is unused externally, which implies the
//     surrounding renderer likely handles any inter-character spacing itself.
//
//   col_bits
//     Returned 8-bit vertical column pattern for the chosen character and
//     column.
//
// GLYPH WIDTH MODEL
//   Each implemented glyph is effectively defined over five columns:
//
//       col_idx = 0, 1, 2, 3, 4
//
//   Any other column index for the same character falls back to 0x00 through
//   the default branches inside the nested column case statements.
//
// FALLBACK POLICY
//   The module uses two layers of fallback:
//
//     1) For a supported character, unsupported column indices return 0x00.
//     2) For an unsupported character code, the entire glyph returns 0x00.
//
//   This makes the font ROM safe and predictable in the presence of unexpected
//   input.
//
// PEDAGOGICAL SUMMARY
//   The module can be understood as a two-level ROM lookup:
//
//       first choose character
//       then choose column within that character
//       then return the corresponding vertical pixel byte
//
//   So this is effectively:
//
//       (char_code, col_idx) -> col_bits
//
//   with blank fallback for unsupported cases.
//------------------------------------------------------------------------------
module oled_font_rom_5x7 (
    //--------------------------------------------------------------------------
    // ASCII character code whose glyph is being queried.
    //--------------------------------------------------------------------------
    input  wire [7:0] char_code,

    //--------------------------------------------------------------------------
    // Glyph column index.
    //
    // Intended glyph columns are 0..4. Other values return blank columns in the
    // per-character default branches.
    //--------------------------------------------------------------------------
    input  wire [2:0] col_idx,

    //--------------------------------------------------------------------------
    // 8-bit vertical pixel column for the selected character and column.
    //
    // Bit meaning:
    //   bit[0] = top pixel
    //   bit[6] = bottom glyph pixel
    //   bit[7] = blank spacer row
    //--------------------------------------------------------------------------
    output reg  [7:0] col_bits
);

    //==========================================================================
    // Combinational glyph ROM decode
    //--------------------------------------------------------------------------
    // Step-by-step interpretation:
    //
    //   1) Default to a blank column.
    //   2) Decode char_code.
    //   3) For characters that require multiple columns, decode col_idx.
    //   4) Return the corresponding vertical bit pattern.
    //
    // This is a pure combinational ROM-style mapping with no internal state and
    // no clock.
    //==========================================================================
    always @(*) begin
        //----------------------------------------------------------------------
        // Default output
        //
        // Start from a blank column so that unsupported characters or columns
        // naturally render as empty space unless explicitly overridden.
        //----------------------------------------------------------------------
        col_bits = 8'h00;

        case (char_code)

            //==================================================================
            // Space ' '
            //------------------------------------------------------------------
            // Rendering meaning:
            //   Always blank for all queried columns.
            //==================================================================
            8'h20: begin
                col_bits = 8'h00;
            end

            //==================================================================
            // Dash '-'
            //------------------------------------------------------------------
            // Glyph interpretation:
            //   A small horizontal stroke in the middle region.
            //
            // Column model:
            //   Five columns are defined explicitly.
            //==================================================================
            8'h2D: begin
                case (col_idx)
                    3'd0: col_bits = 8'b00000000;
                    3'd1: col_bits = 8'b00001000;
                    3'd2: col_bits = 8'b00001000;
                    3'd3: col_bits = 8'b00001000;
                    3'd4: col_bits = 8'b00000000;
                    default: col_bits = 8'h00;
                endcase
            end

            //==================================================================
            // Colon ':'
            //------------------------------------------------------------------
            // Glyph interpretation:
            //   Two separated vertical dot regions.
            //==================================================================
            8'h3A: begin
                case (col_idx)
                    3'd0: col_bits = 8'b00000000;
                    3'd1: col_bits = 8'b00010100;
                    3'd2: col_bits = 8'b00000000;
                    3'd3: col_bits = 8'b00010100;
                    3'd4: col_bits = 8'b00000000;
                    default: col_bits = 8'h00;
                endcase
            end

            //==================================================================
            // Digits 0..9
            //------------------------------------------------------------------
            // Each digit is defined as five explicit columns.
            // These are intended for telemetry values such as counts, IDs,
            // status fields, and numeric measurements.
            //==================================================================

            8'h30: begin // '0'
                case (col_idx)
                    3'd0: col_bits = 8'b00111110;
                    3'd1: col_bits = 8'b01000001;
                    3'd2: col_bits = 8'b01000001;
                    3'd3: col_bits = 8'b01000001;
                    3'd4: col_bits = 8'b00111110;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h31: begin // '1'
                case (col_idx)
                    3'd0: col_bits = 8'b00000000;
                    3'd1: col_bits = 8'b01000010;
                    3'd2: col_bits = 8'b01111111;
                    3'd3: col_bits = 8'b01000000;
                    3'd4: col_bits = 8'b00000000;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h32: begin // '2'
                case (col_idx)
                    3'd0: col_bits = 8'b01100010;
                    3'd1: col_bits = 8'b01010001;
                    3'd2: col_bits = 8'b01001001;
                    3'd3: col_bits = 8'b01000101;
                    3'd4: col_bits = 8'b01000010;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h33: begin // '3'
                case (col_idx)
                    3'd0: col_bits = 8'b00100010;
                    3'd1: col_bits = 8'b01000001;
                    3'd2: col_bits = 8'b01001001;
                    3'd3: col_bits = 8'b01001001;
                    3'd4: col_bits = 8'b00110110;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h34: begin // '4'
                case (col_idx)
                    3'd0: col_bits = 8'b00011000;
                    3'd1: col_bits = 8'b00010100;
                    3'd2: col_bits = 8'b00010010;
                    3'd3: col_bits = 8'b01111111;
                    3'd4: col_bits = 8'b00010000;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h35: begin // '5'
                case (col_idx)
                    3'd0: col_bits = 8'b00100111;
                    3'd1: col_bits = 8'b01000101;
                    3'd2: col_bits = 8'b01000101;
                    3'd3: col_bits = 8'b01000101;
                    3'd4: col_bits = 8'b00111001;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h36: begin // '6'
                case (col_idx)
                    3'd0: col_bits = 8'b00111110;
                    3'd1: col_bits = 8'b01001001;
                    3'd2: col_bits = 8'b01001001;
                    3'd3: col_bits = 8'b01001001;
                    3'd4: col_bits = 8'b00110000;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h37: begin // '7'
                case (col_idx)
                    3'd0: col_bits = 8'b00000001;
                    3'd1: col_bits = 8'b01110001;
                    3'd2: col_bits = 8'b00001001;
                    3'd3: col_bits = 8'b00000101;
                    3'd4: col_bits = 8'b00000011;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h38: begin // '8'
                case (col_idx)
                    3'd0: col_bits = 8'b00110110;
                    3'd1: col_bits = 8'b01001001;
                    3'd2: col_bits = 8'b01001001;
                    3'd3: col_bits = 8'b01001001;
                    3'd4: col_bits = 8'b00110110;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h39: begin // '9'
                case (col_idx)
                    3'd0: col_bits = 8'b00000110;
                    3'd1: col_bits = 8'b01001001;
                    3'd2: col_bits = 8'b01001001;
                    3'd3: col_bits = 8'b00101001;
                    3'd4: col_bits = 8'b00011110;
                    default: col_bits = 8'h00;
                endcase
            end

            //==================================================================
            // Uppercase letters
            //------------------------------------------------------------------
            // Only a telemetry-oriented subset is implemented.
            // These letters are intended for labels such as:
            //   ALT, BMP, CAM, ERR, LOCK, SONAR, etc.
            //==================================================================

            8'h41: begin // 'A'
                case (col_idx)
                    3'd0: col_bits = 8'b01111110;
                    3'd1: col_bits = 8'b00010001;
                    3'd2: col_bits = 8'b00010001;
                    3'd3: col_bits = 8'b00010001;
                    3'd4: col_bits = 8'b01111110;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h42: begin // 'B'
                case (col_idx)
                    3'd0: col_bits = 8'b01111111;
                    3'd1: col_bits = 8'b01001001;
                    3'd2: col_bits = 8'b01001001;
                    3'd3: col_bits = 8'b01001001;
                    3'd4: col_bits = 8'b00110110;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h43: begin // 'C'
                case (col_idx)
                    3'd0: col_bits = 8'b00111110;
                    3'd1: col_bits = 8'b01000001;
                    3'd2: col_bits = 8'b01000001;
                    3'd3: col_bits = 8'b01000001;
                    3'd4: col_bits = 8'b00100010;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h44: begin // 'D'
                case (col_idx)
                    3'd0: col_bits = 8'b01111111;
                    3'd1: col_bits = 8'b01000001;
                    3'd2: col_bits = 8'b01000001;
                    3'd3: col_bits = 8'b00100010;
                    3'd4: col_bits = 8'b00011100;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h45: begin // 'E'
                case (col_idx)
                    3'd0: col_bits = 8'b01111111;
                    3'd1: col_bits = 8'b01001001;
                    3'd2: col_bits = 8'b01001001;
                    3'd3: col_bits = 8'b01001001;
                    3'd4: col_bits = 8'b01000001;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h46: begin // 'F'
                case (col_idx)
                    3'd0: col_bits = 8'b01111111;
                    3'd1: col_bits = 8'b00001001;
                    3'd2: col_bits = 8'b00001001;
                    3'd3: col_bits = 8'b00001001;
                    3'd4: col_bits = 8'b00000001;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h47: begin // 'G'
                case (col_idx)
                    3'd0: col_bits = 8'b00111110;
                    3'd1: col_bits = 8'b01000001;
                    3'd2: col_bits = 8'b01001001;
                    3'd3: col_bits = 8'b01001001;
                    3'd4: col_bits = 8'b01111010;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h48: begin // 'H'
                case (col_idx)
                    3'd0: col_bits = 8'b01111111;
                    3'd1: col_bits = 8'b00001000;
                    3'd2: col_bits = 8'b00001000;
                    3'd3: col_bits = 8'b00001000;
                    3'd4: col_bits = 8'b01111111;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h49: begin // 'I'
                case (col_idx)
                    3'd0: col_bits = 8'b00000000;
                    3'd1: col_bits = 8'b01000001;
                    3'd2: col_bits = 8'b01111111;
                    3'd3: col_bits = 8'b01000001;
                    3'd4: col_bits = 8'b00000000;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h4B: begin // 'K'
                case (col_idx)
                    3'd0: col_bits = 8'b01111111;
                    3'd1: col_bits = 8'b00001000;
                    3'd2: col_bits = 8'b00010100;
                    3'd3: col_bits = 8'b00100010;
                    3'd4: col_bits = 8'b01000001;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h4C: begin // 'L'
                case (col_idx)
                    3'd0: col_bits = 8'b01111111;
                    3'd1: col_bits = 8'b01000000;
                    3'd2: col_bits = 8'b01000000;
                    3'd3: col_bits = 8'b01000000;
                    3'd4: col_bits = 8'b01000000;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h4D: begin // 'M'
                case (col_idx)
                    3'd0: col_bits = 8'b01111111;
                    3'd1: col_bits = 8'b00000010;
                    3'd2: col_bits = 8'b00001100;
                    3'd3: col_bits = 8'b00000010;
                    3'd4: col_bits = 8'b01111111;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h4E: begin // 'N'
                case (col_idx)
                    3'd0: col_bits = 8'b01111111;
                    3'd1: col_bits = 8'b00000100;
                    3'd2: col_bits = 8'b00001000;
                    3'd3: col_bits = 8'b00010000;
                    3'd4: col_bits = 8'b01111111;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h4F: begin // 'O'
                case (col_idx)
                    3'd0: col_bits = 8'b00111110;
                    3'd1: col_bits = 8'b01000001;
                    3'd2: col_bits = 8'b01000001;
                    3'd3: col_bits = 8'b01000001;
                    3'd4: col_bits = 8'b00111110;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h50: begin // 'P'
                case (col_idx)
                    3'd0: col_bits = 8'b01111111;
                    3'd1: col_bits = 8'b00001001;
                    3'd2: col_bits = 8'b00001001;
                    3'd3: col_bits = 8'b00001001;
                    3'd4: col_bits = 8'b00000110;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h52: begin // 'R'
                case (col_idx)
                    3'd0: col_bits = 8'b01111111;
                    3'd1: col_bits = 8'b00001001;
                    3'd2: col_bits = 8'b00011001;
                    3'd3: col_bits = 8'b00101001;
                    3'd4: col_bits = 8'b01000110;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h53: begin // 'S'
                case (col_idx)
                    3'd0: col_bits = 8'b01000110;
                    3'd1: col_bits = 8'b01001001;
                    3'd2: col_bits = 8'b01001001;
                    3'd3: col_bits = 8'b01001001;
                    3'd4: col_bits = 8'b00110001;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h54: begin // 'T'
                case (col_idx)
                    3'd0: col_bits = 8'b00000001;
                    3'd1: col_bits = 8'b00000001;
                    3'd2: col_bits = 8'b01111111;
                    3'd3: col_bits = 8'b00000001;
                    3'd4: col_bits = 8'b00000001;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h55: begin // 'U'
                case (col_idx)
                    3'd0: col_bits = 8'b00111111;
                    3'd1: col_bits = 8'b01000000;
                    3'd2: col_bits = 8'b01000000;
                    3'd3: col_bits = 8'b01000000;
                    3'd4: col_bits = 8'b00111111;
                    default: col_bits = 8'h00;
                endcase
            end

            8'h59: begin // 'Y'
                case (col_idx)
                    3'd0: col_bits = 8'b00000111;
                    3'd1: col_bits = 8'b00001000;
                    3'd2: col_bits = 8'b01110000;
                    3'd3: col_bits = 8'b00001000;
                    3'd4: col_bits = 8'b00000111;
                    default: col_bits = 8'h00;
                endcase
            end

            //==================================================================
            // Default / unsupported character
            //------------------------------------------------------------------
            // Policy:
            //   Unsupported characters render as blank.
            //
            // Why this is useful:
            //   It prevents random ASCII values from producing undefined glyphs
            //   or garbage pixels in telemetry text.
            //==================================================================
            default: begin
                col_bits = 8'h00;
            end
        endcase
    end

endmodule

`default_nettype wire