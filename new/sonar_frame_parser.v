`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// sonar_frame_parser
//------------------------------------------------------------------------------
// ROLE
//   UART-byte-level parser for the sonar measurement frame.
//
// HIGH-LEVEL PURPOSE
//   This module receives individual UART bytes and recognizes a very specific
//   ASCII-formatted sonar message:
//
//       'R' <digit> <digit> <digit> <CR>
//
//   When that exact sequence is observed, the three ASCII digits are converted
//   into a numeric distance value and emitted on:
//
//       distance_in_raw
//
//   together with a one-cycle pulse:
//
//       distance_valid_pulse
//
//   If the byte stream violates the expected pattern at any parsing step, the
//   module emits:
//
//       parse_err_pulse
//
//   and returns to the initial waiting state.
//
// EXPECTED PROTOCOL GRAMMAR
//   The parser expects a 5-byte frame of the form:
//
//       Byte 0 : ASCII 'R'
//       Byte 1 : hundreds digit ('0'..'9')
//       Byte 2 : tens digit     ('0'..'9')
//       Byte 3 : ones digit     ('0'..'9')
//       Byte 4 : carriage return (0x0D)
//
//   Symbolically:
//
//       R d2 d1 d0 CR
//
//   where d2, d1, and d0 are ASCII decimal digits.
//
// EXAMPLE
//   The sequence:
//
//       'R' '1' '2' '3' 0x0D
//
//   is interpreted as:
//
//       distance_in_raw = 123
//
// WHY THIS MODULE EXISTS
//   A UART receiver only reconstructs bytes. It does not know what those bytes
//   mean. This parser adds *protocol meaning* by imposing a frame grammar on
//   the incoming byte stream.
//
//   Conceptually:
//
//       asynchronous serial bits
//           -> sonar_uart_rx
//           -> byte stream
//           -> sonar_frame_parser
//           -> structured sonar measurement
//
// FSM PHILOSOPHY
//   The parser is implemented as a small finite-state machine because the input
//   is naturally sequential and grammar-driven.
//
//   Each parser state corresponds to "what byte is expected next":
//
//     ST_WAIT_R : waiting for frame leader 'R'
//     ST_D2     : waiting for hundreds digit
//     ST_D1     : waiting for tens digit
//     ST_D0     : waiting for ones digit
//     ST_CR     : waiting for carriage return
//
//   This is one of the clearest possible hardware realizations of a simple
//   protocol grammar.
//
// SIGNAL SEMANTICS
//   clk
//     Local synchronous clock.
//
//   rst
//     Active-high synchronous reset.
//
//   rx_byte
//     Most recently received UART byte from the upstream byte receiver.
//
//   rx_valid
//     One-cycle pulse indicating that rx_byte is valid this cycle and should be
//     considered by the parser.
//
//   distance_in_raw
//     Numeric value formed from the three parsed decimal digits.
//     Updated only when a full valid frame is completed.
//
//   distance_valid_pulse
//     One-cycle pulse asserted only when a full valid frame has just been
//     recognized and distance_in_raw has been updated.
//
//   parse_err_pulse
//     One-cycle pulse asserted when the current valid byte violates the
//     expected frame syntax for the current parser state.
//
// PULSE SEMANTICS
//   Both output flags are pulses, not sticky indicators.
//
//   At the start of every active non-reset clock cycle, the module clears:
//
//       distance_valid_pulse <= 0
//       parse_err_pulse      <= 0
//
//   Then, if rx_valid is asserted, one of those pulses may be raised depending
//   on whether the new byte advances a valid frame or violates the grammar.
//
// INTERNAL DIGIT STORAGE
//   d2, d1, d0
//     These hold the numeric values of the three decimal digits after ASCII
//     conversion. They are not stored as ASCII characters; instead, the parser
//     subtracts `"0"` from each ASCII digit byte and stores the resulting
//     numeric nibble.
//
// ASCII DIGIT CONVERSION
//   If rx_byte is the ASCII character "7", then:
//
//       rx_byte - "0"
//
//   yields the numeric value 7.
//
//   This is why the parser can later form the distance numerically as:
//
//       (d2 * 100) + (d1 * 10) + d0
//
// FUNCTIONAL SUMMARY
//   The parser performs the following conceptual sequence:
//
//     Step 1: Wait for the frame leader 'R'
//     Step 2: Capture the hundreds digit
//     Step 3: Capture the tens digit
//     Step 4: Capture the ones digit
//     Step 5: Confirm the carriage return
//     Step 6: Convert the three digits into a numeric distance
//
// RESET BEHAVIOR
//   On rst assertion:
//
//     - parser state returns to ST_WAIT_R
//     - digit registers are cleared to zero
//     - distance_in_raw is cleared to zero
//     - output pulses are cleared
//
// ERROR RECOVERY POLICY
//   On most syntax violations, the parser:
//
//     1) pulses parse_err_pulse
//     2) returns to ST_WAIT_R
//
//   This means the parser does not attempt partial recovery within a malformed
//   frame. Instead, it discards the current parsing attempt and waits for a new
//   frame leader.
//
// IMPORTANT LIMITATION
//   In ST_WAIT_R, any valid byte other than 'R' causes parse_err_pulse.
//   That means the parser treats all non-'R' bytes observed while idle as parse
//   errors, rather than silently ignoring unrelated traffic.
//
//   This is acceptable if the upstream stream is expected to consist only of
//   sonar frames, but it is a design choice worth documenting explicitly.
//
// DESIGN PHILOSOPHY
//   The parser is intentionally strict and simple:
//
//     - strict, because it enforces one exact 5-byte grammar
//     - simple, because each parser state corresponds directly to one expected
//       next symbol in the frame
//
//   This makes the module easy to review, simulate, and extend if later sensor
//   variants require slightly richer framing.
//------------------------------------------------------------------------------
module sonar_frame_parser (
    //--------------------------------------------------------------------------
    // Synchronous clock.
    //--------------------------------------------------------------------------
    input  wire       clk,

    //--------------------------------------------------------------------------
    // Active-high synchronous reset.
    //--------------------------------------------------------------------------
    input  wire       rst,

    //--------------------------------------------------------------------------
    // Received byte from the upstream UART receiver.
    //
    // This byte is only considered by the parser when rx_valid is high.
    //--------------------------------------------------------------------------
    input  wire [7:0] rx_byte,

    //--------------------------------------------------------------------------
    // One-cycle pulse indicating that rx_byte is valid this cycle.
    //--------------------------------------------------------------------------
    input  wire       rx_valid,

    //--------------------------------------------------------------------------
    // Parsed numeric distance value.
    //
    // Updated only when a complete valid sonar frame has been recognized.
    //--------------------------------------------------------------------------
    output reg  [9:0] distance_in_raw,

    //--------------------------------------------------------------------------
    // One-cycle pulse indicating successful frame recognition.
    //--------------------------------------------------------------------------
    output reg        distance_valid_pulse,

    //--------------------------------------------------------------------------
    // One-cycle pulse indicating a syntax violation in the received frame.
    //--------------------------------------------------------------------------
    output reg        parse_err_pulse
);

    //==========================================================================
    // Parser state encoding
    //--------------------------------------------------------------------------
    // ST_WAIT_R
    //   Waiting for the initial frame leader 'R'
    //
    // ST_D2
    //   Waiting for the hundreds digit
    //
    // ST_D1
    //   Waiting for the tens digit
    //
    // ST_D0
    //   Waiting for the ones digit
    //
    // ST_CR
    //   Waiting for the terminating carriage return
    //==========================================================================
    localparam [2:0] ST_WAIT_R = 3'd0;
    localparam [2:0] ST_D2     = 3'd1;
    localparam [2:0] ST_D1     = 3'd2;
    localparam [2:0] ST_D0     = 3'd3;
    localparam [2:0] ST_CR     = 3'd4;

    //==========================================================================
    // Internal parser state and digit registers
    //--------------------------------------------------------------------------
    // state
    //   Current parser state.
    //
    // d2, d1, d0
    //   Numeric values of the three decimal digits once converted from ASCII.
    //
    // Width choice:
    //   4 bits are sufficient to store decimal digits 0 through 9.
    //==========================================================================
    reg [2:0] state;
    reg [3:0] d2, d1, d0;

    //==========================================================================
    // ASCII digit classifier
    //--------------------------------------------------------------------------
    // Purpose:
    //   Determine whether the input character lies in the ASCII decimal digit
    //   range '0' through '9'.
    //
    // Step-by-step logic:
    //   1) Compare ch against ASCII '0'
    //   2) Compare ch against ASCII '9'
    //   3) Return true only if both comparisons indicate that ch lies within
    //      the inclusive digit range
    //
    // Why use a function?
    //   Because digit checks occur in multiple parser states, and the function
    //   keeps the parsing logic compact and readable.
    //==========================================================================
    function is_digit;
        input [7:0] ch;
        begin
            is_digit = (ch >= "0") && (ch <= "9");
        end
    endfunction

    //==========================================================================
    // Parser sequential process
    //--------------------------------------------------------------------------
    // Overall structure:
    //
    //   A) Reset handling
    //   B) Default clearing of pulse outputs
    //   C) Parsing work only when rx_valid is asserted
    //   D) FSM transition/update based on current parser state
    //
    // Important design choice:
    //   The parser does nothing on cycles where rx_valid = 0, other than clear
    //   its one-cycle pulse outputs. This means the parser advances exactly one
    //   step per received UART byte, not per clock cycle.
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            //------------------------------------------------------------------
            // Reset case
            //
            // Reset returns the parser to its initial "waiting for R" state and
            // clears all stored digits, outputs, and parsed distance.
            //------------------------------------------------------------------
            state                <= ST_WAIT_R;
            d2                   <= 4'd0;
            d1                   <= 4'd0;
            d0                   <= 4'd0;
            distance_in_raw      <= 10'd0;
            distance_valid_pulse <= 1'b0;
            parse_err_pulse      <= 1'b0;
        end else begin
            //------------------------------------------------------------------
            // Default pulse clearing
            //
            // Both output indicators are one-cycle pulses, so they are cleared
            // at the beginning of each active non-reset cycle.
            //------------------------------------------------------------------
            distance_valid_pulse <= 1'b0;
            parse_err_pulse      <= 1'b0;

            //------------------------------------------------------------------
            // Parsing proceeds only when a new valid byte arrives.
            //
            // If rx_valid is low, the parser holds its current state and stored
            // digits, waiting for the next byte.
            //------------------------------------------------------------------
            if (rx_valid) begin
                case (state)

                    //==========================================================
                    // ST_WAIT_R
                    //----------------------------------------------------------
                    // Purpose:
                    //   Wait for the frame leader 'R'.
                    //
                    // Step-by-step behavior:
                    //   1) Examine the current rx_byte.
                    //   2) If it is ASCII 'R', accept it as the start of a new
                    //      frame and move to ST_D2.
                    //   3) Otherwise, pulse parse_err_pulse and remain in the
                    //      waiting state.
                    //
                    // Design choice:
                    //   Any valid non-'R' byte while idle is considered a parse
                    //   error rather than being ignored silently.
                    //==========================================================
                    ST_WAIT_R: begin
                        if (rx_byte == "R")
                            state <= ST_D2;
                        else
                            parse_err_pulse <= 1'b1;
                    end

                    //==========================================================
                    // ST_D2
                    //----------------------------------------------------------
                    // Purpose:
                    //   Capture the hundreds digit.
                    //
                    // Step-by-step behavior:
                    //   1) Check whether rx_byte is an ASCII decimal digit.
                    //   2) If yes:
                    //        - convert ASCII digit to numeric value by
                    //          subtracting "0"
                    //        - store the result in d2
                    //        - advance to ST_D1
                    //   3) If not:
                    //        - pulse parse_err_pulse
                    //        - abandon the current frame
                    //        - return to ST_WAIT_R
                    //==========================================================
                    ST_D2: begin
                        if (is_digit(rx_byte)) begin
                            d2    <= rx_byte - "0";
                            state <= ST_D1;
                        end else begin
                            parse_err_pulse <= 1'b1;
                            state <= ST_WAIT_R;
                        end
                    end

                    //==========================================================
                    // ST_D1
                    //----------------------------------------------------------
                    // Purpose:
                    //   Capture the tens digit.
                    //
                    // Step-by-step behavior:
                    //   1) Check whether rx_byte is an ASCII decimal digit.
                    //   2) If yes:
                    //        - convert ASCII digit to numeric value
                    //        - store it in d1
                    //        - advance to ST_D0
                    //   3) If not:
                    //        - pulse parse_err_pulse
                    //        - abandon the current frame
                    //        - return to ST_WAIT_R
                    //==========================================================
                    ST_D1: begin
                        if (is_digit(rx_byte)) begin
                            d1    <= rx_byte - "0";
                            state <= ST_D0;
                        end else begin
                            parse_err_pulse <= 1'b1;
                            state <= ST_WAIT_R;
                        end
                    end

                    //==========================================================
                    // ST_D0
                    //----------------------------------------------------------
                    // Purpose:
                    //   Capture the ones digit.
                    //
                    // Step-by-step behavior:
                    //   1) Check whether rx_byte is an ASCII decimal digit.
                    //   2) If yes:
                    //        - convert ASCII digit to numeric value
                    //        - store it in d0
                    //        - advance to ST_CR
                    //   3) If not:
                    //        - pulse parse_err_pulse
                    //        - abandon the current frame
                    //        - return to ST_WAIT_R
                    //==========================================================
                    ST_D0: begin
                        if (is_digit(rx_byte)) begin
                            d0    <= rx_byte - "0";
                            state <= ST_CR;
                        end else begin
                            parse_err_pulse <= 1'b1;
                            state <= ST_WAIT_R;
                        end
                    end

                    //==========================================================
                    // ST_CR
                    //----------------------------------------------------------
                    // Purpose:
                    //   Confirm that the frame terminates with carriage return.
                    //
                    // Step-by-step behavior:
                    //   1) Check whether rx_byte equals 0x0D (carriage return).
                    //   2) If yes:
                    //        - compute the numeric distance from the previously
                    //          captured decimal digits:
                    //
                    //              distance = d2*100 + d1*10 + d0
                    //
                    //        - store the result in distance_in_raw
                    //        - pulse distance_valid_pulse
                    //   3) If not:
                    //        - pulse parse_err_pulse
                    //   4) In either case, return to ST_WAIT_R so the next frame
                    //      must begin with a fresh 'R'
                    //
                    // Why the multiplication factors are 100, 10, and 1:
                    //   Because d2 is the hundreds digit, d1 is the tens digit,
                    //   and d0 is the ones digit in standard decimal notation.
                    //==========================================================
                    ST_CR: begin
                        if (rx_byte == 8'h0D) begin
                            distance_in_raw <= (d2 * 10'd100) + (d1 * 10'd10) + d0;
                            distance_valid_pulse <= 1'b1;
                        end else begin
                            parse_err_pulse <= 1'b1;
                        end
                        state <= ST_WAIT_R;
                    end

                    //==========================================================
                    // Default recovery
                    //----------------------------------------------------------
                    // Purpose:
                    //   Guarantee that any illegal or unknown state encoding is
                    //   driven back to the initial parser state.
                    //==========================================================
                    default: state <= ST_WAIT_R;
                endcase
            end
        end
    end

endmodule

`default_nettype wire