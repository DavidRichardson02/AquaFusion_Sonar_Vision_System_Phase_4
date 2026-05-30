`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// oled_init_rom
//------------------------------------------------------------------------------
// ROLE
//   SSD1306 initialization command ROM for a 128x32 monochrome OLED.
//
// HIGH-LEVEL PURPOSE
//   This module provides a fixed, index-addressed initialization sequence for an
//   SSD1306-compatible OLED controller. It is intended to be consumed by a
//   higher-level OLED control FSM that steps through the sequence one entry at
//   a time.
//
//   Conceptually, this module answers the question:
//
//       "Given initialization step index N, what command byte should be sent?"
//
//   along with two pieces of control metadata:
//
//       valid
//         indicates whether the current index corresponds to a real command byte
//
//       last
//         indicates that the initialization sequence has ended
//
// WHY THIS MODULE EXISTS
//   OLED initialization is inherently sequence-oriented. A controller typically
//   needs to send a specific ordered list of command bytes after power-up.
//
//   Rather than hard-coding that command list inside a large controller FSM,
//   this ROM separates:
//
//     - command *content*  (this module)
//     from
//     - command *execution* (the controller FSM)
//
//   That separation is valuable because:
//
//     1) the initialization policy is easy to inspect and review,
//     2) the controller FSM can remain generic,
//     3) changing the command sequence does not require restructuring the
//        transaction engine.
//
// DEVICE-SPECIFIC INTENT
//   The comments indicate that this sequence is intended for a:
//
//       128x32 monochrome SSD1306 OLED
//
//   and that:
//
//     - page addressing mode is used during runtime refresh
//     - the sequence intentionally excludes 0xAF (Display ON)
//
// WHY 0xAF IS EXCLUDED
//   Display ON (0xAF) is commonly used to enable panel output after
//   initialization. This ROM intentionally stops short of that final step so
//   that a higher-level controller can:
//
//     1) power VBAT,
//     2) clear or initialize display RAM,
//     3) optionally write a known framebuffer,
//     4) only then enable visible display output
//
//   That is a careful and sensible bring-up policy because it avoids showing
//   uninitialized or stale screen contents during early power-up.
//
// SIGNAL SEMANTICS
//   index
//     6-bit initialization-step selector.
//     Each index corresponds to one position in the command sequence.
//
//   data
//     8-bit command byte for the current index.
//
//   valid
//     Indicates whether `data` should be interpreted as a real command byte.
//
//   last
//     Indicates that the current index corresponds to the sequence terminator.
//
// ROM SEMANTICS
//   This is a combinational ROM-style decoder:
//
//     index -> {data, valid, last}
//
//   There is no internal state, no clock, and no counter.
//   The calling controller is responsible for:
//
//     - supplying index values,
//     - advancing through the sequence,
//     - deciding what to do when `last` is asserted.
//
// DEFAULT POLICY
//   At the beginning of the combinational block, the outputs are assigned:
//
//       data  = 8'h00
//       valid = 1'b1
//       last  = 1'b0
//
//   This establishes a default assumption that the selected index is a valid
//   command byte unless later overridden by the case statement.
//
// END-OF-SEQUENCE POLICY
//   The sequence terminates explicitly at index 24:
//
//       data  = 8'h00
//       valid = 1'b0
//       last  = 1'b1
//
//   The same end-of-sequence signaling is also used in the default case, which
//   means any out-of-range or unspecified index is treated as:
//
//     - no valid command
//     - sequence ended
//
//   This is a defensive and useful policy because it prevents accidental
//   traversal beyond the defined script from being misinterpreted as a valid
//   command stream.
//
// ADDRESSING MODE NOTE
//   The ROM programs:
//
//       0x20, 0x02
//
//   which selects page addressing mode according to the design comments. That
//   means the runtime OLED refresh logic is expected to operate in page-address
//   organization rather than some alternative memory addressing mode.
//
// PEDAGOGICAL SUMMARY
//   The module can be understood as a read-only script:
//
//     index 0  -> first command byte
//     index 1  -> second command byte
//     ...
//     index 23 -> final real initialization command byte
//     index 24 -> explicit "end of script"
//
//   So this module describes *what* to send, while another module decides
//   *when* and *how* to send it.
//------------------------------------------------------------------------------
module oled_init_rom (
    //--------------------------------------------------------------------------
    // Initialization sequence index.
    //
    // The surrounding controller presents an index value to select one entry in
    // the command sequence.
    //--------------------------------------------------------------------------
    input  wire [5:0] index,

    //--------------------------------------------------------------------------
    // Command byte associated with the selected sequence index.
    //--------------------------------------------------------------------------
    output reg  [7:0] data,

    //--------------------------------------------------------------------------
    // Indicates that the selected index corresponds to a real command byte.
    //
    // valid = 1 -> `data` is a meaningful initialization command byte
    // valid = 0 -> no command byte should be sent for this index
    //--------------------------------------------------------------------------
    output reg        valid,

    //--------------------------------------------------------------------------
    // Indicates that the initialization sequence has ended.
    //
    // last = 1 means the calling controller has reached the terminator.
    //--------------------------------------------------------------------------
    output reg        last
);

    //==========================================================================
    // Combinational ROM decode
    //--------------------------------------------------------------------------
    // Step-by-step evaluation strategy:
    //
    //   1) Start from a default assumption:
    //        - data  = 0x00
    //        - valid = 1
    //        - last  = 0
    //
    //   2) Use `index` to select a specific command byte from the case table.
    //
    //   3) For the explicit sequence terminator and for any unspecified index:
    //        - invalidate the command byte
    //        - assert `last`
    //
    // Interpretation:
    //   The case table is the actual ROM contents, while the default block at
    //   the top provides a predictable fallback behavior for unspecified fields.
    //==========================================================================
    always @(*) begin
        //----------------------------------------------------------------------
        // Default outputs
        //
        // By default, treat the selected index as an ordinary valid command
        // entry with data initialized to zero unless later overwritten by the
        // case statement.
        //----------------------------------------------------------------------
        data  = 8'h00;
        valid = 1'b1;
        last  = 1'b0;

        case (index)

            //------------------------------------------------------------------
            // 0xAE : Display OFF
            //
            // Purpose:
            //   Ensure the panel output is disabled while configuration is
            //   taking place.
            //------------------------------------------------------------------
            6'd0:  data = 8'hAE;

            //------------------------------------------------------------------
            // 0xD5, 0x80 : Display clock divide / oscillator setting
            //
            // Purpose:
            //   Configure internal display timing behavior.
            //------------------------------------------------------------------
            6'd1:  data = 8'hD5;
            6'd2:  data = 8'h80;

            //------------------------------------------------------------------
            // 0xA8, 0x1F : Multiplex ratio = 1/32
            //
            // Purpose:
            //   Match the panel height for a 32-row display.
            //------------------------------------------------------------------
            6'd3:  data = 8'hA8;
            6'd4:  data = 8'h1F;

            //------------------------------------------------------------------
            // 0xD3, 0x00 : Display offset = 0
            //
            // Purpose:
            //   Set vertical display offset to zero.
            //------------------------------------------------------------------
            6'd5:  data = 8'hD3;
            6'd6:  data = 8'h00;

            //------------------------------------------------------------------
            // 0x40 : Start line = 0
            //
            // Purpose:
            //   Select display RAM line mapping starting at line 0.
            //------------------------------------------------------------------
            6'd7:  data = 8'h40;

            //------------------------------------------------------------------
            // 0x8D, 0x14 : Charge pump enable
            //
            // Purpose:
            //   Enable the internal charge pump as required by the intended
            //   SSD1306 operating mode.
            //------------------------------------------------------------------
            6'd8:  data = 8'h8D;
            6'd9:  data = 8'h14;

            //------------------------------------------------------------------
            // 0x20, 0x02 : Memory addressing mode = page addressing
            //
            // Purpose:
            //   Configure the device so runtime refresh logic can operate in
            //   page-addressed mode.
            //------------------------------------------------------------------
            6'd10: data = 8'h20;
            6'd11: data = 8'h02;

            //------------------------------------------------------------------
            // 0xA1 : Segment remap
            //
            // Purpose:
            //   Configure horizontal segment orientation.
            //------------------------------------------------------------------
            6'd12: data = 8'hA1;

            //------------------------------------------------------------------
            // 0xC8 : COM scan direction decrement
            //
            // Purpose:
            //   Configure vertical scan direction/orientation.
            //------------------------------------------------------------------
            6'd13: data = 8'hC8;

            //------------------------------------------------------------------
            // 0xDA, 0x02 : COM pins hardware configuration
            //
            // Purpose:
            //   Configure COM pin layout appropriate to the target panel.
            //------------------------------------------------------------------
            6'd14: data = 8'hDA;
            6'd15: data = 8'h02;

            //------------------------------------------------------------------
            // 0x81, 0x8F : Contrast control
            //
            // Purpose:
            //   Set display contrast.
            //------------------------------------------------------------------
            6'd16: data = 8'h81;
            6'd17: data = 8'h8F;

            //------------------------------------------------------------------
            // 0xD9, 0xF1 : Precharge period
            //
            // Purpose:
            //   Configure panel precharge characteristics.
            //------------------------------------------------------------------
            6'd18: data = 8'hD9;
            6'd19: data = 8'hF1;

            //------------------------------------------------------------------
            // 0xDB, 0x40 : VCOM detect level
            //
            // Purpose:
            //   Configure VCOMH-related display behavior.
            //------------------------------------------------------------------
            6'd20: data = 8'hDB;
            6'd21: data = 8'h40;

            //------------------------------------------------------------------
            // 0xA4 : Display follows RAM contents
            //
            // Purpose:
            //   Ensure panel output reflects display RAM rather than forcing all
            //   pixels on.
            //------------------------------------------------------------------
            6'd22: data = 8'hA4;

            //------------------------------------------------------------------
            // 0xA6 : Normal display mode
            //
            // Purpose:
            //   Select normal, non-inverted display rendering.
            //------------------------------------------------------------------
            6'd23: data = 8'hA6;

            //------------------------------------------------------------------
            // Explicit end-of-sequence marker
            //
            // Policy:
            //   No valid command byte is produced here.
            //   Instead, the caller is informed that the ROM script has ended.
            //------------------------------------------------------------------
            6'd24: begin
                data  = 8'h00;
                valid = 1'b0;
                last  = 1'b1;
            end

            //------------------------------------------------------------------
            // Default / out-of-range handling
            //
            // Policy:
            //   Treat all unspecified indices as end-of-sequence.
            //
            // Why this is useful:
            //   It prevents accidental traversal beyond the defined ROM contents
            //   from generating spurious valid command bytes.
            //------------------------------------------------------------------
            default: begin
                data  = 8'h00;
                valid = 1'b0;
                last  = 1'b1;
            end
        endcase
    end

endmodule

`default_nettype wire