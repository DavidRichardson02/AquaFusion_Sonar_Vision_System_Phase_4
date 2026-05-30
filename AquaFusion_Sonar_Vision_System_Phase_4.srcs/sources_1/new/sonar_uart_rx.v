`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// sonar_uart_rx
//------------------------------------------------------------------------------
// ROLE
//   Byte-oriented UART receiver for the sonar serial stream.
//
// HIGH-LEVEL PURPOSE
//   This module converts an asynchronous serial input line, `rx_i`, into:
//
//     - `rx_byte`   : the received 8-bit data byte
//     - `rx_valid`  : a one-cycle pulse indicating that a valid UART frame has
//                     just been received
//     - `frame_err` : a one-cycle pulse indicating that the stop bit was not
//                     valid, meaning the received frame was malformed
//
// UART ASSUMPTION
//   The module is written for a conventional UART-style frame consisting of:
//
//       idle line = logic 1
//       start bit = logic 0
//       8 data bits, LSB first
//       stop bit  = logic 1
//
//   So the expected frame structure is:
//
//       [idle=1 ...] [start=0] [b0] [b1] [b2] [b3] [b4] [b5] [b6] [b7] [stop=1]
//
// WHY THIS MODULE EXISTS
//   The sonar sensor transmits characters asynchronously. That means the FPGA
//   does not receive a shared transmit clock. Instead, the receiver must:
//
//     1) detect the falling edge / low level indicating a start bit
//     2) wait appropriate fractions of a bit time
//     3) sample each data bit at the proper times
//     4) verify that the stop bit is high
//
//   This module performs that reconstruction using the local FPGA clock and a
//   simple finite-state machine.
//
// IMPORTANT TIMING CONCEPT
//   The receiver does not know the sender's clock directly. It approximates the
//   sender's bit timing using:
//
//       CLKS_PER_BIT = CLK_HZ / BAUD
//
//   This means one UART bit interval is modeled as `CLKS_PER_BIT` cycles of the
//   local `clk`.
//
// START-BIT STRATEGY
//   The start bit is treated specially.
//
//   When the line first goes low in IDLE, the module does *not* immediately
//   trust that event as a valid frame. Instead, it moves to ST_START and waits
//   until approximately half a bit time has elapsed. It then samples the line
//   again.
//
//   Why?
//   Because this helps reject short glitches or noise spikes that momentarily
//   pull the line low. A real start bit should still be low near the middle of
//   the bit cell.
//
// DATA-BIT STRATEGY
//   Once the start bit is confirmed, the receiver enters ST_DATA and samples one
//   data bit per bit interval. The sampled bits are written into `shreg` using:
//
//       shreg[bit_idx] <= rx_i;
//
//   That means the receiver stores bits LSB-first, which matches standard UART
//   ordering.
//
// STOP-BIT STRATEGY
//   After 8 data bits are collected, the receiver enters ST_STOP and waits one
//   more full bit interval. At that sample point:
//
//     - if `rx_i == 1`, the frame is accepted
//     - if `rx_i != 1`, the frame is rejected with `frame_err`
//
//   In both cases, the state machine returns to IDLE afterward.
//
// SIGNAL SEMANTICS
//   clk
//     Local FPGA clock used to measure UART bit intervals.
//
//   rst
//     Active-high synchronous reset.
//
//   rx_i
//     Asynchronous UART receive input line.
//
//   rx_byte
//     Received byte value. Updated when the stop bit sample is reached.
//
//   rx_valid
//     One-clock pulse indicating that `rx_byte` contains a newly received valid
//     byte from a frame with a correct stop bit.
//
//   frame_err
//     One-clock pulse indicating that a frame ended with an invalid stop bit.
//
// PULSE SEMANTICS
//   `rx_valid` and `frame_err` are intentionally pulsed outputs, not sticky
//   flags. At the start of every non-reset clock cycle, both are cleared to 0,
//   and they are asserted only in the specific cycle where the stop-bit sample
//   is evaluated.
//
// PARAMETER SEMANTICS
//   CLK_HZ
//     Frequency of the local FPGA clock in hertz.
//
//   BAUD
//     UART symbol rate in bits per second.
//
// DERIVED CONSTANT
//   CLKS_PER_BIT = CLK_HZ / BAUD
//
//   This is the integer number of local clock cycles assigned to one UART bit.
//   Since integer division is used, exact timing is only achieved when CLK_HZ is
//   an exact multiple of BAUD. Otherwise, the implementation uses the truncated
//   integer approximation.
//
// FSM OVERVIEW
//   ST_IDLE
//     Wait for the line to go low, indicating possible start of frame.
//
//   ST_START
//     Wait roughly half a bit time and confirm that the line is still low.
//
//   ST_DATA
//     Sample 8 data bits, one per full bit interval.
//
//   ST_STOP
//     Sample the stop bit and either accept or reject the frame.
//
// RESET BEHAVIOR
//   On rst assertion:
//
//     - state     <= ST_IDLE
//     - clk_ctr   <= 0
//     - bit_idx   <= 0
//     - shreg     <= 0
//     - rx_byte   <= 0
//     - rx_valid  <= 0
//     - frame_err <= 0
//
//   Therefore, reset returns the receiver to a clean "waiting for a new frame"
//   condition.
//
// DESIGN LIMITATIONS / ASSUMPTIONS
//   1) No explicit input synchronizer is present for rx_i.
//      In a stricter implementation, a 2-flop synchronizer or oversampling
//      front-end could be considered, although care is needed because UART
//      reception is itself a timing-sensitive asynchronous-input problem.
//
//   2) No parity bit is supported.
//      This receiver expects only start + 8 data + stop.
//
//   3) No multiple-stop-bit support is explicitly modeled.
//      Only one stop-bit sample point is checked.
//
//   4) No oversampling is used.
//      This is a simple single-sample-per-bit receiver using the system clock
//      as a coarse timing base.
//
//   5) Timing error tolerance depends on CLK_HZ / BAUD ratio.
//      Higher clock-to-baud ratios provide finer timing granularity.
//
// PEDAGOGICAL SUMMARY
//   The module can be read as a four-step serial decoding process:
//
//     Step 1: Wait for a falling line (possible start bit)
//     Step 2: Re-check halfway into the start bit to confirm it
//     Step 3: Sample 8 data bits, storing them into a shift register
//     Step 4: Sample the stop bit and classify the frame as valid or erroneous
//------------------------------------------------------------------------------
module sonar_uart_rx #(
    //--------------------------------------------------------------------------
    // Local FPGA clock frequency in hertz.
    //--------------------------------------------------------------------------
    parameter integer CLK_HZ = 100_000_000,

    //--------------------------------------------------------------------------
    // UART baud rate in bits per second.
    //--------------------------------------------------------------------------
    parameter integer BAUD   = 9600
)(
    //--------------------------------------------------------------------------
    // Local receiver clock.
    //
    // All internal timing is derived from this clock.
    //--------------------------------------------------------------------------
    input  wire       clk,

    //--------------------------------------------------------------------------
    // Active-high synchronous reset.
    //--------------------------------------------------------------------------
    input  wire       rst,

    //--------------------------------------------------------------------------
    // Asynchronous UART receive input line.
    //
    // Idle state is expected to be logic 1.
    //--------------------------------------------------------------------------
    input  wire       rx_i,

    //--------------------------------------------------------------------------
    // Received data byte.
    //
    // Updated when the stop-bit sample point is reached.
    //--------------------------------------------------------------------------
    output reg  [7:0] rx_byte,

    //--------------------------------------------------------------------------
    // One-cycle pulse indicating that a valid byte has just been received.
    //--------------------------------------------------------------------------
    output reg        rx_valid,

    //--------------------------------------------------------------------------
    // One-cycle pulse indicating that the stop bit was invalid.
    //--------------------------------------------------------------------------
    output reg        frame_err
);

    //==========================================================================
    // Bit timing constant
    //--------------------------------------------------------------------------
    // Number of local clk cycles corresponding to one UART bit interval.
    //
    // Example:
    //   CLK_HZ = 100_000_000
    //   BAUD   = 9600
    //
    //   CLKS_PER_BIT = 100_000_000 / 9600 = 10416 (integer truncation)
    //
    // This value drives the timing of start-bit confirmation, data sampling,
    // and stop-bit sampling.
    //==========================================================================
    localparam integer CLKS_PER_BIT = CLK_HZ / BAUD;

    //==========================================================================
    // State encoding
    //--------------------------------------------------------------------------
    // ST_IDLE  : wait for possible start bit
    // ST_START : verify start bit near its midpoint
    // ST_DATA  : collect 8 data bits
    // ST_STOP  : sample stop bit and finalize frame status
    //==========================================================================
    localparam [1:0] ST_IDLE  = 2'd0;
    localparam [1:0] ST_START = 2'd1;
    localparam [1:0] ST_DATA  = 2'd2;
    localparam [1:0] ST_STOP  = 2'd3;

    //==========================================================================
    // Internal state registers
    //--------------------------------------------------------------------------
    // state
    //   Current FSM state.
    //
    // clk_ctr
    //   Counts local clock cycles inside the current UART bit interval.
    //
    // bit_idx
    //   Identifies which data bit is currently being sampled.
    //   Since the receiver stores 8 data bits, a 3-bit index is sufficient.
    //
    // shreg
    //   Temporary storage for the 8 received data bits before they are copied
    //   into rx_byte.
    //==========================================================================
    reg [1:0]  state;
    reg [31:0] clk_ctr;
    reg [2:0]  bit_idx;
    reg [7:0]  shreg;

    //==========================================================================
    // UART receive state machine
    //--------------------------------------------------------------------------
    // Step-by-step structure:
    //
    //   A) Reset handling
    //      Establish a known initial state.
    //
    //   B) Default pulse clearing
    //      rx_valid and frame_err are cleared every cycle unless explicitly
    //      asserted later in this clock step.
    //
    //   C) State-specific behavior
    //      The FSM determines whether the receiver is waiting for a start bit,
    //      confirming a start bit, collecting data bits, or validating the stop
    //      bit.
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            //------------------------------------------------------------------
            // Reset case
            //
            // Reset returns the FSM to the waiting state and clears all timing,
            // indexing, and output state.
            //------------------------------------------------------------------
            state     <= ST_IDLE;
            clk_ctr   <= 32'd0;
            bit_idx   <= 3'd0;
            shreg     <= 8'd0;
            rx_byte   <= 8'd0;
            rx_valid  <= 1'b0;
            frame_err <= 1'b0;
        end else begin
            //------------------------------------------------------------------
            // Default pulse behavior
            //
            // rx_valid and frame_err are one-cycle indicators. Therefore they
            // are cleared at the start of each active cycle and only asserted in
            // the precise cycle where a frame result is determined.
            //------------------------------------------------------------------
            rx_valid  <= 1'b0;
            frame_err <= 1'b0;

            case (state)

                //==============================================================
                // ST_IDLE
                //--------------------------------------------------------------
                // Purpose:
                //   Wait for the receive line to transition from idle-high to
                //   low, indicating a possible start bit.
                //
                // Step-by-step behavior:
                //   1) Force clk_ctr to zero so no stale timing carries over.
                //   2) Force bit_idx to zero so the next frame starts at bit 0.
                //   3) If rx_i is low, tentatively treat that as start-bit
                //      onset and move to ST_START.
                //
                // Important nuance:
                //   The module does not yet trust the low level fully. It only
                //   treats it as a candidate start bit. The actual confirmation
                //   occurs in ST_START at the mid-bit sample point.
                //==============================================================
                ST_IDLE: begin
                    clk_ctr <= 32'd0;
                    bit_idx <= 3'd0;
                    if (rx_i == 1'b0)
                        state <= ST_START;
                end

                //==============================================================
                // ST_START
                //--------------------------------------------------------------
                // Purpose:
                //   Confirm that the detected low level is truly a start bit.
                //
                // Step-by-step behavior:
                //   1) Count local clock cycles from the moment the low level
                //      was detected.
                //   2) When approximately half a bit time has elapsed
                //      (CLKS_PER_BIT/2), sample rx_i again.
                //   3) If rx_i is still low, accept the start bit and proceed
                //      to ST_DATA.
                //   4) If rx_i is no longer low, treat the event as noise or a
                //      false start and return to ST_IDLE.
                //
                // Why sample at half a bit?
                //   Because the midpoint of the start bit is a more reliable
                //   place to validate that the low level is real and stable.
                //==============================================================
                ST_START: begin
                    if (clk_ctr == (CLKS_PER_BIT/2)) begin
                        clk_ctr <= 32'd0;
                        if (rx_i == 1'b0)
                            state <= ST_DATA;
                        else
                            state <= ST_IDLE;
                    end else begin
                        clk_ctr <= clk_ctr + 32'd1;
                    end
                end

                //==============================================================
                // ST_DATA
                //--------------------------------------------------------------
                // Purpose:
                //   Sample and store the 8 UART data bits.
                //
                // Step-by-step behavior:
                //   1) Wait until one full bit interval has elapsed.
                //   2) Sample rx_i and store it into shreg[bit_idx].
                //   3) If this was bit 7, all 8 bits are now captured:
                //        - reset bit_idx to zero
                //        - move to ST_STOP
                //      Otherwise:
                //        - increment bit_idx
                //        - remain in ST_DATA to capture the next bit
                //
                // Bit ordering:
                //   The assignment
                //
                //       shreg[bit_idx] <= rx_i;
                //
                //   means the first received data bit goes into bit 0, the next
                //   into bit 1, and so on. This matches LSB-first UART format.
                //
                // Timing note:
                //   Because clk_ctr is cleared before entering ST_DATA and then
                //   allowed to run for a full CLKS_PER_BIT interval between
                //   samples, each data sample occurs one bit period apart in the
                //   receiver's local timing model.
                //==============================================================
                ST_DATA: begin
                    if (clk_ctr == CLKS_PER_BIT-1) begin
                        clk_ctr <= 32'd0;
                        shreg[bit_idx] <= rx_i;
                        if (bit_idx == 3'd7) begin
                            bit_idx <= 3'd0;
                            state   <= ST_STOP;
                        end else begin
                            bit_idx <= bit_idx + 3'd1;
                        end
                    end else begin
                        clk_ctr <= clk_ctr + 32'd1;
                    end
                end

                //==============================================================
                // ST_STOP
                //--------------------------------------------------------------
                // Purpose:
                //   Sample the stop bit and decide whether the frame is valid.
                //
                // Step-by-step behavior:
                //   1) Wait one full bit interval from the previous data sample.
                //   2) At the stop-bit sample point:
                //        - copy shreg into rx_byte
                //        - check whether rx_i is high
                //   3) If rx_i is high:
                //        - assert rx_valid for one cycle
                //        - do not assert frame_err
                //   4) If rx_i is not high:
                //        - assert frame_err for one cycle
                //        - do not assert rx_valid
                //   5) Return to ST_IDLE to wait for the next frame
                //
                // Important note:
                //   rx_byte is updated regardless of whether the stop bit is
                //   valid. The frame validity information is carried separately
                //   by rx_valid and frame_err.
                //==============================================================
                ST_STOP: begin
                    if (clk_ctr == CLKS_PER_BIT-1) begin
                        clk_ctr   <= 32'd0;
                        rx_byte   <= shreg;
                        rx_valid  <= (rx_i == 1'b1);
                        frame_err <= (rx_i != 1'b1);
                        state     <= ST_IDLE;
                    end else begin
                        clk_ctr <= clk_ctr + 32'd1;
                    end
                end

                //==============================================================
                // Default recovery
                //--------------------------------------------------------------
                // Purpose:
                //   Ensure that any illegal or unknown state encoding falls back
                //   to the idle state rather than leaving the FSM stranded.
                //==============================================================
                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire