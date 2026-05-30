`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// MODULE: i2c_master_engine
//------------------------------------------------------------------------------
// ROLE
//   Reusable open-drain I2C/SCCB transaction engine.
//
// SYSTEM CONTEXT
//   This module is the physical serial-bus engine used by camera-control logic.
//   Higher-level modules issue abstract commands such as:
//
//       write I2C switch control byte
//       write OV5640 register
//       read OV5640 register
//
//   This engine converts those commands into actual open-drain SCL/SDA activity.
//
// SUPPORTED TRANSACTION TYPES
//   1) Write, no register phase
//
//        START
//        DEV_ADDR + W
//        DATA
//        STOP
//
//      Used for simple I2C devices such as the FMC Pcam Adapter I2C switch.
//
//   2) Write, 16-bit register phase
//
//        START
//        DEV_ADDR + W
//        REG_ADDR[15:8]
//        REG_ADDR[7:0]
//        DATA
//        STOP
//
//      Used for OV5640 SCCB register writes.
//
//   3) Read, 16-bit register phase with repeated START
//
//        START
//        DEV_ADDR + W
//        REG_ADDR[15:8]
//        REG_ADDR[7:0]
//        REPEATED_START
//        DEV_ADDR + R
//        READ_DATA
//        NACK
//        STOP
//
//      Used for OV5640 SCCB register reads.
//
// UNSUPPORTED TRANSACTION TYPE
//   Read without register phase is not supported in this revision.
//
// OWNERSHIP BOUNDARY
//   This module owns:
//     - physical SCL/SDA drive-low/release-high-Z behavior,
//     - START / repeated START / STOP generation,
//     - byte serialization,
//     - byte reception,
//     - ACK/NACK sampling,
//     - transaction completion response generation.
//
//   This module does not own:
//     - retry policy,
//     - interpretation of register contents,
//     - camera initialization sequencing,
//     - FMC switch channel policy,
//     - clock-domain crossing.
//
// CLOCK / RESET CONTRACT
//   clk:
//     Control-plane clock. All internal registers are synchronous to clk.
//
//   rst:
//     Active-high synchronous reset.
//
// COMMAND HANDSHAKE CONTRACT
//   A command is accepted on a rising clk edge when:
//
//       cmd_valid == 1
//       cmd_ready == 1
//
//   The command producer must hold all cmd_* fields stable while cmd_valid is
//   high and until the accepting edge.
//
// RESPONSE CONTRACT
//   rsp_valid pulses for exactly one clk cycle when the transaction finishes.
//
//   rsp_err:
//     General transaction failure.
//
//   rsp_ack_error:
//     Specific error class indicating a target NACK.
//
//   rsp_rd_data:
//     Valid only for successful read transactions when rsp_valid is high and
//     rsp_err/rsp_ack_error are both low.
//
// OPEN-DRAIN ELECTRICAL CONTRACT
//   scl_io and sda_io are modeled as open-drain signals:
//
//       output 0   -> drive the line low
//       output 1/Z -> release the line so pull-up can bring it high
//
//   Therefore, this module never drives a logic 1 onto SCL/SDA. It either drives
//   0 or releases high-Z.
//
// CLOCK-STRETCHING LIMITATION
//   This engine does not implement target clock stretching. When SCL is released,
//   the design assumes the pull-up and target allow SCL to go high within the
//   programmed timing interval.
//
// TIMING NOTE
//   The internal bit sequencer uses three timing phases for each data bit:
//
//       setup while SCL low
//       SCL high
//       SCL low / advance
//
//   The divider is chosen conservatively so this three-phase bit period does not
//   exceed the requested SCL_HZ under normal integer rounding.
//
// FAILURE / ILLEGAL-STATE POLICY
//   Any illegal state releases SCL/SDA, reports rsp_err, pulses rsp_valid, and
//   returns to idle. This prevents the bus from being held low indefinitely.
//
// SYNTHESIS NOTES
//   - Verilog-2001 compatible.
//   - No SystemVerilog constructs.
//   - Constant functions are used only for elaboration-time parameter math.
//   - In FPGA synthesis, top-level I/O constraints or IOBUF inference must
//     preserve open-drain behavior on scl_io and sda_io.
//==============================================================================

module i2c_master_engine #(
    //--------------------------------------------------------------------------
    // CLK_HZ
    //--------------------------------------------------------------------------
    // Frequency of clk in hertz.
    //
    // Used to derive the internal timing divider.
    //--------------------------------------------------------------------------
    parameter integer CLK_HZ = 100_000_000,

    //--------------------------------------------------------------------------
    // SCL_HZ
    //--------------------------------------------------------------------------
    // Target maximum SCL frequency.
    //
    // The engine chooses an integer divider such that the generated SCL data-bit
    // rate is at or below this target for normal transactions.
    //--------------------------------------------------------------------------
    parameter integer SCL_HZ = 400_000
)(
    //--------------------------------------------------------------------------
    // Clock/reset
    //--------------------------------------------------------------------------
    input  wire        clk,
    input  wire        rst,

    //--------------------------------------------------------------------------
    // Command request channel
    //--------------------------------------------------------------------------
    input  wire        cmd_valid,
    output reg         cmd_ready,

    input  wire        cmd_has_reg,
    input  wire        cmd_is_read,
    input  wire [6:0]  cmd_dev_addr,
    input  wire [15:0] cmd_reg_addr,
    input  wire [7:0]  cmd_wr_data,

    //--------------------------------------------------------------------------
    // Response channel
    //--------------------------------------------------------------------------
    output reg         rsp_valid,
    output reg         rsp_err,
    output reg         rsp_ack_error,
    output reg [7:0]   rsp_rd_data,

    //--------------------------------------------------------------------------
    // Engine status
    //--------------------------------------------------------------------------
    output reg         busy,

    //--------------------------------------------------------------------------
    // Physical open-drain I2C/SCCB pins
    //--------------------------------------------------------------------------
    inout  wire        scl_io,
    inout  wire        sda_io
);

    //==========================================================================
    // FUNCTION: div_ceil_at_least_one
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Compute ceil(numerator / denominator), clamped to at least one.
    //
    // INPUT CONTRACT
    //   numerator:
    //     Positive or non-positive integer numerator.
    //
    //   denominator:
    //     Positive or non-positive integer denominator.
    //
    // OUTPUT CONTRACT
    //   Returns an integer result.
    //   The result is always at least 1.
    //
    // WHY CEILING DIVISION IS USED
    //   A timing divider that rounds down can make the generated SCL faster than
    //   requested. Ceiling division biases the divider upward, which keeps the
    //   resulting bus frequency at or below the target.
    //
    // MATHEMATICAL BASIS
    //   For positive integers:
    //
    //       ceil(N / D) = (N + D - 1) / D
    //
    // STEP-BY-STEP
    //   1) If numerator <= 0, return 1.
    //   2) If denominator <= 0, return 1.
    //   3) Otherwise compute `(numerator + denominator - 1) / denominator`.
    //   4) Clamp any accidental non-positive result back to 1.
    //
    // WORKED EXAMPLE
    //   numerator   = 100_000_000
    //   denominator = 1_200_000
    //
    //   result = ceil(83.333...)
    //          = 84
    //
    // SYNTHESIS NOTE
    //   This is a constant function used for parameter elaboration only.
    //==========================================================================
    function integer div_ceil_at_least_one;
        input integer numerator;
        input integer denominator;

        integer result;

        begin
            if ((numerator <= 0) || (denominator <= 0)) begin
                result = 1;
            end else begin
                result = (numerator + denominator - 1) / denominator;
            end

            if (result < 1)
                div_ceil_at_least_one = 1;
            else
                div_ceil_at_least_one = result;
        end
    endfunction

    //==========================================================================
    // FUNCTION: clog2_at_least_one
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Compute a Verilog-2001 replacement for SystemVerilog `$clog2`.
    //
    // INPUT CONTRACT
    //   value:
    //     Number of distinct count states that must be representable.
    //
    // OUTPUT CONTRACT
    //   Returns the minimum bit width W such that:
    //
    //       2^W >= value
    //
    //   Return value is always at least 1.
    //
    // STEP-BY-STEP
    //   1) If value <= 1, return 1.
    //   2) Subtract one from value.
    //      This makes exact powers of two produce the expected width.
    //   3) Repeatedly right-shift the temporary value.
    //   4) Count the shifts.
    //
    // WORKED EXAMPLES
    //   value = 1:
    //     width = 1
    //
    //   value = 2:
    //     tmp = 1
    //     shifts once
    //     width = 1
    //
    //   value = 3:
    //     tmp = 2
    //     shifts: 2 -> 1 -> 0
    //     width = 2
    //
    //   value = 84:
    //     width = 7 because 2^6 < 84 <= 2^7.
    //
    // SYNTHESIS NOTE
    //   This is a constant function. It controls vector width only.
    //==========================================================================
    function integer clog2_at_least_one;
        input integer value;

        integer tmp;

        begin
            if (value <= 1) begin
                clog2_at_least_one = 1;
            end else begin
                tmp = value - 1;
                clog2_at_least_one = 0;

                while (tmp > 0) begin
                    clog2_at_least_one = clog2_at_least_one + 1;
                    tmp = tmp >> 1;
                end
            end
        end
    endfunction

    //==========================================================================
    // FUNCTION: dev_write_byte
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Convert a 7-bit I2C device address into the 8-bit address byte used for
    //   a write transaction.
    //
    // INPUT CONTRACT
    //   dev_addr:
    //     7-bit I2C/SCCB address.
    //
    // OUTPUT CONTRACT
    //   Returns:
    //
    //       {dev_addr, 1'b0}
    //
    //   where the low bit is the I2C write bit.
    //
    // STEP-BY-STEP
    //   1) Place the 7-bit address into bits [7:1].
    //   2) Place 0 into bit [0] to indicate write.
    //
    // WORKED EXAMPLE
    //   OV5640 address:
    //     dev_addr       = 7'h3C
    //     dev_write_byte = 8'h78
    //
    // SYNTHESIS NOTE
    //   Pure concatenation; no logic depth of concern.
    //==========================================================================
    function [7:0] dev_write_byte;
        input [6:0] dev_addr;
        begin
            dev_write_byte = {dev_addr, 1'b0};
        end
    endfunction

    //==========================================================================
    // FUNCTION: dev_read_byte
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Convert a 7-bit I2C device address into the 8-bit address byte used for
    //   a read transaction.
    //
    // INPUT CONTRACT
    //   dev_addr:
    //     7-bit I2C/SCCB address.
    //
    // OUTPUT CONTRACT
    //   Returns:
    //
    //       {dev_addr, 1'b1}
    //
    //   where the low bit is the I2C read bit.
    //
    // STEP-BY-STEP
    //   1) Place the 7-bit address into bits [7:1].
    //   2) Place 1 into bit [0] to indicate read.
    //
    // WORKED EXAMPLE
    //   OV5640 address:
    //     dev_addr      = 7'h3C
    //     dev_read_byte = 8'h79
    //
    // SYNTHESIS NOTE
    //   Pure concatenation; no logic depth of concern.
    //==========================================================================
    function [7:0] dev_read_byte;
        input [6:0] dev_addr;
        begin
            dev_read_byte = {dev_addr, 1'b1};
        end
    endfunction

    //==========================================================================
    // Timing-divider constants
    //--------------------------------------------------------------------------
    // SCL_PHASES_PER_BIT:
    //   The data-bit sequencer uses three tick intervals per serialized bit:
    //
    //     1) setup while SCL is low
    //     2) SCL high
    //     3) SCL low / advance
    //
    // DIVIDER:
    //   Number of clk cycles per internal tick.
    //
    //   The divider uses ceiling division so the generated SCL bit rate does not
    //   exceed SCL_HZ due to integer truncation.
    //
    // DIVW:
    //   Counter width required to represent DIVIDER - 1.
    //==========================================================================
    localparam integer SCL_PHASES_PER_BIT = 3;

    localparam integer SCL_HZ_SAFE =
        (SCL_HZ <= 0) ? 1 : SCL_HZ;

    localparam integer DIVIDER =
        div_ceil_at_least_one(CLK_HZ, SCL_HZ_SAFE * SCL_PHASES_PER_BIT);

    localparam integer DIVW =
        clog2_at_least_one(DIVIDER);

    localparam [DIVW-1:0] DIVIDER_M1 = DIVIDER - 1;

    //==========================================================================
    // FSM state encoding
    //--------------------------------------------------------------------------
    // ST_IDLE:
    //   Bus released. Waiting for a command.
    //
    // ST_START_A / ST_START_B:
    //   Generate I2C START condition.
    //
    // ST_SEND_BIT_*:
    //   Serialize one bit from tx_byte.
    //
    // ST_RECV_BIT_*:
    //   Receive one bit into rx_byte.
    //
    // ST_ACK_*:
    //   Release SDA and sample target ACK after transmitting a byte.
    //
    // ST_MASTER_NACK_*:
    //   Send master NACK after one-byte read.
    //
    // ST_STOP_*:
    //   Generate I2C STOP condition.
    //
    // ST_NEXT_BYTE:
    //   Decide which byte/phase comes next.
    //
    // ST_RSTART_*:
    //   Generate repeated START for register read transactions.
    //
    // ST_DONE_PULSE:
    //   Successful transaction response pulse.
    //
    // ST_FAIL_PULSE:
    //   Failed transaction response pulse.
    //==========================================================================
    localparam [5:0]
        ST_IDLE              = 6'd0,
        ST_START_A           = 6'd1,
        ST_START_B           = 6'd2,

        ST_SEND_BIT_SETUP    = 6'd3,
        ST_SEND_BIT_SCL_H    = 6'd4,
        ST_SEND_BIT_SCL_L    = 6'd5,

        ST_RECV_BIT_SETUP    = 6'd6,
        ST_RECV_BIT_SCL_H    = 6'd7,
        ST_RECV_BIT_SCL_L    = 6'd8,

        ST_ACK_SETUP         = 6'd9,
        ST_ACK_SCL_H         = 6'd10,
        ST_ACK_SCL_L         = 6'd11,

        ST_MASTER_NACK_SET   = 6'd12,
        ST_MASTER_NACK_H     = 6'd13,
        ST_MASTER_NACK_L     = 6'd14,

        ST_STOP_A            = 6'd15,
        ST_STOP_B            = 6'd16,
        ST_STOP_C            = 6'd17,

        ST_NEXT_BYTE         = 6'd18,
        ST_RSTART_A          = 6'd19,
        ST_RSTART_B          = 6'd20,

        ST_DONE_PULSE        = 6'd21,
        ST_FAIL_PULSE        = 6'd22;

    //==========================================================================
    // Transaction phase encoding
    //--------------------------------------------------------------------------
    // PH_DEVW:
    //   Device address with write bit.
    //
    // PH_REGH:
    //   Register address high byte.
    //
    // PH_REGL:
    //   Register address low byte.
    //
    // PH_WRDATA:
    //   Write data byte.
    //
    // PH_DEVR:
    //   Device address with read bit after repeated START.
    //
    // PH_RDDATA:
    //   Single received data byte.
    //==========================================================================
    localparam [2:0]
        PH_DEVW    = 3'd0,
        PH_REGH    = 3'd1,
        PH_REGL    = 3'd2,
        PH_WRDATA  = 3'd3,
        PH_DEVR    = 3'd4,
        PH_RDDATA  = 3'd5;

    //==========================================================================
    // Internal state registers
    //==========================================================================
    reg [5:0]      state;
    reg [2:0]      phase;

    reg [DIVW-1:0] div_ctr;

    reg            scl_oe_n;
    reg            sda_oe_n;

    reg            lat_cmd_has_reg;
    reg            lat_cmd_is_read;
    reg [6:0]      lat_cmd_dev_addr;
    reg [15:0]     lat_cmd_reg_addr;
    reg [7:0]      lat_cmd_wr_data;

    reg [7:0]      tx_byte;
    reg [7:0]      rx_byte;
    reg [2:0]      bit_idx;
    reg            ack_seen;

    //==========================================================================
    // Tick generation
    //--------------------------------------------------------------------------
    // tick_q:
    //   One-cycle internal timing strobe. All I2C state transitions occur only
    //   on tick_q, except command acceptance and response pulses.
    //
    // DIVIDER behavior:
    //   If DIVIDER == 1, tick_q is true every clk cycle.
    //==========================================================================
    wire tick_q;
    assign tick_q = (div_ctr == DIVIDER_M1);

    //==========================================================================
    // Open-drain pin driving
    //--------------------------------------------------------------------------
    // scl_oe_n / sda_oe_n meaning:
    //
    //   0 -> drive the bus line low
    //   1 -> release the bus line to high-Z
    //
    // External pull-ups, internal pull-ups, or board-level pull-ups are expected
    // to create the logic-high level when released.
    //==========================================================================
    assign scl_io = scl_oe_n ? 1'bz : 1'b0;
    assign sda_io = sda_oe_n ? 1'bz : 1'b0;

    wire sda_in;
    assign sda_in = sda_io;

    //==========================================================================
    // SEQUENTIAL BLOCK: I2C/SCCB transaction engine
    //--------------------------------------------------------------------------
    // STATE OWNER
    //   This always block is the sole owner of every register in this module.
    //
    // RESET BEHAVIOR
    //   Reset releases both bus lines, clears all response fields, asserts
    //   cmd_ready, and returns the engine to ST_IDLE.
    //
    // COMMAND ACCEPTANCE
    //   In ST_IDLE, a command is captured when cmd_valid and cmd_ready are both
    //   high. All command fields are latched so the producer may change them
    //   after the handshake.
    //
    // BYTE-SEND ALGORITHM
    //   For every transmitted byte:
    //     1) Drive SCL low.
    //     2) Drive or release SDA according to the current bit.
    //     3) Release SCL high.
    //     4) Bring SCL low again.
    //     5) Advance to the next bit.
    //     6) After bit 0, release SDA and sample ACK.
    //
    // BYTE-RECEIVE ALGORITHM
    //   For a single received byte:
    //     1) Drive SCL low.
    //     2) Release SDA.
    //     3) Release SCL high.
    //     4) Sample SDA into rx_byte[bit_idx].
    //     5) Bring SCL low.
    //     6) Advance to the next bit.
    //     7) After bit 0, publish rx_byte into rsp_rd_data.
    //     8) Send master NACK because only one-byte reads are supported.
    //
    // RESPONSE POLICY
    //   ST_DONE_PULSE and ST_FAIL_PULSE both assert rsp_valid for one clk.
    //   The producer determines success from rsp_err/rsp_ack_error.
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            div_ctr         <= {DIVW{1'b0}};

            state           <= ST_IDLE;
            phase           <= PH_DEVW;

            cmd_ready       <= 1'b1;
            rsp_valid       <= 1'b0;
            rsp_err         <= 1'b0;
            rsp_ack_error   <= 1'b0;
            rsp_rd_data     <= 8'h00;
            busy            <= 1'b0;

            scl_oe_n        <= 1'b1;
            sda_oe_n        <= 1'b1;

            lat_cmd_has_reg  <= 1'b0;
            lat_cmd_is_read  <= 1'b0;
            lat_cmd_dev_addr <= 7'd0;
            lat_cmd_reg_addr <= 16'd0;
            lat_cmd_wr_data  <= 8'd0;

            tx_byte         <= 8'd0;
            rx_byte         <= 8'd0;
            bit_idx         <= 3'd7;
            ack_seen        <= 1'b0;
        end else begin
            //------------------------------------------------------------------
            // Default one-cycle response behavior.
            //------------------------------------------------------------------
            rsp_valid <= 1'b0;

            //------------------------------------------------------------------
            // Internal tick divider.
            //------------------------------------------------------------------
            if (tick_q)
                div_ctr <= {DIVW{1'b0}};
            else
                div_ctr <= div_ctr + 1'b1;

            case (state)

                //--------------------------------------------------------------
                // ST_IDLE: bus released, ready for a new command.
                //--------------------------------------------------------------
                ST_IDLE: begin
                    cmd_ready <= 1'b1;
                    busy      <= 1'b0;
                    scl_oe_n  <= 1'b1;
                    sda_oe_n  <= 1'b1;

                    if (cmd_valid && cmd_ready) begin
                        cmd_ready        <= 1'b0;
                        busy             <= 1'b1;

                        rsp_err          <= 1'b0;
                        rsp_ack_error    <= 1'b0;
                        rsp_rd_data      <= 8'h00;

                        lat_cmd_has_reg  <= cmd_has_reg;
                        lat_cmd_is_read  <= cmd_is_read;
                        lat_cmd_dev_addr <= cmd_dev_addr;
                        lat_cmd_reg_addr <= cmd_reg_addr;
                        lat_cmd_wr_data  <= cmd_wr_data;

                        phase            <= PH_DEVW;
                        tx_byte          <= dev_write_byte(cmd_dev_addr);
                        bit_idx          <= 3'd7;

                        state            <= ST_START_A;
                    end
                end

                //--------------------------------------------------------------
                // START generation, phase A.
                //
                // Both SCL and SDA are released high before the START edge.
                //--------------------------------------------------------------
                ST_START_A: begin
                    if (tick_q) begin
                        scl_oe_n <= 1'b1;
                        sda_oe_n <= 1'b1;
                        state    <= ST_START_B;
                    end
                end

                //--------------------------------------------------------------
                // START generation, phase B.
                //
                // I2C START condition:
                //   SDA transitions high -> low while SCL is high.
                //--------------------------------------------------------------
                ST_START_B: begin
                    if (tick_q) begin
                        scl_oe_n <= 1'b1;
                        sda_oe_n <= 1'b0;
                        tx_byte  <= dev_write_byte(lat_cmd_dev_addr);
                        bit_idx  <= 3'd7;
                        state    <= ST_SEND_BIT_SETUP;
                    end
                end

                //--------------------------------------------------------------
                // Transmit-bit setup while SCL is low.
                //--------------------------------------------------------------
                ST_SEND_BIT_SETUP: begin
                    if (tick_q) begin
                        scl_oe_n <= 1'b0;
                        sda_oe_n <= tx_byte[bit_idx] ? 1'b1 : 1'b0;
                        state    <= ST_SEND_BIT_SCL_H;
                    end
                end

                //--------------------------------------------------------------
                // Transmit-bit high phase.
                //--------------------------------------------------------------
                ST_SEND_BIT_SCL_H: begin
                    if (tick_q) begin
                        scl_oe_n <= 1'b1;
                        state    <= ST_SEND_BIT_SCL_L;
                    end
                end

                //--------------------------------------------------------------
                // Transmit-bit low/advance phase.
                //--------------------------------------------------------------
                ST_SEND_BIT_SCL_L: begin
                    if (tick_q) begin
                        scl_oe_n <= 1'b0;

                        if (bit_idx == 3'd0) begin
                            state <= ST_ACK_SETUP;
                        end else begin
                            bit_idx <= bit_idx - 3'd1;
                            state   <= ST_SEND_BIT_SETUP;
                        end
                    end
                end

                //--------------------------------------------------------------
                // ACK setup.
                //
                // The target owns SDA during the ACK bit, so the master releases
                // SDA before raising SCL.
                //--------------------------------------------------------------
                ST_ACK_SETUP: begin
                    if (tick_q) begin
                        sda_oe_n <= 1'b1;
                        state    <= ST_ACK_SCL_H;
                    end
                end

                //--------------------------------------------------------------
                // ACK sample.
                //
                // ACK is active-low:
                //   sda_in == 0 -> ACK
                //   sda_in == 1 -> NACK
                //--------------------------------------------------------------
                ST_ACK_SCL_H: begin
                    if (tick_q) begin
                        scl_oe_n <= 1'b1;
                        ack_seen <= ~sda_in;
                        state    <= ST_ACK_SCL_L;
                    end
                end

                //--------------------------------------------------------------
                // ACK low/decision phase.
                //--------------------------------------------------------------
                ST_ACK_SCL_L: begin
                    if (tick_q) begin
                        scl_oe_n <= 1'b0;

                        if (!ack_seen) begin
                            rsp_err       <= 1'b1;
                            rsp_ack_error <= 1'b1;
                            state         <= ST_STOP_A;
                        end else begin
                            state <= ST_NEXT_BYTE;
                        end
                    end
                end

                //--------------------------------------------------------------
                // Decide next transaction phase.
                //--------------------------------------------------------------
                ST_NEXT_BYTE: begin
                    if (phase == PH_DEVW) begin
                        if (lat_cmd_has_reg) begin
                            tx_byte <= lat_cmd_reg_addr[15:8];
                            bit_idx <= 3'd7;
                            phase   <= PH_REGH;
                            state   <= ST_SEND_BIT_SETUP;
                        end else if (lat_cmd_is_read) begin
                            //--------------------------------------------------
                            // Unsupported command: read without register phase.
                            //--------------------------------------------------
                            rsp_err <= 1'b1;
                            state   <= ST_STOP_A;
                        end else begin
                            tx_byte <= lat_cmd_wr_data;
                            bit_idx <= 3'd7;
                            phase   <= PH_WRDATA;
                            state   <= ST_SEND_BIT_SETUP;
                        end

                    end else if (phase == PH_REGH) begin
                        tx_byte <= lat_cmd_reg_addr[7:0];
                        bit_idx <= 3'd7;
                        phase   <= PH_REGL;
                        state   <= ST_SEND_BIT_SETUP;

                    end else if (phase == PH_REGL) begin
                        if (lat_cmd_is_read) begin
                            phase <= PH_DEVR;
                            state <= ST_RSTART_A;
                        end else begin
                            tx_byte <= lat_cmd_wr_data;
                            bit_idx <= 3'd7;
                            phase   <= PH_WRDATA;
                            state   <= ST_SEND_BIT_SETUP;
                        end

                    end else if (phase == PH_WRDATA) begin
                        state <= ST_STOP_A;

                    end else if (phase == PH_DEVR) begin
                        rx_byte <= 8'h00;
                        bit_idx <= 3'd7;
                        phase   <= PH_RDDATA;
                        state   <= ST_RECV_BIT_SETUP;

                    end else if (phase == PH_RDDATA) begin
                        state <= ST_MASTER_NACK_SET;

                    end else begin
                        rsp_err <= 1'b1;
                        state   <= ST_STOP_A;
                    end
                end

                //--------------------------------------------------------------
                // Repeated START phase A.
                //
                // Prepare bus high before issuing the repeated START edge.
                //--------------------------------------------------------------
                ST_RSTART_A: begin
                    if (tick_q) begin
                        sda_oe_n <= 1'b1;
                        scl_oe_n <= 1'b1;
                        state    <= ST_RSTART_B;
                    end
                end

                //--------------------------------------------------------------
                // Repeated START phase B.
                //
                // SDA falls while SCL is high, then the read address byte is
                // prepared for transmission.
                //--------------------------------------------------------------
                ST_RSTART_B: begin
                    if (tick_q) begin
                        sda_oe_n <= 1'b0;
                        tx_byte  <= dev_read_byte(lat_cmd_dev_addr);
                        bit_idx  <= 3'd7;
                        state    <= ST_SEND_BIT_SETUP;
                    end
                end

                //--------------------------------------------------------------
                // Receive-bit setup.
                //
                // Release SDA because the target drives read data.
                //--------------------------------------------------------------
                ST_RECV_BIT_SETUP: begin
                    if (tick_q) begin
                        scl_oe_n <= 1'b0;
                        sda_oe_n <= 1'b1;
                        state    <= ST_RECV_BIT_SCL_H;
                    end
                end

                //--------------------------------------------------------------
                // Receive-bit high/sample phase.
                //--------------------------------------------------------------
                ST_RECV_BIT_SCL_H: begin
                    if (tick_q) begin
                        scl_oe_n         <= 1'b1;
                        rx_byte[bit_idx] <= sda_in;
                        state            <= ST_RECV_BIT_SCL_L;
                    end
                end

                //--------------------------------------------------------------
                // Receive-bit low/advance phase.
                //--------------------------------------------------------------
                ST_RECV_BIT_SCL_L: begin
                    if (tick_q) begin
                        scl_oe_n <= 1'b0;

                        if (bit_idx == 3'd0) begin
                            rsp_rd_data <= rx_byte;
                            state       <= ST_NEXT_BYTE;
                        end else begin
                            bit_idx <= bit_idx - 3'd1;
                            state   <= ST_RECV_BIT_SETUP;
                        end
                    end
                end

                //--------------------------------------------------------------
                // Master NACK setup after one-byte read.
                //
                // A NACK is represented by releasing SDA during the ninth clock.
                //--------------------------------------------------------------
                ST_MASTER_NACK_SET: begin
                    if (tick_q) begin
                        scl_oe_n <= 1'b0;
                        sda_oe_n <= 1'b1;
                        state    <= ST_MASTER_NACK_H;
                    end
                end

                //--------------------------------------------------------------
                // Master NACK high phase.
                //--------------------------------------------------------------
                ST_MASTER_NACK_H: begin
                    if (tick_q) begin
                        scl_oe_n <= 1'b1;
                        state    <= ST_MASTER_NACK_L;
                    end
                end

                //--------------------------------------------------------------
                // Master NACK low phase.
                //
                // SDA is pulled low after the NACK clock so STOP can be formed
                // as low-to-high SDA while SCL is high.
                //--------------------------------------------------------------
                ST_MASTER_NACK_L: begin
                    if (tick_q) begin
                        scl_oe_n <= 1'b0;
                        sda_oe_n <= 1'b0;
                        state    <= ST_STOP_A;
                    end
                end

                //--------------------------------------------------------------
                // STOP phase A.
                //
                // Ensure SDA is low while SCL is low.
                //--------------------------------------------------------------
                ST_STOP_A: begin
                    if (tick_q) begin
                        scl_oe_n <= 1'b0;
                        sda_oe_n <= 1'b0;
                        state    <= ST_STOP_B;
                    end
                end

                //--------------------------------------------------------------
                // STOP phase B.
                //
                // Release SCL high while keeping SDA low.
                //--------------------------------------------------------------
                ST_STOP_B: begin
                    if (tick_q) begin
                        scl_oe_n <= 1'b1;
                        sda_oe_n <= 1'b0;
                        state    <= ST_STOP_C;
                    end
                end

                //--------------------------------------------------------------
                // STOP phase C.
                //
                // I2C STOP condition:
                //   SDA transitions low -> high while SCL is high.
                //--------------------------------------------------------------
                ST_STOP_C: begin
                    if (tick_q) begin
                        sda_oe_n <= 1'b1;

                        if (rsp_err)
                            state <= ST_FAIL_PULSE;
                        else
                            state <= ST_DONE_PULSE;
                    end
                end

                //--------------------------------------------------------------
                // Successful response pulse.
                //--------------------------------------------------------------
                ST_DONE_PULSE: begin
                    rsp_valid <= 1'b1;
                    busy      <= 1'b0;
                    cmd_ready <= 1'b1;
                    state     <= ST_IDLE;
                end

                //--------------------------------------------------------------
                // Failed response pulse.
                //--------------------------------------------------------------
                ST_FAIL_PULSE: begin
                    rsp_valid <= 1'b1;
                    busy      <= 1'b0;
                    cmd_ready <= 1'b1;
                    state     <= ST_IDLE;
                end

                //--------------------------------------------------------------
                // Illegal-state recovery.
                //--------------------------------------------------------------
                default: begin
                    scl_oe_n      <= 1'b1;
                    sda_oe_n      <= 1'b1;
                    busy          <= 1'b0;
                    cmd_ready     <= 1'b1;
                    rsp_err       <= 1'b1;
                    rsp_ack_error <= 1'b0;
                    rsp_valid     <= 1'b1;
                    state         <= ST_IDLE;
                end
            endcase
        end
    end

endmodule

`default_nettype wire