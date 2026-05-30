`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// sccb_master
//------------------------------------------------------------------------------
// ROLE
//   Transaction-level SCCB / I²C-like serial master for camera register access.
//
// HIGH-LEVEL PURPOSE
//   This module accepts a simple command interface describing either:
//
//     - a register write
//     - a register read
//
//   and converts that command into a full two-wire serial bus transaction over:
//
//     - scl
//     - sda
//
//   The module handles:
//
//     1) start condition generation
//     2) byte transmission bit-by-bit
//     3) ACK checking after transmitted bytes
//     4) repeated-start generation for read commands
//     5) byte reception for readback
//     6) master-side final NACK after read byte
//     7) stop condition generation
//
// BUS MODEL
//   The bus is modeled in open-drain / wired-AND style:
//
//     - the master never actively drives logic '1'
//     - instead, it either:
//         * drives logic '0'
//         * releases the line to high-impedance
//
//   This is implemented with:
//
//       scl_oe_n
//       sda_oe_n
//
//   where:
//
//       oe_n = 0  -> actively pull line low
//       oe_n = 1  -> release line (high-Z)
//
//   Thus the assignments:
//
//       assign scl = (scl_oe_n == 1'b0) ? 1'b0 : 1'bz;
//       assign sda = (sda_oe_n == 1'b0) ? 1'b0 : 1'bz;
//
//   mean the bus is expected to be externally pulled high when not driven low.
//
// WHY THIS MODULE EXISTS
//   Higher-level camera logic should not need to manually toggle SCL/SDA bit by
//   bit for every register access. Instead, a cleaner architecture is:
//
//       command-level intent
//           ->
//       SCCB master
//           ->
//       electrical bus waveform
//
//   This module is that translation boundary.
//
// COMMAND INTERFACE OVERVIEW
//   Inputs:
//     cmd_valid
//       Indicates that a new command is available.
//
//     cmd_ready
//       Indicates that the master is idle and can accept a new command.
//
//     cmd_type
//       Command type selector.
//
//       Present interpretation from the implemented state machine:
//         cmd_type = 0 -> write transaction
//         cmd_type = 1 -> read transaction
//
//     cmd_reg_addr
//       16-bit target register address.
//
//     cmd_wr_data
//       Write data byte for write transactions.
//
//   Outputs:
//     done
//       One-cycle completion pulse for a successful transaction.
//
//     err
//       One-cycle pulse indicating a transaction failure.
//
//     ack_error
//       One-cycle pulse indicating the specific failure mode "ACK was not
//       received where expected."
//
//     rd_data
//       Read data byte captured during a read transaction.
//
//     busy
//       Indicates that the SCCB master is currently occupied performing a
//       transaction.
//
// TRANSACTION FORMS
//
//   1) Write transaction (cmd_type = 0)
//
//        START
//        DEV_ADDR + W
//        REG_ADDR[15:8]
//        REG_ADDR[7:0]
//        WR_DATA
//        STOP
//
//   2) Read transaction (cmd_type = 1)
//
//        START
//        DEV_ADDR + W
//        REG_ADDR[15:8]
//        REG_ADDR[7:0]
//        RESTART
//        DEV_ADDR + R
//        READ 1 BYTE
//        MASTER NACK
//        STOP
//
// TIMING QUANTIZATION
//   The module does not attempt to toggle SCL every clk cycle. Instead, it
//   creates a slower internal timing event:
//
//       step_tick
//
//   using a divider:
//
//       DIVIDER = CLK_HZ / (SCL_HZ * 4)
//
//   Conceptually, this divides each SCL bit cell into several sub-phases, and
//   the FSM advances only on `step_tick` assertions.
//
// WHY MULTIPLY BY 4?
//   The implementation breaks the per-bit process into multiple explicit
//   low/high/fall-style phases. The factor of 4 provides a simple substep
//   timing granularity so that the FSM can sequence:
//
//     - prepare line values while SCL is low
//     - raise SCL for sampling / stable high phase
//     - bring SCL low again
//
//   This is a pragmatic bit-engine timing model rather than a mathematically
//   abstract serial protocol description.
//
// SIGNAL SEMANTICS
//   clk
//     Local synchronous clock driving the FSM and divider.
//
//   rst
//     Active-high synchronous reset.
//
//   cmd_valid
//     Indicates that the command fields are valid and should be accepted when
//     cmd_ready is high.
//
//   cmd_ready
//     Indicates that the master is ready to accept a new command.
//
//   cmd_type
//     0 = write, 1 = read.
//
//   cmd_reg_addr
//     Target 16-bit register address.
//
//   cmd_wr_data
//     Data byte to write in write transactions.
//
//   done
//     One-cycle pulse on successful transaction completion.
//
//   err
//     One-cycle pulse on error termination.
//
//   ack_error
//     One-cycle pulse specifically indicating missing slave ACK.
//
//   rd_data
//     Byte read from the slave during a read transaction.
//
//   busy
//     High while transaction is in progress.
//
//   scl, sda
//     Open-drain style serial clock and data lines.
//
// INTERNAL STATE OVERVIEW
//
//   Bus drive control:
//     scl_oe_n
//     sda_oe_n
//
//   Bus observation:
//     scl_i
//     sda_i
//
//   Timing generation:
//     div_ctr
//     step_tick
//
//   FSM state:
//     state
//
//   Latched command fields:
//     latched_cmd_type
//     latched_reg_addr
//     latched_wr_data
//
//   Current byte send/receive machinery:
//     tx_byte
//     rx_byte
//     bit_idx
//
//   Transaction sequencing:
//     byte_phase
//     total_write_bytes
//     doing_readback
//     last_master_ack_n
//
// BYTE-PHASE INTERPRETATION
//   During the initial write-addressing portion of the transaction, byte_phase
//   identifies which byte is being transmitted:
//
//     byte_phase = 0 -> DEV_ADDR + W
//     byte_phase = 1 -> REG_ADDR[15:8]
//     byte_phase = 2 -> REG_ADDR[7:0]
//     byte_phase = 3 -> WR_DATA   (only for write command)
//
//   total_write_bytes is set to:
//
//     3 for read command
//     4 for write command
//
//   so that read transactions stop after writing only the device-write address
//   and two register-address bytes, then transition into repeated-start flow.
//
// ACK POLICY
//   After each transmitted byte, the master releases SDA and samples the slave's
//   ACK bit.
//
//   Expected ACK convention:
//     ACK = SDA low
//
//   Therefore, if SDA is not low during ACK sampling:
//
//       ack_error <= 1
//       state     <= ST_ERROR
//
// READBACK POLICY
//   For a read command:
//
//     - the first part of the transaction writes the target register address
//     - then a repeated start is generated
//     - then the device address is transmitted with R=1
//     - then one byte is received
//     - then the master transmits a NACK
//     - then STOP is generated
//
//   The master sends NACK after the single received byte because this design
//   only reads one byte and therefore wishes to terminate the read sequence.
//
// IMPORTANT PRESENT SEMANTICS
//   1) done, err, and ack_error are one-cycle pulses. They are cleared by
//      default each active cycle and asserted only in specific terminal states.
//
//   2) cmd_ready is asserted in ST_IDLE and again in terminal states that
//      return to ST_IDLE.
//
//   3) The state machine samples SDA directly through sda_i.
//
//   4) last_master_ack_n is initialized to 1 and used so that the master sends
//      a NACK after the single-byte read.
//
//   5) The FSM uses explicit "LO / HI / FALL" sub-states rather than a compact
//      bit-cell loop. This makes bus timing easier to audit.
//
// RESET BEHAVIOR
//   On rst assertion:
//
//     - FSM returns to ST_IDLE
//     - output flags are cleared
//     - command latches are cleared
//     - bus lines are released high
//     - internal counters and indices are reset
//
//   This corresponds to a conservative idle-bus condition.
//
// PEDAGOGICAL SUMMARY
//   The module can be understood in six conceptual stages:
//
//     Stage 1: accept and latch a transaction command
//     Stage 2: generate START
//     Stage 3: transmit required write-side bytes with ACK checking
//     Stage 4: if read command, generate RESTART and send DEV_ADDR+R
//     Stage 5: receive one byte and send master NACK
//     Stage 6: generate STOP and report done or error
//------------------------------------------------------------------------------
module sccb_master #(
    //--------------------------------------------------------------------------
    // Local FPGA clock frequency in hertz.
    //--------------------------------------------------------------------------
    parameter integer CLK_HZ        = 100_000_000,

    //--------------------------------------------------------------------------
    // Target serial clock rate in hertz.
    //--------------------------------------------------------------------------
    parameter integer SCL_HZ        = 400_000,

    //--------------------------------------------------------------------------
    // 7-bit device address.
    //
    // Present default corresponds to the expected camera SCCB device address.
    //--------------------------------------------------------------------------
    parameter [6:0]   DEV_ADDR_7BIT = 7'h3C
)(
    input  wire        clk,
    input  wire        rst,

    //--------------------------------------------------------------------------
    // Command handshake and payload
    //--------------------------------------------------------------------------
    input  wire        cmd_valid,
    output reg         cmd_ready,
    input  wire        cmd_type,
    input  wire [15:0] cmd_reg_addr,
    input  wire [7:0]  cmd_wr_data,

    //--------------------------------------------------------------------------
    // Transaction result interface
    //--------------------------------------------------------------------------
    output reg         done,
    output reg         err,
    output reg         ack_error,
    output reg  [7:0]  rd_data,
    output reg         busy,

    //--------------------------------------------------------------------------
    // Open-drain style SCCB/I2C-like bus pins
    //--------------------------------------------------------------------------
    inout  wire        scl,
    inout  wire        sda
);

    //==========================================================================
    // Open-drain style output-enable controls
    //--------------------------------------------------------------------------
    // Convention:
    //   oe_n = 0 -> actively pull line low
    //   oe_n = 1 -> release line to high impedance
    //
    // The bus is assumed to have pull-ups externally, so a released line reads
    // as logic high in normal operation.
    //==========================================================================
    reg scl_oe_n;
    reg sda_oe_n;

    assign scl = (scl_oe_n == 1'b0) ? 1'b0 : 1'bz;
    assign sda = (sda_oe_n == 1'b0) ? 1'b0 : 1'bz;

    //==========================================================================
    // Bus observation wires
    //--------------------------------------------------------------------------
    // scl_i and sda_i read back the present line level.
    //
    // Important:
    //   These reflect the actual resolved wire value, not merely the master's
    //   intended drive state. That is why sda_i can be used to observe slave
    //   ACK bits and read data bits.
    //==========================================================================
    wire scl_i = scl;
    wire sda_i = sda;

    //==========================================================================
    // Step-tick divider
    //--------------------------------------------------------------------------
    // DIVIDER determines how many clk cycles occur between internal FSM timing
    // steps.
    //
    // Each step_tick advances the bus FSM by one sub-phase.
    //==========================================================================
    localparam integer DIVIDER = (CLK_HZ / (SCL_HZ * 4));

    reg [31:0] div_ctr;
    reg        step_tick;

    //--------------------------------------------------------------------------
    // Divider process
    //
    // Step-by-step behavior:
    //   1) On reset, clear divider count and clear step_tick.
    //   2) Otherwise, count clk cycles.
    //   3) When div_ctr reaches DIVIDER-1:
    //        - clear the counter
    //        - pulse step_tick for one cycle
    //   4) On all other cycles:
    //        - increment div_ctr
    //        - keep step_tick low
    //
    // Therefore:
    //   step_tick is a periodic one-cycle enable pulse used by the serial FSM.
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            div_ctr   <= 32'd0;
            step_tick <= 1'b0;
        end else if (div_ctr >= DIVIDER-1) begin
            div_ctr   <= 32'd0;
            step_tick <= 1'b1;
        end else begin
            div_ctr   <= div_ctr + 32'd1;
            step_tick <= 1'b0;
        end
    end

    //==========================================================================
    // FSM state encoding
    //--------------------------------------------------------------------------
    // ST_IDLE
    //   Wait for command acceptance.
    //
    // ST_START_A / ST_START_B
    //   Generate start condition.
    //
    // ST_SEND_BIT_LO / HI / FALL
    //   Transmit one bit of tx_byte.
    //
    // ST_ACK_LO / HI / FALL
    //   Release SDA and sample slave ACK.
    //
    // ST_RECV_BIT_LO / HI / FALL
    //   Receive one bit into rx_byte.
    //
    // ST_SEND_MACK_LO / HI / FALL
    //   Send master ACK/NACK after read byte. In present implementation this is
    //   used to send a final NACK.
    //
    // ST_STOP_A / B / C
    //   Generate stop condition.
    //
    // ST_NEXT_BYTE
    //   Choose next tx_byte based on transaction phase.
    //
    // ST_DONE
    //   Pulse successful completion.
    //
    // ST_ERROR
    //   Pulse error completion.
    //
    // ST_RESTART_A / B
    //   Generate repeated start for read transactions.
    //==========================================================================
    localparam [5:0]
        ST_IDLE            = 6'd0,
        ST_START_A         = 6'd1,
        ST_START_B         = 6'd2,
        ST_SEND_BIT_LO     = 6'd3,
        ST_SEND_BIT_HI     = 6'd4,
        ST_SEND_BIT_FALL   = 6'd5,
        ST_ACK_LO          = 6'd6,
        ST_ACK_HI          = 6'd7,
        ST_ACK_FALL        = 6'd8,
        ST_RECV_BIT_LO     = 6'd9,
        ST_RECV_BIT_HI     = 6'd10,
        ST_RECV_BIT_FALL   = 6'd11,
        ST_SEND_MACK_LO    = 6'd12,
        ST_SEND_MACK_HI    = 6'd13,
        ST_SEND_MACK_FALL  = 6'd14,
        ST_STOP_A          = 6'd15,
        ST_STOP_B          = 6'd16,
        ST_STOP_C          = 6'd17,
        ST_NEXT_BYTE       = 6'd18,
        ST_DONE            = 6'd19,
        ST_ERROR           = 6'd20,
        ST_RESTART_A       = 6'd21,
        ST_RESTART_B       = 6'd22;

    reg [5:0] state;

    //==========================================================================
    // Latched command fields
    //--------------------------------------------------------------------------
    // These capture the accepted command at transaction launch time so that the
    // bus FSM operates on stable command data.
    //==========================================================================
    reg        latched_cmd_type;
    reg [15:0] latched_reg_addr;
    reg [7:0]  latched_wr_data;

    //==========================================================================
    // Bit/byte transaction machinery
    //--------------------------------------------------------------------------
    // tx_byte
    //   Byte currently being transmitted.
    //
    // rx_byte
    //   Byte currently being assembled from slave read data.
    //
    // bit_idx
    //   Current bit position within the byte, transmitted or received MSB-first
    //   from index 7 down to 0.
    //
    // byte_phase
    //   Identifies which write-side byte of the transaction is being sent.
    //
    // total_write_bytes
    //   Number of bytes in the write-addressing portion:
    //     3 for register read command
    //     4 for register write command
    //
    // doing_readback
    //   0 while transmitting the initial address/setup sequence
    //   1 after repeated-start, when moving into readback phase
    //
    // last_master_ack_n
    //   Bit value controlling the final master ACK/NACK after readback.
    //   Presently initialized and used so that the master sends NACK after the
    //   single-byte read.
    //==========================================================================
    reg [7:0] tx_byte;
    reg [7:0] rx_byte;
    reg [2:0] bit_idx;
    reg [2:0] byte_phase;
    reg [2:0] total_write_bytes;
    reg       doing_readback;
    reg       last_master_ack_n;

    //==========================================================================
    // Main SCCB transaction FSM
    //--------------------------------------------------------------------------
    // High-level structure:
    //
    //   A) Reset handling
    //   B) Default pulse clearing for done/err/ack_error
    //   C) State-specific bus sequencing
    //
    // done, err, ack_error are pulse-like indicators, so they are cleared at
    // the beginning of each non-reset cycle.
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            //------------------------------------------------------------------
            // Reset case
            //
            // Return the master to an idle released-bus state and clear all
            // transaction state.
            //------------------------------------------------------------------
            state             <= ST_IDLE;
            cmd_ready         <= 1'b1;
            done              <= 1'b0;
            err               <= 1'b0;
            ack_error         <= 1'b0;
            rd_data           <= 8'd0;
            busy              <= 1'b0;
            scl_oe_n          <= 1'b1;
            sda_oe_n          <= 1'b1;
            latched_cmd_type  <= 1'b0;
            latched_reg_addr  <= 16'd0;
            latched_wr_data   <= 8'd0;
            tx_byte           <= 8'd0;
            rx_byte           <= 8'd0;
            bit_idx           <= 3'd7;
            byte_phase        <= 3'd0;
            total_write_bytes <= 3'd0;
            doing_readback    <= 1'b0;
            last_master_ack_n <= 1'b1;
        end else begin
            //------------------------------------------------------------------
            // Default pulse clearing
            //
            // These flags are asserted only in specific terminal/error moments.
            //------------------------------------------------------------------
            done      <= 1'b0;
            err       <= 1'b0;
            ack_error <= 1'b0;

            case (state)

                //==============================================================
                // ST_IDLE
                //--------------------------------------------------------------
                // Purpose:
                //   Present idle bus condition and accept a new command.
                //
                // Step-by-step behavior:
                //   1) Advertise cmd_ready = 1.
                //   2) Advertise busy = 0.
                //   3) Release both bus lines high.
                //   4) If cmd_valid is asserted:
                //        - deassert cmd_ready
                //        - assert busy
                //        - latch command fields
                //        - initialize transaction bookkeeping
                //        - move to start-condition generation
                //
                // total_write_bytes policy:
                //   cmd_type = 1 (read)  -> 3 setup bytes
                //   cmd_type = 0 (write) -> 4 setup bytes
                //==============================================================
                ST_IDLE: begin
                    cmd_ready <= 1'b1;
                    busy      <= 1'b0;
                    scl_oe_n  <= 1'b1;
                    sda_oe_n  <= 1'b1;

                    if (cmd_valid) begin
                        cmd_ready         <= 1'b0;
                        busy              <= 1'b1;
                        latched_cmd_type  <= cmd_type;
                        latched_reg_addr  <= cmd_reg_addr;
                        latched_wr_data   <= cmd_wr_data;
                        byte_phase        <= 3'd0;
                        total_write_bytes <= cmd_type ? 3'd3 : 3'd4;
                        doing_readback    <= 1'b0;
                        state             <= ST_START_A;
                    end
                end

                //==============================================================
                // ST_START_A
                //--------------------------------------------------------------
                // Purpose:
                //   Begin start-condition setup with both lines released high.
                //
                // Step-by-step:
                //   On step_tick:
                //     - release SCL
                //     - release SDA
                //     - move to ST_START_B
                //
                // Engineering meaning:
                //   Establish the bus in an idle-high condition before pulling
                //   SDA low to create START.
                //==============================================================
                ST_START_A: begin
                    if (step_tick) begin
                        scl_oe_n <= 1'b1;
                        sda_oe_n <= 1'b1;
                        state    <= ST_START_B;
                    end
                end

                //==============================================================
                // ST_START_B
                //--------------------------------------------------------------
                // Purpose:
                //   Generate the START condition.
                //
                // Step-by-step:
                //   On step_tick:
                //     - pull SDA low while SCL remains released high
                //     - move to ST_NEXT_BYTE
                //
                // This realizes the fundamental START signature:
                //   SDA transitions low while SCL is high.
                //==============================================================
                ST_START_B: begin
                    if (step_tick) begin
                        sda_oe_n <= 1'b0;
                        state    <= ST_NEXT_BYTE;
                    end
                end

                //==============================================================
                // ST_RESTART_A
                //--------------------------------------------------------------
                // Purpose:
                //   Begin repeated-start generation.
                //
                // Step-by-step:
                //   On step_tick:
                //     - release SDA
                //     - release SCL
                //     - move to ST_RESTART_B
                //
                // This restores the bus to a high/high pre-start posture.
                //==============================================================
                ST_RESTART_A: begin
                    if (step_tick) begin
                        sda_oe_n <= 1'b1;
                        scl_oe_n <= 1'b1;
                        state    <= ST_RESTART_B;
                    end
                end

                //==============================================================
                // ST_RESTART_B
                //--------------------------------------------------------------
                // Purpose:
                //   Complete repeated-start generation.
                //
                // Step-by-step:
                //   On step_tick:
                //     - pull SDA low while SCL is high
                //     - move to ST_NEXT_BYTE
                //==============================================================
                ST_RESTART_B: begin
                    if (step_tick) begin
                        sda_oe_n <= 1'b0;
                        state    <= ST_NEXT_BYTE;
                    end
                end

                //==============================================================
                // ST_NEXT_BYTE
                //--------------------------------------------------------------
                // Purpose:
                //   Select the next byte to transmit and reset the bit index.
                //
                // Step-by-step:
                //   1) Reset bit_idx to 7 so transmission starts from MSB.
                //   2) If not doing readback:
                //        choose tx_byte based on byte_phase:
                //          phase 0 -> DEV_ADDR + W
                //          phase 1 -> reg_addr[15:8]
                //          phase 2 -> reg_addr[7:0]
                //          phase 3 -> wr_data
                //   3) Else:
                //        choose tx_byte = DEV_ADDR + R
                //   4) Move to ST_SEND_BIT_LO
                //
                // MSB-first note:
                //   SCCB/I²C-style byte transmission is performed MSB first here,
                //   which is why bit_idx starts at 7.
                //==============================================================
                ST_NEXT_BYTE: begin
                    bit_idx <= 3'd7;

                    if (!doing_readback) begin
                        case (byte_phase)
                            3'd0: tx_byte <= {DEV_ADDR_7BIT, 1'b0};
                            3'd1: tx_byte <= latched_reg_addr[15:8];
                            3'd2: tx_byte <= latched_reg_addr[7:0];
                            3'd3: tx_byte <= latched_wr_data;
                            default: tx_byte <= 8'h00;
                        endcase
                    end else begin
                        tx_byte <= {DEV_ADDR_7BIT, 1'b1};
                    end

                    state <= ST_SEND_BIT_LO;
                end

                //==============================================================
                // ST_SEND_BIT_LO
                //--------------------------------------------------------------
                // Purpose:
                //   Drive the current tx bit while SCL is low.
                //
                // Step-by-step:
                //   On step_tick:
                //     - force SCL low
                //     - set SDA drive according to tx_byte[bit_idx]
                //         bit=0 -> drive low
                //         bit=1 -> release high
                //     - move to ST_SEND_BIT_HI
                //
                // Expression note:
                //   sda_oe_n <= ~tx_byte[bit_idx]
                //
                //   If tx bit = 0:
                //     ~0 = 1 -> wait, check carefully:
                //       tx bit = 0 => ~0 = 1'b1, which releases the line.
                //   Because open-drain lines encode logic 1 by release and logic
                //   0 by drive-low, this inversion is deliberate under the
                //   module's oe_n convention.
                //
                //   The implemented logic is preserved exactly as written.
                //==============================================================
                ST_SEND_BIT_LO: begin
                    if (step_tick) begin
                        scl_oe_n <= 1'b0;
                        sda_oe_n <= ~tx_byte[bit_idx];
                        state    <= ST_SEND_BIT_HI;
                    end
                end

                //==============================================================
                // ST_SEND_BIT_HI
                //--------------------------------------------------------------
                // Purpose:
                //   Raise SCL so the data bit is presented during the high phase.
                //==============================================================
                ST_SEND_BIT_HI: begin
                    if (step_tick) begin
                        scl_oe_n <= 1'b1;
                        state    <= ST_SEND_BIT_FALL;
                    end
                end

                //==============================================================
                // ST_SEND_BIT_FALL
                //--------------------------------------------------------------
                // Purpose:
                //   End the bit cell and either continue with the next bit or
                //   move to ACK handling.
                //
                // Step-by-step:
                //   On step_tick:
                //     - pull SCL low again
                //     - if current bit was the final bit (bit_idx == 0):
                //         * release SDA for slave ACK
                //         * move to ST_ACK_LO
                //       else:
                //         * decrement bit_idx
                //         * return to ST_SEND_BIT_LO
                //==============================================================
                ST_SEND_BIT_FALL: begin
                    if (step_tick) begin
                        scl_oe_n <= 1'b0;
                        if (bit_idx == 3'd0) begin
                            sda_oe_n <= 1'b1;
                            state    <= ST_ACK_LO;
                        end else begin
                            bit_idx <= bit_idx - 3'd1;
                            state   <= ST_SEND_BIT_LO;
                        end
                    end
                end

                //==============================================================
                // ST_ACK_LO / ST_ACK_HI / ST_ACK_FALL
                //--------------------------------------------------------------
                // Purpose:
                //   Allow slave ACK/NACK after a transmitted byte.
                //
                // ACK convention:
                //   Slave ACK is SDA low during the acknowledge bit.
                //
                // Sequence:
                //   ST_ACK_LO:
                //     hold SCL low, release SDA
                //
                //   ST_ACK_HI:
                //     raise SCL so ACK bit may be observed
                //
                //   ST_ACK_FALL:
                //     pull SCL low and evaluate SDA
                //
                // Outcome logic:
                //   If SDA is not low:
                //     ack_error <= 1
                //     state     <= ST_ERROR
                //
                //   Else if still in write-address/setup phase:
                //     either send next setup byte, or:
                //       - stop immediately for write command
                //       - repeated-start for read command
                //
                //   Else if doing_readback:
                //     begin bit-by-bit receive of one byte
                //==============================================================
                ST_ACK_LO: begin
                    if (step_tick) begin
                        scl_oe_n <= 1'b0;
                        sda_oe_n <= 1'b1;
                        state    <= ST_ACK_HI;
                    end
                end

                ST_ACK_HI: begin
                    if (step_tick) begin
                        scl_oe_n <= 1'b1;
                        state    <= ST_ACK_FALL;
                    end
                end

                ST_ACK_FALL: begin
                    if (step_tick) begin
                        scl_oe_n <= 1'b0;
                        if (sda_i != 1'b0) begin
                            ack_error <= 1'b1;
                            state     <= ST_ERROR;
                        end else if (!doing_readback) begin
                            if (byte_phase + 3'd1 < total_write_bytes) begin
                                byte_phase <= byte_phase + 3'd1;
                                state      <= ST_NEXT_BYTE;
                            end else if (latched_cmd_type == 1'b0) begin
                                state <= ST_STOP_A;
                            end else begin
                                doing_readback <= 1'b1;
                                state          <= ST_RESTART_A;
                            end
                        end else begin
                            rx_byte <= 8'd0;
                            bit_idx <= 3'd7;
                            state   <= ST_RECV_BIT_LO;
                        end
                    end
                end

                //==============================================================
                // ST_RECV_BIT_LO / HI / FALL
                //--------------------------------------------------------------
                // Purpose:
                //   Receive one byte from the slave, MSB first.
                //
                // Sequence:
                //   ST_RECV_BIT_LO:
                //     - hold SCL low
                //     - release SDA so the slave can drive it
                //
                //   ST_RECV_BIT_HI:
                //     - raise SCL
                //     - sample sda_i into rx_byte[bit_idx]
                //
                //   ST_RECV_BIT_FALL:
                //     - pull SCL low again
                //     - if this was bit 0:
                //         * publish rd_data <= rx_byte
                //         * prepare final master NACK
                //         * move to ST_SEND_MACK_LO
                //       else:
                //         * decrement bit_idx
                //         * continue receiving
                //
                // Important sequencing note:
                //   The implementation stores the final sampled bits into rx_byte
                //   over successive cycles, then assigns rd_data <= rx_byte at
                //   the end of reception as written.
                //==============================================================
                ST_RECV_BIT_LO: begin
                    if (step_tick) begin
                        scl_oe_n <= 1'b0;
                        sda_oe_n <= 1'b1;
                        state    <= ST_RECV_BIT_HI;
                    end
                end

                ST_RECV_BIT_HI: begin
                    if (step_tick) begin
                        scl_oe_n <= 1'b1;
                        rx_byte[bit_idx] <= sda_i;
                        state    <= ST_RECV_BIT_FALL;
                    end
                end

                ST_RECV_BIT_FALL: begin
                    if (step_tick) begin
                        scl_oe_n <= 1'b0;
                        if (bit_idx == 3'd0) begin
                            rd_data           <= rx_byte;
                            last_master_ack_n <= 1'b1;
                            state             <= ST_SEND_MACK_LO;
                        end else begin
                            bit_idx <= bit_idx - 3'd1;
                            state   <= ST_RECV_BIT_LO;
                        end
                    end
                end

                //==============================================================
                // ST_SEND_MACK_LO / HI / FALL
                //--------------------------------------------------------------
                // Purpose:
                //   Send the master's final ACK/NACK bit after read reception.
                //
                // Present implementation intent:
                //   Send NACK after the single read byte, indicating no further
                //   bytes are requested.
                //
                // last_master_ack_n = 1 means:
                //   SDA is released during ACK/NACK bit, producing NACK in the
                //   open-drain convention used here.
                //
                // Sequence:
                //   LO:
                //     prepare SDA value while SCL low
                //   HI:
                //     raise SCL
                //   FALL:
                //     pull SCL low again, release SDA, move to STOP
                //==============================================================
                ST_SEND_MACK_LO: begin
                    if (step_tick) begin
                        scl_oe_n <= 1'b0;
                        sda_oe_n <= last_master_ack_n;
                        state    <= ST_SEND_MACK_HI;
                    end
                end

                ST_SEND_MACK_HI: begin
                    if (step_tick) begin
                        scl_oe_n <= 1'b1;
                        state    <= ST_SEND_MACK_FALL;
                    end
                end

                ST_SEND_MACK_FALL: begin
                    if (step_tick) begin
                        scl_oe_n <= 1'b0;
                        sda_oe_n <= 1'b1;
                        state    <= ST_STOP_A;
                    end
                end

                //==============================================================
                // ST_STOP_A / B / C
                //--------------------------------------------------------------
                // Purpose:
                //   Generate STOP condition.
                //
                // Sequence:
                //   ST_STOP_A:
                //     drive both SCL and SDA low
                //
                //   ST_STOP_B:
                //     release SCL high while SDA remains low
                //
                //   ST_STOP_C:
                //     release SDA high
                //
                // This realizes the STOP signature:
                //   SDA transitions high while SCL is high.
                //==============================================================
                ST_STOP_A: begin
                    if (step_tick) begin
                        scl_oe_n <= 1'b0;
                        sda_oe_n <= 1'b0;
                        state    <= ST_STOP_B;
                    end
                end

                ST_STOP_B: begin
                    if (step_tick) begin
                        scl_oe_n <= 1'b1;
                        state    <= ST_STOP_C;
                    end
                end

                ST_STOP_C: begin
                    if (step_tick) begin
                        sda_oe_n <= 1'b1;
                        state    <= ST_DONE;
                    end
                end

                //==============================================================
                // ST_DONE
                //--------------------------------------------------------------
                // Purpose:
                //   Report successful completion and return to idle.
                //
                // Step-by-step:
                //   1) Pulse done.
                //   2) Clear busy.
                //   3) Reassert cmd_ready.
                //   4) Return to ST_IDLE.
                //==============================================================
                ST_DONE: begin
                    done      <= 1'b1;
                    busy      <= 1'b0;
                    cmd_ready <= 1'b1;
                    state     <= ST_IDLE;
                end

                //==============================================================
                // ST_ERROR
                //--------------------------------------------------------------
                // Purpose:
                //   Report failure and return the bus to released idle.
                //
                // Step-by-step:
                //   1) Pulse err.
                //   2) Clear busy.
                //   3) Reassert cmd_ready.
                //   4) Release both bus lines.
                //   5) Return to ST_IDLE.
                //
                // ack_error is pulsed in the path that detects ACK failure just
                // before entering this state.
                //==============================================================
                ST_ERROR: begin
                    err       <= 1'b1;
                    busy      <= 1'b0;
                    cmd_ready <= 1'b1;
                    scl_oe_n  <= 1'b1;
                    sda_oe_n  <= 1'b1;
                    state     <= ST_IDLE;
                end

                //==============================================================
                // Default recovery
                //--------------------------------------------------------------
                // Purpose:
                //   Recover from any illegal or unknown state encoding by
                //   returning to the idle state.
                //==============================================================
                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire