`timescale 1ns/1ps
`default_nettype none


//==============================================================================
// ov5640_init_table
//------------------------------------------------------------------------------
// ROLE
//   Minimal Stage-A OV5640 initialization table.
//
// NOTES
//   - This table is intentionally conservative.
//   - Control-plane validation target:
//       power sequencing
//       sensor ID reads
//       basic MIPI two-lane + RAW10 path setup writes
//   - Full image-mode tuning remains a later Stage-B/Stage-C artifact.
//==============================================================================

module ov5640_init_table #(
    parameter integer LANE_CNT = 2
)(
    input  wire [7:0]  addr,
    output reg         valid,
    output reg         is_delay,
    output reg         last,
    output reg [15:0]  reg_addr,
    output reg [7:0]   reg_data,
    output reg [31:0]  delay_ms
);

    always @(*) begin
        valid    = 1'b1;
        is_delay = 1'b0;
        last     = 1'b0;
        reg_addr = 16'h0000;
        reg_data = 8'h00;
        delay_ms = 32'd0;

        case (addr)
            8'd0: begin reg_addr = 16'h3103; reg_data = 8'h11; end
            8'd1: begin reg_addr = 16'h3008; reg_data = 8'h82; end
            8'd2: begin is_delay = 1'b1; delay_ms = 32'd10; end
            8'd3: begin reg_addr = 16'h3008; reg_data = 8'h42; end
            8'd4: begin reg_addr = 16'h3103; reg_data = 8'h03; end
            8'd5: begin reg_addr = 16'h300E; reg_data = (LANE_CNT == 2) ? 8'h45 : 8'h25; end
            8'd6: begin reg_addr = 16'h4800; reg_data = 8'h14; end
            8'd7: begin reg_addr = 16'h4300; reg_data = 8'h00; end
            8'd8: begin reg_addr = 16'h501F; reg_data = 8'h03; end

            8'd9: begin
                valid = 1'b0;
                last  = 1'b1;
            end

            default: begin
                valid = 1'b0;
                last  = 1'b1;
            end
        endcase
    end

endmodule

`default_nettype wire
//
/*
//==============================================================================
// ov5640_init_table
//------------------------------------------------------------------------------
// PURPOSE
//   This module implements a small combinational initialization table for the
//   OV5640 image sensor.
//
//   The table is indexed by an external step counter, `index`, and returns the
//   control fields associated with that step:
//
//     - operation type               (`op`)
//     - target register address      (`reg_addr`)
//     - write payload                (`wr_data`)
//     - expected readback value      (`expect_data`)
//     - delay duration in ms         (`delay_ms`)
//
//   Conceptually, this module behaves like a ROM-backed script:
//
//       step index  --->  decoded initialization command
//
//   It contains no clock, no internal state, and no memory elements. Its
//   outputs are purely combinational functions of:
//
//       1) the current table index
//       2) the static parameter LANE_CNT
//
// ARCHITECTURAL ROLE
//   In a larger camera-control subsystem, this table is typically consumed by a
//   supervisory initialization FSM. That FSM:
//
//     1) drives `index`
//     2) reads the corresponding command fields from this module
//     3) interprets `op`
//     4) issues SCCB/I2C transactions or delays as required
//     5) advances to the next table entry when the current entry completes
//
//   Thus, this module does not "perform initialization" by itself.
//   It only *describes* the scripted initialization sequence.
//
// DESIGN STYLE
//   - Fully combinational
//   - Deterministic
//   - Single decode table
//   - No hidden state
//   - Easy to inspect in simulation and waveform review
//
// PARAMETERIZATION
//   INDEX_W
//     Width of the table index input. The present case table uses explicit
//     entries 0 through 12, so INDEX_W must be wide enough to represent at
//     least those values. In practice, INDEX_W=8 is more than sufficient.
//
//   LANE_CNT
//     Camera CSI lane-count selection parameter.
//     This parameter affects at least one script entry, allowing the table to
//     emit different register values for 1-lane versus 2-lane configurations.
//
// OPERATION ENCODING
//   The downstream FSM must interpret `op` consistently. The encoding implied
//   by this table is:
//
//     op = 2'd0 : NOP / END / INVALID-ENTRY
//         No action required.
//         In many systems this acts as the end-of-table marker.
//
//     op = 2'd1 : WRITE
//         Perform a register write:
//             register[reg_addr] <- wr_data
//
//     op = 2'd2 : READ-EXPECT / VERIFY
//         Perform a register read and compare the returned value against
//         `expect_data`.
//
//     op = 2'd3 : DELAY
//         Wait for `delay_ms` milliseconds before advancing.
//
// IMPORTANT CONTRACT NOTE
//   This module assumes that the consumer knows and agrees with the above
//   operation encoding. The encoding should therefore be documented centrally
//   in the camera initialization subsystem.
//
// DEFAULTING STRATEGY
//   At the top of the combinational block, all outputs are assigned safe
//   defaults. This is a standard and important combinational RTL practice.
//
//   Benefits:
//     - prevents inferred latches
//     - makes unspecified case fields deterministic
//     - allows each case item to override only the fields it needs
//
//   In particular, for entries such as DELAY or END, only a subset of fields
//   have meaningful values. The default assignments ensure all other outputs
//   remain well-defined.
//
// SCRIPT CONTENT OVERVIEW
//   The current script performs the following high-level actions:
//
//     - Verify sensor ID high byte
//     - Verify sensor ID low byte
//     - Select system clock source / pre-reset configuration
//     - Issue software reset
//     - Delay after reset
//     - Bring sensor out of reset / power-down state
//     - Configure clocking / lane-related interface setup
//     - Configure selected format / ISP-related registers
//     - Program one framing-related register
//     - Terminate the script with a NOP/END entry
//
// IMPORTANT MAINTENANCE NOTE
//   This table is intentionally short and likely only a bring-up skeleton, not
//   a complete production OV5640 configuration. Many practical OV5640 setups
//   require substantially longer register scripts for PLL, timing, crop,
//   scaling, output format, MIPI timing, exposure defaults, and ISP controls.
//
// SYNTHESIS NOTE
//   Synthesis will typically map this module into combinational LUT logic,
//   behaving as a small decode ROM.
//
//==============================================================================
module ov5640_init_table #(
    parameter integer INDEX_W  = 8,
    parameter integer LANE_CNT = 2
)(
    //--------------------------------------------------------------------------
    // Table address / step selector
    //--------------------------------------------------------------------------
    // The supervisory FSM drives this value to select which script entry is
    // currently being requested from the table.
    //--------------------------------------------------------------------------
    input  wire [INDEX_W-1:0] index,

    //--------------------------------------------------------------------------
    // Decoded operation type for the selected entry
    //--------------------------------------------------------------------------
    // See OPERATION ENCODING above.
    //--------------------------------------------------------------------------
    output reg  [1:0]         op,

    //--------------------------------------------------------------------------
    // Target register address
    //--------------------------------------------------------------------------
    // Meaningful primarily for WRITE and READ-EXPECT operations.
    //--------------------------------------------------------------------------
    output reg  [15:0]        reg_addr,

    //--------------------------------------------------------------------------
    // Write payload
    //--------------------------------------------------------------------------
    // Meaningful primarily for WRITE operations.
    //--------------------------------------------------------------------------
    output reg  [7:0]         wr_data,

    //--------------------------------------------------------------------------
    // Expected readback value
    //--------------------------------------------------------------------------
    // Meaningful primarily for READ-EXPECT operations.
    //--------------------------------------------------------------------------
    output reg  [7:0]         expect_data,

    //--------------------------------------------------------------------------
    // Delay duration in milliseconds
    //--------------------------------------------------------------------------
    // Meaningful primarily for DELAY operations.
    //--------------------------------------------------------------------------
    output reg  [15:0]        delay_ms
);

    //--------------------------------------------------------------------------
    // Combinational decode of the initialization script
    //--------------------------------------------------------------------------
    // This block must assign every output for every possible `index` value.
    // To guarantee this, safe defaults are established first, and then the
    // selected case entry overrides only the fields relevant to that entry.
    //
    // Step-by-step behavior:
    //
    //   1) Assume a benign "do nothing" entry by default.
    //   2) Examine `index`.
    //   3) If `index` matches a known table entry, override the outputs with
    //      that entry's operation and associated operands.
    //   4) If `index` is outside the defined script range, leave the outputs at
    //      their safe default values, which correspond to NOP/END behavior.
    //
    // This pattern produces deterministic combinational logic without latches.
    //--------------------------------------------------------------------------
    always @(*) begin
        //----------------------------------------------------------------------
        // Default entry:
        //   op = 0 means "no action" / "end of table" unless overridden.
        //
        // All fields are initialized to known zero values so that:
        //   - unspecified fields never float conceptually
        //   - no latches are inferred
        //   - default / out-of-range index behavior is deterministic
        //----------------------------------------------------------------------
        op          = 2'd0;
        reg_addr    = 16'h0000;
        wr_data     = 8'h00;
        expect_data = 8'h00;
        delay_ms    = 16'd0;

        //----------------------------------------------------------------------
        // Table decode
        //----------------------------------------------------------------------
        case (index)

            //==================================================================
            // Entry 0
            //------------------------------------------------------------------
            // READ-EXPECT: Verify OV5640 chip ID high byte.
            //
            // Register 0x300A is commonly used as the high byte of the sensor
            // ID. The expected value here is 0x56.
            //
            // System meaning:
            //   "Read register 0x300A and confirm that it equals 0x56."
            //
            // Why this matters:
            //   This helps confirm that:
            //     - SCCB communication is alive
            //     - the addressed device is plausibly the intended sensor
            //     - the bus is not floating or talking to the wrong target
            //==================================================================
            8'd0: begin
                op          = 2'd2;
                reg_addr    = 16'h300A;
                expect_data = 8'h56;
            end

            //==================================================================
            // Entry 1
            //------------------------------------------------------------------
            // READ-EXPECT: Verify OV5640 chip ID low byte.
            //
            // Register 0x300B is commonly used as the low byte of the sensor
            // ID. The expected value here is 0x40.
            //
            // Together, entries 0 and 1 verify the full chip ID:
            //
            //     0x300A -> 0x56
            //     0x300B -> 0x40
            //
            // which corresponds to sensor ID 0x5640.
            //==================================================================
            8'd1: begin
                op          = 2'd2;
                reg_addr    = 16'h300B;
                expect_data = 8'h40;
            end

            //==================================================================
            // Entry 2
            //------------------------------------------------------------------
            // WRITE: Program register 0x3103 with 0x11.
            //
            // In many OV5640 initialization flows, this register participates
            // in system clocking or clock-source selection. The exact semantic
            // interpretation should be checked against the sensor documentation
            // used by the project.
            //
            // System meaning:
            //   "Write 0x11 to register 0x3103."
            //
            // Engineering note:
            //   Early clock/source configuration often precedes reset-release
            //   and subsequent interface programming.
            //==================================================================
            8'd2: begin
                op       = 2'd1;
                reg_addr = 16'h3103;
                wr_data  = 8'h11;
            end

            //==================================================================
            // Entry 3
            //------------------------------------------------------------------
            // WRITE: Software reset / system control operation.
            //
            // Register 0x3008 is widely associated with system control in many
            // OV5640 register scripts. Writing 0x82 is commonly used as a reset
            // action in initialization sequences.
            //
            // System meaning:
            //   "Issue a reset-oriented control write."
            //
            // Engineering consequence:
            //   After reset is issued, the script must generally wait long
            //   enough for the sensor to complete its internal reset behavior.
            //==================================================================
            8'd3: begin
                op       = 2'd1;
                reg_addr = 16'h3008;
                wr_data  = 8'h82;
            end

            //==================================================================
            // Entry 4
            //------------------------------------------------------------------
            // DELAY: Wait 10 milliseconds.
            //
            // This entry exists to provide post-reset settling time.
            //
            // System meaning:
            //   "Do not issue another SCCB operation until 10 ms have elapsed."
            //
            // Why delay matters:
            //   After reset-related writes, the sensor may need time for:
            //     - internal state reinitialization
            //     - PLL / clock recovery
            //     - digital logic stabilization
            //     - register accessibility restoration
            //==================================================================
            8'd4: begin
                op       = 2'd3;
                delay_ms = 16'd10;
            end

            //==================================================================
            // Entry 5
            //------------------------------------------------------------------
            // WRITE: Bring the sensor into the next operational state after the
            // reset delay.
            //
            // Register 0x3008 again participates in system control. Here the
            // write value 0x42 likely corresponds to a non-reset operational
            // mode used by this script.
            //
            // System meaning:
            //   "Write 0x42 to register 0x3008."
            //==================================================================
            8'd5: begin
                op       = 2'd1;
                reg_addr = 16'h3008;
                wr_data  = 8'h42;
            end

            //==================================================================
            // Entry 6
            //------------------------------------------------------------------
            // WRITE: Reprogram 0x3103 after reset-related sequencing.
            //
            // The value 0x03 differs from entry 2 and suggests that the script
            // is transitioning from an early pre-reset/pre-bring-up setting
            // into a subsequent normal operating configuration.
            //
            // System meaning:
            //   "Write 0x03 to register 0x3103."
            //==================================================================
            8'd6: begin
                op       = 2'd1;
                reg_addr = 16'h3103;
                wr_data  = 8'h03;
            end

            //==================================================================
            // Entry 7
            //------------------------------------------------------------------
            // WRITE: Configure lane-dependent interface behavior.
            //
            // Register 0x300E is assigned based on the LANE_CNT parameter:
            //
            //   if LANE_CNT == 2  -> write 0x45
            //   otherwise         -> write 0x25
            //
            // System meaning:
            //   The initialization script adapts one register field to the
            //   selected camera-lane configuration.
            //
            // Why parameterization matters:
            //   This allows one RTL table to support more than one physical link
            //   configuration without duplicating the entire script.
            //
            // Maintenance note:
            //   The exact lane-count meaning and legal values should be frozen
            //   at the subsystem level. As written, any non-2 value maps to the
            //   "else" case.
            //==================================================================
            8'd7: begin
                op       = 2'd1;
                reg_addr = 16'h300E;
                wr_data  = (LANE_CNT == 2) ? 8'h45 : 8'h25;
            end

            //==================================================================
            // Entry 8
            //------------------------------------------------------------------
            // WRITE: Program register 0x4800 with 0x14.
            //
            // In many OV5640 scripts, registers in this region relate to output
            // interface formatting / control. The exact meaning should be
            // verified against the chosen datasheet or reference script.
            //
            // System meaning:
            //   "Write 0x14 to register 0x4800."
            //==================================================================
            8'd8: begin
                op       = 2'd1;
                reg_addr = 16'h4800;
                wr_data  = 8'h14;
            end

            //==================================================================
            // Entry 9
            //------------------------------------------------------------------
            // WRITE: Program register 0x4300 with 0x00.
            //
            // This register is often associated with output format selection in
            // OV5640-related configurations. Exact interpretation again depends
            // on the project's chosen device reference.
            //
            // System meaning:
            //   "Write 0x00 to register 0x4300."
            //==================================================================
            8'd9: begin
                op       = 2'd1;
                reg_addr = 16'h4300;
                wr_data  = 8'h00;
            end

            //==================================================================
            // Entry 10
            //------------------------------------------------------------------
            // WRITE: Program register 0x501F with 0x03.
            //
            // Registers in this region are often tied to ISP / format /
            // processing-path configuration.
            //
            // System meaning:
            //   "Write 0x03 to register 0x501F."
            //==================================================================
            8'd10: begin
                op       = 2'd1;
                reg_addr = 16'h501F;
                wr_data  = 8'h03;
            end

            //==================================================================
            // Entry 11
            //------------------------------------------------------------------
            // WRITE: Program register 0x3800 with 0x02.
            //
            // The 0x380x region is frequently associated with image windowing
            // and geometry-related controls. In a complete production script,
            // this region often includes several adjacent registers.
            //
            // System meaning:
            //   "Write 0x02 to register 0x3800."
            //==================================================================
            8'd11: begin
                op       = 2'd1;
                reg_addr = 16'h3800;
                wr_data  = 8'h02;
            end

            //==================================================================
            // Entry 12
            //------------------------------------------------------------------
            // END-OF-TABLE marker.
            //
            // This entry intentionally leaves the default values in place:
            //
            //   op = 2'd0
            //
            // A supervisory FSM may interpret this as:
            //   - script complete
            //   - no further commands to execute
            //
            // Keeping an explicit terminal entry in the table is often cleaner
            // than relying only on the `default` branch, because it makes the
            // intended script length explicit.
            //==================================================================
            8'd12: begin
                op = 2'd0;
            end

            //==================================================================
            // Default case
            //------------------------------------------------------------------
            // Any out-of-range index decodes to NOP/END behavior.
            //
            // This provides safe behavior if:
            //   - the index overruns the intended script range
            //   - the FSM is misconfigured
            //   - unused upper address space is accessed
            //
            // Because defaults were assigned before the case statement, this
            // branch only needs to preserve the NOP semantics.
            //==================================================================
            default: begin
                op = 2'd0;
            end
        endcase
    end

endmodule

`default_nettype wire
*/