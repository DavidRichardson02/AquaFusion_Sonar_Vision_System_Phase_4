`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// camera_init_fsm
//------------------------------------------------------------------------------
// Frozen Stage-A contract:
//   - Edge-triggered start
//   - Direct cam_pwup drive
//   - Registered cmd/rsp handshake matching pcam_i2c_switch_ctrl
//
// Minimal public, reviewable script:
//   Power sequence:
//     1) PWUP low  for 100 ms
//     2) PWUP high and wait 50 ms
//
//   SCCB sequence:
//     3) Read/check 0x300A == 0x56
//     4) Read/check 0x300B == 0x40
//     5) 0x3103 <= 0x11   // choose system input clock from pad
//     6) 0x3008 <= 0x82   // software reset
//     7) wait 10 ms
//     8) 0x3008 <= 0x42   // de-assert reset, keep power-down until cfg done
//     9) 0x3103 <= 0x03   // choose system input clock from PLL
//    10) 0x300E <= 0x45   // 2-lane MIPI
//        or 0x25          // 1-lane MIPI
//    11) 0x4800 <= 0x14   // free-run clock, LP11 when idle
//    12) 0x4300 <= 0x00   // RAW10 format part 1
//    13) 0x501F <= 0x03   // RAW10 format part 2
//
// Intentionally omitted here:
//   - PLL programming registers
//   - broader imaging configuration register set
//   - final wake-up write from the public manual, because the publicly posted
//     text lists an address that should be cross-checked against vendor/demo
//     sources before freezing into RTL.
//==============================================================================

module camera_init_fsm #(
    parameter integer CLK_HZ              = 100_000_000,
    parameter integer LANE_CNT            = 2,
    parameter [6:0]   OV5640_ADDR_7B      = 7'h3C,
    parameter integer PWUP_LOW_MS         = 100,
    parameter integer PWUP_TO_SCCB_MS     = 50,
    parameter integer SWRESET_HOLD_MS     = 10
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        start,

    output reg         cam_pwup,

    output reg         busy,
    output reg         init_done,
    output reg         init_fail,
    output reg         sensor_id_ok,
    output reg         nack_sticky,
    output reg [7:0]   last_err,
    output reg [15:0]  step_code,

    output reg         cmd_valid,
    input  wire        cmd_ready,
    output reg         cmd_has_reg,
    output reg         cmd_is_read,
    output reg [6:0]   cmd_dev_addr,
    output reg [15:0]  cmd_reg_addr,
    output reg [7:0]   cmd_wr_data,

    input  wire        rsp_valid,
    input  wire        rsp_err,
    input  wire        rsp_ack_error,
    input  wire [7:0]  rsp_rd_data
);

    localparam integer TICKS_PER_MS = CLK_HZ / 1000;

    localparam [2:0]
        OP_END   = 3'd0,
        OP_WRITE = 3'd1,
        OP_READ  = 3'd2,
        OP_DELAY = 3'd3;

    localparam [3:0]
        ST_IDLE       = 4'd0,
        ST_PWUP_LOW   = 4'd1,
        ST_PWUP_HIGH  = 4'd2,
        ST_FETCH      = 4'd3,
        ST_CMD_ARM    = 4'd4,
        ST_CMD_WAIT   = 4'd5,
        ST_WAIT_RSP   = 4'd6,
        ST_CHECK      = 4'd7,
        ST_DELAY      = 4'd8,
        ST_ADVANCE    = 4'd9,
        ST_DONE       = 4'd10,
        ST_FAIL       = 4'd11;

    localparam [7:0]
        ERR_NONE      = 8'h00,
        ERR_SCCB      = 8'h01,
        ERR_ID_HI     = 8'h02,
        ERR_ID_LO     = 8'h03,
        ERR_BAD_STEP  = 8'h04;

    reg [3:0]  state;
    reg        start_d1;
    reg [7:0]  step_idx;
    reg [31:0] delay_ctr;
    reg        id_hi_ok;
    reg        id_lo_ok;

    reg [2:0]  cur_op;
    reg [15:0] cur_reg_addr;
    reg [7:0]  cur_wr_data;
    reg [7:0]  cur_expect_data;
    reg [15:0] cur_delay_ms;

    wire start_rise;
    assign start_rise = start & ~start_d1;

    wire [7:0] mipi_lane_wr_data;
    assign mipi_lane_wr_data = (LANE_CNT == 1) ? 8'h25 : 8'h45;

    //--------------------------------------------------------------------------
    // Minimal explicit script
    //--------------------------------------------------------------------------
    always @* begin
        cur_op          = OP_END;
        cur_reg_addr    = 16'h0000;
        cur_wr_data     = 8'h00;
        cur_expect_data = 8'h00;
        cur_delay_ms    = 16'd0;

        case (step_idx)
            8'd0: begin
                cur_op          = OP_READ;
                cur_reg_addr    = 16'h300A;
                cur_expect_data = 8'h56;
            end

            8'd1: begin
                cur_op          = OP_READ;
                cur_reg_addr    = 16'h300B;
                cur_expect_data = 8'h40;
            end

            8'd2: begin
                cur_op       = OP_WRITE;
                cur_reg_addr = 16'h3103;
                cur_wr_data  = 8'h11;
            end

            8'd3: begin
                cur_op       = OP_WRITE;
                cur_reg_addr = 16'h3008;
                cur_wr_data  = 8'h82;
            end

            8'd4: begin
                cur_op       = OP_DELAY;
                cur_delay_ms = SWRESET_HOLD_MS[15:0];
            end

            8'd5: begin
                cur_op       = OP_WRITE;
                cur_reg_addr = 16'h3008;
                cur_wr_data  = 8'h42;
            end

            8'd6: begin
                cur_op       = OP_WRITE;
                cur_reg_addr = 16'h3103;
                cur_wr_data  = 8'h03;
            end

            8'd7: begin
                cur_op       = OP_WRITE;
                cur_reg_addr = 16'h300E;
                cur_wr_data  = mipi_lane_wr_data;
            end

            8'd8: begin
                cur_op       = OP_WRITE;
                cur_reg_addr = 16'h4800;
                cur_wr_data  = 8'h14;
            end

            8'd9: begin
                cur_op       = OP_WRITE;
                cur_reg_addr = 16'h4300;
                cur_wr_data  = 8'h00;
            end

            8'd10: begin
                cur_op       = OP_WRITE;
                cur_reg_addr = 16'h501F;
                cur_wr_data  = 8'h03;
            end

            8'd11: begin
                cur_op = OP_END;
            end

            default: begin
                cur_op = OP_END;
            end
        endcase
    end

    always @(posedge clk) begin
        if (rst) begin
            state        <= ST_IDLE;
            start_d1     <= 1'b0;
            cam_pwup     <= 1'b0;
            busy         <= 1'b0;
            init_done    <= 1'b0;
            init_fail    <= 1'b0;
            sensor_id_ok <= 1'b0;
            nack_sticky  <= 1'b0;
            last_err     <= ERR_NONE;
            step_code    <= 16'h0000;
            cmd_valid    <= 1'b0;
            cmd_has_reg  <= 1'b0;
            cmd_is_read  <= 1'b0;
            cmd_dev_addr <= 7'd0;
            cmd_reg_addr <= 16'd0;
            cmd_wr_data  <= 8'd0;
            step_idx     <= 8'd0;
            delay_ctr    <= 32'd0;
            id_hi_ok     <= 1'b0;
            id_lo_ok     <= 1'b0;
        end else begin
            start_d1  <= start;
            cmd_valid <= 1'b0;
            step_code <= {4'h0, state, step_idx};

            if (rsp_valid && rsp_ack_error)
                nack_sticky <= 1'b1;

            case (state)
                ST_IDLE: begin
                    busy         <= 1'b0;
                    init_done    <= 1'b0;
                    init_fail    <= 1'b0;
                    sensor_id_ok <= 1'b0;
                    nack_sticky  <= 1'b0;
                    last_err     <= ERR_NONE;
                    step_idx     <= 8'd0;
                    delay_ctr    <= 32'd0;
                    id_hi_ok     <= 1'b0;
                    id_lo_ok     <= 1'b0;
                    cam_pwup     <= 1'b0;

                    if (start_rise) begin
                        busy      <= 1'b1;
                        cam_pwup  <= 1'b0;
                        delay_ctr <= 32'd0;
                        state     <= ST_PWUP_LOW;
                    end
                end

                ST_PWUP_LOW: begin
                    busy     <= 1'b1;
                    cam_pwup <= 1'b0;

                    if (delay_ctr >= (PWUP_LOW_MS * TICKS_PER_MS - 1)) begin
                        delay_ctr <= 32'd0;
                        cam_pwup  <= 1'b1;
                        state     <= ST_PWUP_HIGH;
                    end else begin
                        delay_ctr <= delay_ctr + 32'd1;
                    end
                end

                ST_PWUP_HIGH: begin
                    busy     <= 1'b1;
                    cam_pwup <= 1'b1;

                    if (delay_ctr >= (PWUP_TO_SCCB_MS * TICKS_PER_MS - 1)) begin
                        delay_ctr <= 32'd0;
                        step_idx  <= 8'd0;
                        state     <= ST_FETCH;
                    end else begin
                        delay_ctr <= delay_ctr + 32'd1;
                    end
                end

                ST_FETCH: begin
                    busy     <= 1'b1;
                    cam_pwup <= 1'b1;

                    case (cur_op)
                        OP_END: begin
                            sensor_id_ok <= id_hi_ok & id_lo_ok;
                            init_done    <= 1'b1;
                            busy         <= 1'b0;
                            state        <= ST_DONE;
                        end

                        OP_WRITE: begin
                            cmd_has_reg  <= 1'b1;
                            cmd_is_read  <= 1'b0;
                            cmd_dev_addr <= OV5640_ADDR_7B;
                            cmd_reg_addr <= cur_reg_addr;
                            cmd_wr_data  <= cur_wr_data;
                            state        <= ST_CMD_ARM;
                        end

                        OP_READ: begin
                            cmd_has_reg  <= 1'b1;
                            cmd_is_read  <= 1'b1;
                            cmd_dev_addr <= OV5640_ADDR_7B;
                            cmd_reg_addr <= cur_reg_addr;
                            cmd_wr_data  <= 8'h00;
                            state        <= ST_CMD_ARM;
                        end

                        OP_DELAY: begin
                            delay_ctr <= 32'd0;
                            state     <= ST_DELAY;
                        end

                        default: begin
                            last_err  <= ERR_BAD_STEP;
                            init_fail <= 1'b1;
                            busy      <= 1'b0;
                            state     <= ST_FAIL;
                        end
                    endcase
                end

                ST_CMD_ARM: begin
                    busy      <= 1'b1;
                    cam_pwup  <= 1'b1;
                    cmd_valid <= 1'b1;
                    state     <= ST_CMD_WAIT;
                end

                ST_CMD_WAIT: begin
                    busy      <= 1'b1;
                    cam_pwup  <= 1'b1;
                    cmd_valid <= 1'b1;

                    if (cmd_ready)
                        state <= ST_WAIT_RSP;
                end

                ST_WAIT_RSP: begin
                    busy     <= 1'b1;
                    cam_pwup <= 1'b1;

                    if (rsp_valid) begin
                        if (rsp_err || rsp_ack_error) begin
                            last_err  <= ERR_SCCB;
                            init_fail <= 1'b1;
                            busy      <= 1'b0;
                            state     <= ST_FAIL;
                        end else begin
                            if (cmd_is_read)
                                state <= ST_CHECK;
                            else
                                state <= ST_ADVANCE;
                        end
                    end
                end

                ST_CHECK: begin
                    busy     <= 1'b1;
                    cam_pwup <= 1'b1;

                    if (step_idx == 8'd0) begin
                        if (rsp_rd_data == cur_expect_data) begin
                            id_hi_ok <= 1'b1;
                            state    <= ST_ADVANCE;
                        end else begin
                            last_err  <= ERR_ID_HI;
                            init_fail <= 1'b1;
                            busy      <= 1'b0;
                            state     <= ST_FAIL;
                        end
                    end else if (step_idx == 8'd1) begin
                        if (rsp_rd_data == cur_expect_data) begin
                            id_lo_ok <= 1'b1;
                            state    <= ST_ADVANCE;
                        end else begin
                            last_err  <= ERR_ID_LO;
                            init_fail <= 1'b1;
                            busy      <= 1'b0;
                            state     <= ST_FAIL;
                        end
                    end else begin
                        if (rsp_rd_data == cur_expect_data) begin
                            state <= ST_ADVANCE;
                        end else begin
                            last_err  <= ERR_SCCB;
                            init_fail <= 1'b1;
                            busy      <= 1'b0;
                            state     <= ST_FAIL;
                        end
                    end
                end

                ST_DELAY: begin
                    busy     <= 1'b1;
                    cam_pwup <= 1'b1;

                    if (delay_ctr >= (cur_delay_ms * TICKS_PER_MS - 1)) begin
                        delay_ctr <= 32'd0;
                        state     <= ST_ADVANCE;
                    end else begin
                        delay_ctr <= delay_ctr + 32'd1;
                    end
                end

                ST_ADVANCE: begin
                    busy     <= 1'b1;
                    cam_pwup <= 1'b1;
                    step_idx <= step_idx + 8'd1;
                    state    <= ST_FETCH;
                end

                ST_DONE: begin
                    busy     <= 1'b0;
                    cam_pwup <= 1'b1;

                    if (!start)
                        state <= ST_IDLE;
                end

                ST_FAIL: begin
                    busy     <= 1'b0;
                    cam_pwup <= 1'b1;

                    if (!start)
                        state <= ST_IDLE;
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule

`default_nettype wire
//
/*
`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// camera_init_fsm
//------------------------------------------------------------------------------
// ROLE
//   Camera initialization sequencer driven by a table of operations.
//
// HIGH-LEVEL PURPOSE
//   This module orchestrates the full camera control-plane bring-up sequence by
//   coordinating three kinds of actions:
//
//     1) camera power-up timing through `pwup_start` and the returned
//        `pwup_done` / `pwup_ready_for_sccb` indications
//
//     2) SCCB transactions through a simple command interface to the lower-level
//        `sccb_master`
//
//     3) interpretation of a table-driven initialization script via
//        `table_index`, `table_op`, `table_reg_addr`, `table_wr_data`,
//        `table_expect_data`, and `table_delay_ms`
//
//   Conceptually, this module is the "bring-up program counter" for the camera.
//
// WHY THIS MODULE EXISTS
//   Camera initialization is not a single transaction. It is a structured
//   sequence that typically includes:
//
//     - waiting for power-up timing requirements
//     - reading identification registers
//     - writing configuration registers
//     - inserting mandatory delays
//     - checking whether certain readbacks match expected values
//
//   Implementing that flow directly in ad hoc top-level glue logic would be hard
//   to review and hard to maintain. Instead, this FSM provides a formal control
//   structure:
//
//       start request
//           ->
//       power-up phase
//           ->
//       fetch next table entry
//           ->
//       execute write/read/delay
//           ->
//       validate where required
//           ->
//       advance or fail
//
// TABLE-DRIVEN EXECUTION MODEL
//   The module does not hard-code every register operation internally.
//   Instead, it uses:
//
//       table_index
//
//   to address an external table provider. The table provider returns one entry
//   at a time through:
//
//       table_op
//       table_reg_addr
//       table_wr_data
//       table_expect_data
//       table_delay_ms
//
//   The FSM then interprets that entry according to `table_op`.
//
// TABLE OPERATION ENCODING
//   Present interpretation:
//
//     table_op = 0
//       End-of-table sentinel.
//       Initialization is considered complete.
//
//     table_op = 1
//       Register write.
//       Issue SCCB write command using table_reg_addr and table_wr_data.
//
//     table_op = 2
//       Register read-and-check.
//       Issue SCCB read command using table_reg_addr, then compare returned
//       `sccb_rd_data` against `table_expect_data`.
//
//     table_op = 3
//       Delay entry.
//       Wait for `table_delay_ms` milliseconds.
//
// FSM STATE OVERVIEW
//   ST_IDLE
//     Wait for initialization start request.
//
//   ST_PWUP
//     Wait for the external power-up FSM to complete and declare SCCB-ready.
//
//   ST_FETCH
//     Read and interpret the current table entry.
//
//   ST_ISSUE_CMD
//     Present a command to the SCCB master when it is ready.
//
//   ST_WAIT_CMD
//     Wait for the SCCB command to complete or fail.
//
//   ST_CHECK_READ
//     Validate readback data against the expected table value.
//
//   ST_DELAY
//     Wait for the requested table delay interval.
//
//   ST_ADVANCE
//     Increment table_index and return to ST_FETCH.
//
//   ST_DONE
//     Hold successful-completion state.
//
//   ST_FAIL
//     Hold failure state.
//
// SIGNAL SEMANTICS
//   clk
//     Local synchronous control clock.
//
//   rst
//     Active-high synchronous reset.
//
//   start
//     Start request for initialization.
//
//     IMPORTANT PRESENT-BEHAVIOR NOTE:
//       The signal is treated as level-sensitive. If asserted while in ST_DONE
//       or ST_FAIL, it restarts the initialization sequence.
//
//   pwup_start
//     One-cycle request pulse to the lower-level power-up FSM.
//
//   pwup_done
//     Indicates that the lower-level power-up FSM has completed.
//
//   pwup_ready_for_sccb
//     Indicates that SCCB transactions are now allowed.
//
//   sccb_cmd_valid
//     One-cycle command issue pulse to the SCCB master.
//
//   sccb_cmd_ready
//     Indicates that the SCCB master can accept a new command.
//
//   sccb_cmd_type
//     SCCB command type:
//       0 = write
//       1 = read
//
//   sccb_cmd_reg_addr
//     Register address for the SCCB transaction.
//
//   sccb_cmd_wr_data
//     Write data for SCCB write transactions.
//
//   sccb_done
//     One-cycle pulse indicating successful SCCB transaction completion.
//
//   sccb_err
//     One-cycle pulse indicating SCCB transaction failure.
//
//   sccb_ack_error
//     One-cycle pulse indicating missing ACK during SCCB transaction.
//
//   sccb_rd_data
//     Read data returned by the SCCB master after a read transaction.
//
//   table_index
//     Current index into the external initialization table.
//
//   table_op
//     Operation code for the current table entry.
//
//   table_reg_addr
//     Register address field from current table entry.
//
//   table_wr_data
//     Write-data field from current table entry.
//
//   table_expect_data
//     Expected readback value for read-and-check entries.
//
//   table_delay_ms
//     Delay duration for delay entries.
//
//   busy
//     Indicates that initialization is in progress.
//
//   init_done
//     Indicates successful completion.
//
//   init_fail
//     Indicates failure.
//
//   sensor_id_ok
//     Indicates that the sensor ID high and low bytes were both successfully
//     read and matched expected values.
//
//   last_err
//     Encoded reason for most recent failure.
//
// ERROR CODE SEMANTICS
//   ERR_NONE
//     No error.
//
//   ERR_PWUP
//     Reserved placeholder for power-up failure.
//
//     IMPORTANT PRESENT-BEHAVIOR NOTE:
//       This code is defined but not assigned anywhere in the implemented FSM.
//
//   ERR_SCCB
//     SCCB transaction failure or generic readback mismatch for non-ID reads.
//
//   ERR_ID_HI
//     Sensor ID high byte did not match expected value.
//
//   ERR_ID_LO
//     Sensor ID low byte did not match expected value.
//
//   ERR_BAD_OP
//     Table entry contained an unsupported operation code.
//
// SENSOR-ID POLICY
//   The FSM treats the first two read-check entries specially by table index:
//
//     table_index == 0
//       expected to be sensor ID high byte check
//
//     table_index == 1
//       expected to be sensor ID low byte check
//
//   Successful comparison at these positions sets:
//
//       id_hi_ok
//       id_lo_ok
//
//   At end-of-table, the module computes:
//
//       sensor_id_ok <= id_hi_ok & id_lo_ok
//
//   Therefore, `sensor_id_ok` is a summary of whether both early ID checks
//   passed.
//
// TIMING MODEL FOR DELAY ENTRIES
//   Delay entries use:
//
//       TICKS_PER_MS = CLK_HZ / 1000
//
//   and compare:
//
//       delay_ctr >= (table_delay_ms * TICKS_PER_MS) - 1
//
//   so that delay requests are implemented as cycle-counted waits.
//
// RESET BEHAVIOR
//   On rst assertion:
//
//     - state returns to ST_IDLE
//     - outputs and status fields are cleared
//     - table index resets to zero
//     - ID-check latches are cleared
//     - delay counter is cleared
//
//   Engineering meaning:
//     Reset returns the initialization controller to a clean "not started yet"
//     state.
//
// PULSE BEHAVIOR
//   Two outputs are intentionally pulse-like:
//
//     pwup_start
//     sccb_cmd_valid
//
//   At the start of every non-reset cycle, both are defaulted low:
//
//       pwup_start     <= 0
//       sccb_cmd_valid <= 0
//
//   and are then asserted only in the specific states/cycles that launch an
//   action.
//
// DESIGN PHILOSOPHY
//   This module is a classic example of layered control abstraction:
//
//     - power-up timing is delegated to `camera_pwup_fsm`
//     - individual bus transactions are delegated to `sccb_master`
//     - initialization policy is encoded as a table
//     - this FSM sequences those pieces into one coherent bring-up flow
//
// PEDAGOGICAL SUMMARY
//   The full initialization algorithm can be read as:
//
//     Step 1: wait for start
//     Step 2: trigger and wait for power-up readiness
//     Step 3: fetch the current initialization-table entry
//     Step 4: if entry is a write, issue SCCB write
//     Step 5: if entry is a read, issue SCCB read and verify result
//     Step 6: if entry is a delay, wait required time
//     Step 7: advance to next table entry
//     Step 8: stop successfully when end-of-table is reached
//     Step 9: stop with failure if any required check fails
//------------------------------------------------------------------------------
module camera_init_fsm #(
    //--------------------------------------------------------------------------
    // Local control clock frequency in hertz.
    //--------------------------------------------------------------------------
    parameter integer CLK_HZ   = 100_000_000,

    //--------------------------------------------------------------------------
    // Width of the table index register.
    //--------------------------------------------------------------------------
    parameter integer INDEX_W  = 8,

    //--------------------------------------------------------------------------
    // Camera lane-count parameter.
    //
    // IMPORTANT PRESENT-BEHAVIOR NOTE:
    //   This parameter is not functionally consumed by the FSM logic here. It
    //   is only referenced in the unused sink at the end of the module.
    //--------------------------------------------------------------------------
    parameter integer LANE_CNT = 2
)(
    input  wire clk,
    input  wire rst,
    input  wire start,

    //--------------------------------------------------------------------------
    // Interface to camera_pwup_fsm
    //--------------------------------------------------------------------------
    output reg  pwup_start,
    input  wire pwup_done,
    input  wire pwup_ready_for_sccb,

    //--------------------------------------------------------------------------
    // Interface to sccb_master
    //--------------------------------------------------------------------------
    output reg         sccb_cmd_valid,
    input  wire        sccb_cmd_ready,
    output reg         sccb_cmd_type,
    output reg  [15:0] sccb_cmd_reg_addr,
    output reg  [7:0]  sccb_cmd_wr_data,

    input  wire        sccb_done,
    input  wire        sccb_err,
    input  wire        sccb_ack_error,
    input  wire [7:0]  sccb_rd_data,

    //--------------------------------------------------------------------------
    // Interface to initialization table source
    //--------------------------------------------------------------------------
    output reg  [INDEX_W-1:0] table_index,
    input  wire [1:0]         table_op,
    input  wire [15:0]        table_reg_addr,
    input  wire [7:0]         table_wr_data,
    input  wire [7:0]         table_expect_data,
    input  wire [15:0]        table_delay_ms,

    //--------------------------------------------------------------------------
    // Status outputs
    //--------------------------------------------------------------------------
    output reg  busy,
    output reg  init_done,
    output reg  init_fail,
    output reg  sensor_id_ok,
    output reg  [7:0] last_err
);

    //==========================================================================
    // FSM state encoding
    //==========================================================================
    localparam [4:0]
        ST_IDLE       = 5'd0,
        ST_PWUP       = 5'd1,
        ST_FETCH      = 5'd2,
        ST_ISSUE_CMD  = 5'd3,
        ST_WAIT_CMD   = 5'd4,
        ST_CHECK_READ = 5'd5,
        ST_DELAY      = 5'd6,
        ST_ADVANCE    = 5'd7,
        ST_DONE       = 5'd8,
        ST_FAIL       = 5'd9;

    //==========================================================================
    // Error-code encoding
    //==========================================================================
    localparam [7:0]
        ERR_NONE   = 8'h00,
        ERR_PWUP   = 8'h01,
        ERR_SCCB   = 8'h02,
        ERR_ID_HI  = 8'h03,
        ERR_ID_LO  = 8'h04,
        ERR_BAD_OP = 8'h05;

    //==========================================================================
    // Millisecond timing conversion for delay entries
    //==========================================================================
    localparam integer TICKS_PER_MS = CLK_HZ / 1000;

    //==========================================================================
    // Internal registers
    //--------------------------------------------------------------------------
    // state
    //   Current FSM state.
    //
    // delay_ctr
    //   Cycle counter used during delay entries.
    //
    // id_hi_ok / id_lo_ok
    //   Sticky-success latches for the first two expected sensor ID reads.
    //==========================================================================
    reg [4:0]  state;
    reg [31:0] delay_ctr;
    reg        id_hi_ok;
    reg        id_lo_ok;

    //==========================================================================
    // Main initialization FSM
    //--------------------------------------------------------------------------
    // High-level structure:
    //
    //   A) Reset handling
    //   B) Default pulse clearing for pwup_start and sccb_cmd_valid
    //   C) State-specific bring-up policy execution
    //
    // The FSM advances one conceptual initialization step at a time:
    //   power-up, fetch entry, execute action, verify, delay, advance, done/fail
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            //------------------------------------------------------------------
            // Reset case
            //
            // Return the FSM to a clean not-started state.
            //------------------------------------------------------------------
            state             <= ST_IDLE;
            pwup_start        <= 1'b0;
            sccb_cmd_valid    <= 1'b0;
            sccb_cmd_type     <= 1'b0;
            sccb_cmd_reg_addr <= 16'd0;
            sccb_cmd_wr_data  <= 8'd0;
            table_index       <= {INDEX_W{1'b0}};
            busy              <= 1'b0;
            init_done         <= 1'b0;
            init_fail         <= 1'b0;
            sensor_id_ok      <= 1'b0;
            last_err          <= ERR_NONE;
            delay_ctr         <= 32'd0;
            id_hi_ok          <= 1'b0;
            id_lo_ok          <= 1'b0;
        end else begin
            //------------------------------------------------------------------
            // Default pulse clearing
            //
            // pwup_start and sccb_cmd_valid are launch pulses, not level-held
            // control outputs, so they are cleared each cycle unless explicitly
            // asserted in a specific state.
            //------------------------------------------------------------------
            pwup_start     <= 1'b0;
            sccb_cmd_valid <= 1'b0;

            case (state)

                //==============================================================
                // ST_IDLE
                //--------------------------------------------------------------
                // Purpose:
                //   Hold the controller in its baseline "not running" state and
                //   wait for a start request.
                //
                // Step-by-step behavior:
                //   1) Mark the controller not busy.
                //   2) Clear init_done and init_fail.
                //   3) Clear sensor_id_ok and last_err.
                //   4) Reset table_index, delay counter, and ID-success latches.
                //   5) If start is asserted:
                //        - assert busy
                //        - pulse pwup_start
                //        - transition to ST_PWUP
                //
                // Engineering meaning:
                //   Initialization does not begin by issuing SCCB traffic.
                //   It begins by requiring the lower-level power-up FSM to
                //   perform the timing-safe power-up sequence first.
                //==============================================================
                ST_IDLE: begin
                    busy         <= 1'b0;
                    init_done    <= 1'b0;
                    init_fail    <= 1'b0;
                    sensor_id_ok <= 1'b0;
                    last_err     <= ERR_NONE;
                    table_index  <= {INDEX_W{1'b0}};
                    delay_ctr    <= 32'd0;
                    id_hi_ok     <= 1'b0;
                    id_lo_ok     <= 1'b0;

                    if (start) begin
                        busy       <= 1'b1;
                        pwup_start <= 1'b1;
                        state      <= ST_PWUP;
                    end
                end

                //==============================================================
                // ST_PWUP
                //--------------------------------------------------------------
                // Purpose:
                //   Wait for the external power-up controller to declare that
                //   SCCB traffic is now safe.
                //
                // Step-by-step behavior:
                //   1) Hold busy high.
                //   2) Wait until both:
                //        - pwup_ready_for_sccb is asserted
                //        - pwup_done is asserted
                //   3) Then:
                //        - reset table_index to zero
                //        - move to ST_FETCH
                //
                // Note:
                //   There is no explicit power-up timeout or power-up failure
                //   detection in the current implementation.
                //==============================================================
                ST_PWUP: begin
                    busy <= 1'b1;
                    if (pwup_ready_for_sccb && pwup_done) begin
                        table_index <= {INDEX_W{1'b0}};
                        state       <= ST_FETCH;
                    end
                end

                //==============================================================
                // ST_FETCH
                //--------------------------------------------------------------
                // Purpose:
                //   Interpret the current table entry and decide what kind of
                //   action is required next.
                //
                // table_op meanings:
                //   0 -> end of table / success
                //   1 -> write command
                //   2 -> read-and-check command
                //   3 -> timed delay
                //
                // Step-by-step behavior:
                //   If table_op == 0:
                //     - compute sensor_id_ok from id_hi_ok & id_lo_ok
                //     - assert init_done
                //     - clear busy
                //     - move to ST_DONE
                //
                //   If table_op == 1:
                //     - prepare SCCB write command fields
                //     - move to ST_ISSUE_CMD
                //
                //   If table_op == 2:
                //     - prepare SCCB read command fields
                //     - move to ST_ISSUE_CMD
                //
                //   If table_op == 3:
                //     - clear delay counter
                //     - move to ST_DELAY
                //
                //   Otherwise:
                //     - record ERR_BAD_OP
                //     - assert init_fail
                //     - clear busy
                //     - move to ST_FAIL
                //==============================================================
                ST_FETCH: begin
                    busy <= 1'b1;
                    case (table_op)
                        2'd0: begin
                            sensor_id_ok <= id_hi_ok & id_lo_ok;
                            init_done    <= 1'b1;
                            busy         <= 1'b0;
                            state        <= ST_DONE;
                        end

                        2'd1: begin
                            sccb_cmd_type     <= 1'b0;
                            sccb_cmd_reg_addr <= table_reg_addr;
                            sccb_cmd_wr_data  <= table_wr_data;
                            state             <= ST_ISSUE_CMD;
                        end

                        2'd2: begin
                            sccb_cmd_type     <= 1'b1;
                            sccb_cmd_reg_addr <= table_reg_addr;
                            sccb_cmd_wr_data  <= 8'h00;
                            state             <= ST_ISSUE_CMD;
                        end

                        2'd3: begin
                            delay_ctr <= 32'd0;
                            state     <= ST_DELAY;
                        end

                        default: begin
                            last_err  <= ERR_BAD_OP;
                            init_fail <= 1'b1;
                            busy      <= 1'b0;
                            state     <= ST_FAIL;
                        end
                    endcase
                end

                //==============================================================
                // ST_ISSUE_CMD
                //--------------------------------------------------------------
                // Purpose:
                //   Launch the prepared SCCB command once the SCCB master is
                //   ready to accept it.
                //
                // Step-by-step behavior:
                //   1) Hold busy high.
                //   2) Wait until sccb_cmd_ready is asserted.
                //   3) Pulse sccb_cmd_valid for one cycle.
                //   4) Transition to ST_WAIT_CMD.
                //
                // Note:
                //   The actual command contents have already been prepared in
                //   ST_FETCH.
                //==============================================================
                ST_ISSUE_CMD: begin
                    busy <= 1'b1;
                    if (sccb_cmd_ready) begin
                        sccb_cmd_valid <= 1'b1;
                        state          <= ST_WAIT_CMD;
                    end
                end

                //==============================================================
                // ST_WAIT_CMD
                //--------------------------------------------------------------
                // Purpose:
                //   Wait for the lower-level SCCB transaction to complete or
                //   fail.
                //
                // Step-by-step behavior:
                //   1) Hold busy high.
                //   2) If sccb_err or sccb_ack_error is asserted:
                //        - record ERR_SCCB
                //        - assert init_fail
                //        - clear busy
                //        - move to ST_FAIL
                //   3) Else if sccb_done is asserted:
                //        - if current table operation was a read entry:
                //            move to ST_CHECK_READ
                //          else:
                //            move to ST_ADVANCE
                //
                // Important detail:
                //   The branch decision after sccb_done uses the *current*
                //   table_op, which is assumed to remain stable for the current
                //   table_index while the command is in flight.
                //==============================================================
                ST_WAIT_CMD: begin
                    busy <= 1'b1;
                    if (sccb_err || sccb_ack_error) begin
                        last_err  <= ERR_SCCB;
                        init_fail <= 1'b1;
                        busy      <= 1'b0;
                        state     <= ST_FAIL;
                    end else if (sccb_done) begin
                        if (table_op == 2'd2)
                            state <= ST_CHECK_READ;
                        else
                            state <= ST_ADVANCE;
                    end
                end

                //==============================================================
                // ST_CHECK_READ
                //--------------------------------------------------------------
                // Purpose:
                //   Validate returned read data against the current table entry's
                //   expected value.
                //
                // Special treatment of first two read entries:
                //   table_index == 0 -> sensor ID high byte check
                //   table_index == 1 -> sensor ID low byte check
                //
                // Step-by-step behavior:
                //
                //   If table_index == 0:
                //     - compare sccb_rd_data against table_expect_data
                //     - on match:
                //         * set id_hi_ok
                //         * advance
                //       on mismatch:
                //         * set ERR_ID_HI
                //         * fail
                //
                //   Else if table_index == 1:
                //     - compare sccb_rd_data against table_expect_data
                //     - on match:
                //         * set id_lo_ok
                //         * advance
                //       on mismatch:
                //         * set ERR_ID_LO
                //         * fail
                //
                //   Else:
                //     - generic read-check
                //     - on match: advance
                //     - on mismatch: fail with ERR_SCCB
                //==============================================================
                ST_CHECK_READ: begin
                    busy <= 1'b1;

                    if (table_index == 8'd0) begin
                        if (sccb_rd_data == table_expect_data) begin
                            id_hi_ok <= 1'b1;
                            state    <= ST_ADVANCE;
                        end else begin
                            last_err  <= ERR_ID_HI;
                            init_fail <= 1'b1;
                            busy      <= 1'b0;
                            state     <= ST_FAIL;
                        end
                    end else if (table_index == 8'd1) begin
                        if (sccb_rd_data == table_expect_data) begin
                            id_lo_ok <= 1'b1;
                            state    <= ST_ADVANCE;
                        end else begin
                            last_err  <= ERR_ID_LO;
                            init_fail <= 1'b1;
                            busy      <= 1'b0;
                            state     <= ST_FAIL;
                        end
                    end else if (sccb_rd_data == table_expect_data) begin
                        state <= ST_ADVANCE;
                    end else begin
                        last_err  <= ERR_SCCB;
                        init_fail <= 1'b1;
                        busy      <= 1'b0;
                        state     <= ST_FAIL;
                    end
                end

                //==============================================================
                // ST_DELAY
                //--------------------------------------------------------------
                // Purpose:
                //   Implement a millisecond-counted wait requested by the
                //   current table entry.
                //
                // Step-by-step behavior:
                //   1) Hold busy high.
                //   2) Compare delay_ctr against:
                //
                //        (table_delay_ms * TICKS_PER_MS) - 1
                //
                //   3) If the interval is complete:
                //        - clear delay_ctr
                //        - move to ST_ADVANCE
                //   4) Otherwise:
                //        - increment delay_ctr
                //
                // Engineering meaning:
                //   This state allows the initialization table to request timing
                //   gaps between SCCB operations without hard-coding those
                //   delays into the FSM itself.
                //==============================================================
                ST_DELAY: begin
                    busy <= 1'b1;
                    if (delay_ctr >= (table_delay_ms * TICKS_PER_MS) - 1) begin
                        delay_ctr <= 32'd0;
                        state     <= ST_ADVANCE;
                    end else begin
                        delay_ctr <= delay_ctr + 32'd1;
                    end
                end

                //==============================================================
                // ST_ADVANCE
                //--------------------------------------------------------------
                // Purpose:
                //   Move to the next table entry.
                //
                // Step-by-step behavior:
                //   1) Hold busy high.
                //   2) Increment table_index by one.
                //   3) Return to ST_FETCH.
                //
                // Architectural meaning:
                //   ST_FETCH interprets the current table entry; ST_ADVANCE moves
                //   the "program counter" to the next one.
                //==============================================================
                ST_ADVANCE: begin
                    busy        <= 1'b1;
                    table_index <= table_index + {{(INDEX_W-1){1'b0}},1'b1};
                    state       <= ST_FETCH;
                end

                //==============================================================
                // ST_DONE
                //--------------------------------------------------------------
                // Purpose:
                //   Hold successful completion state and optionally restart if a
                //   new start request arrives.
                //
                // Step-by-step behavior:
                //   1) Keep busy low.
                //   2) If start is asserted again:
                //        - clear success/failure flags
                //        - clear sensor_id_ok and last_err
                //        - reset table_index and ID latches
                //        - pulse pwup_start
                //        - assert busy
                //        - move back to ST_PWUP
                //
                // Present semantics:
                //   start acts as a restart request in ST_DONE.
                //==============================================================
                ST_DONE: begin
                    busy <= 1'b0;
                    if (start) begin
                        init_done    <= 1'b0;
                        init_fail    <= 1'b0;
                        sensor_id_ok <= 1'b0;
                        last_err     <= ERR_NONE;
                        table_index  <= {INDEX_W{1'b0}};
                        id_hi_ok     <= 1'b0;
                        id_lo_ok     <= 1'b0;
                        pwup_start   <= 1'b1;
                        busy         <= 1'b1;
                        state        <= ST_PWUP;
                    end
                end

                //==============================================================
                // ST_FAIL
                //--------------------------------------------------------------
                // Purpose:
                //   Hold failure state and optionally restart if a new start
                //   request arrives.
                //
                // Step-by-step behavior:
                //   1) Keep busy low.
                //   2) If start is asserted again:
                //        - clear success/failure flags
                //        - clear sensor_id_ok and last_err
                //        - reset table_index, ID latches, and delay counter
                //        - pulse pwup_start
                //        - assert busy
                //        - move back to ST_PWUP
                //
                // Present semantics:
                //   start acts as a retry request in ST_FAIL.
                //==============================================================
                ST_FAIL: begin
                    busy <= 1'b0;
                    if (start) begin
                        init_done    <= 1'b0;
                        init_fail    <= 1'b0;
                        sensor_id_ok <= 1'b0;
                        last_err     <= ERR_NONE;
                        table_index  <= {INDEX_W{1'b0}};
                        id_hi_ok     <= 1'b0;
                        id_lo_ok     <= 1'b0;
                        delay_ctr    <= 32'd0;
                        pwup_start   <= 1'b1;
                        busy         <= 1'b1;
                        state        <= ST_PWUP;
                    end
                end

                //==============================================================
                // Default recovery
                //--------------------------------------------------------------
                // Purpose:
                //   Recover from illegal or unknown state encodings by returning
                //   to ST_IDLE.
                //==============================================================
                default: state <= ST_IDLE;
            endcase
        end
    end

    //==========================================================================
    // Unused-signal sink
    //--------------------------------------------------------------------------
    // Purpose:
    //   Prevent warnings for presently unused design artifacts.
    //
    // ERR_PWUP
    //   Reserved error code not yet used in active logic.
    //
    // LANE_CNT
    //   Presently not consumed by the state machine behavior here.
    //==========================================================================
    wire _unused;
    assign _unused = ERR_PWUP[0] ^ LANE_CNT[0];

endmodule

`default_nettype wire
*/