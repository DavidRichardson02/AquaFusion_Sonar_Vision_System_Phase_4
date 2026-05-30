`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// pcam_i2c_switch_ctrl
//------------------------------------------------------------------------------
// ROLE
//   Select exactly one FMC Pcam Adapter port before any OV5640 SCCB traffic.
//
// NOTES
//   - One-hot channel byte:
//       Port A -> 0x01
//       Port B -> 0x02
//       Port C -> 0x04
//       Port D -> 0x08
//   - No register phase is used for the switch transaction.
//==============================================================================

module pcam_i2c_switch_ctrl #(
    parameter [6:0] I2C_SWITCH_ADDR_7B = 7'h70
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire [1:0]  cam_sel,

    output reg         busy,
    output reg         done,
    output reg         err,

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

    localparam [2:0]
        ST_IDLE = 3'd0,
        ST_CMD  = 3'd1,
        ST_WAIT = 3'd2,
        ST_DONE = 3'd3,
        ST_FAIL = 3'd4;

    reg [2:0] state;
    reg       start_d1;

    wire start_rise;
    assign start_rise = start & ~start_d1;

    wire [7:0] chan_1hot;
    assign chan_1hot =
        (cam_sel == 2'd0) ? 8'h01 :
        (cam_sel == 2'd1) ? 8'h02 :
        (cam_sel == 2'd2) ? 8'h04 :
                            8'h08;

    always @(posedge clk) begin
        if (rst) begin
            state        <= ST_IDLE;
            start_d1     <= 1'b0;
            busy         <= 1'b0;
            done         <= 1'b0;
            err          <= 1'b0;
            cmd_valid    <= 1'b0;
            cmd_has_reg  <= 1'b0;
            cmd_is_read  <= 1'b0;
            cmd_dev_addr <= 7'd0;
            cmd_reg_addr <= 16'd0;
            cmd_wr_data  <= 8'd0;
        end else begin
            start_d1  <= start;
            done      <= 1'b0;
            cmd_valid <= 1'b0;

            case (state)
                ST_IDLE: begin
                    busy <= 1'b0;
                    err  <= 1'b0;

                    if (start_rise) begin
                        busy         <= 1'b1;
                        cmd_has_reg  <= 1'b0;
                        cmd_is_read  <= 1'b0;
                        cmd_dev_addr <= I2C_SWITCH_ADDR_7B;
                        cmd_reg_addr <= 16'h0000;
                        cmd_wr_data  <= chan_1hot;
                        state        <= ST_CMD;
                    end
                end

                ST_CMD: begin
                    cmd_valid <= 1'b1;
                    if (cmd_ready)
                        state <= ST_WAIT;
                end

                ST_WAIT: begin
                    if (rsp_valid) begin
                        if (rsp_err || rsp_ack_error) begin
                            err   <= 1'b1;
                            busy  <= 1'b0;
                            state <= ST_FAIL;
                        end else begin
                            busy  <= 1'b0;
                            state <= ST_DONE;
                        end
                    end
                end

                ST_DONE: begin
                    done  <= 1'b1;
                    state <= ST_IDLE;
                end

                ST_FAIL: begin
                    if (!start)
                        state <= ST_IDLE;
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

    wire _unused;
    assign _unused = rsp_rd_data[0];

endmodule

`default_nettype wire