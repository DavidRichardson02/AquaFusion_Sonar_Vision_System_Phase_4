`timescale 1ns/1ps
`default_nettype none

module camera_ctrl_subsystem_sys #(
    parameter integer CLK_HZ   = 100_000_000,
    parameter integer SCL_HZ   = 400_000,
    parameter integer FRAME_W  = 640,
    parameter integer FRAME_H  = 480,
    parameter integer H_TOTAL  = 800,
    parameter integer V_TOTAL  = 525,
    parameter integer SNAP_W   = 192
)(
    input  wire              clk_sys,
    input  wire              rst_sys,
    input  wire              start,
    input  wire              clk_vid,
    input  wire              rst_vid,
    input  wire [11:0]       pix_x_vid,
    input  wire [11:0]       pix_y_vid,
    input  wire              de_vid,
    input  wire              frame_tick_vid,

    output wire              cam_pwup,
    inout  wire              cam_scl,
    inout  wire              cam_sda,
    input  wire              cam_mipi_clk_p,
    input  wire              cam_mipi_clk_n,
    input  wire              cam_mipi_lane0_p,
    input  wire              cam_mipi_lane0_n,
    input  wire              cam_mipi_lane1_p,
    input  wire              cam_mipi_lane1_n,

    output wire              busy,
    output wire              init_done,
    output wire              init_fail,
    output wire              sensor_id_ok,
    output wire              camera_ready,
    output wire [7:0]        last_err,

    output wire [23:0]       camera_rgb_vid,
    output wire              camera_rgb_valid_vid,

    output wire [SNAP_W-1:0] cam_status_snap_sys,
    output wire              cam_status_snap_upd_sys
);

    localparam [6:0] I2C_SWITCH_ADDR_7B = 7'h70;
    localparam [6:0] OV5640_ADDR_7B     = 7'h3C;

    wire sw_busy;
    wire sw_done;
    wire sw_err;

    wire power_busy;
    wire power_done;
    wire power_good;
    wire ready_for_sccb;
    wire [2:0] power_state_dbg;

    wire sccb_init_busy;
    wire sccb_init_done_i;
    wire sccb_init_fail_i;
    wire sensor_id_ok_i;
    wire sccb_busy_i;
    wire sccb_error_i;
    wire sccb_done_pulse_i;
    wire [7:0] sccb_last_err_i;
    wire [15:0] sccb_table_index_i;
    wire [15:0] sccb_retry_count_i;

    wire cmd_valid_sw;
    wire cmd_has_reg_sw;
    wire cmd_is_read_sw;
    wire [6:0]  cmd_dev_addr_sw;
    wire [15:0] cmd_reg_addr_sw;
    wire [7:0]  cmd_wr_data_sw;

    wire cmd_valid_init;
    wire cmd_has_reg_init;
    wire cmd_is_read_init;
    wire [6:0]  cmd_dev_addr_init;
    wire [15:0] cmd_reg_addr_init;
    wire [7:0]  cmd_wr_data_init;

    wire cmd_ready;
    wire rsp_valid;
    wire rsp_err;
    wire rsp_ack_error;
    wire [7:0] rsp_rd_data;

    wire use_sensor_init = sw_done | power_done | sccb_init_busy | sccb_init_done_i;

    wire        cmd_valid_mux    = use_sensor_init ? cmd_valid_init    : cmd_valid_sw;
    wire        cmd_has_reg_mux  = use_sensor_init ? cmd_has_reg_init  : cmd_has_reg_sw;
    wire        cmd_is_read_mux  = use_sensor_init ? cmd_is_read_init  : cmd_is_read_sw;
    wire [6:0]  cmd_dev_addr_mux = use_sensor_init ? cmd_dev_addr_init : cmd_dev_addr_sw;
    wire [15:0] cmd_reg_addr_mux = use_sensor_init ? cmd_reg_addr_init : cmd_reg_addr_sw;
    wire [7:0]  cmd_wr_data_mux  = use_sensor_init ? cmd_wr_data_init  : cmd_wr_data_sw;

    wire        cam_rx_clk;
    wire        rx_axis_tvalid;
    wire [15:0] rx_axis_tdata;
    wire        rx_axis_tuser;
    wire        rx_axis_tlast;
    wire        rx_frame_done;
    wire        rx_csi_locked;
    wire [7:0]  rx_csi_error_flags;
    wire        rx_overflow_pulse;

    wire        stream_pixel_valid;
    wire [15:0] stream_pixel_data;
    wire        stream_frame_start;
    wire        stream_line_start;
    wire        stream_frame_done;
    wire [11:0] stream_pixel_x;
    wire [11:0] stream_pixel_y;
    wire        stream_timing_error_pulse;

    wire        frame_event_toggle;
    wire        drop_event_toggle;
    wire        overflow_event_toggle;
    wire [23:0] frame_rgb_vid;
    wire        frame_rgb_valid_vid;
    wire        frame_store_valid_vid;

    reg  [7:0]  csi_error_flags_hold_rx;

    always @(posedge cam_rx_clk or posedge rst_sys) begin
        if (rst_sys) begin
            csi_error_flags_hold_rx <= 8'h00;
        end else begin
            if (stream_timing_error_pulse)
                csi_error_flags_hold_rx[0] <= 1'b1;
            if (rx_overflow_pulse)
                csi_error_flags_hold_rx[1] <= 1'b1;
            if (rx_csi_error_flags != 8'h00)
                csi_error_flags_hold_rx <= csi_error_flags_hold_rx | rx_csi_error_flags;
        end
    end

    pcam_i2c_switch_ctrl #(
        .I2C_SWITCH_ADDR_7B(I2C_SWITCH_ADDR_7B)
    ) u_pcam_i2c_switch_ctrl (
        .clk          (clk_sys),
        .rst          (rst_sys),
        .start        (start),
        .cam_sel      (2'd0),
        .busy         (sw_busy),
        .done         (sw_done),
        .err          (sw_err),
        .cmd_valid    (cmd_valid_sw),
        .cmd_ready    (cmd_ready),
        .cmd_has_reg  (cmd_has_reg_sw),
        .cmd_is_read  (cmd_is_read_sw),
        .cmd_dev_addr (cmd_dev_addr_sw),
        .cmd_reg_addr (cmd_reg_addr_sw),
        .cmd_wr_data  (cmd_wr_data_sw),
        .rsp_valid    (rsp_valid),
        .rsp_err      (rsp_err),
        .rsp_ack_error(rsp_ack_error),
        .rsp_rd_data  (rsp_rd_data)
    );

    camera_power_fsm #(
        .CLK_HZ       (CLK_HZ),
        .HOLD_LOW_MS  (2),
        .POST_PWUP_MS (10)
    ) u_camera_power_fsm (
        .clk               (clk_sys),
        .rst               (rst_sys),
        .start             (sw_done),
        .cam_pwup          (cam_pwup),
        .busy              (power_busy),
        .done              (power_done),
        .camera_power_good (power_good),
        .ready_for_sccb    (ready_for_sccb),
        .state_dbg         (power_state_dbg)
    );

    ov5640_sccb_init #(
        .OV5640_ADDR_7B (OV5640_ADDR_7B),
        .CLK_HZ         (CLK_HZ)
    ) u_ov5640_sccb_init (
        .clk          (clk_sys),
        .rst          (rst_sys),
        .start        (power_done),
        .power_ready  (ready_for_sccb),
        .busy         (sccb_init_busy),
        .init_done    (sccb_init_done_i),
        .init_fail    (sccb_init_fail_i),
        .sensor_id_ok (sensor_id_ok_i),
        .sccb_busy    (sccb_busy_i),
        .sccb_error   (sccb_error_i),
        .sccb_done    (sccb_done_pulse_i),
        .last_err     (sccb_last_err_i),
        .table_index  (sccb_table_index_i),
        .retry_count  (sccb_retry_count_i),
        .cmd_valid    (cmd_valid_init),
        .cmd_ready    (cmd_ready),
        .cmd_has_reg  (cmd_has_reg_init),
        .cmd_is_read  (cmd_is_read_init),
        .cmd_dev_addr (cmd_dev_addr_init),
        .cmd_reg_addr (cmd_reg_addr_init),
        .cmd_wr_data  (cmd_wr_data_init),
        .rsp_valid    (rsp_valid),
        .rsp_err      (rsp_err),
        .rsp_ack_error(rsp_ack_error),
        .rsp_rd_data  (rsp_rd_data)
    );

    i2c_master_engine #(
        .CLK_HZ (CLK_HZ),
        .SCL_HZ (SCL_HZ)
    ) u_i2c_master_engine (
        .clk          (clk_sys),
        .rst          (rst_sys),
        .cmd_valid    (cmd_valid_mux),
        .cmd_ready    (cmd_ready),
        .cmd_has_reg  (cmd_has_reg_mux),
        .cmd_is_read  (cmd_is_read_mux),
        .cmd_dev_addr (cmd_dev_addr_mux),
        .cmd_reg_addr (cmd_reg_addr_mux),
        .cmd_wr_data  (cmd_wr_data_mux),
        .rsp_valid    (rsp_valid),
        .rsp_err      (rsp_err),
        .rsp_ack_error(rsp_ack_error),
        .rsp_rd_data  (rsp_rd_data),
        .busy         (),
        .scl_io       (cam_scl),
        .sda_io       (cam_sda)
    );

    pcam_csi_rx_wrapper #(
        .DATA_W  (16),
        .FRAME_W (FRAME_W),
        .FRAME_H (FRAME_H)
    ) u_pcam_csi_rx_wrapper (
        .clk_ref          (clk_sys),
        .rst              (rst_sys),
        .enable           (sccb_init_done_i),
        .cam_mipi_clk_p   (cam_mipi_clk_p),
        .cam_mipi_clk_n   (cam_mipi_clk_n),
        .cam_mipi_lane0_p (cam_mipi_lane0_p),
        .cam_mipi_lane0_n (cam_mipi_lane0_n),
        .cam_mipi_lane1_p (cam_mipi_lane1_p),
        .cam_mipi_lane1_n (cam_mipi_lane1_n),
        .cam_rx_clk       (cam_rx_clk),
        .axis_tvalid      (rx_axis_tvalid),
        .axis_tdata       (rx_axis_tdata),
        .axis_tuser       (rx_axis_tuser),
        .axis_tlast       (rx_axis_tlast),
        .frame_done       (rx_frame_done),
        .csi_locked       (rx_csi_locked),
        .csi_error_flags  (rx_csi_error_flags),
        .overflow_pulse   (rx_overflow_pulse)
    );

    camera_pixel_stream #(
        .FRAME_W (FRAME_W),
        .FRAME_H (FRAME_H)
    ) u_camera_pixel_stream (
        .clk                (cam_rx_clk),
        .rst                (rst_sys),
        .axis_tvalid        (rx_axis_tvalid),
        .axis_tdata         (rx_axis_tdata),
        .axis_tuser         (rx_axis_tuser),
        .axis_tlast         (rx_axis_tlast),
        .frame_done_in      (rx_frame_done),
        .pixel_valid        (stream_pixel_valid),
        .pixel_data         (stream_pixel_data),
        .frame_start        (stream_frame_start),
        .line_start         (stream_line_start),
        .frame_done         (stream_frame_done),
        .pixel_x            (stream_pixel_x),
        .pixel_y            (stream_pixel_y),
        .timing_error_pulse (stream_timing_error_pulse)
    );

    camera_frame_sync #(
        .FRAME_W (FRAME_W),
        .FRAME_H (FRAME_H),
        .H_TOTAL (H_TOTAL),
        .V_TOTAL (V_TOTAL)
    ) u_camera_frame_sync (
        .cam_clk               (cam_rx_clk),
        .cam_rst               (rst_sys),
        .pixel_valid           (stream_pixel_valid),
        .pixel_data            (stream_pixel_data),
        .frame_start           (stream_frame_start),
        .frame_done            (stream_frame_done),
        .frame_event_toggle    (frame_event_toggle),
        .drop_event_toggle     (drop_event_toggle),
        .overflow_event_toggle (overflow_event_toggle),
        .vid_clk               (clk_vid),
        .vid_rst               (rst_vid),
        .pix_x_vid             (pix_x_vid),
        .pix_y_vid             (pix_y_vid),
        .de_vid                (de_vid),
        .frame_tick_vid        (frame_tick_vid),
        .rgb_vid               (frame_rgb_vid),
        .rgb_valid_vid         (frame_rgb_valid_vid),
        .frame_store_valid_vid (frame_store_valid_vid)
    );

    camera_status_regs #(
        .CLK_HZ  (CLK_HZ),
        .SNAP_W  (SNAP_W),
        .FRAME_W (FRAME_W),
        .FRAME_H (FRAME_H)
    ) u_camera_status_regs (
        .clk                    (clk_sys),
        .rst                    (rst_sys),
        .power_good             (power_good),
        .sccb_busy              (sccb_busy_i),
        .sccb_done_pulse        (sccb_done_pulse_i),
        .sccb_error             (sccb_error_i | sw_err),
        .sccb_init_done         (sccb_init_done_i),
        .init_fail_in           (sccb_init_fail_i | sw_err),
        .sensor_id_ok           (sensor_id_ok_i),
        .last_err               (sw_err ? 8'hF0 : sccb_last_err_i),
        .table_index            (sccb_table_index_i),
        .retry_count            (sccb_retry_count_i),
        .csi_locked_async       (rx_csi_locked),
        .csi_error_flags_async  (csi_error_flags_hold_rx),
        .frame_event_toggle_async(frame_event_toggle),
        .drop_event_toggle_async(drop_event_toggle),
        .overflow_event_toggle_async(overflow_event_toggle),
        .frame_store_valid_async(frame_store_valid_vid),
        .camera_ready           (camera_ready),
        .busy                   (),
        .init_fail              (init_fail),
        .frame_count            (),
        .freshness_ms           (),
        .snap_data              (cam_status_snap_sys),
        .snap_upd               (cam_status_snap_upd_sys)
    );

    assign busy                = sw_busy | power_busy | sccb_init_busy | (~camera_ready);
    assign init_done           = sccb_init_done_i;
    assign sensor_id_ok        = sensor_id_ok_i;
    assign last_err            = sw_err ? 8'hF0 : sccb_last_err_i;
    assign camera_rgb_vid      = frame_rgb_vid;
    assign camera_rgb_valid_vid= frame_rgb_valid_vid;

    wire _unused_stream_xy;
    wire _unused_line_start;
    wire _unused_power_state;
    assign _unused_stream_xy   = ^stream_pixel_x ^ ^stream_pixel_y;
    assign _unused_line_start  = stream_line_start;
    assign _unused_power_state = ^power_state_dbg;

endmodule

`default_nettype wire
