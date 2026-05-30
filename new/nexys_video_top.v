`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// nexys_video_top
//------------------------------------------------------------------------------
// CANONICAL INTEGRATION SHELL
//
// Subsystems integrated:
//   1) SYS-domain camera control / SCCB bring-up
//   2) SYS-domain OLED telemetry formatting and SSD1306 driver
//   3) HDMI clock generation, PIX-domain video pipeline, TMDS output
//
// NOTE
//   This block freezes the logical system contract.
//   Exact FMC camera-adapter physical pin breakout is intentionally left to a
//   board-specific adapter wrapper / XDC pair.
//==============================================================================
module nexys_video_top #(
    parameter integer CLK_HZ          = 100_000_000,
    parameter integer OLED_REFRESH_HZ = 5,
    parameter integer OLED_LINE_CHARS = 21,
    parameter integer OLED_SPI_HZ     = 10_000_000,
    parameter integer HDMI_H_ACTIVE   = 640,
    parameter integer HDMI_H_FP       = 16,
    parameter integer HDMI_H_SYNC     = 96,
    parameter integer HDMI_H_BP       = 48,
    parameter integer HDMI_V_ACTIVE   = 480,
    parameter integer HDMI_V_FP       = 10,
    parameter integer HDMI_V_SYNC     = 2,
    parameter integer HDMI_V_BP       = 33,
    parameter integer HDMI_HSYNC_POL  = 0,
    parameter integer HDMI_VSYNC_POL  = 0,
    parameter integer CAM_SNAP_W      = 64
)(
    //--------------------------------------------------------------------------
    // Board clocks / reset
    //--------------------------------------------------------------------------
    input  wire        clk100,
    input  wire        rst_btn,

    //--------------------------------------------------------------------------
    // HDMI source-port sideband
    //--------------------------------------------------------------------------
    input  wire        hdmi_hpd_in,
    output wire        hdmi_tx_en,

    //--------------------------------------------------------------------------
    // HDMI TMDS outputs (J8 source)
    //--------------------------------------------------------------------------
    output wire        tmds_clk_p,
    output wire        tmds_clk_n,
    output wire [2:0]  tmds_data_p,
    output wire [2:0]  tmds_data_n,

    //--------------------------------------------------------------------------
    // On-board OLED control pins
    //--------------------------------------------------------------------------
    output wire        oled_res_n,
    output wire        oled_dc,
    output wire        oled_sclk,
    output wire        oled_sdin,
    output wire        oled_vbat_n,
    output wire        oled_vdd_n,

    //--------------------------------------------------------------------------
    // Camera control pins
    //--------------------------------------------------------------------------
    output wire        cam_pwup,
    inout  wire        cam_scl,
    inout  wire        cam_sda

    //--------------------------------------------------------------------------
    // Camera adapter / link physical pins are intentionally omitted here.
    // They belong in a board-specific FMC/adapter wrapper once the exact
    // Pcam-FMC mapping is frozen.
    //--------------------------------------------------------------------------
);

    //==========================================================================
    // 1) SYS-domain reset
    //==========================================================================
    wire rst_sys;
    assign rst_sys = rst_btn;

    //==========================================================================
    // 2) HDMI clock generation
    //==========================================================================
    wire clk_pix;
    wire clk_tmds_5x;
    wire hdmi_clk_locked;

    clk_wiz_hdmi_640x480 u_clk_wiz_hdmi_640x480 (
        .clk_in1 (clk100),
        .reset   (rst_sys),
        .clk_out1(clk_pix),
        .clk_out2(clk_tmds_5x),
        .locked  (hdmi_clk_locked)
    );

    //--------------------------------------------------------------------------
    // Pixel/TMDS resets
    //--------------------------------------------------------------------------
    // Canonical form:
    //   downstream logic held in reset until MMCM lock is achieved.
    //--------------------------------------------------------------------------
    reg [3:0] rst_pix_sync;
    reg [3:0] rst_tmds_sync;

    always @(posedge clk_pix or posedge rst_sys) begin
        if (rst_sys)
            rst_pix_sync <= 4'hF;
        else if (!hdmi_clk_locked)
            rst_pix_sync <= 4'hF;
        else
            rst_pix_sync <= {rst_pix_sync[2:0], 1'b0};
    end

    always @(posedge clk_tmds_5x or posedge rst_sys) begin
        if (rst_sys)
            rst_tmds_sync <= 4'hF;
        else if (!hdmi_clk_locked)
            rst_tmds_sync <= 4'hF;
        else
            rst_tmds_sync <= {rst_tmds_sync[2:0], 1'b0};
    end

    wire rst_pix     = rst_pix_sync[3];
    wire rst_tmds_5x = rst_tmds_sync[3];

    //==========================================================================
    // 3) Camera control subsystem (SYS domain)
    //==========================================================================
    wire                  cam_ctrl_busy;
    wire                  cam_init_done;
    wire                  cam_init_fail;
    wire                  cam_sensor_id_ok;
    wire [7:0]            cam_last_err;
    wire [CAM_SNAP_W-1:0] cam_status_snap_sys;
    wire                  cam_status_snap_upd_sys;

    camera_ctrl_subsystem_sys #(
        .CLK_HZ  (CLK_HZ),
        .SCL_HZ  (400_000),
        .LANE_CNT(2),
        .SNAP_W  (CAM_SNAP_W)
    ) u_camera_ctrl_subsystem_sys (
        .clk                    (clk100),
        .rst                    (rst_sys),
        .start                  (1'b1),

        .cam_pwup               (cam_pwup),
        .cam_scl                (cam_scl),
        .cam_sda                (cam_sda),

        .busy                   (cam_ctrl_busy),
        .init_done              (cam_init_done),
        .init_fail              (cam_init_fail),
        .sensor_id_ok           (cam_sensor_id_ok),
        .last_err               (cam_last_err),

        .cam_status_snap_sys    (cam_status_snap_sys),
        .cam_status_snap_upd_sys(cam_status_snap_upd_sys)
    );

    //==========================================================================
    // 4) Camera video/link receive path
    //==========================================================================
    // Placeholder canonical contract:
    //   This wrapper belongs to the adapter-specific receive side and converts
    //   physical camera link signals into pixel-domain RGB/video timing.
    //
    //   Exact port list intentionally omitted until FMC/Pcam mapping is frozen.
    //--------------------------------------------------------------------------
    wire [23:0] cam_rgb_pix;
    wire        cam_hsync_pix;
    wire        cam_vsync_pix;
    wire        cam_de_pix;
    wire        cam_frame_alive_sys;

    assign cam_rgb_pix        = 24'h000000;
    assign cam_hsync_pix      = 1'b0;
    assign cam_vsync_pix      = 1'b0;
    assign cam_de_pix         = 1'b0;
    assign cam_frame_alive_sys= 1'b0;

    //==========================================================================
    // 5) Minimal PIX-domain video generator / compositor
    //==========================================================================
    reg [11:0] h_ctr;
    reg [11:0] v_ctr;

    wire [11:0] h_total = HDMI_H_ACTIVE + HDMI_H_FP + HDMI_H_SYNC + HDMI_H_BP;
    wire [11:0] v_total = HDMI_V_ACTIVE + HDMI_V_FP + HDMI_V_SYNC + HDMI_V_BP;

    always @(posedge clk_pix) begin
        if (rst_pix) begin
            h_ctr <= 12'd0;
            v_ctr <= 12'd0;
        end else begin
            if (h_ctr == h_total - 1) begin
                h_ctr <= 12'd0;
                if (v_ctr == v_total - 1)
                    v_ctr <= 12'd0;
                else
                    v_ctr <= v_ctr + 12'd1;
            end else begin
                h_ctr <= h_ctr + 12'd1;
            end
        end
    end

    wire active_video_pix;
    assign active_video_pix = (h_ctr < HDMI_H_ACTIVE) && (v_ctr < HDMI_V_ACTIVE);

    wire hsync_pix_i;
    wire vsync_pix_i;

    assign hsync_pix_i =
        (HDMI_HSYNC_POL != 0) ?
        ~((h_ctr >= (HDMI_H_ACTIVE + HDMI_H_FP)) &&
          (h_ctr <  (HDMI_H_ACTIVE + HDMI_H_FP + HDMI_H_SYNC))) :
         ((h_ctr >= (HDMI_H_ACTIVE + HDMI_H_FP)) &&
          (h_ctr <  (HDMI_H_ACTIVE + HDMI_H_FP + HDMI_H_SYNC)));

    assign vsync_pix_i =
        (HDMI_VSYNC_POL != 0) ?
        ~((v_ctr >= (HDMI_V_ACTIVE + HDMI_V_FP)) &&
          (v_ctr <  (HDMI_V_ACTIVE + HDMI_V_FP + HDMI_V_SYNC))) :
         ((v_ctr >= (HDMI_V_ACTIVE + HDMI_V_FP)) &&
          (v_ctr <  (HDMI_V_ACTIVE + HDMI_V_FP + HDMI_V_SYNC)));

    //--------------------------------------------------------------------------
    // Base pattern: simple deterministic color bars / activity field
    //--------------------------------------------------------------------------
    reg [23:0] rgb_base_pix;

    always @(*) begin
        if (!active_video_pix) begin
            rgb_base_pix = 24'h000000;
        end else if (h_ctr < (HDMI_H_ACTIVE/3)) begin
            rgb_base_pix = 24'h400000;
        end else if (h_ctr < ((HDMI_H_ACTIVE*2)/3)) begin
            rgb_base_pix = 24'h004000;
        end else begin
            rgb_base_pix = 24'h000040;
        end
    end

    //--------------------------------------------------------------------------
    // Example overlay layers
    //--------------------------------------------------------------------------
    reg [23:0] rgb_layer0_pix;
    reg [23:0] rgb_layer1_pix;
    reg [23:0] rgb_layer2_pix;
    reg [23:0] rgb_layer3_pix;

    always @(*) begin
        rgb_layer0_pix = 24'h000000;
        rgb_layer1_pix = 24'h000000;
        rgb_layer2_pix = 24'h000000;
        rgb_layer3_pix = 24'h000000;

        // Border if HDMI not hot-plugged
        if (active_video_pix && !hdmi_hpd_in) begin
            if ((h_ctr < 8) || (h_ctr >= HDMI_H_ACTIVE-8) ||
                (v_ctr < 8) || (v_ctr >= HDMI_V_ACTIVE-8))
                rgb_layer0_pix = 24'h400000;
        end

        // Camera-init status marker
        if (active_video_pix && cam_init_done) begin
            if ((h_ctr >= 16) && (h_ctr < 64) && (v_ctr >= 16) && (v_ctr < 32))
                rgb_layer1_pix = 24'h004000;
        end else if (active_video_pix && cam_init_fail) begin
            if ((h_ctr >= 16) && (h_ctr < 64) && (v_ctr >= 16) && (v_ctr < 32))
                rgb_layer1_pix = 24'h400000;
        end else if (active_video_pix && cam_ctrl_busy) begin
            if ((h_ctr >= 16) && (h_ctr < 64) && (v_ctr >= 16) && (v_ctr < 32))
                rgb_layer1_pix = 24'h404000;
        end

        // Placeholder camera overlay region
        if (active_video_pix && cam_de_pix) begin
            rgb_layer2_pix = cam_rgb_pix;
        end
    end

    wire [23:0] rgb_final_pix;

    video_compositor u_video_compositor (
        .rgb_base  (rgb_base_pix),
        .rgb_layer0(rgb_layer0_pix),
        .rgb_layer1(rgb_layer1_pix),
        .rgb_layer2(rgb_layer2_pix),
        .rgb_layer3(rgb_layer3_pix),
        .rgb_out   (rgb_final_pix)
    );

    //==========================================================================
    // 6) TMDS encoding
    //==========================================================================
    wire [9:0] tmds_word_b;
    wire [9:0] tmds_word_g;
    wire [9:0] tmds_word_r;

    tmds_encoder u_tmds_encoder_b (
        .clk (clk_pix),
        .rst (rst_pix),
        .din (rgb_final_pix[7:0]),
        .c0  (hsync_pix_i),
        .c1  (vsync_pix_i),
        .de  (active_video_pix),
        .dout(tmds_word_b)
    );

    tmds_encoder u_tmds_encoder_g (
        .clk (clk_pix),
        .rst (rst_pix),
        .din (rgb_final_pix[15:8]),
        .c0  (1'b0),
        .c1  (1'b0),
        .de  (active_video_pix),
        .dout(tmds_word_g)
    );

    tmds_encoder u_tmds_encoder_r (
        .clk (clk_pix),
        .rst (rst_pix),
        .din (rgb_final_pix[23:16]),
        .c0  (1'b0),
        .c1  (1'b0),
        .de  (active_video_pix),
        .dout(tmds_word_r)
    );

    //==========================================================================
    // 7) TMDS serialization lanes
    //==========================================================================
    tmds_oserdes_lane u_tmds_oserdes_lane_b (
        .PixelClk (clk_pix),
        .SerialClk(clk_tmds_5x),
        .rst      (rst_tmds_5x),
        .tmds_word(tmds_word_b),
        .tmds_p   (tmds_data_p[0]),
        .tmds_n   (tmds_data_n[0])
    );

    tmds_oserdes_lane u_tmds_oserdes_lane_g (
        .PixelClk (clk_pix),
        .SerialClk(clk_tmds_5x),
        .rst      (rst_tmds_5x),
        .tmds_word(tmds_word_g),
        .tmds_p   (tmds_data_p[1]),
        .tmds_n   (tmds_data_n[1])
    );

    tmds_oserdes_lane u_tmds_oserdes_lane_r (
        .PixelClk (clk_pix),
        .SerialClk(clk_tmds_5x),
        .rst      (rst_tmds_5x),
        .tmds_word(tmds_word_r),
        .tmds_p   (tmds_data_p[2]),
        .tmds_n   (tmds_data_n[2])
    );

    //--------------------------------------------------------------------------
    // TMDS forwarded clock lane
    //--------------------------------------------------------------------------
    // For first-light integration, the pixel clock is forwarded through a
    // differential output buffer.
    //--------------------------------------------------------------------------
    OBUFDS #(
        .IOSTANDARD("TMDS_33"),
        .SLEW("FAST")
    ) u_obufds_tmds_clk (
        .I  (clk_pix),
        .O  (tmds_clk_p),
        .OB (tmds_clk_n)
    );

    //--------------------------------------------------------------------------
    // HDMI transmitter enable
    //--------------------------------------------------------------------------
    assign hdmi_tx_en = hdmi_clk_locked;

    //==========================================================================
    // 8) OLED telemetry stack
    //==========================================================================
    nexys_video_oled_telemetry_top #(
        .CLK_HZ    (CLK_HZ),
        .REFRESH_HZ(OLED_REFRESH_HZ),
        .LINE_CHARS(OLED_LINE_CHARS),
        .SPI_HZ    (OLED_SPI_HZ)
    ) u_nexys_video_oled_telemetry_top (
        .clk               (clk100),
        .rst               (rst_sys),

        .sonar1_distance_in(10'd0),
        .sonar1_valid      (1'b0),
        .sonar1_stale      (1'b0),
        .sonar1_timeout_err(1'b0),
        .sonar1_age_ticks  (16'd0),

        .sonar2_distance_in(10'd0),
        .sonar2_valid      (1'b0),
        .sonar2_stale      (1'b0),
        .sonar2_timeout_err(1'b0),
        .sonar2_age_ticks  (16'd0),

        .cam_busy          (cam_ctrl_busy),
        .cam_init_done     (cam_init_done),
        .cam_init_fail     (cam_init_fail),
        .cam_sensor_id_ok  (cam_sensor_id_ok),
        .cam_last_err      (cam_last_err),

        .sys_locked        (hdmi_clk_locked),
        .hdmi_hpd          (hdmi_hpd_in),
        .heartbeat         (cam_frame_alive_sys),
        .frame_count_lsb   ({4'd0, v_ctr}),

        .oled_res_n        (oled_res_n),
        .oled_dc           (oled_dc),
        .oled_sclk         (oled_sclk),
        .oled_sdin         (oled_sdin),
        .oled_vbat_n       (oled_vbat_n),
        .oled_vdd_n        (oled_vdd_n)
    );

endmodule

`default_nettype wire