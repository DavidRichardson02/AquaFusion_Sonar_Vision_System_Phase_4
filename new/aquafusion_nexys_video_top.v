`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// aquafusion_nexys_video_top
//------------------------------------------------------------------------------
// ROLE
//   Canonical top-level integration shell for the AquaFusion Nexys Video
//   platform.
//
// SYSTEM PURPOSE
//   Integrates:
//
//     1) dual sonar UART acquisition pipelines
//     2) camera control / status publication
//     3) SYS-domain telemetry packing
//     4) explicit SYS -> VID snapshot crossing
//     5) frame-boundary-qualified VID-domain rendering
//     6) HDMI output generation
//     7) OLED telemetry generation
//     8) CLS debug telemetry publication
//
// CENTRAL ENGINEERING DOCTRINE
//   The design follows a strict producer/publication/rendering discipline:
//
//       SYS-domain producers
//           -> stable packed snapshots / telemetry buses
//           -> explicit CDC crossing
//           -> VID-domain frame-boundary commit
//           -> purely raster-derived rendering
//
//   Therefore, no VID-domain renderer directly consumes a moving SYS-domain bus.
//
// INTEGRATION INVARIANTS
//   1) No VID-domain renderer samples unsynchronized SYS-domain state.
//   2) Multi-bit SYS->VID transfers occur only through explicit snapshot CDC.
//   3) Visible HUD state changes only on VID frame boundaries.
//   4) Each physical output has exactly one owner in this top-level.
//   5) Exactly one leaf-interface family is used in this revision.
//
// CLOCK-DOMAIN PARTITION
//   clk_sys / rst_sys_local:
//     acquisition, parsing, filtering, watchdogs, telemetry packers,
//     camera control, OLED generation, CLS publication
//
//   clk_vid / rst_vid:
//     raster timing, frame-committed telemetry consumption, HUD rendering,
//     HDMI scanout
//
// RENDERING DISCIPLINE
//   Renderers are expected to behave like functions of:
//
//       current pixel coordinate
//       + active-video qualifier
//       + frame-stable committed telemetry
//       -> current pixel color
//
// LEAF-INTERFACE FAMILY FROZEN IN THIS REVISION
//   - sonar_occupancy_painter with TELEM_MODE / hud_mode_pix interface
//   - sonar_map_renderer with MAP_W / MAP_H / DATA_W / map_rd_addr interface
//   - hud_sonar_radar_overlay_pix
//   - byte_event_sys2vid
//   - vga_uart_terminal_overlay
//   - hud_sonar_tile_rich using snap_data_pix / snap_upd_pix
//   - oled_telemetry_pack_sys
//
// NOTES
// -----
// 1) OLED and CLS paths are intentionally isolated from HDMI composition.
// 2) The bearing-publication path is architecturally complete, but the current
//    source still publishes a fixed placeholder bearing.
// 3) A safe SYS-local synthetic fallback angle is used for painter direction
//    generation whenever bearing validity is absent.
//==============================================================================

module aquafusion_nexys_video_top #(
    //--------------------------------------------------------------------------
    // Global timing
    //--------------------------------------------------------------------------
    parameter integer SYS_CLK_HZ       = 100_000_000,

    //--------------------------------------------------------------------------
    // Video timing
    //--------------------------------------------------------------------------
    parameter integer H_ACTIVE         = 640,
    parameter integer H_FP             = 16,
    parameter integer H_SYNC           = 96,
    parameter integer H_BP             = 48,
    parameter integer V_ACTIVE         = 480,
    parameter integer V_FP             = 10,
    parameter integer V_SYNC           = 2,
    parameter integer V_BP             = 33,
    parameter integer HSYNC_POL        = 0,
    parameter integer VSYNC_POL        = 0,

    //--------------------------------------------------------------------------
    // Snapshot widths
    //--------------------------------------------------------------------------
    parameter integer SONAR_SNAPSHOT_W = 64,
    parameter integer CAM_SNAPSHOT_W   = 256,

    //--------------------------------------------------------------------------
    // Sonar protocol
    //--------------------------------------------------------------------------
    parameter integer SONAR_BAUD       = 9600
)(
    //--------------------------------------------------------------------------
    // Board IO
    //--------------------------------------------------------------------------
    input  wire        sys_clk_in,
    input  wire        cpu_resetn_in,
    input  wire [7:0]  sw_in,
    input  wire [4:0]  btn_in,
    output wire [7:0]  led_out,

    //--------------------------------------------------------------------------
    // HDMI
    //--------------------------------------------------------------------------
    output wire        hdmi_tx_clk_p,
    output wire        hdmi_tx_clk_n,
    output wire [2:0]  hdmi_tx_data_p,
    output wire [2:0]  hdmi_tx_data_n,
    output wire        hdmi_tx_en,
    input  wire        hdmi_tx_hpd,

    //--------------------------------------------------------------------------
    // FMC Pcam Adapter / Pcam 5C port A
    //--------------------------------------------------------------------------
    output wire [1:0]  set_vadj,
    output wire        vadj_en,
    output wire        cam_pwup,
    inout  wire        cam_scl,
    inout  wire        cam_sda,
    output wire        cam_gpio1_oen_n,
    output wire        cam_gpio1_dir,
    output wire        cam_a_bta_o,
    input  wire        cam_a_hs_clk_p,
    input  wire        cam_a_hs_clk_n,
    input  wire        cam_a_hs_lane0_p,
    input  wire        cam_a_hs_lane0_n,
    input  wire        cam_a_hs_lane1_p,
    input  wire        cam_a_hs_lane1_n,
    input  wire        cam_a_lp_clk_p,
    input  wire        cam_a_lp_clk_n,
    input  wire        cam_a_lp_lane0_p,
    input  wire        cam_a_lp_lane0_n,
    input  wire        cam_a_lp_lane1_p,
    input  wire        cam_a_lp_lane1_n,

    //--------------------------------------------------------------------------
    // Sonars
    //--------------------------------------------------------------------------
    input  wire        sonar1_uart_i,
    input  wire        sonar1_pwm_i,
    input  wire        sonar2_uart_i,
    input  wire        sonar2_pwm_i,

    //--------------------------------------------------------------------------
    // Debug
    //--------------------------------------------------------------------------
    output wire        dbg_frame_tick,
    output wire        dbg_sonar_update,
    output wire        dbg_heartbeat,
    output wire        dbg_cam_sccb_transaction,
    output wire        dbg_cam_init_done,
    output wire        dbg_cam_overflow_event,

    //--------------------------------------------------------------------------
    // OLED
    //--------------------------------------------------------------------------
    output wire        oled_res_n,
    output wire        oled_dc,
    output wire        oled_sclk,
    output wire        oled_sdin,
    output wire        oled_vbat_n,
    output wire        oled_vdd_n,

    //--------------------------------------------------------------------------
    // CLS telemetry UART
    //--------------------------------------------------------------------------
    output wire        cls_txd_o
);

    //==========================================================================
    // Layout constants
    //--------------------------------------------------------------------------
    // Layout philosophy:
    //   - top band    : global debug + camera + compact sonar tiles
    //   - lower band  : sonar spatial views
    //   - terminal    : text diagnostics
    //==========================================================================
    localparam integer RGB_W                   = 24;
    localparam integer SONAR_RICH_SNAPSHOT_W   = 128;
    localparam integer SONAR_BUS48_W           = 48;
    localparam integer SONAR_STALE_MS          = 200;
    localparam [9:0]   SONAR_STALE_MS_U10      = SONAR_STALE_MS[9:0];
    localparam integer MAP_AW                  = 16;
    localparam integer MAP_DW                  = 8;

    localparam integer DBG_PANEL_X0            = 8;
    localparam integer DBG_PANEL_Y0            = 8;
    localparam integer DBG_PANEL_W             = 224;
    localparam integer DBG_PANEL_H             = 112;

    localparam integer CAM_PANEL_X0            = 240;
    localparam integer CAM_PANEL_Y0            = 8;
    localparam integer CAM_VIEW_X0             = 456;
    localparam integer CAM_VIEW_Y0             = 8;
    localparam integer CAM_VIEW_W              = 176;
    localparam integer CAM_VIEW_H              = 132;

    localparam integer UART_BOX_X0             = 176;
    localparam integer UART_BOX_Y0             = 188;
    localparam integer UART_BOX_COLS           = 32;
    localparam integer UART_BOX_ROWS           = 12;
    localparam integer UART_BOX_CHAR_W         = 8;
    localparam integer UART_BOX_CHAR_H         = 8;
    localparam integer UART_BOX_WPX            = UART_BOX_COLS * UART_BOX_CHAR_W;
    localparam integer UART_BOX_HPX            = UART_BOX_ROWS * UART_BOX_CHAR_H;

    localparam integer CAM_PANEL_W             = 208;
    localparam integer CAM_PANEL_H             = 112;

    localparam integer SONAR1_RICH_X0          = 8;
    localparam integer SONAR1_RICH_Y0          = 208;
    localparam integer SONAR_RICH_TILE_W       = 144;
    localparam integer SONAR_RICH_TILE_H       = 112;

    localparam integer SONAR2_BASIC_X0         = 456;
    localparam integer SONAR2_BASIC_Y0         = 8;
    localparam integer SONAR2_RICH_X0          = 456;
    localparam integer SONAR2_RICH_Y0          = 88;

    localparam integer SONAR_MAP_WPX           = 128;
    localparam integer SONAR_MAP_HPX           = 128;
    // Keep map cells 1:1 with the visible 128x128 map panel.
    // 128 mm/cell fits the 255 in sonar range inside the centered map.
    localparam integer SONAR_MAP_CELLS_W       = 128;
    localparam integer SONAR_MAP_CELLS_H       = 128;
    localparam integer SONAR_MAP_ORIGIN_X      = (SONAR_MAP_CELLS_W/2);
    localparam integer SONAR_MAP_ORIGIN_Y      = (SONAR_MAP_CELLS_H/2);
    localparam integer SONAR_MAP_CELL_MM_SHIFT = 7;
    localparam integer SONAR_MAP_MAX_RAY_STEPS = (SONAR_MAP_CELLS_W/2) - 1;
    localparam integer SONAR_MAP_FIT_X_SHIFT   = 0;
    localparam integer SONAR_MAP_FIT_Y_SHIFT   = 0;
    localparam integer SONAR1_MAP_RD_LAT       = 2;

    localparam integer SONAR_RADAR_W           = 144;
    localparam integer SONAR_RADAR_H           = 144;
    localparam integer SONAR_RADAR_RMAX        = 56;

    localparam integer SONAR1_MAP_X0           = 8;
    localparam integer SONAR1_MAP_Y0           = 344;
    localparam integer SONAR1_RADAR_X0         = 144;
    localparam integer SONAR1_RADAR_Y0         = 328;

    localparam integer SONAR2_MAP_X0           = 360;
    localparam integer SONAR2_MAP_Y0           = 344;
    localparam integer SONAR2_RADAR_X0         = 496;
    localparam integer SONAR2_RADAR_Y0         = 328;

    localparam integer SONAR1_PAINTER_PANEL_X0 = 280;
    localparam integer SONAR1_PAINTER_PANEL_W  = 160;
    localparam integer SONAR1_PAINTER_GRID_ROWS = 4;
    localparam integer SONAR1_PAINTER_GRID_COLS = 2;
    localparam integer SONAR1_PAINTER_ROW_IDX  = 3;
    localparam integer SONAR1_PAINTER_COL_IDX  = 0;
    localparam integer SONAR1_PAINTER_CELL_W   = SONAR1_PAINTER_PANEL_W / SONAR1_PAINTER_GRID_COLS;
    localparam integer SONAR1_PAINTER_CELL_H   = V_ACTIVE / SONAR1_PAINTER_GRID_ROWS;
    localparam integer SONAR1_PAINTER_X0       = SONAR1_PAINTER_PANEL_X0 +
                                                 (SONAR1_PAINTER_COL_IDX * SONAR1_PAINTER_CELL_W);
    localparam integer SONAR1_PAINTER_Y0       = SONAR1_PAINTER_ROW_IDX * SONAR1_PAINTER_CELL_H;
    localparam integer SONAR1_PAINTER_X1       = SONAR1_PAINTER_X0 + SONAR1_PAINTER_CELL_W - 1;
    localparam integer SONAR1_PAINTER_Y1       = SONAR1_PAINTER_Y0 + SONAR1_PAINTER_CELL_H - 1;

    //==========================================================================
    // Runtime HDMI auxiliary view enum
    //--------------------------------------------------------------------------
    // Sonar selector state was removed from the live top-level path. The final
    // compositor directly consumes the enabled sonar1 surfaces.
    //==========================================================================
    localparam [2:0] AUX_VIEW_TEST             = 3'd0;
    localparam [2:0] AUX_VIEW_CAMERA_ONLY      = 3'd1;
    localparam [2:0] AUX_VIEW_CAMERA_HUD       = 3'd2;
    localparam [2:0] AUX_VIEW_UART             = 3'd3;
    localparam [2:0] AUX_VIEW_MAP_DEBUG        = 3'd4;

    localparam [2:0] AUX_VIEW_RESET            = AUX_VIEW_CAMERA_HUD;

    //==========================================================================
    // Board switch assignments
    //==========================================================================
    localparam integer SW_OLED_TEST_MODE        = 0;
    localparam integer SW_SONAR1_RAW_MODE       = 1;
    localparam integer SW_SONAR1_BRUSH_MODE     = 2;

    //==========================================================================
    // Helper functions
    //--------------------------------------------------------------------------
    // These helpers are intentionally local because they represent simple,
    // reviewable integration transforms rather than reusable subsystem logic.
    //==========================================================================

    //--------------------------------------------------------------------------
    // FUNCTION: inch_u10_to_mm_u16
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Convert a whole-inch unsigned quantity into a whole-mm unsigned
    //   quantity with nearest-integer rounding.
    //
    // PHYSICAL BASIS
    //   1 inch = 25.4 mm
    //
    // INTEGER FORM
    //   mm = inch * 25.4 = inch * 254 / 10
    //
    // ROUNDING POLICY
    //   Add half the divisor before division:
    //       rounded_mm = (inch * 254 + 5) / 10
    //
    // STEP-BY-STEP
    //   1) Multiply the inch input by 254, producing tenths-of-a-mm.
    //   2) Add 5, which is half of 10, to implement nearest-integer rounding.
    //   3) Divide by 10 to return to whole millimeters.
    //
    // RANGE / SAFETY NOTE
    //   A 32-bit intermediate is used so the multiply cannot overflow for the
    //   intended sensor range.
    //--------------------------------------------------------------------------
    function [15:0] inch_u10_to_mm_u16;
        input [9:0] inch_u10;
        reg [31:0] num;
        begin
            num = (inch_u10 * 32'd254) + 32'd5;
            inch_u10_to_mm_u16 = num / 32'd10;
        end
    endfunction

    //--------------------------------------------------------------------------
    // FUNCTION: mm_u16_to_inch_u9
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Convert a whole-millimeter unsigned quantity into a whole-inch unsigned
    //   quantity with nearest-integer rounding.
    //
    // PHYSICAL BASIS
    //   1 inch = 25.4 mm
    //
    // INTEGER FORM
    //   inch = mm / 25.4 = mm * 10 / 254
    //
    // ROUNDING POLICY
    //   Add half the divisor before division:
    //       rounded_in = (mm * 10 + 127) / 254
    //
    // STEP-BY-STEP
    //   1) Multiply the mm input by 10.
    //   2) Add 127, which is half of 254 rounded down.
    //   3) Divide by 254 to obtain whole inches.
    //--------------------------------------------------------------------------
    function [8:0] mm_u16_to_inch_u9;
        input [15:0] mm_u16;
        reg [31:0] num;
        begin
            num = (mm_u16 * 32'd10) + 32'd127;
            mm_u16_to_inch_u9 = num / 32'd254;
        end
    endfunction

    //--------------------------------------------------------------------------
    // FUNCTION: deg_u9_to_q10
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Convert a degree-domain angle into a 10-bit 1024-count revolution code.
    //
    // INTEGER FORM
    //   q10 = deg * 1024 / 360
    //
    // ROUNDING POLICY
    //   Add half of 360 before division:
    //       q10 = (deg * 1024 + 180) / 360
    //
    // STEP-BY-STEP
    //   1) Scale degrees into a 1024-count turn.
    //   2) Add rounding bias.
    //   3) Divide by 360 to obtain the final code.
    //--------------------------------------------------------------------------
    function [9:0] deg_u9_to_q10;
        input [8:0] deg_u9;
        reg [19:0] num;
        begin
            num = (deg_u9 * 20'd1024) + 20'd180;
            deg_u9_to_q10 = num / 20'd360;
        end
    endfunction

    //--------------------------------------------------------------------------
    // FUNCTION: sanitize_aux_view
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Accept only legal auxiliary-view enum values. Unsupported inputs map to
    //   OFF.
    //--------------------------------------------------------------------------
    function [2:0] sanitize_aux_view;
        input [2:0] view_in;
        begin
            case (view_in)
                AUX_VIEW_TEST,
                AUX_VIEW_CAMERA_ONLY,
                AUX_VIEW_CAMERA_HUD,
                AUX_VIEW_UART,
                AUX_VIEW_MAP_DEBUG: sanitize_aux_view = view_in;
                default:       sanitize_aux_view = AUX_VIEW_TEST;
            endcase
        end
    endfunction

    //==========================================================================
    // Signal classification guide
    //--------------------------------------------------------------------------
    // *_sys
    //   Owned by SYS domain.
    //
    // *_vid / *_pix
    //   Owned by VID domain after CDC or native VID generation.
    //
    // *_upd_sys / *_upd_pix
    //   Event-style publication pulse.
    //
    // *_commit_vid
    //   One-cycle pulse indicating a frame-boundary commit into VID domain.
    //
    // *_stale_*
    //   Level status, not a pulse.
    //
    // *_sticky_*
    //   Sticky fault state, reset-cleared only.
    //
    // *_rgb444 / *_rgb888
    //   Pixel surfaces intended for later composition.
    //==========================================================================

    //==========================================================================
    // Clocks / resets
    //--------------------------------------------------------------------------
    // Consumers:
    //   global timing, CDC, all subsystems
    // Persistence:
    //   levels
    //==========================================================================
    wire clk_sys;
    wire rst_sys;
    wire sys_locked;

    wire clk_vid;
    wire rst_vid;
    wire clk_tmds_5x;
    wire rst_tmds_5x;

    //--------------------------------------------------------------------------
    // Local operational reset
    //--------------------------------------------------------------------------
    // Consumers:
    //   most SYS-domain data-plane modules
    // Persistence:
    //   level
    //==========================================================================
    wire rst_sys_local;

    //==========================================================================
    // Board-conditioned controls
    //--------------------------------------------------------------------------
    // Domain:
    //   SYS
    // Consumers:
    //   control extraction, selector commit, mode straps
    // Persistence:
    //   stable sampled levels
    //==========================================================================
    wire [7:0] sw_sys;
    wire [4:0] btn_sys;
    wire       reset_req_sys;

    //==========================================================================
    // Video timing
    //--------------------------------------------------------------------------
    // Domain:
    //   VID
    // Consumers:
    //   all raster renderers and HDMI wrapper
    // Persistence:
    //   continuously varying counters and control levels
    //==========================================================================
    wire [11:0] pix_x;
    wire [11:0] pix_y;
    wire        de;
    wire        hsync;
    wire        vsync;
    wire        frame_tick;

    //==========================================================================
    // Base and final RGB streams
    //--------------------------------------------------------------------------
    // Domain:
    //   VID
    // Consumers:
    //   compositor, HDMI wrapper
    //==========================================================================
    wire [RGB_W-1:0] rgb_debug_panel;
    wire [RGB_W-1:0] rgb_camera_debug_tile;
    wire [RGB_W-1:0] rgb_camera_bg_vid;
    wire [RGB_W-1:0] rgb_final;


    //==============================================================================
    // Dual-sonar canonical acquisition / selected-publication integration
    //------------------------------------------------------------------------------
    // PURPOSE
    //   Provide matched sonar-1 and sonar-2 integration blocks with explicit:
    //
    //     1) raw parser path
    //     2) true filter-core path
    //     3) selected downstream publication path
    //
    //   All official downstream publication surfaces consume the selected path:
    //     - watchdog
    //     - basic status snapshot
    //     - bus48 telemetry publication
    //     - mm conversion used by painters
    //     - CLS bridge (sonar 1 in current design)
    //
    //   Rich diagnostics remain dual-sourced:
    //     - raw_in  = parser output
    //     - filt_in = true filter-core output
    //
    // RUNTIME MODE POLICY
    //   SONAR_FILTER_BYPASS has highest priority.
    //   Otherwise:
    //     sw_sys[1] selects sonar-1 raw-vs-filtered published mode
    //     sw_sys[2] selects sonar-1 painter brush footprint mode
    //
    //     mode_raw = 1 -> publish raw parser output directly
    //     mode_raw = 0 -> publish true filter-core output
    //
    // IMPORTANT NAMING RULE
    //   *_filter_core
    //     actual output of sonar_filter
    //
    //   *_selected
    //     canonical downstream-published value/valid pair
    //
    //   *_filt
    //     retained only as compatibility aliases for legacy code sections still
    //     expecting historical names
    //==============================================================================
    
    //------------------------------------------------------------------------------
    // Sonar 1 acquisition-chain signals
    //------------------------------------------------------------------------------
    wire [7:0]  sonar1_rx_byte;
    wire        sonar1_rx_valid;
    wire        sonar1_rx_frame_err;
    
    wire [9:0]  sonar1_distance_in_uart;
    wire        sonar1_distance_valid_uart;
    wire        sonar1_parse_err_pulse;

    wire [7:0]  sonar1_distance_in_pwm_u8;
    wire [15:0] sonar1_distance_mm_pwm;
    wire        sonar1_distance_valid_pwm;

    wire [9:0]  sonar1_distance_in_raw_sample;
    wire        sonar1_distance_valid_pulse;
    reg  [9:0]  sonar1_distance_in_raw;
    
    wire [9:0]  sonar1_distance_in_filter_core;
    wire        sonar1_distance_valid_filter_core;
    
    wire [9:0]  sonar1_distance_in_selected;
    wire        sonar1_distance_valid_selected;
    
    wire        sonar1_mode_raw_sys;
    
    wire        sonar1_stale_sys;
    wire [15:0] sonar1_age_ticks_sys;
    wire        sonar1_timeout_err_sys;
    wire [15:0] sonar1_update_count_sys;
    
    wire [SONAR_SNAPSHOT_W-1:0] sonar1_snap_sys;
    wire                        sonar1_snap_upd_sys;
    wire [SONAR_SNAPSHOT_W-1:0] sonar1_snap_vid;
    wire                        sonar1_snap_commit_vid;
    
    wire [SONAR_RICH_SNAPSHOT_W-1:0] sonar1_rich_snap_sys;
    wire                             sonar1_rich_snap_upd_sys;
    wire [SONAR_RICH_SNAPSHOT_W-1:0] sonar1_rich_snap_vid;
    wire                             sonar1_rich_snap_commit_vid;
    
    wire [47:0] sonar1_bus48_sys;
    wire        sonar1_bus48_upd_sys;
    wire [SONAR_BUS48_W-1:0] sonar1_bus48_vid;
    wire                     sonar1_bus48_commit_vid;
    
    wire [15:0] sonar1_dist_mm_sys;
    
    // Historical compatibility aliases
    wire [9:0]  sonar1_distance_in_filt;
    wire        sonar1_distance_valid_filt;
    
    //------------------------------------------------------------------------------
    // Sonar 2 acquisition-chain signals
    //------------------------------------------------------------------------------
    wire [7:0]  sonar2_rx_byte;
    wire        sonar2_rx_valid;
    wire        sonar2_rx_frame_err;
    
    wire [9:0]  sonar2_distance_in_raw;
    wire        sonar2_distance_valid_pulse;
    wire        sonar2_parse_err_pulse;
    
    wire [9:0]  sonar2_distance_in_filter_core;
    wire        sonar2_distance_valid_filter_core;
    
    wire [9:0]  sonar2_distance_in_selected;
    wire        sonar2_distance_valid_selected;
    
    wire        sonar2_stale_sys;
    wire [15:0] sonar2_age_ticks_sys;
    wire        sonar2_timeout_err_sys;
    wire [15:0] sonar2_update_count_sys;
    
    wire [SONAR_SNAPSHOT_W-1:0] sonar2_snap_sys;
    wire                        sonar2_snap_upd_sys;
    wire [SONAR_SNAPSHOT_W-1:0] sonar2_snap_vid;
    wire                        sonar2_snap_commit_vid;
    
    wire [SONAR_RICH_SNAPSHOT_W-1:0] sonar2_rich_snap_sys;
    wire                             sonar2_rich_snap_upd_sys;
    wire [SONAR_RICH_SNAPSHOT_W-1:0] sonar2_rich_snap_vid;
    wire                             sonar2_rich_snap_commit_vid;
    
    wire [47:0] sonar2_bus48_sys;
    wire        sonar2_bus48_upd_sys;
    wire [SONAR_BUS48_W-1:0] sonar2_bus48_vid;
    wire                     sonar2_bus48_commit_vid;
    
    // Historical compatibility aliases
    wire [9:0]  sonar2_distance_in_filt;
    wire        sonar2_distance_valid_filt;
    
    //------------------------------------------------------------------------------
    // Sticky parser / UART fault state
    //------------------------------------------------------------------------------
    reg sonar1_parse_err_sticky_sys;
    reg sonar1_uart_frame_err_sticky_sys;

    reg cam_auto_start_pending_sys;
    
    //==========================================================================
    // Camera control/status
    //--------------------------------------------------------------------------
    // Domain:
    //   SYS for control plane, VID for committed status tile
    //==========================================================================
    wire                      cam_ctrl_busy_sys;
    wire                      cam_init_done_sys;
    wire                      cam_init_fail_sys;
    wire                      cam_sensor_id_ok_sys;
    wire                      cam_ready_sys;
    wire [7:0]                cam_last_err_sys;

    wire [CAM_SNAPSHOT_W-1:0] cam_status_snap_sys;
    wire                      cam_status_snap_upd_sys;
    wire [CAM_SNAPSHOT_W-1:0] cam_status_snap_vid;
    wire                      cam_status_snap_commit_vid;
    wire                      cam_rgb_valid_vid;
    wire [31:0]               cam_frame_count_sys;

    wire                      cam_init_start_sys;
    wire                      cam_auto_start_pulse_sys;
    wire [31:0]               status_word_sys;
    wire                      video_ready_vid;

    wire signed [15:0] sonar1_cos_q15_sys;
    wire signed [15:0] sonar1_sin_q15_sys;
    wire signed [15:0] sonar1_map_sin_q15_sys;
    wire signed [15:0] sonar2_cos_q15_sys;
    wire signed [15:0] sonar2_sin_q15_sys;

    //==========================================================================
    // Bearing publication path
    //--------------------------------------------------------------------------
    // Domain:
    //   SYS publication -> VID committed
    // Persistence:
    //   snapshot + update event
    //==========================================================================
    wire [8:0]  cam_reg_bearing_deg_sys;
    wire        cam_reg_bearing_valid_sys;
    wire        cam_reg_bearing_upd_sys;

    reg         cam_reg_bearing_upd_d1_sys;
    wire        cam_reg_bearing_upd_pulse_sys;

    wire [31:0] bearing_snap_sys;
    wire        bearing_snap_upd_sys;
    wire [31:0] bearing_snap_vid;
    wire        bearing_snap_commit_vid;

    wire [8:0]  cam_align_bearing_deg_vid;
    wire [9:0]  cam_align_angle_q10_vid;
    wire        cam_align_valid_vid;

    //--------------------------------------------------------------------------
    // Placeholder bearing source
    //
    // No real camera/fusion bearing is currently published in this revision.
    // Keep validity low so the SYS painter and VID radar use their fallback
    // sweep policies instead of locking all sonar rendering to 0 degrees.
    //--------------------------------------------------------------------------
    assign cam_reg_bearing_deg_sys   = 9'd0;
    assign cam_reg_bearing_valid_sys = 1'b0;
    assign cam_reg_bearing_upd_sys   = cam_init_done_sys;

    wire        sonar1_map_we_sys;
    wire [15:0] sonar1_map_addr_wr_sys;
    wire [7:0]  sonar1_map_din_sys;
    wire [63:0] sonar1_painter_telem_sys;
    wire        sonar1_painter_telem_vld_sys;

    wire        sonar2_map_we_sys;
    wire [15:0] sonar2_map_addr_wr_sys;
    wire [7:0]  sonar2_map_din_sys;
    wire [63:0] sonar2_painter_telem_sys;
    wire        sonar2_painter_telem_vld_sys;

    wire [15:0] sonar1_map_addr_rd_vid;
    wire [7:0]  sonar1_map_dout_vid;
    wire        sonar1_map_valid_vid;
    wire [11:0] sonar1_map_rgb444_vid;

    wire [15:0] sonar2_map_addr_rd_vid;
    wire [7:0]  sonar2_map_dout_vid;
    wire        sonar2_map_valid_vid;
    wire [11:0] sonar2_map_rgb444_vid;

    wire [63:0] sonar1_painter_telem_vid;
    wire        sonar1_painter_telem_commit_vid;
    wire [63:0] sonar2_painter_telem_vid;
    wire        sonar2_painter_telem_commit_vid;

    wire [63:0] sonar1_renderer_telem_vid;
    wire        sonar1_renderer_telem_upd_vid;
    wire [63:0] sonar2_renderer_telem_vid;
    wire        sonar2_renderer_telem_upd_vid;

    //==========================================================================
    // Radar overlays
    //==========================================================================
    wire [9:0]  sonar1_angle_q10_sys;
    wire [9:0]  sonar1_angle_q10_vid;

    reg  [9:0]  synth_angle_q10_sys;

    wire        sonar1_bus_stale_vid;

    wire [63:0] sonar1_radar_telem_vid;
    wire        sonar1_radar_telem_vld_vid;
    wire [63:0] sonar2_radar_telem_vid;
    wire        sonar2_radar_telem_vld_vid;

    //==========================================================================
    // Debug-panel helper signals
    //--------------------------------------------------------------------------
    // Domain:
    //   VID
    // Persistence:
    //   frame-stable levels and update toggles
    //==========================================================================
    wire [15:0] sonar_dbg_mm_vid;
    wire [8:0]  sonar_dbg_in_vid;
    wire [15:0] sonar_dbg_age_ms_vid;
    wire        sonar_dbg_stale_vid;
    wire        sonar_dbg_clk_locked_vid;
    reg         sonar_dbg_valid_vid;
    reg         sonar_dbg_update_toggle_vid;
    wire [3:0]  sonar_dbg_fault_flags_vid;

    //--------------------------------------------------------------------------
    // Current policy: debug panel displays sonar 1 committed bus48 telemetry
    //--------------------------------------------------------------------------
    assign sonar_dbg_mm_vid          = sonar1_bus48_vid[47:32];
    assign sonar_dbg_in_vid          = mm_u16_to_inch_u9(sonar_dbg_mm_vid);
    assign sonar_dbg_age_ms_vid      = {6'd0, sonar1_bus48_vid[31:22]};
    assign sonar_dbg_stale_vid       = sonar1_bus_stale_vid;
    assign sonar_dbg_clk_locked_vid  = video_ready_vid;
    assign sonar_dbg_fault_flags_vid = 4'b0000;

    //--------------------------------------------------------------------------
    // SEQUENTIAL CONTRACT: debug-panel activity latch
    //
    // STATE OWNER
    //   clk_vid / rst_vid
    //
    // STATE MEANING
    //   sonar_dbg_valid_vid:
    //     indicates at least one committed sonar1 bus48 update has occurred.
    //
    //   sonar_dbg_update_toggle_vid:
    //     toggles on each committed sonar1 bus48 update so the panel can render
    //     visible liveness.
    //
    // UPDATE RULE
    //   1) Reset clears both state elements.
    //   2) A committed bus48 update sets valid and toggles the activity bit.
    //   3) Otherwise state holds.
    //--------------------------------------------------------------------------
    always @(posedge clk_vid) begin
        if (rst_vid) begin
            sonar_dbg_valid_vid         <= 1'b0;
            sonar_dbg_update_toggle_vid <= 1'b0;
        end else if (sonar1_bus48_commit_vid) begin
            sonar_dbg_valid_vid         <= 1'b1;
            sonar_dbg_update_toggle_vid <= ~sonar_dbg_update_toggle_vid;
        end
    end

    //==========================================================================
    // Render-surface buses
    //--------------------------------------------------------------------------
    // Domain:
    //   VID
    // Consumers:
    //   RGB promotion, muxing, compositor
    //==========================================================================
    wire [11:0] rgb_sonar1_map_444;
    wire [11:0] rgb_sonar1_radar_444;
    wire [11:0] rgb_sonar1_rich_tile_444;
    wire [11:0] rgb_uart_term_444;
    wire [11:0] sonar1_map_overlay_444;
    wire [11:0] rgb_sonar1_painter_444;
    wire [23:0] rgb_camera_viewport_widget;
    wire        sonar1_painter_tile_vid;

    wire [23:0] rgb_sonar1_map_888;
    wire [23:0] rgb_sonar1_radar_888;
    wire [23:0] rgb_sonar1_rich_tile_888;
    wire [23:0] rgb_uart_term_888;
    wire [23:0] rgb_sonar1_painter_888;

    wire [23:0] rgb_test_pattern_vid;
    wire [23:0] rgb_base_vid;

    wire        debug_panel_region_vid;
    wire        camera_viewport_region_vid;
    wire        camera_debug_tile_region_vid;
    wire        uart_term_region_vid;
    wire        sonar1_rich_tile_region_vid;
    wire        sonar1_map_region_vid;
    wire        sonar1_painter_region_vid;
    wire        sonar1_radar_region_now_vid;
    wire        sonar1_radar_region_vid;

    reg         sonar1_radar_region_d1_vid;
    reg         sonar1_radar_region_d2_vid;

    wire        cam_status_ready_vid;
    wire        cam_status_frame_store_valid_vid;
    wire        cam_status_init_fail_vid;
    wire        cam_status_init_done_vid;

    assign sonar1_painter_tile_vid =
        de &&
        (pix_x >= SONAR1_PAINTER_X0[11:0]) && (pix_x <= SONAR1_PAINTER_X1[11:0]) &&
        (pix_y >= SONAR1_PAINTER_Y0[11:0]) && (pix_y <= SONAR1_PAINTER_Y1[11:0]);

    assign sonar1_painter_region_vid = sonar1_painter_tile_vid;

    assign debug_panel_region_vid =
        de &&
        (pix_x >= DBG_PANEL_X0) && (pix_x < (DBG_PANEL_X0 + DBG_PANEL_W)) &&
        (pix_y >= DBG_PANEL_Y0) && (pix_y < (DBG_PANEL_Y0 + DBG_PANEL_H));

    assign camera_viewport_region_vid =
        de &&
        (pix_x >= CAM_VIEW_X0) && (pix_x < (CAM_VIEW_X0 + CAM_VIEW_W)) &&
        (pix_y >= CAM_VIEW_Y0) && (pix_y < (CAM_VIEW_Y0 + CAM_VIEW_H));

    assign camera_debug_tile_region_vid =
        de &&
        (pix_x >= CAM_PANEL_X0) && (pix_x < (CAM_PANEL_X0 + CAM_PANEL_W)) &&
        (pix_y >= CAM_PANEL_Y0) && (pix_y < (CAM_PANEL_Y0 + CAM_PANEL_H));

    assign uart_term_region_vid =
        de &&
        (pix_x >= UART_BOX_X0) && (pix_x < (UART_BOX_X0 + UART_BOX_WPX)) &&
        (pix_y >= UART_BOX_Y0) && (pix_y < (UART_BOX_Y0 + UART_BOX_HPX));

    assign sonar1_rich_tile_region_vid =
        de &&
        (pix_x >= SONAR1_RICH_X0) && (pix_x < (SONAR1_RICH_X0 + SONAR_RICH_TILE_W)) &&
        (pix_y >= SONAR1_RICH_Y0) && (pix_y < (SONAR1_RICH_Y0 + SONAR_RICH_TILE_H));

    assign sonar1_map_region_vid =
        de &&
        (pix_x >= SONAR1_MAP_X0) &&
        (pix_x <  (SONAR1_MAP_X0 + SONAR_MAP_WPX)) &&
        (pix_y >= SONAR1_MAP_Y0) &&
        (pix_y <  (SONAR1_MAP_Y0 + SONAR_MAP_HPX));

    assign sonar1_radar_region_now_vid =
        de &&
        (pix_x >= SONAR1_RADAR_X0) && (pix_x < (SONAR1_RADAR_X0 + SONAR_RADAR_W)) &&
        (pix_y >= SONAR1_RADAR_Y0) && (pix_y < (SONAR1_RADAR_Y0 + SONAR_RADAR_H));

    always @(posedge clk_vid) begin
        if (rst_vid) begin
            sonar1_radar_region_d1_vid <= 1'b0;
            sonar1_radar_region_d2_vid <= 1'b0;
        end else begin
            sonar1_radar_region_d1_vid <= sonar1_radar_region_now_vid;
            sonar1_radar_region_d2_vid <= sonar1_radar_region_d1_vid;
        end
    end

    assign sonar1_radar_region_vid = sonar1_radar_region_d2_vid;

    assign rgb_test_pattern_vid =
        (!de) ? 24'h000000 :
        (((pix_x[5:0] == 6'd0) || (pix_y[5:0] == 6'd0)) ? 24'h404040 :
         {pix_x[7:0], pix_y[7:0], (pix_x[7:0] ^ pix_y[7:0])});

    assign cam_status_ready_vid             = cam_status_snap_vid[15];
    assign cam_status_frame_store_valid_vid = cam_status_snap_vid[14];
    assign cam_status_init_fail_vid         = cam_status_snap_vid[12];
    assign cam_status_init_done_vid         = cam_status_snap_vid[10];


    //--------------------------------------------------------------------------
    // Sonar 2 disabled tie-offs
    //--------------------------------------------------------------------------
    // All active sonar2 instantiations are intentionally commented out in this
    // revision. These constants keep the remaining top-level status, selector,
    // OLED, and compositor glue deterministic while sonar1 remains live.
    //--------------------------------------------------------------------------
    assign sonar2_rx_byte                    = 8'd0;
    assign sonar2_rx_valid                   = 1'b0;
    assign sonar2_rx_frame_err               = 1'b0;
    assign sonar2_distance_in_raw            = 10'd0;
    assign sonar2_distance_valid_pulse       = 1'b0;
    assign sonar2_parse_err_pulse            = 1'b0;
    assign sonar2_distance_in_filter_core    = 10'd0;
    assign sonar2_distance_valid_filter_core = 1'b0;
    assign sonar2_distance_in_selected       = 10'd0;
    assign sonar2_distance_valid_selected    = 1'b0;
    assign sonar2_distance_in_filt           = 10'd0;
    assign sonar2_distance_valid_filt        = 1'b0;

    assign sonar2_stale_sys                  = 1'b0;
    assign sonar2_age_ticks_sys              = 16'd0;
    assign sonar2_timeout_err_sys            = 1'b0;
    assign sonar2_update_count_sys           = 16'd0;

    assign sonar2_snap_sys                   = {SONAR_SNAPSHOT_W{1'b0}};
    assign sonar2_snap_upd_sys               = 1'b0;
    assign sonar2_snap_vid                   = {SONAR_SNAPSHOT_W{1'b0}};
    assign sonar2_snap_commit_vid            = 1'b0;

    assign sonar2_rich_snap_sys              = {SONAR_RICH_SNAPSHOT_W{1'b0}};
    assign sonar2_rich_snap_upd_sys          = 1'b0;
    assign sonar2_rich_snap_vid              = {SONAR_RICH_SNAPSHOT_W{1'b0}};
    assign sonar2_rich_snap_commit_vid       = 1'b0;

    assign sonar2_bus48_sys                  = 48'd0;
    assign sonar2_bus48_upd_sys              = 1'b0;
    assign sonar2_bus48_vid                  = {SONAR_BUS48_W{1'b0}};
    assign sonar2_bus48_commit_vid           = 1'b0;

    assign sonar2_cos_q15_sys                = 16'sd0;
    assign sonar2_sin_q15_sys                = 16'sd0;

    assign sonar2_map_we_sys                 = 1'b0;
    assign sonar2_map_addr_wr_sys            = 16'd0;
    assign sonar2_map_din_sys                = 8'd0;
    assign sonar2_painter_telem_sys          = 64'd0;
    assign sonar2_painter_telem_vld_sys      = 1'b0;

    assign sonar2_map_addr_rd_vid            = 16'd0;
    assign sonar2_map_dout_vid               = 8'd0;
    assign sonar2_map_valid_vid              = 1'b0;
    assign sonar2_map_rgb444_vid             = 12'h000;

    assign sonar2_painter_telem_vid          = 64'd0;
    assign sonar2_painter_telem_commit_vid   = 1'b0;
    assign sonar2_renderer_telem_vid         = 64'd0;
    assign sonar2_renderer_telem_upd_vid     = 1'b0;
    assign sonar2_radar_telem_vid            = 64'd0;
    assign sonar2_radar_telem_vld_vid        = 1'b0;

    //==========================================================================
    // UART terminal CDC
    //==========================================================================
    wire [7:0] uart_evt_byte_vid;
    wire       uart_evt_vld_vid;
    wire       uart_evt_busy_sys;

    //==========================================================================
    // Runtime view selector state
    //--------------------------------------------------------------------------
    // Domain:
    //   SYS for storage, VID after committed snapshot
    //==========================================================================
    wire [2:0] view_code_sys;
    wire [2:0] aux_view_code_sys;
    reg        btn_aux_d1_sys;

    wire btn_load_aux_rise_sys;

    reg  [2:0] aux_view_sel_sys;
    reg        aux_view_cfg_upd_sys;
    reg        aux_view_boot_pending_sys;
    wire [2:0] aux_view_sel_vid;

    //==========================================================================
    // Full-overlay compositor stages
    //==========================================================================
    wire [23:0] rgb_stage0;
    wire [23:0] rgb_stage1;
    wire [23:0] rgb_stage2;
    wire [23:0] rgb_stage3;
    wire [23:0] rgb_stage4;
    wire [23:0] rgb_stage5;
    wire [23:0] rgb_stage6;
    wire [23:0] rgb_stage7;
    wire [23:0] rgb_stage8;

    //==========================================================================
    // Frame counters and VID->SYS mirror
    //--------------------------------------------------------------------------
    // frame_count_vid:
    //   native VID-domain counter
    //
    // frame_count_sys:
    //   synchronized SYS-domain mirror using toggle-based CDC
    //==========================================================================
    reg  [15:0] frame_count_vid;
    reg  [15:0] frame_count_sys;

    reg         frame_tick_toggle_vid;
    reg  [1:0]  frame_tick_toggle_sync_sys;
    wire        frame_tick_pulse_sys;

    //==========================================================================
    // OLED text pipeline
    //--------------------------------------------------------------------------
    // Domain:
    //   SYS
    // Consumers:
    //   OLED console and controller
    //==========================================================================
    wire [167:0] oled_line0_ascii;
    wire [167:0] oled_line1_ascii;
    wire [167:0] oled_line2_ascii;
    wire [167:0] oled_line3_ascii;
    wire         oled_line_upd;

    wire [8:0]   oled_byte_addr;
    wire [7:0]   oled_byte_data_text;
    wire [7:0]   oled_byte_data_test;
    wire [7:0]   oled_byte_data_mux;
    wire         oled_test_mode_sys;

    //==========================================================================
    // Basic derived wires
    //--------------------------------------------------------------------------
    // This section contains only low-complexity top-level glue.
    //==========================================================================
    assign rst_sys_local            = rst_sys | reset_req_sys;
    assign cam_auto_start_pulse_sys = cam_auto_start_pending_sys & ~rst_sys_local;
    assign cam_init_start_sys       = cam_auto_start_pulse_sys | btn_sys[1];

    assign view_code_sys            = sw_sys[7:5];
    assign aux_view_code_sys        = sanitize_aux_view(view_code_sys);

    assign btn_load_aux_rise_sys    = btn_sys[4] & ~btn_aux_d1_sys;

    assign oled_test_mode_sys       = sw_sys[SW_OLED_TEST_MODE];

    //--------------------------------------------------------------------------
    // Bearing snapshot bit contract
    //   [31:12] reserved / zero
    //   [11]    valid
    //   [10:2]  bearing in degrees
    //   [1:0]   reserved / zero
    //--------------------------------------------------------------------------
    assign bearing_snap_sys          = {20'd0, cam_reg_bearing_valid_sys, cam_reg_bearing_deg_sys, 2'b00};
    assign bearing_snap_upd_sys      = cam_reg_bearing_upd_pulse_sys;

    assign cam_align_valid_vid       = bearing_snap_vid[11];
    assign cam_align_bearing_deg_vid = bearing_snap_vid[10:2];
    assign cam_align_angle_q10_vid   = deg_u9_to_q10(cam_align_bearing_deg_vid);

    assign sonar1_bus_stale_vid      = (sonar1_bus48_vid[31:22] >= SONAR_STALE_MS_U10);
    assign cam_frame_count_sys       = cam_status_snap_sys[159:128];

    assign video_ready_vid           = ~rst_vid;
    // Painter map rows increase downward; invert Y so map bearings match radar.
    assign sonar1_map_sin_q15_sys    = (sonar1_sin_q15_sys == 16'sh8000) ?
                                       16'sh7FFF : -sonar1_sin_q15_sys;

    always @(posedge clk_sys) begin
        if (rst_sys_local)
            cam_auto_start_pending_sys <= 1'b1;
        else if (cam_auto_start_pending_sys)
            cam_auto_start_pending_sys <= 1'b0;
    end

    //--------------------------------------------------------------------------
    // VID-side radar angle policy
    //   preferred source: committed bearing snapshot
    //   fallback source : frame-driven synthetic sweep
    //--------------------------------------------------------------------------
    assign sonar1_angle_q10_vid      = cam_align_valid_vid ?
                                       cam_align_angle_q10_vid :
                                       {frame_count_vid[7:0], 2'b00};

    assign rgb_sonar1_map_444        = sonar1_map_valid_vid ? sonar1_map_rgb444_vid : 12'h000;

    assign oled_byte_data_mux        = (oled_test_mode_sys != 1'b0) ? oled_byte_data_test
                                                                    : oled_byte_data_text;

    assign frame_tick_pulse_sys      = frame_tick_toggle_sync_sys[1] ^ frame_tick_toggle_sync_sys[0];

    //==========================================================================
    // 1) System clock/reset conditioning
    //==========================================================================
    clk_reset_mgr #(
        .SYS_CLK_HZ(SYS_CLK_HZ)
    ) u_clk_reset_mgr (
        .sys_clk_in (sys_clk_in),
        .resetn_in  (cpu_resetn_in),
        .clk_sys    (clk_sys),
        .rst_sys    (rst_sys),
        .locked     (sys_locked)
    );

    //==========================================================================
    // 2) Board input conditioning
    //==========================================================================
    board_io_ctrl u_board_io_ctrl (
        .clk_sys       (clk_sys),
        .rst_sys       (rst_sys),
        .sw_in         (sw_in),
        .btn_in        (btn_in),
        .sw_sys        (sw_sys),
        .btn_sys       (btn_sys),
        .reset_req_sys (reset_req_sys)
    );

    //==========================================================================
    // 3) Status-word assembly for LED publication
    //==========================================================================
    assign status_word_sys[0]     = sys_locked;
    assign status_word_sys[1]     = sonar1_stale_sys;
    assign status_word_sys[2]     = sonar1_timeout_err_sys;
    assign status_word_sys[3]     = sonar1_rx_frame_err;
    assign status_word_sys[4]     = sonar2_stale_sys;
    assign status_word_sys[5]     = sonar2_timeout_err_sys;
    assign status_word_sys[6]     = sonar2_rx_frame_err;
    assign status_word_sys[7]     = cam_ctrl_busy_sys;
    assign status_word_sys[15:8]  = cam_last_err_sys;
    assign status_word_sys[23:16] = sonar1_update_count_sys[7:0];
    assign status_word_sys[31:24] = sonar2_update_count_sys[7:0];

    //==========================================================================
    // 4) LED publisher
    //==========================================================================
    led_status_mux u_led_status_mux (
        .clk_sys     (clk_sys),
        .rst_sys     (rst_sys_local),
        .status_word (status_word_sys),
        .led_out     (led_out)
    );

    //==========================================================================
    // 5) Heartbeat
    //==========================================================================
    heartbeat_gen #(
        .CLK_HZ    (SYS_CLK_HZ),
        .TOGGLE_HZ (2)
    ) u_heartbeat_gen (
        .clk         (clk_sys),
        .rst         (rst_sys_local),
        .heartbeat_o (dbg_heartbeat)
    );

    //==========================================================================
    // 6) Video timing
    //==========================================================================
    video_timing_core #(
        .H_ACTIVE  (H_ACTIVE),
        .H_FP      (H_FP),
        .H_SYNC    (H_SYNC),
        .H_BP      (H_BP),
        .V_ACTIVE  (V_ACTIVE),
        .V_FP      (V_FP),
        .V_SYNC    (V_SYNC),
        .V_BP      (V_BP),
        .HSYNC_POL (HSYNC_POL),
        .VSYNC_POL (VSYNC_POL)
    ) u_video_timing_core (
        .clk_vid    (clk_vid),
        .rst_vid    (rst_vid),
        .pix_x      (pix_x),
        .pix_y      (pix_y),
        .de         (de),
        .hsync      (hsync),
        .vsync      (vsync),
        .frame_tick (frame_tick)
    );

    //==========================================================================
    // 7) Base pattern
    //--------------------------------------------------------------------------
    // Present policy: black base. A future test-pattern generator may replace
    // this constant without changing compositor structure.
    //==========================================================================
    //
    /*
    test_pattern_gen u_test_pattern_gen (
        .pix_x   (pix_x),
        .pix_y   (pix_y),
        .de      (de),
        .rgb_out (rgb_test)
    );
    //*/

    //==========================================================================
    // 8) Rich numeric debug panel
    //==========================================================================
    hud_debug_panel_rich_numeric #(
        .PANEL_X0              (DBG_PANEL_X0),
        .PANEL_Y0              (DBG_PANEL_Y0),
        .PANEL_W               (DBG_PANEL_W),
        .PANEL_H               (DBG_PANEL_H),
        .TEXT_REGISTER_SELECT  (0),
        .TEXT_REGISTER_GLYPH   (0)
    ) u_hud_debug_panel_rich_numeric (
        .clk_pix              (clk_vid),
        .rst_pix              (rst_vid),
        .pix_x                (pix_x),
        .pix_y                (pix_y),
        .de                   (de),
        .clk_locked           (sonar_dbg_clk_locked_vid),
        .sonar_valid          (sonar_dbg_valid_vid),
        .sonar_stale          (sonar_dbg_stale_vid),
        .sonar_age_ms         (sonar_dbg_age_ms_vid),
        .sonar_distance_in    (sonar_dbg_in_vid),
        .sonar_update_toggle  (sonar_dbg_update_toggle_vid),
        .frame_ctr_lsb        (frame_count_vid[7:0]),
        .fault_flags          (sonar_dbg_fault_flags_vid),
        .rgb_out              (rgb_debug_panel)
    );

    //==========================================================================
    // 9) Sonar 1 acquisition chain
    //--------------------------------------------------------------------------
    // PIPELINE
    //   UART RX -> parse -> filter -> watchdog -> SYS snapshot -> SYS->VID CDC
    //==========================================================================
    //==========================================================================
    // Sonar 1 acquisition / parse / filter / selected-publication pipeline
    //------------------------------------------------------------------------------
    // PURPOSE
    //   Provide a deterministic runtime-selectable publication boundary for
    //   sonar 1.
    //
    // PIPELINE LAYERS
    //   1) Raw parser path
    //        sonar1_distance_in_raw
    //        sonar1_distance_valid_pulse
    //
    //   2) True filter-core path
    //        sonar1_distance_in_filter_core
    //        sonar1_distance_valid_filter_core
    //
    //   3) Selected published path
    //        sonar1_distance_in_selected
    //        sonar1_distance_valid_selected
    //
    // RUNTIME MODE POLICY
    //   sonar1_mode_raw_sys = 1
    //     publish raw parser output directly
    //
    //   sonar1_mode_raw_sys = 0
    //     publish true filter-core output
    //
    // ENGINEERING INTENT
    //   This makes the raw-vs-filtered boundary explicit and testable without
    //   disturbing the downstream watchdog / snapshot / CDC / renderer structure.
    //
    // IMPORTANT DIAGNOSTIC RULE
    //   The rich snapshot path should continue to receive:
    //     - raw value from parser
    //     - true filter-core value from sonar_filter
    //
    //   so that the rendered rich tile can still expose both paths even while the
    //   official downstream publication path is switched to raw mode.
    //==========================================================================
    
    //------------------------------------------------------------------------------
    // Runtime publication-mode selection
    //------------------------------------------------------------------------------
    // sonar1:
    //   SW_SONAR1_RAW_MODE = 1 -> raw selected publication
    //   SW_SONAR1_RAW_MODE = 0 -> filtered selected publication
    //
    // Compile-time SONAR_FILTER_BYPASS forces raw mode for sonar1.
    //------------------------------------------------------------------------------
`ifdef SONAR_FILTER_BYPASS
    assign sonar1_mode_raw_sys = 1'b1;
`else
    assign sonar1_mode_raw_sys = sw_sys[SW_SONAR1_RAW_MODE];
`endif
   
   
    //==============================================================================
    // Sonar 1
    //------------------------------------------------------------------------------
    // Accept either the MaxSonar ASCII UART stream or the PWM pulse-width output.
    // UART wins only on the rare cycle where both sources publish together.
    //==============================================================================
    sonar_uart_rx #(
        .CLK_HZ (SYS_CLK_HZ),
        .BAUD   (SONAR_BAUD)
    ) u_sonar1_uart_rx (
        .clk       (clk_sys),
        .rst       (rst_sys_local),
        .rx_i      (sonar1_uart_i),
        .rx_byte   (sonar1_rx_byte),
        .rx_valid  (sonar1_rx_valid),
        .frame_err (sonar1_rx_frame_err)
    );
     
    sonar_frame_parser u_sonar1_frame_parser (
        .clk                  (clk_sys),
        .rst                  (rst_sys_local),
        .rx_byte              (sonar1_rx_byte),
        .rx_valid             (sonar1_rx_valid),
        .distance_in_raw      (sonar1_distance_in_uart),
        .distance_valid_pulse (sonar1_distance_valid_uart),
        .parse_err_pulse      (sonar1_parse_err_pulse)
    );

    maxsonar_pwm_capture #(
        .CLK_HZ (SYS_CLK_HZ)
    ) u_sonar1_pwm_capture (
        .clk        (clk_sys),
        .rst        (rst_sys_local),
        .pwm_in     (sonar1_pwm_i),
        .dist_in    (sonar1_distance_in_pwm_u8),
        .dist_mm    (sonar1_distance_mm_pwm),
        .dist_valid (sonar1_distance_valid_pwm)
    );

    assign sonar1_distance_valid_pulse =
        sonar1_distance_valid_uart | sonar1_distance_valid_pwm;

    assign sonar1_distance_in_raw_sample =
        sonar1_distance_valid_uart ? sonar1_distance_in_uart :
        sonar1_distance_valid_pwm  ? {2'd0, sonar1_distance_in_pwm_u8} :
                                     sonar1_distance_in_uart;

    always @(posedge clk_sys) begin
        if (rst_sys_local) begin
            sonar1_distance_in_raw <= 10'd0;
        end else if (sonar1_distance_valid_uart) begin
            sonar1_distance_in_raw <= sonar1_distance_in_uart;
        end else if (sonar1_distance_valid_pwm) begin
            sonar1_distance_in_raw <= {2'd0, sonar1_distance_in_pwm_u8};
        end
    end
    
    sonar_filter u_sonar1_filter (
        .clk                  (clk_sys),
        .rst                  (rst_sys_local),
        .distance_in_raw      (sonar1_distance_in_raw_sample),
        .distance_valid_pulse (sonar1_distance_valid_pulse),
        .distance_out_filt    (sonar1_distance_in_filter_core),
        .distance_valid_out   (sonar1_distance_valid_filter_core)
    );
    
    // Selected downstream publication path
    assign sonar1_distance_in_selected =
        (sonar1_mode_raw_sys != 1'b0) ? sonar1_distance_in_raw_sample
                                      : sonar1_distance_in_filter_core;
    
    assign sonar1_distance_valid_selected =
        (sonar1_mode_raw_sys != 1'b0) ? sonar1_distance_valid_pulse
                                      : sonar1_distance_valid_filter_core;
    
    // Compatibility aliases
    assign sonar1_distance_in_filt    = sonar1_distance_in_selected;
    assign sonar1_distance_valid_filt = sonar1_distance_valid_selected;
    
    // Sticky fault accumulation
    always @(posedge clk_sys) begin
        if (rst_sys_local) begin
            sonar1_parse_err_sticky_sys      <= 1'b0;
            sonar1_uart_frame_err_sticky_sys <= 1'b0;
        end else begin
            if (sonar1_parse_err_pulse)
                sonar1_parse_err_sticky_sys <= 1'b1;
    
            if (sonar1_rx_frame_err)
                sonar1_uart_frame_err_sticky_sys <= 1'b1;
        end
    end
    
    // Official downstream state/update path
    sonar_watchdog #(
        .CLK_HZ   (SYS_CLK_HZ),
        .STALE_MS (SONAR_STALE_MS)
    ) u_sonar1_watchdog (
        .clk                  (clk_sys),
        .rst                  (rst_sys_local),
        .distance_valid_pulse (sonar1_distance_valid_selected),
        .stale                (sonar1_stale_sys),
        .age_ticks            (sonar1_age_ticks_sys),
        .timeout_err          (sonar1_timeout_err_sys),
        .update_count         (sonar1_update_count_sys)
    );
    
    sonar_status_snapshot_sys #(
        .SNAP_W           (SONAR_SNAPSHOT_W),
        .AGE_BUCKET_SHIFT (4)
    ) u_sonar1_status_snapshot_sys (
        .clk                  (clk_sys),
        .rst                  (rst_sys_local),
        .distance_in          (sonar1_distance_in_selected),
        .distance_valid       (sonar1_distance_valid_selected),
        .stale                (sonar1_stale_sys),
        .timeout_err          (sonar1_timeout_err_sys),
        .parse_err_sticky_set (sonar1_parse_err_pulse),
        .age_ticks            (sonar1_age_ticks_sys),
        .update_count         (sonar1_update_count_sys),
        .snap_data            (sonar1_snap_sys),
        .snap_upd             (sonar1_snap_upd_sys)
    );
    
    snapshot_sys2vid #(
        .W (SONAR_SNAPSHOT_W)
    ) u_snapshot_sys2vid_sonar1 (
        .clk_src            (clk_sys),
        .rst_src            (rst_sys_local),
        .snap_src           (sonar1_snap_sys),
        .snap_upd_src       (sonar1_snap_upd_sys),
        .clk_dst            (clk_vid),
        .rst_dst            (rst_vid),
        .frame_tick_dst     (frame_tick),
        .snap_dst_committed (sonar1_snap_vid),
        .commit_pulse_dst   (sonar1_snap_commit_vid)
    );
    
    // Rich diagnostics remain dual-sourced
    sonar_snapshot_pack_rich_sys #(
        .CLK_HZ (SYS_CLK_HZ),
        .SNAP_W (SONAR_RICH_SNAPSHOT_W),
        .SRC_ID (2'd0)
    ) u_sonar1_snapshot_pack_rich_sys (
        .clk_sys          (clk_sys),
        .rst_sys          (rst_sys_local),
        .raw_in           (sonar1_distance_in_raw),
        .filt_in          (sonar1_distance_in_filter_core),
        .stale            (sonar1_stale_sys),
        .timeout_err      (sonar1_timeout_err_sys),
        .parse_err_sticky (sonar1_parse_err_sticky_sys),
        .age_ticks        (sonar1_age_ticks_sys),
        .update_count     ({16'd0, sonar1_update_count_sys}),
        .snap_data        (sonar1_rich_snap_sys),
        .snap_upd         (sonar1_rich_snap_upd_sys)
    );
    
    snapshot_sys2vid #(
        .W (SONAR_RICH_SNAPSHOT_W)
    ) u_snapshot_sys2vid_sonar1_rich (
        .clk_src            (clk_sys),
        .rst_src            (rst_sys_local),
        .snap_src           (sonar1_rich_snap_sys),
        .snap_upd_src       (sonar1_rich_snap_upd_sys),
        .clk_dst            (clk_vid),
        .rst_dst            (rst_vid),
        .frame_tick_dst     (frame_tick),
        .snap_dst_committed (sonar1_rich_snap_vid),
        .commit_pulse_dst   (sonar1_rich_snap_commit_vid)
    );
    
    // Official telemetry/publication bus uses selected path
    sonar_bus48_pack_sys #(
        .CLK_HZ (SYS_CLK_HZ)
    ) u_sonar1_bus48_pack_sys (
        .clk_sys         (clk_sys),
        .rst_sys         (rst_sys_local),
        .filt_in_inch    (sonar1_distance_in_selected),
        .sample_vld      (sonar1_distance_valid_selected),
        .stale           (sonar1_stale_sys),
        .age_ticks       (sonar1_age_ticks_sys),
        .no_target_evt   (1'b0),
        .sonar_bus48     (sonar1_bus48_sys),
        .sonar_bus48_upd (sonar1_bus48_upd_sys)
    );
    
    // Official derived physical quantity uses selected path
    assign sonar1_dist_mm_sys = inch_u10_to_mm_u16(sonar1_distance_in_selected);
    
    //==============================================================================
    // Sonar 2
    //==============================================================================
    /*
    sonar_uart_rx #(
        .CLK_HZ (SYS_CLK_HZ),
        .BAUD   (SONAR_BAUD)
    ) u_sonar2_uart_rx (
        .clk       (clk_sys),
        .rst       (rst_sys_local),
        .rx_i      (sonar2_uart_i),
        .rx_byte   (sonar2_rx_byte),
        .rx_valid  (sonar2_rx_valid),
        .frame_err (sonar2_rx_frame_err)
    );
    
    sonar_frame_parser u_sonar2_frame_parser (
        .clk                  (clk_sys),
        .rst                  (rst_sys_local),
        .rx_byte              (sonar2_rx_byte),
        .rx_valid             (sonar2_rx_valid),
        .distance_in_raw      (sonar2_distance_in_raw),
        .distance_valid_pulse (sonar2_distance_valid_pulse),
        .parse_err_pulse      (sonar2_parse_err_pulse)
    );
    
    sonar_filter u_sonar2_filter (
        .clk                  (clk_sys),
        .rst                  (rst_sys_local),
        .distance_in_raw      (sonar2_distance_in_raw),
        .distance_valid_pulse (sonar2_distance_valid_pulse),
        .distance_out_filt    (sonar2_distance_in_filter_core),
        .distance_valid_out   (sonar2_distance_valid_filter_core)
    );
    */
    
    // Official downstream state/update path
    /*
    sonar_watchdog #(
        .CLK_HZ   (SYS_CLK_HZ),
        .STALE_MS (200)
    ) u_sonar2_watchdog (
        .clk                  (clk_sys),
        .rst                  (rst_sys_local),
        .distance_valid_pulse (sonar2_distance_valid_selected),
        .stale                (sonar2_stale_sys),
        .age_ticks            (sonar2_age_ticks_sys),
        .timeout_err          (sonar2_timeout_err_sys),
        .update_count         (sonar2_update_count_sys)
    );
    
    sonar_status_snapshot_sys #(
        .SNAP_W           (SONAR_SNAPSHOT_W),
        .AGE_BUCKET_SHIFT (4)
    ) u_sonar2_status_snapshot_sys (
        .clk                  (clk_sys),
        .rst                  (rst_sys_local),
        .distance_in          (sonar2_distance_in_selected),
        .distance_valid       (sonar2_distance_valid_selected),
        .stale                (sonar2_stale_sys),
        .timeout_err          (sonar2_timeout_err_sys),
        .parse_err_sticky_set (sonar2_parse_err_pulse),
        .age_ticks            (sonar2_age_ticks_sys),
        .update_count         (sonar2_update_count_sys),
        .snap_data            (sonar2_snap_sys),
        .snap_upd             (sonar2_snap_upd_sys)
    );
    
    snapshot_sys2vid #(
        .W (SONAR_SNAPSHOT_W)
    ) u_snapshot_sys2vid_sonar2 (
        .clk_src            (clk_sys),
        .rst_src            (rst_sys_local),
        .snap_src           (sonar2_snap_sys),
        .snap_upd_src       (sonar2_snap_upd_sys),
        .clk_dst            (clk_vid),
        .rst_dst            (rst_vid),
        .frame_tick_dst     (frame_tick),
        .snap_dst_committed (sonar2_snap_vid),
        .commit_pulse_dst   (sonar2_snap_commit_vid)
    );
    
    // Rich diagnostics remain dual-sourced
    sonar_snapshot_pack_rich_sys #(
        .CLK_HZ (SYS_CLK_HZ),
        .SNAP_W (SONAR_RICH_SNAPSHOT_W),
        .SRC_ID (2'd1)
    ) u_sonar2_snapshot_pack_rich_sys (
        .clk_sys          (clk_sys),
        .rst_sys          (rst_sys_local),
        .raw_in           (sonar2_distance_in_raw),
        .filt_in          (sonar2_distance_in_filter_core),
        .stale            (sonar2_stale_sys),
        .timeout_err      (sonar2_timeout_err_sys),
        .parse_err_sticky (sonar2_parse_err_sticky_sys),
        .age_ticks        (sonar2_age_ticks_sys),
        .update_count     ({16'd0, sonar2_update_count_sys}),
        .snap_data        (sonar2_rich_snap_sys),
        .snap_upd         (sonar2_rich_snap_upd_sys)
    );
    
    snapshot_sys2vid #(
        .W (SONAR_RICH_SNAPSHOT_W)
    ) u_snapshot_sys2vid_sonar2_rich (
        .clk_src            (clk_sys),
        .rst_src            (rst_sys_local),
        .snap_src           (sonar2_rich_snap_sys),
        .snap_upd_src       (sonar2_rich_snap_upd_sys),
        .clk_dst            (clk_vid),
        .rst_dst            (rst_vid),
        .frame_tick_dst     (frame_tick),
        .snap_dst_committed (sonar2_rich_snap_vid),
        .commit_pulse_dst   (sonar2_rich_snap_commit_vid)
    );
    
    // Official telemetry/publication bus uses selected path
    sonar_bus48_pack_sys #(
        .CLK_HZ (SYS_CLK_HZ)
    ) u_sonar2_bus48_pack_sys (
        .clk_sys         (clk_sys),
        .rst_sys         (rst_sys_local),
        .filt_in_inch    (sonar2_distance_in_selected),
        .sample_vld      (sonar2_distance_valid_selected),
        .stale           (sonar2_stale_sys),
        .age_ticks       (sonar2_age_ticks_sys),
        .no_target_evt   (1'b0),
        .sonar_bus48     (sonar2_bus48_sys),
        .sonar_bus48_upd (sonar2_bus48_upd_sys)
    );
    */
    
    //==============================================================================
    // Minimal matched downstream-consumer guidance
    //------------------------------------------------------------------------------
    // 1) The live painter consumes:
    //      sonar1_dist_mm_sys / sonar1_distance_valid_selected
    //
    // 2) Any fallback-angle or debug-liveness logic should use selected-valid
    //    explicitly, not hidden legacy alias names.
    //
    // 3) dbg_sonar_update follows the live sonar1 publication pulse only.
    //==============================================================================
    assign dbg_sonar_update = sonar1_bus48_upd_sys;

    //==========================================================================
    // 11) Camera control/status subsystem
    //==========================================================================
    camera_top #(
        .CLK_HZ    (SYS_CLK_HZ),
        .SCL_HZ    (400_000),
        .CAM_MODE  (0),
        .CAM_FORMAT(0),
        .STORE_W   (H_ACTIVE),
        .STORE_H   (V_ACTIVE),
        .H_TOTAL   (H_ACTIVE + H_FP + H_SYNC + H_BP),
        .V_TOTAL   (V_ACTIVE + V_FP + V_SYNC + V_BP),
        .SNAP_W    (CAM_SNAPSHOT_W)
    ) u_camera_top (
        .clk_sys                 (clk_sys),
        .rst_sys                 (rst_sys_local),
        .start                   (cam_init_start_sys),
        .clk_vid                 (clk_vid),
        .rst_vid                 (rst_vid),
        .pix_x_vid               (pix_x),
        .pix_y_vid               (pix_y),
        .de_vid                  (de),
        .frame_tick_vid          (frame_tick),
        .set_vadj                (set_vadj),
        .vadj_en                 (vadj_en),
        .cam_pwup                (cam_pwup),
        .cam_scl                 (cam_scl),
        .cam_sda                 (cam_sda),
        .cam_gpio1_oen_n         (cam_gpio1_oen_n),
        .cam_gpio1_dir           (cam_gpio1_dir),
        .cam_a_hs_clk_p          (cam_a_hs_clk_p),
        .cam_a_hs_clk_n          (cam_a_hs_clk_n),
        .cam_a_hs_lane0_p        (cam_a_hs_lane0_p),
        .cam_a_hs_lane0_n        (cam_a_hs_lane0_n),
        .cam_a_hs_lane1_p        (cam_a_hs_lane1_p),
        .cam_a_hs_lane1_n        (cam_a_hs_lane1_n),
        .cam_a_lp_clk_p          (cam_a_lp_clk_p),
        .cam_a_lp_clk_n          (cam_a_lp_clk_n),
        .cam_a_lp_lane0_p        (cam_a_lp_lane0_p),
        .cam_a_lp_lane0_n        (cam_a_lp_lane0_n),
        .cam_a_lp_lane1_p        (cam_a_lp_lane1_p),
        .cam_a_lp_lane1_n        (cam_a_lp_lane1_n),
        .cam_a_bta_o             (cam_a_bta_o),
        .camera_power_good       (),
        .camera_reset_done       (),
        .sccb_init_done          (),
        .sccb_busy               (),
        .sccb_done               (),
        .sccb_error              (),
        .busy                    (cam_ctrl_busy_sys),
        .init_done               (cam_init_done_sys),
        .init_fail               (cam_init_fail_sys),
        .sensor_id_ok            (cam_sensor_id_ok_sys),
        .camera_ready            (cam_ready_sys),
        .last_err                (cam_last_err_sys),
        .camera_rgb_vid          (rgb_camera_bg_vid),
        .camera_rgb_valid_vid    (cam_rgb_valid_vid),
        .cam_status_snap_sys     (cam_status_snap_sys),
        .cam_status_snap_upd_sys (cam_status_snap_upd_sys),
        .dbg_frame_tick          (dbg_frame_tick),
        .dbg_sccb_transaction    (dbg_cam_sccb_transaction),
        .dbg_init_done           (dbg_cam_init_done),
        .dbg_overflow_event      (dbg_cam_overflow_event)
    );

    snapshot_sys2vid_camstatus #(
        .W(CAM_SNAPSHOT_W)
    ) u_snapshot_sys2vid_camstatus (
        .clk_sys               (clk_sys),
        .rst_sys               (rst_sys_local),
        .snap_sys              (cam_status_snap_sys),
        .snap_upd_sys          (cam_status_snap_upd_sys),
        .clk_vid               (clk_vid),
        .rst_vid               (rst_vid),
        .frame_tick_vid        (frame_tick),
        .snap_vid_committed    (cam_status_snap_vid),
        .snap_commit_pulse_vid (cam_status_snap_commit_vid)
    );

    hud_camera_debug_tile #(
        .TILE_X0 (CAM_PANEL_X0),
        .TILE_Y0 (CAM_PANEL_Y0),
        .TILE_W  (CAM_PANEL_W),
        .TILE_H  (CAM_PANEL_H),
        .SNAP_W  (CAM_SNAPSHOT_W)
    ) u_hud_camera_debug_tile (
        .clk_vid             (clk_vid),
        .rst_vid             (rst_vid),
        .pix_x               (pix_x),
        .pix_y               (pix_y),
        .de                  (de),
        .cam_snap_vid        (cam_status_snap_vid),
        .cam_snap_commit_vid (cam_status_snap_commit_vid),
        .rgb_out             (rgb_camera_debug_tile)
    );

    camera_viewport_widget #(
        .X0       (CAM_VIEW_X0),
        .Y0       (CAM_VIEW_Y0),
        .W        (CAM_VIEW_W),
        .H        (CAM_VIEW_H),
        .BORDER   (2),
        .STATUS_H (10)
    ) u_camera_viewport_widget (
        .pix_x           (pix_x),
        .pix_y           (pix_y),
        .de              (de),
        .cam_rgb         (rgb_camera_bg_vid),
        .cam_rgb_valid   (cam_rgb_valid_vid),
        .cam_frame_valid (cam_status_frame_store_valid_vid),
        .cam_ready       (cam_status_ready_vid),
        .cam_init_done   (cam_status_init_done_vid),
        .cam_init_fail   (cam_status_init_fail_vid),
        .rgb_out         (rgb_camera_viewport_widget)
    );

    //==========================================================================
    // 12) Bearing publication / rising-edge event generation
    //--------------------------------------------------------------------------
    // STATE OWNER
    //   clk_sys / rst_sys_local
    //
    // STATE MEANING
    //   cam_reg_bearing_upd_d1_sys holds the previous cycle's publish level.
    //
    // UPDATE RULE
    //   1) Reset clears the delay register.
    //   2) Each cycle captures the current source level.
    //
    // RESULT
    //   A one-cycle publication pulse is derived by comparing current and
    //   previous values.
    //--------------------------------------------------------------------------
    always @(posedge clk_sys) begin
        if (rst_sys_local)
            cam_reg_bearing_upd_d1_sys <= 1'b0;
        else
            cam_reg_bearing_upd_d1_sys <= cam_reg_bearing_upd_sys;
    end

    assign cam_reg_bearing_upd_pulse_sys =
        cam_reg_bearing_upd_sys & ~cam_reg_bearing_upd_d1_sys;

    snapshot_sys2vid #(
        .W(32)
    ) u_snapshot_sys2vid_bearing (
        .clk_src            (clk_sys),
        .rst_src            (rst_sys_local),
        .snap_src           (bearing_snap_sys),
        .snap_upd_src       (bearing_snap_upd_sys),
        .clk_dst            (clk_vid),
        .rst_dst            (rst_vid),
        .frame_tick_dst     (frame_tick),
        .snap_dst_committed (bearing_snap_vid),
        .commit_pulse_dst   (bearing_snap_commit_vid)
    );

    //==========================================================================
    // 13) SYS-local synthetic angle generator
    //--------------------------------------------------------------------------
    // ROLE
    //   Provide a deterministic fallback direction source for the occupancy
    //   painters when no true registration/fusion bearing is available.
    //
    // STATE OWNER
    //   clk_sys / rst_sys_local
    //
    // STATE MEANING
    //   synth_angle_q10_sys stores the fallback sweep angle.
    //
    // UPDATE RULE
    //   1) Reset initializes the fallback angle to zero.
    //   2) Any filtered sonar-valid event increments the angle by a small step.
    //   3) Otherwise the angle holds.
    //
    // RATIONALE
    //   Advancing only on accepted sonar samples ties the fallback sweep to
    //   sensing activity rather than raw clock cycles.
    //--------------------------------------------------------------------------
    always @(posedge clk_sys) begin
        if (rst_sys_local) begin
            synth_angle_q10_sys <= 10'd0;
        end else if (sonar1_distance_valid_filt) begin
            synth_angle_q10_sys <= synth_angle_q10_sys + 10'd8;
        end
    end

    assign sonar1_angle_q10_sys =
        cam_reg_bearing_valid_sys ? deg_u9_to_q10(cam_reg_bearing_deg_sys)
                                  : synth_angle_q10_sys;

    angle_q10_to_dir_q15 #(
        .REGISTER_OUTPUTS(1)
    ) u_sonar1_angle_q10_to_dir_q15 (
        .clk       (clk_sys),
        .rst_n     (~rst_sys_local),
        .angle_q10 (sonar1_angle_q10_sys),
        .dir_x_q15 (sonar1_cos_q15_sys),
        .dir_y_q15 (sonar1_sin_q15_sys)
    );

    /*
    angle_q10_to_dir_q15 #(
        .REGISTER_OUTPUTS(1)
    ) u_sonar2_angle_q10_to_dir_q15 (
        .clk       (clk_sys),
        .rst_n     (~rst_sys_local),
        .angle_q10 (sonar2_angle_q10_sys),
        .dir_x_q15 (sonar2_cos_q15_sys),
        .dir_y_q15 (sonar2_sin_q15_sys)
    );
    */

    //==========================================================================
    // 14) Rich snapshot publication
    //==========================================================================
    //
    /*
    sonar_snapshot_pack_rich_sys #(
        .CLK_HZ (SYS_CLK_HZ),
        .SNAP_W (SONAR_RICH_SNAPSHOT_W),
        .SRC_ID (2'd0)
    ) u_sonar1_snapshot_pack_rich_sys (
        .clk_sys          (clk_sys),
        .rst_sys          (rst_sys_local),
        .raw_in           (sonar1_distance_in_raw),
        .filt_in          (sonar1_distance_in_filter_core),
        .stale            (sonar1_stale_sys),
        .timeout_err      (sonar1_timeout_err_sys),
        .parse_err_sticky (sonar1_parse_err_pulse),
        .age_ticks        (sonar1_age_ticks_sys),
        .update_count     ({16'd0, sonar1_update_count_sys}),
        .snap_data        (sonar1_rich_snap_sys),
        .snap_upd         (sonar1_rich_snap_upd_sys)
    );

    sonar_snapshot_pack_rich_sys #(
        .CLK_HZ (SYS_CLK_HZ),
        .SNAP_W (SONAR_RICH_SNAPSHOT_W),
        .SRC_ID (2'd1)
    ) u_sonar2_snapshot_pack_rich_sys (
        .clk_sys          (clk_sys),
        .rst_sys          (rst_sys_local),
        .raw_in           (sonar2_distance_in_raw),
        .filt_in          (sonar2_distance_in_filt),
        .stale            (sonar2_stale_sys),
        .timeout_err      (sonar2_timeout_err_sys),
        .parse_err_sticky (sonar2_parse_err_pulse),
        .age_ticks        (sonar2_age_ticks_sys),
        .update_count     ({16'd0, sonar2_update_count_sys}),
        .snap_data        (sonar2_rich_snap_sys),
        .snap_upd         (sonar2_rich_snap_upd_sys)
    );
    //*/
    //==========================================================================
    // 15) SONAR 1 48-bit telemetry bus publication
    //--------------------------------------------------------------------------
    // CONTRACT
    //   - filt_in_inch : current filtered sonar distance in inches
    //   - sample_vld   : one-cycle accepted-sample publication pulse
    //   - stale        : watchdog stale state
    //   - age_ticks    : watchdog age counter
    //   - no_target_evt: explicit no-target event (not yet produced here, so 0)
    //
    // PUBLICATION POLICY
    //   sonar_bus48/upd form an event-published SYS-domain snapshot pair for
    //   downstream snapshot_sys2vid crossing.
    //==========================================================================
    //
    /*
    sonar_bus48_pack_sys #(
        .CLK_HZ (SYS_CLK_HZ)
    ) u_sonar1_bus48_pack_sys (
        .clk_sys       (clk_sys),
        .rst_sys       (rst_sys_local),
        .filt_in_inch  (sonar1_distance_in_selected),
        .sample_vld    (sonar1_distance_valid_selected),
        .stale         (sonar1_stale_sys),
        .age_ticks     (sonar1_age_ticks_sys),
        .no_target_evt (1'b0),
        .sonar_bus48   (sonar1_bus48_sys),
        .sonar_bus48_upd(sonar1_bus48_upd_sys)
    );
    //*/


    //==========================================================================
    // 16) SYS->VID CDC for rich snapshots and bus48
    //==========================================================================
    //
    /*
    snapshot_sys2vid #(
        .W(SONAR_RICH_SNAPSHOT_W)
    ) u_snapshot_sys2vid_sonar1_rich (
        .clk_src            (clk_sys),
        .rst_src            (rst_sys_local),
        .snap_src           (sonar1_rich_snap_sys),
        .snap_upd_src       (sonar1_rich_snap_upd_sys),
        .clk_dst            (clk_vid),
        .rst_dst            (rst_vid),
        .frame_tick_dst     (frame_tick),
        .snap_dst_committed (sonar1_rich_snap_vid),
        .commit_pulse_dst   (sonar1_rich_snap_commit_vid)
    );

    snapshot_sys2vid #(
        .W(SONAR_RICH_SNAPSHOT_W)
    ) u_snapshot_sys2vid_sonar2_rich (
        .clk_src            (clk_sys),
        .rst_src            (rst_sys_local),
        .snap_src           (sonar2_rich_snap_sys),
        .snap_upd_src       (sonar2_rich_snap_upd_sys),
        .clk_dst            (clk_vid),
        .rst_dst            (rst_vid),
        .frame_tick_dst     (frame_tick),
        .snap_dst_committed (sonar2_rich_snap_vid),
        .commit_pulse_dst   (sonar2_rich_snap_commit_vid)
    );

    snapshot_sys2vid #(
        .W(SONAR_BUS48_W)
    ) u_snapshot_sys2vid_sonar1_bus48 (
        .clk_src            (clk_sys),
        .rst_src            (rst_sys_local),
        .snap_src           (sonar1_bus48_sys),
        .snap_upd_src       (sonar1_bus48_upd_sys),
        .clk_dst            (clk_vid),
        .rst_dst            (rst_vid),
        .frame_tick_dst     (frame_tick),
        .snap_dst_committed (sonar1_bus48_vid),
        .commit_pulse_dst   (sonar1_bus48_commit_vid)
    );

    snapshot_sys2vid #(
        .W(SONAR_BUS48_W)
    ) u_snapshot_sys2vid_sonar2_bus48 (
        .clk_src            (clk_sys),
        .rst_src            (rst_sys_local),
        .snap_src           (sonar2_bus48_sys),
        .snap_upd_src       (sonar2_bus48_upd_sys),
        .clk_dst            (clk_vid),
        .rst_dst            (rst_vid),
        .frame_tick_dst     (frame_tick),
        .snap_dst_committed (sonar2_bus48_vid),
        .commit_pulse_dst   (sonar2_bus48_commit_vid)
    );

    //==========================================================================
    // 17) Painter telemetry CDC and dual occupancy painters
    //--------------------------------------------------------------------------
    // POLICY
    //   Painters own SYS-side map writes.
    //   Their PIX-side RGB outputs are retained as valid candidate surfaces.
    //==========================================================================
    snapshot_sys2vid #(
        .W(64)
    ) u_snapshot_sys2vid_sonar1_painter_telem (
        .clk_src            (clk_sys),
        .rst_src            (rst_sys_local),
        .snap_src           (sonar1_painter_telem_sys),
        .snap_upd_src       (sonar1_painter_telem_vld_sys),
        .clk_dst            (clk_vid),
        .rst_dst            (rst_vid),
        .frame_tick_dst     (frame_tick),
        .snap_dst_committed (sonar1_painter_telem_vid),
        .commit_pulse_dst   (sonar1_painter_telem_commit_vid)
    );

    snapshot_sys2vid #(
        .W(64)
    ) u_snapshot_sys2vid_sonar2_painter_telem (
        .clk_src            (clk_sys),
        .rst_src            (rst_sys_local),
        .snap_src           (sonar2_painter_telem_sys),
        .snap_upd_src       (sonar2_painter_telem_vld_sys),
        .clk_dst            (clk_vid),
        .rst_dst            (rst_vid),
        .frame_tick_dst     (frame_tick),
        .snap_dst_committed (sonar2_painter_telem_vid),
        .commit_pulse_dst   (sonar2_painter_telem_commit_vid)
    );
    //*/

    //==========================================================================
    // Active SYS->VID CDC for bus48 and painter telemetry
    //--------------------------------------------------------------------------
    // These instances complete the live telemetry path from the SYS-domain
    // sonar publishers into VID-domain radar/debug/map-overlay consumers.
    //
    // The legacy block above remains commented because it also contains older
    // duplicate rich-snapshot CDC instances that are already instantiated in
    // the canonical sonar acquisition sections.
    //==========================================================================
    snapshot_sys2vid #(
        .W(SONAR_BUS48_W)
    ) u_snapshot_sys2vid_sonar1_bus48 (
        .clk_src            (clk_sys),
        .rst_src            (rst_sys_local),
        .snap_src           (sonar1_bus48_sys),
        .snap_upd_src       (sonar1_bus48_upd_sys),
        .clk_dst            (clk_vid),
        .rst_dst            (rst_vid),
        .frame_tick_dst     (frame_tick),
        .snap_dst_committed (sonar1_bus48_vid),
        .commit_pulse_dst   (sonar1_bus48_commit_vid)
    );

    /*
    snapshot_sys2vid #(
        .W(SONAR_BUS48_W)
    ) u_snapshot_sys2vid_sonar2_bus48 (
        .clk_src            (clk_sys),
        .rst_src            (rst_sys_local),
        .snap_src           (sonar2_bus48_sys),
        .snap_upd_src       (sonar2_bus48_upd_sys),
        .clk_dst            (clk_vid),
        .rst_dst            (rst_vid),
        .frame_tick_dst     (frame_tick),
        .snap_dst_committed (sonar2_bus48_vid),
        .commit_pulse_dst   (sonar2_bus48_commit_vid)
    );
    */

    snapshot_sys2vid #(
        .W(64)
    ) u_snapshot_sys2vid_sonar1_painter_telem (
        .clk_src            (clk_sys),
        .rst_src            (rst_sys_local),
        .snap_src           (sonar1_painter_telem_sys),
        .snap_upd_src       (sonar1_painter_telem_vld_sys),
        .clk_dst            (clk_vid),
        .rst_dst            (rst_vid),
        .frame_tick_dst     (frame_tick),
        .snap_dst_committed (sonar1_painter_telem_vid),
        .commit_pulse_dst   (sonar1_painter_telem_commit_vid)
    );

    /*
    snapshot_sys2vid #(
        .W(64)
    ) u_snapshot_sys2vid_sonar2_painter_telem (
        .clk_src            (clk_sys),
        .rst_src            (rst_sys_local),
        .snap_src           (sonar2_painter_telem_sys),
        .snap_upd_src       (sonar2_painter_telem_vld_sys),
        .clk_dst            (clk_vid),
        .rst_dst            (rst_vid),
        .frame_tick_dst     (frame_tick),
        .snap_dst_committed (sonar2_painter_telem_vid),
        .commit_pulse_dst   (sonar2_painter_telem_commit_vid)
    );
    */


    sonar_occupancy_painter #(
        .MAP_W          (SONAR_MAP_CELLS_W),
        .MAP_H          (SONAR_MAP_CELLS_H),
        .ORIGIN_X       (SONAR_MAP_ORIGIN_X),
        .ORIGIN_Y       (SONAR_MAP_ORIGIN_Y),
        .CELL_MM_SHIFT  (SONAR_MAP_CELL_MM_SHIFT),
        .MAX_RAY_STEPS  (SONAR_MAP_MAX_RAY_STEPS),
        .TELEM_MODE     (1),
        .RIGHT_PANEL_X0 (SONAR1_PAINTER_PANEL_X0),
        .RIGHT_PANEL_W  (SONAR1_PAINTER_PANEL_W),
        .SCREEN_H       (V_ACTIVE),
        .GRID_ROWS      (SONAR1_PAINTER_GRID_ROWS),
        .GRID_COLS      (SONAR1_PAINTER_GRID_COLS),
        .ROW_IDX        (SONAR1_PAINTER_ROW_IDX),
        .COL_IDX        (SONAR1_PAINTER_COL_IDX)
    ) u_sonar1_occupancy_painter (
        .clk_sys                (clk_sys),
        .rst_sys                (rst_sys_local),
        .dist_mm_sys            (sonar1_dist_mm_sys),
        .dist_vld_sys           (sonar1_distance_valid_selected),
        .sin_q15_sys            (sonar1_map_sin_q15_sys),
        .cos_q15_sys            (sonar1_cos_q15_sys),
        .brush_mode_sys         (sw_sys[SW_SONAR1_BRUSH_MODE]),
        .busy_sys               (),
        .done_pulse_sys         (),
        .map_we_sys             (sonar1_map_we_sys),
        .map_addr_sys           (sonar1_map_addr_wr_sys),
        .map_din_sys            (sonar1_map_din_sys),
        .painter_telem_sys      (sonar1_painter_telem_sys),
        .painter_telem_vld_sys  (sonar1_painter_telem_vld_sys),
        .pix_clk                (clk_vid),
        .pix_rst                (rst_vid),
        .hud_mode_pix           (1'b1),
        .hcount                 (pix_x[9:0]),
        .vcount                 (pix_y[9:0]),
        .active_video           (sonar1_painter_tile_vid),
        .frame_tick             (frame_tick),
        .sonar_mm_pix           (16'd0),
        .sonar_update_pix       (8'd0),
        .sonar_telem_pix        (sonar1_bus48_vid),
        .sonar_telem_upd_pix    (sonar1_bus48_commit_vid),
        .src_pix                (2'd0),
        .uart_pkt_pulse_pix     (1'b0),
        .uart_crc_err_pulse_pix (1'b0),
        .hud_flags              (8'h81),
        .painter_telem_pix      (sonar1_painter_telem_vid),
        .painter_telem_upd_pix  (sonar1_painter_telem_commit_vid),
        .rgb_bg                 (12'h000),
        .rgb_out                (rgb_sonar1_painter_444)
    );

    /*
    sonar_occupancy_painter #(
        .TELEM_MODE     (1),
        .RIGHT_PANEL_X0 (280),
        .RIGHT_PANEL_W  (160),
        .SCREEN_H       (480),
        .ROW_IDX        (3),
        .COL_IDX        (0)
    ) u_sonar2_occupancy_painter (
        .clk_sys                (clk_sys),
        .rst_sys                (rst_sys_local),
        .dist_mm_sys            (sonar2_dist_mm_sys),
        .dist_vld_sys           (sonar2_distance_valid_filt),
        .sin_q15_sys            (sonar2_sin_q15_sys),
        .cos_q15_sys            (sonar2_cos_q15_sys),
        .brush_mode_sys         (sw_sys[SW_SONAR1_BRUSH_MODE]),
        .busy_sys               (),
        .done_pulse_sys         (),
        .map_we_sys             (sonar2_map_we_sys),
        .map_addr_sys           (sonar2_map_addr_wr_sys),
        .map_din_sys            (sonar2_map_din_sys),
        .painter_telem_sys      (sonar2_painter_telem_sys),
        .painter_telem_vld_sys  (sonar2_painter_telem_vld_sys),
        .pix_clk                (clk_vid),
        .pix_rst                (rst_vid),
        .hud_mode_pix           (1'b0),
        .hcount                 (pix_x[9:0]),
        .vcount                 (pix_y[9:0]),
        .active_video           (de),
        .frame_tick             (frame_tick),
        .sonar_mm_pix           (16'd0),
        .sonar_update_pix       (8'd0),
        .sonar_telem_pix        (sonar2_bus48_vid),
        .sonar_telem_upd_pix    (sonar2_bus48_commit_vid),
        .src_pix                (2'd0),
        .uart_pkt_pulse_pix     (1'b0),
        .uart_crc_err_pulse_pix (1'b0),
        .hud_flags              (8'h80),
        .painter_telem_pix      (sonar2_painter_telem_vid),
        .painter_telem_upd_pix  (sonar2_painter_telem_commit_vid),
        .rgb_bg                 (12'h000),
        .rgb_out                (rgb_sonar2_painter_444)
    );
    */

    //==========================================================================
    // 18) Dual map memories
    //==========================================================================
    sonar_map_mem_tdp_dc #(
        .AW(MAP_AW),
        .DW(MAP_DW)
    ) u_sonar1_map_mem (
        .clk_sys  (clk_sys),
        .we_sys   (sonar1_map_we_sys),
        .addr_sys (sonar1_map_addr_wr_sys),
        .din_sys  (sonar1_map_din_sys),
        .clk_vid  (clk_vid),
        .addr_vid (sonar1_map_addr_rd_vid),
        .dout_vid (sonar1_map_dout_vid)
    );

    /*
    sonar_map_mem_tdp_dc #(
        .AW(MAP_AW),
        .DW(MAP_DW)
    ) u_sonar2_map_mem (
        .clk_sys  (clk_sys),
        .we_sys   (sonar2_map_we_sys),
        .addr_sys (sonar2_map_addr_wr_sys),
        .din_sys  (sonar2_map_din_sys),
        .clk_vid  (clk_vid),
        .addr_vid (sonar2_map_addr_rd_vid),
        .dout_vid (sonar2_map_dout_vid)
    );
    */

    //==========================================================================
    // 19) Dual map renderers
    //--------------------------------------------------------------------------
    // Interface family frozen to the MAP_W/MAP_H/DATA_W contract that appears
    // in the uploaded top-level.
    //==========================================================================
    sonar_map_renderer #(
        .MAP_W      (SONAR_MAP_CELLS_W),
        .MAP_H      (SONAR_MAP_CELLS_H),
        .DATA_W     (8),
        .PANEL_X0   (SONAR1_MAP_X0),
        .PANEL_Y0   (SONAR1_MAP_Y0),
        .PANEL_WPX  (SONAR_MAP_WPX),
        .PANEL_HPX  (SONAR_MAP_HPX),
        .FIT_TO_PANEL (1),
        .FIT_X_SHIFT  (SONAR_MAP_FIT_X_SHIFT),
        .FIT_Y_SHIFT  (SONAR_MAP_FIT_Y_SHIFT),
        .RD_LAT     (SONAR1_MAP_RD_LAT),
        .EN_BORDER  (1),
        .EN_GRID    (1)
    ) u_sonar1_map_renderer (
        .pix_clk                (clk_vid),
        .pix_rst                (rst_vid),
        .hcount                 (pix_x[9:0]),
        .vcount                 (pix_y[9:0]),
        .active_video           (de),
        .frame_tick             (frame_tick),
        .map_rd_addr            (sonar1_map_addr_rd_vid),
        .map_rd_data            (sonar1_map_dout_vid),
        .painter_valid          (sonar1_map_valid_vid),
        .painter_rgb            (sonar1_map_rgb444_vid),
        .renderer_telem_pix     (sonar1_renderer_telem_vid),
        .renderer_telem_upd_pix (sonar1_renderer_telem_upd_vid)
    );

    /*
    sonar_map_renderer #(
        .MAP_W      (256),
        .MAP_H      (256),
        .DATA_W     (8),
        .PANEL_X0   (SONAR2_MAP_X0),
        .PANEL_Y0   (SONAR2_MAP_Y0),
        .PANEL_WPX  (SONAR_MAP_WPX),
        .PANEL_HPX  (SONAR_MAP_HPX),
        .RD_LAT     (1),
        .EN_BORDER  (1),
        .EN_GRID    (1)
    ) u_sonar2_map_renderer (
        .pix_clk                (clk_vid),
        .pix_rst                (rst_vid),
        .hcount                 (pix_x[9:0]),
        .vcount                 (pix_y[9:0]),
        .active_video           (de),
        .frame_tick             (frame_tick),
        .map_rd_addr            (sonar2_map_addr_rd_vid),
        .map_rd_data            (sonar2_map_dout_vid),
        .painter_valid          (sonar2_map_valid_vid),
        .painter_rgb            (sonar2_map_rgb444_vid),
        .renderer_telem_pix     (sonar2_renderer_telem_vid),
        .renderer_telem_upd_pix (sonar2_renderer_telem_upd_vid)
    );
    */

    //==========================================================================
    // 20) Map overlay telemetry
    //--------------------------------------------------------------------------
    // This overlay adds painter and renderer telemetry on top of the base map.
    //==========================================================================
    sonar_map_overlay_telem #(
        .PANEL_X0    (SONAR1_MAP_X0),
        .PANEL_Y0    (SONAR1_MAP_Y0),
        .PANEL_W     (SONAR_MAP_WPX),
        .PANEL_H     (SONAR_MAP_HPX),
        .GLYPH_SCALE (2)
    ) u_sonar1_map_overlay_telem (
        .pix_clk                 (clk_vid),
        .pix_rst                 (rst_vid),
        .hcount                  (pix_x[9:0]),
        .vcount                  (pix_y[9:0]),
        .active_video            (de),
        .sonar_telem_pix         (sonar1_bus48_vid),
        .sonar_telem_upd_pix     (sonar1_bus48_commit_vid),
        .painter_telem_pix       (sonar1_painter_telem_vid),
        .painter_telem_upd_pix   (sonar1_painter_telem_commit_vid),
        .renderer_telem_pix      (sonar1_renderer_telem_vid),
        .renderer_telem_upd_pix  (sonar1_renderer_telem_upd_vid),
        .rgb_bg                  (rgb_sonar1_map_444),
        .rgb_out                 (sonar1_map_overlay_444)
    );

    /*
    sonar_map_overlay_telem #(
        .PANEL_X0    (SONAR2_MAP_X0),
        .PANEL_Y0    (SONAR2_MAP_Y0),
        .PANEL_W     (SONAR_MAP_WPX),
        .PANEL_H     (SONAR_MAP_HPX),
        .GLYPH_SCALE (2)
    ) u_sonar2_map_overlay_telem (
        .pix_clk                 (clk_vid),
        .pix_rst                 (rst_vid),
        .hcount                  (pix_x[9:0]),
        .vcount                  (pix_y[9:0]),
        .active_video            (de),
        .sonar_telem_pix         (sonar2_bus48_vid),
        .sonar_telem_upd_pix     (sonar2_bus48_commit_vid),
        .painter_telem_pix       (sonar2_painter_telem_vid),
        .painter_telem_upd_pix   (sonar2_painter_telem_commit_vid),
        .renderer_telem_pix      (sonar2_renderer_telem_vid),
        .renderer_telem_upd_pix  (sonar2_renderer_telem_upd_vid),
        .rgb_bg                  (rgb_sonar2_map_444),
        .rgb_out                 (sonar2_map_overlay_444)
    );
    */

    //==========================================================================
    // 21) Dual radar overlays
    //--------------------------------------------------------------------------
    // Interface family frozen to hud_sonar_radar_overlay_pix.
    //==========================================================================
    hud_sonar_radar_overlay_pix #(
        .X0                  (SONAR1_RADAR_X0),
        .Y0                  (SONAR1_RADAR_Y0),
        .W                   (SONAR_RADAR_W),
        .H                   (SONAR_RADAR_H),
        .R_MAX_PX            (SONAR_RADAR_RMAX),
        .TELEMETRY_EN        (1),
        .SWEEP_USE_EXT_ANGLE (1),
        .EN_PHOS             (1)
    ) u_sonar1_radar_overlay_pix (
        .clk_pix              (clk_vid),
        .rst_pix              (rst_vid),
        .pix_x                (pix_x[9:0]),
        .pix_y                (pix_y[9:0]),
        .active_video         (de),
        .frame_tick           (frame_tick),
        .sample_upd_pix       (sonar1_bus48_commit_vid),
        .dist_mm_pix          (sonar1_bus48_vid[47:32]),
        .angle_q10_pix        (sonar1_angle_q10_vid),
        .dist_stale_pix       (sonar1_bus_stale_vid),
        .ring_en_pix          (1'b1),
        .trail_en_pix         (1'b1),
        .rgb_bg               (12'h000),
        .rgb_out              (rgb_sonar1_radar_444),
        .radar_telem_pix      (sonar1_radar_telem_vid),
        .radar_telem_vld_pix  (sonar1_radar_telem_vld_vid)
    );

    /*
    hud_sonar_radar_overlay_pix #(
        .X0                  (SONAR2_RADAR_X0),
        .Y0                  (SONAR2_RADAR_Y0),
        .W                   (SONAR_RADAR_W),
        .H                   (SONAR_RADAR_H),
        .R_MAX_PX            (SONAR_RADAR_RMAX),
        .TELEMETRY_EN        (1),
        .SWEEP_USE_EXT_ANGLE (1),
        .EN_PHOS             (1)
    ) u_sonar2_radar_overlay_pix (
        .clk_pix              (clk_vid),
        .rst_pix              (rst_vid),
        .pix_x                (pix_x[9:0]),
        .pix_y                (pix_y[9:0]),
        .active_video         (de),
        .frame_tick           (frame_tick),
        .sample_upd_pix       (sonar2_bus48_commit_vid),
        .dist_mm_pix          (sonar2_bus48_vid[47:32]),
        .angle_q10_pix        (sonar2_angle_q10_vid),
        .dist_stale_pix       (sonar2_bus_stale_vid),
        .ring_en_pix          (1'b1),
        .trail_en_pix         (1'b1),
        .rgb_bg               (12'h000),
        .rgb_out              (rgb_sonar2_radar_444),
        .radar_telem_pix      (sonar2_radar_telem_vid),
        .radar_telem_vld_pix  (sonar2_radar_telem_vld_vid)
    );
    */

    //==========================================================================
    // 22) Dual rich tiles
    //--------------------------------------------------------------------------
    // Interface family frozen to pix_clk / rst_n / snap_data_pix form.
    //==========================================================================
    hud_sonar_tile_rich #(
        .TILE_X0 (SONAR1_RICH_X0),
        .TILE_Y0 (SONAR1_RICH_Y0),
        .TILE_W  (SONAR_RICH_TILE_W),
        .TILE_H  (SONAR_RICH_TILE_H)
    ) u_sonar1_rich_tile (
        .pix_clk       (clk_vid),
        .rst_n         (~rst_vid),
        .pix_x         (pix_x[9:0]),
        .pix_y         (pix_y[9:0]),
        .active_video  (de),
        .frame_tick    (frame_tick),
        .snap_data_pix (sonar1_rich_snap_vid),
        .snap_upd_pix  (sonar1_rich_snap_commit_vid),
        .rgb_bg        (12'h000),
        .rgb_out       (rgb_sonar1_rich_tile_444)
    );

    /*
    hud_sonar_tile_rich #(
        .TILE_X0 (SONAR2_RICH_X0),
        .TILE_Y0 (SONAR2_RICH_Y0)
    ) u_sonar2_rich_tile (
        .pix_clk       (clk_vid),
        .rst_n         (~rst_vid),
        .pix_x         (pix_x[9:0]),
        .pix_y         (pix_y[9:0]),
        .active_video  (de),
        .frame_tick    (frame_tick),
        .snap_data_pix (sonar2_rich_snap_vid),
        .snap_upd_pix  (sonar2_rich_snap_commit_vid),
        .rgb_bg        (12'h000),
        .rgb_out       (rgb_sonar2_rich_tile_444)
    );
    */

    //==========================================================================
    // 23) UART byte-event CDC and terminal
    //--------------------------------------------------------------------------
    // Present integration forwards sonar1 RX bytes into the terminal.
    //==========================================================================
    byte_event_sys2vid u_uart_byte_event_sys2vid (
        .clk_sys      (clk_sys),
        .rst_sys      (rst_sys_local),
        .byte_sys     (sonar1_rx_byte),
        .byte_vld_sys (sonar1_rx_valid),
        .clk_vid      (clk_vid),
        .rst_vid      (rst_vid),
        .byte_vid     (uart_evt_byte_vid),
        .byte_vld_vid (uart_evt_vld_vid),
        .busy_sys     (uart_evt_busy_sys)
    );

    vga_uart_terminal_overlay #(
        .BOX_X0   (UART_BOX_X0),
        .BOX_Y0   (UART_BOX_Y0),
        .CHAR_W   (UART_BOX_CHAR_W),
        .CHAR_H   (UART_BOX_CHAR_H),
        .BOX_COLS (UART_BOX_COLS),
        .BOX_ROWS (UART_BOX_ROWS)
    ) u_vga_uart_terminal_overlay (
        .pix_clk      (clk_vid),
        .rst          (rst_vid),
        .hcount       (pix_x[9:0]),
        .vcount       (pix_y[9:0]),
        .active_video (de),
        .frame_tick   (frame_tick),
        .rx_byte      (uart_evt_byte_vid),
        .rx_vld       (uart_evt_vld_vid),
        .tx_byte      (8'd0),
        .tx_vld       (1'b0),
        .sw_mode      (sw_sys[4]),
        .rgb_bg       (12'h000),
        .rgb_out      (rgb_uart_term_444)
    );

    //==========================================================================
    // 24) RGB444 -> RGB888 promotion
    //==========================================================================
    // rgb444_to_rgb888 u_rgb444_to_rgb888_sonar2_painter   (.rgb444(rgb_sonar2_painter_444),   .rgb888(rgb_sonar2_painter_888));
    rgb444_to_rgb888 u_rgb444_to_rgb888_sonar1_map       (.rgb444(sonar1_map_overlay_444),   .rgb888(rgb_sonar1_map_888));
    // rgb444_to_rgb888 u_rgb444_to_rgb888_sonar2_map       (.rgb444(sonar2_map_overlay_444),   .rgb888(rgb_sonar2_map_888));
    rgb444_to_rgb888 u_rgb444_to_rgb888_sonar1_radar     (.rgb444(rgb_sonar1_radar_444),     .rgb888(rgb_sonar1_radar_888));
    // rgb444_to_rgb888 u_rgb444_to_rgb888_sonar2_radar     (.rgb444(rgb_sonar2_radar_444),     .rgb888(rgb_sonar2_radar_888));
    rgb444_to_rgb888 u_rgb444_to_rgb888_sonar1_rich_tile (.rgb444(rgb_sonar1_rich_tile_444), .rgb888(rgb_sonar1_rich_tile_888));
    // rgb444_to_rgb888 u_rgb444_to_rgb888_sonar2_rich_tile (.rgb444(rgb_sonar2_rich_tile_444), .rgb888(rgb_sonar2_rich_tile_888));
    rgb444_to_rgb888 u_rgb444_to_rgb888_uart_term        (.rgb444(rgb_uart_term_444),        .rgb888(rgb_uart_term_888));
    rgb444_to_rgb888 u_rgb444_to_rgb888_sonar1_painter (.rgb444 (rgb_sonar1_painter_444),      .rgb888 (rgb_sonar1_painter_888));
    
    
    
    
    //==========================================================================
    // 25) Runtime view-selector state
    //--------------------------------------------------------------------------
    // STATE OWNER
    //   clk_sys / rst_sys_local
    //
    // STATE MEANING
    //   btn_aux_d1_sys       : previous sampled auxiliary-load button state
    //   aux_view_sel_sys     : committed auxiliary selector
    //   aux_view_cfg_upd_sys : selector publication pulse
    //
    // UPDATE RULE
    //   1) Reset initializes the auxiliary selector to its canonical default.
    //   2) Each cycle samples the auxiliary-load button.
    //   3) On a qualifying button edge, the auxiliary selector is updated and
    //      republished.
    //   4) Otherwise selector state holds.
    //
    // RESULT
    //   Auxiliary display-mode changes become visible in VID only after
    //   snapshot commit. The older sonar surface selectors were removed from
    //   active logic because the final compositor no longer consumes their
    //   selected RGB outputs.
    //--------------------------------------------------------------------------
    always @(posedge clk_sys) begin
        if (rst_sys_local) begin
            btn_aux_d1_sys             <= 1'b0;
            aux_view_sel_sys           <= AUX_VIEW_RESET;
            aux_view_cfg_upd_sys       <= 1'b0;
            aux_view_boot_pending_sys  <= 1'b1;
        end else begin
            btn_aux_d1_sys       <= btn_sys[4];
            aux_view_cfg_upd_sys <= 1'b0;

            if (aux_view_boot_pending_sys) begin
                aux_view_boot_pending_sys <= 1'b0;
                aux_view_cfg_upd_sys      <= 1'b1;
            end else if (btn_load_aux_rise_sys) begin
                aux_view_sel_sys     <= aux_view_code_sys;
                aux_view_cfg_upd_sys <= 1'b1;
            end
        end
    end

    snapshot_sys2vid #(
        .W(3)
    ) u_snapshot_sys2vid_aux_view_cfg (
        .clk_src            (clk_sys),
        .rst_src            (rst_sys_local),
        .snap_src           (aux_view_sel_sys),
        .snap_upd_src       (aux_view_cfg_upd_sys),
        .clk_dst            (clk_vid),
        .rst_dst            (rst_vid),
        .frame_tick_dst     (frame_tick),
        .snap_dst_committed (aux_view_sel_vid),
        .commit_pulse_dst   ()
    );

    //==========================================================================
    // 26) VID-domain display-mode decode
    //--------------------------------------------------------------------------
    // The final compositor consumes fixed overlay surfaces directly. Explicit
    // per-layer region masks are used instead of black-key transparency so HUD
    // panels can intentionally draw black interiors over the camera feed.
    //==========================================================================
    wire test_view_vid;
    wire map_debug_view_vid;
    wire camera_base_mode_vid;
    wire camera_widget_enable_vid;
    wire hud_enable_vid;
    wire uart_enable_vid;
    wire map_enable_vid;

    assign test_view_vid        = (aux_view_sel_vid == AUX_VIEW_TEST);
    assign map_debug_view_vid   = (aux_view_sel_vid == AUX_VIEW_MAP_DEBUG);
    assign camera_base_mode_vid = (aux_view_sel_vid == AUX_VIEW_CAMERA_ONLY);
    assign camera_widget_enable_vid =
                                  (aux_view_sel_vid == AUX_VIEW_CAMERA_HUD)  ||
                                  (aux_view_sel_vid == AUX_VIEW_UART);
    assign hud_enable_vid       = (aux_view_sel_vid == AUX_VIEW_CAMERA_HUD) ||
                                  (aux_view_sel_vid == AUX_VIEW_UART);
    assign uart_enable_vid      = (aux_view_sel_vid == AUX_VIEW_UART);
    assign map_enable_vid       = hud_enable_vid || map_debug_view_vid;

    assign rgb_base_vid =
        test_view_vid ? rgb_test_pattern_vid :
        ((camera_base_mode_vid && cam_rgb_valid_vid) ? rgb_camera_bg_vid : 24'h000000);

    //==========================================================================
    // 27) Region-qualified HDMI compositor cascade
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Composite all generated RGB overlays simultaneously.
    //
    // LAYERING POLICY
    //   base:    test pattern, full-screen camera diagnostic, or black
    //   stage 0: bounded camera viewport widget
    //   stage 1: global debug panel
    //   stage 2: camera debug tile
    //   stage 3: UART terminal
    //   stage 4: rich sonar tile
    //   stage 5: map panel / telemetry overlay
    //   stage 6: painter diagnostic tile
    //   stage 7: radar overlay
    //
    // NOTE
    //   The older shared video_compositor uses RGB black as an implicit
    //   transparency key. That is unsuitable here because these renderers use
    //   black as a real panel/background color. The mux below uses explicit
    //   geometry-valid masks so black pixels remain visible inside a layer.
    //==========================================================================
    assign rgb_stage0 = (camera_widget_enable_vid && camera_viewport_region_vid) ?
                        rgb_camera_viewport_widget : rgb_base_vid;

    assign rgb_stage1 = (hud_enable_vid && debug_panel_region_vid) ?
                        rgb_debug_panel : rgb_stage0;

    assign rgb_stage2 = (hud_enable_vid && camera_debug_tile_region_vid) ?
                        rgb_camera_debug_tile : rgb_stage1;

    assign rgb_stage3 = (uart_enable_vid && uart_term_region_vid) ?
                        rgb_uart_term_888 : rgb_stage2;

    assign rgb_stage4 = (hud_enable_vid && sonar1_rich_tile_region_vid) ?
                        rgb_sonar1_rich_tile_888 : rgb_stage3;

    assign rgb_stage5 = (map_enable_vid && sonar1_map_region_vid) ?
                        rgb_sonar1_map_888 : rgb_stage4;

    assign rgb_stage6 = (hud_enable_vid && sonar1_painter_region_vid) ?
                        rgb_sonar1_painter_888 : rgb_stage5;

    assign rgb_stage7 = (hud_enable_vid && sonar1_radar_region_vid) ?
                        rgb_sonar1_radar_888 : rgb_stage6;

    assign rgb_stage8 = de ? rgb_stage7 : 24'h000000;
    assign rgb_final  = rgb_stage8;
    
    
    //==========================================================================
    // 28) HDMI/TMDS wrapper
    //==========================================================================
    hdmi_tx_wrapper_rgb2dvi u_hdmi_tx_wrapper_rgb2dvi (
        .clk_ref      (clk_sys),
        .rst_ref      (rst_sys),
        .rgb_in       (rgb_final),
        .hsync_in     (hsync),
        .vsync_in     (vsync),
        .de_in        (de),
        .hpd_in       (hdmi_tx_hpd),
        .clk_vid      (clk_vid),
        .rst_vid      (rst_vid),
        .clk_tmds_5x  (clk_tmds_5x),
        .rst_tmds_5x  (rst_tmds_5x),
        .tx_en_o      (hdmi_tx_en),
        .tmds_clk_p   (hdmi_tx_clk_p),
        .tmds_clk_n   (hdmi_tx_clk_n),
        .tmds_data_p  (hdmi_tx_data_p),
        .tmds_data_n  (hdmi_tx_data_n)
    );

    //==========================================================================
    // 29) Frame counters and VID->SYS mirror
    //--------------------------------------------------------------------------
    // STATE OWNER
    //   frame_count_vid / frame_tick_toggle_vid         : clk_vid / rst_vid
    //   frame_tick_toggle_sync_sys / frame_count_sys    : clk_sys / rst_sys_local
    //
    // STEP-BY-STEP
    //   1) Each VID frame_tick increments frame_count_vid.
    //   2) Each VID frame_tick toggles a single-bit marker.
    //   3) That marker is synchronized into SYS through a 2-flop chain.
    //   4) A change in the synchronized marker reconstructs a SYS-domain pulse.
    //   5) That pulse increments frame_count_sys.
    //
    // RATIONALE
    //   This avoids direct unsafe sampling of a multi-bit VID counter from SYS.
    //--------------------------------------------------------------------------
    always @(posedge clk_vid) begin
        if (rst_vid)
            frame_count_vid <= 16'd0;
        else if (frame_tick)
            frame_count_vid <= frame_count_vid + 16'd1;
    end

    always @(posedge clk_vid) begin
        if (rst_vid)
            frame_tick_toggle_vid <= 1'b0;
        else if (frame_tick)
            frame_tick_toggle_vid <= ~frame_tick_toggle_vid;
    end

    always @(posedge clk_sys) begin
        if (rst_sys_local)
            frame_tick_toggle_sync_sys <= 2'b00;
        else
            frame_tick_toggle_sync_sys <= {frame_tick_toggle_sync_sys[0], frame_tick_toggle_vid};
    end

    always @(posedge clk_sys) begin
        if (rst_sys_local)
            frame_count_sys <= 16'd0;
        else if (frame_tick_pulse_sys)
            frame_count_sys <= frame_count_sys + 16'd1;
    end

    //==========================================================================
    // 30) OLED telemetry packer
    //--------------------------------------------------------------------------
    // SYS-domain only consumer. The frame counter source is the synchronized
    // SYS mirror, not the native VID counter.
    //==========================================================================
    oled_telemetry_pack_sys #(
        .CLK_HZ     (SYS_CLK_HZ),
        .REFRESH_HZ (5),
        .LINE_CHARS (21)
    ) u_oled_telemetry_pack_sys (
        .clk                (clk_sys),
        .rst                (rst_sys_local),
        .sonar1_distance_in (sonar1_distance_in_filt),
        .sonar1_valid       (sonar1_distance_valid_filt),
        .sonar1_stale       (sonar1_stale_sys),
        .sonar1_timeout_err (sonar1_timeout_err_sys),
        .sonar1_age_ticks   (sonar1_age_ticks_sys),
        .sonar2_distance_in (sonar2_distance_in_filt),
        .sonar2_valid       (sonar2_distance_valid_filt),
        .sonar2_stale       (sonar2_stale_sys),
        .sonar2_timeout_err (sonar2_timeout_err_sys),
        .sonar2_age_ticks   (sonar2_age_ticks_sys),
        .cam_busy           (cam_ctrl_busy_sys),
        .cam_init_done      (cam_init_done_sys),
        .cam_init_fail      (cam_init_fail_sys),
        .cam_sensor_id_ok   (cam_sensor_id_ok_sys),
        .cam_last_err       (cam_last_err_sys),
        .sys_locked         (sys_locked),
        .hdmi_hpd           (hdmi_tx_hpd),
        .heartbeat          (dbg_heartbeat),
        .frame_count_lsb    (frame_count_sys),
        .line0_ascii        (oled_line0_ascii),
        .line1_ascii        (oled_line1_ascii),
        .line2_ascii        (oled_line2_ascii),
        .line3_ascii        (oled_line3_ascii),
        .line_upd           (oled_line_upd)
    );

    //==========================================================================
    // 31) OLED text/test consoles and controller
    //==========================================================================
    oled_text_console #(
        .LINE_CHARS (21)
    ) u_oled_text_console (
        .line0_ascii (oled_line0_ascii),
        .line1_ascii (oled_line1_ascii),
        .line2_ascii (oled_line2_ascii),
        .line3_ascii (oled_line3_ascii),
        .byte_addr   (oled_byte_addr),
        .byte_data   (oled_byte_data_text)
    );

    oled_test_pattern_console u_oled_test_pattern_console (
        .byte_addr (oled_byte_addr),
        .byte_data (oled_byte_data_test)
    );

    oled_ctrl_ssd1306_spi #(
        .CLK_HZ          (SYS_CLK_HZ),
        .SPI_HZ          (10_000_000),
        .POWERUP_WAIT_MS (1),
        .RESET_LOW_US    (10),
        .RESET_HIGH_US   (10),
        .VBAT_WAIT_MS    (100),
        .REFRESH_HZ      (5)
    ) u_oled_ctrl_ssd1306_spi (
        .clk         (clk_sys),
        .rst         (rst_sys_local),
        .line_upd    (oled_line_upd),
        .byte_addr   (oled_byte_addr),
        .byte_data   (oled_byte_data_mux),
        .oled_res_n  (oled_res_n),
        .oled_dc     (oled_dc),
        .oled_sclk   (oled_sclk),
        .oled_sdin   (oled_sdin),
        .oled_vbat_n (oled_vbat_n),
        .oled_vdd_n  (oled_vdd_n)
    );

    //==========================================================================
    // 32) Debug outputs
    //==========================================================================



    //
    /*
    //==========================================================================
    // 33) CLS debug UART bridge
    //--------------------------------------------------------------------------
    // Single owner of cls_txd_o.
    // Publishes canonical sonar1 SYS-domain health semantics.
    //==========================================================================
    aquafusion_cls_debug_bridge #(
        .CLK_HZ          (SYS_CLK_HZ),
        .CLS_BAUD        (9600),
        .CLS_STARTUP_MS  (100),
        .CLS_REFRESH_MS  (250),
        .ENABLE_TWO_PAGE (1),
        .PAGE_HOLD_MS    (2000),
        .AGE_WARN_MS     (250),
        .STALE_MS        (1000)
    ) u_aquafusion_cls_debug_bridge (
        .clk                             (clk_sys),
        .rst                             (rst_sys_local),
        .sonar_range_in_sys              (sonar1_distance_in_selected[7:0]),
        .sonar_valid_sys                 (sonar1_distance_valid_selected),
        .sonar_stale_ms_sys              (sonar1_age_ticks_sys),
        .sonar_timeout_sys               (sonar1_timeout_err_sys),
        .sonar_parse_err_sticky_sys      (sonar1_parse_err_sticky_sys),
        .sonar_uart_frame_err_sticky_sys (sonar1_uart_frame_err_sticky_sys),
        .cam_frame_ctr_sys               (cam_frame_count_sys),
        .cam_valid_sys                   (cam_ready_sys),
        .cls_txd_o                       (cls_txd_o)
    );
    //*/

    //==========================================================================
    // 35) Reserved-input sink
    //--------------------------------------------------------------------------
    // Benign acknowledgment sink for presently unused physical inputs.
    //==========================================================================
    wire _unused_cam_inputs;
    assign _unused_cam_inputs = sonar2_uart_i ^ sonar2_pwm_i;

endmodule

`default_nettype wire
