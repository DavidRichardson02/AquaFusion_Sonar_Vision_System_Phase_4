##==============================================================================
## AquaFusion Sonar + Camera Fusion HUD
## Canonical merged XDC for Nexys Video
## Top: aquafusion_nexys_video_top
##
## Active-port-only final constraint set
##
## Mapping summary
##   - HDMI TX: native Nexys Video HDMI source port J8
##   - Sonar 1: JA3/JA4
##   - Sonar 2: JC3/JC4
##   - Camera: FMC LPC + Digilent FMC Pcam Adapter, port A only
##   - VADJ: 2.5 V for FMC banks 15/16 and attached user IO
##   - Debug outputs: JA8/JA9/JA10 plus freed JA camera pins
##
## Button ordering assumed by top-level:
##   btn_in[0]=BTNC, btn_in[1]=BTNU, btn_in[2]=BTNL, btn_in[3]=BTNR, btn_in[4]=BTND
##==============================================================================

##----------------------------------------------------------------------------
## System clock
##----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN R4 IOSTANDARD LVCMOS33 } [get_ports { sys_clk_in }]
create_clock -add -name sys_clk_in -period 10.000 -waveform {0 5} [get_ports { sys_clk_in }]

##----------------------------------------------------------------------------
## CPU reset button (active-low at top level)
##----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN G4 IOSTANDARD LVCMOS15 } [get_ports { cpu_resetn_in }]

##----------------------------------------------------------------------------
## Slide switches [7:0]
## VADJ is intentionally driven to 2.5 V for the FMC adapter, so these bank-15
## user IOs must use LVCMOS25 instead of the board-default LVCMOS12.
##----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN E22 IOSTANDARD LVCMOS25 } [get_ports { sw_in[0] }]
set_property -dict { PACKAGE_PIN F21 IOSTANDARD LVCMOS25 } [get_ports { sw_in[1] }]
set_property -dict { PACKAGE_PIN G21 IOSTANDARD LVCMOS25 } [get_ports { sw_in[2] }]
set_property -dict { PACKAGE_PIN G22 IOSTANDARD LVCMOS25 } [get_ports { sw_in[3] }]
set_property -dict { PACKAGE_PIN H17 IOSTANDARD LVCMOS25 } [get_ports { sw_in[4] }]
set_property -dict { PACKAGE_PIN J16 IOSTANDARD LVCMOS25 } [get_ports { sw_in[5] }]
set_property -dict { PACKAGE_PIN K13 IOSTANDARD LVCMOS25 } [get_ports { sw_in[6] }]
set_property -dict { PACKAGE_PIN M17 IOSTANDARD LVCMOS25 } [get_ports { sw_in[7] }]

##----------------------------------------------------------------------------
## Pushbuttons [4:0]
##----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN B22 IOSTANDARD LVCMOS25 } [get_ports { btn_in[0] }]
set_property -dict { PACKAGE_PIN F15 IOSTANDARD LVCMOS25 } [get_ports { btn_in[1] }]
set_property -dict { PACKAGE_PIN C22 IOSTANDARD LVCMOS25 } [get_ports { btn_in[2] }]
set_property -dict { PACKAGE_PIN D14 IOSTANDARD LVCMOS25 } [get_ports { btn_in[3] }]
set_property -dict { PACKAGE_PIN D22 IOSTANDARD LVCMOS25 } [get_ports { btn_in[4] }]

##----------------------------------------------------------------------------
## LEDs [7:0]
##----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN T14 IOSTANDARD LVCMOS25 } [get_ports { led_out[0] }]
set_property -dict { PACKAGE_PIN T15 IOSTANDARD LVCMOS25 } [get_ports { led_out[1] }]
set_property -dict { PACKAGE_PIN T16 IOSTANDARD LVCMOS25 } [get_ports { led_out[2] }]
set_property -dict { PACKAGE_PIN U16 IOSTANDARD LVCMOS25 } [get_ports { led_out[3] }]
set_property -dict { PACKAGE_PIN V15 IOSTANDARD LVCMOS25 } [get_ports { led_out[4] }]
set_property -dict { PACKAGE_PIN W16 IOSTANDARD LVCMOS25 } [get_ports { led_out[5] }]
set_property -dict { PACKAGE_PIN W15 IOSTANDARD LVCMOS25 } [get_ports { led_out[6] }]
set_property -dict { PACKAGE_PIN Y13 IOSTANDARD LVCMOS25 } [get_ports { led_out[7] }]

##----------------------------------------------------------------------------
## HDMI TX (native HDMI source port J8)
##----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN T1  IOSTANDARD TMDS_33 } [get_ports { hdmi_tx_clk_p }]
set_property -dict { PACKAGE_PIN U1  IOSTANDARD TMDS_33 } [get_ports { hdmi_tx_clk_n }]

set_property -dict { PACKAGE_PIN W1  IOSTANDARD TMDS_33 } [get_ports { hdmi_tx_data_p[0] }]
set_property -dict { PACKAGE_PIN Y1  IOSTANDARD TMDS_33 } [get_ports { hdmi_tx_data_n[0] }]

set_property -dict { PACKAGE_PIN AA1 IOSTANDARD TMDS_33 } [get_ports { hdmi_tx_data_p[1] }]
set_property -dict { PACKAGE_PIN AB1 IOSTANDARD TMDS_33 } [get_ports { hdmi_tx_data_n[1] }]

set_property -dict { PACKAGE_PIN AB3 IOSTANDARD TMDS_33 } [get_ports { hdmi_tx_data_p[2] }]
set_property -dict { PACKAGE_PIN AB2 IOSTANDARD TMDS_33 } [get_ports { hdmi_tx_data_n[2] }]

set_property -dict { PACKAGE_PIN AB13 IOSTANDARD LVCMOS25 } [get_ports { hdmi_tx_hpd }]

## Present top-level contract includes hdmi_tx_en.
## Board-native TXEN pin exists on the buffered HDMI sink side; mapped here to preserve
## the active top-level port contract.
set_property -dict { PACKAGE_PIN R3 IOSTANDARD LVCMOS33 } [get_ports { hdmi_tx_en }]





##==============================================================================
## AquaFusion Nexys Video — CLS first-light debug XDC slice
## PURPOSE
##   - Keep the existing CLS/UART debug path alive
##   - Bring up OV5640 SCCB control on JA
##   - Bring in the Pcam 5C MIPI clock + 2 data lanes on JB
##
## POLICY
##   - CLS UART TX from FPGA uses JB10
##   - JB10 maps to FPGA pin Y7 on Nexys Video
##   - Camera auxiliary control pins stay on JA
##   - Camera MIPI differential inputs occupy dedicated JB pairs
##   - D-PHY / CSI timing exceptions and generated clocks belong with the
##     receiver IP wrapper, not as ad hoc hand-written constraints here
##==============================================================================

##----------------------------------------------------------------------------
## VADJ control for FMC banks 15/16
##----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN AA13 IOSTANDARD LVCMOS25 PULLUP true } [get_ports { set_vadj[0] }]
set_property -dict { PACKAGE_PIN AB17 IOSTANDARD LVCMOS25 PULLUP true } [get_ports { set_vadj[1] }]
set_property -dict { PACKAGE_PIN V14  IOSTANDARD LVCMOS25 PULLUP true } [get_ports { vadj_en }]

##----------------------------------------------------------------------------
## FMC Pcam Adapter control + SCCB
##----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN M13 IOSTANDARD LVCMOS25 } [get_ports { cam_pwup }]
set_property -dict { PACKAGE_PIN L13 IOSTANDARD LVCMOS25 } [get_ports { cam_gpio1_oen_n }]
set_property -dict { PACKAGE_PIN J22 IOSTANDARD LVCMOS25 } [get_ports { cam_gpio1_dir }]
set_property -dict { PACKAGE_PIN G17 IOSTANDARD LVCMOS25 } [get_ports { cam_a_bta_o }]
## W5/V5 share bank 34 with sys_clk_in, so these dedicated SCCB/I2C pins must
## use the bank-34 3.3 V I/O standard. Do not constrain them as VADJ/LVCMOS25.
set_property -dict { PACKAGE_PIN W5  IOSTANDARD LVCMOS33 PULLUP true } [get_ports { cam_scl }]
set_property -dict { PACKAGE_PIN V5  IOSTANDARD LVCMOS33 PULLUP true } [get_ports { cam_sda }]

##----------------------------------------------------------------------------
## FMC Pcam Adapter port A HS receive pairs
## Use DIFF_TERM TRUE for 100-ohm on-die differential termination when external
## termination is absent on the receive path.
##----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN J20 IOSTANDARD LVDS_25 DIFF_TERM TRUE } [get_ports { cam_a_hs_clk_p }]
set_property -dict { PACKAGE_PIN J21 IOSTANDARD LVDS_25 DIFF_TERM TRUE } [get_ports { cam_a_hs_clk_n }]
set_property -dict { PACKAGE_PIN K21 IOSTANDARD LVDS_25 DIFF_TERM TRUE } [get_ports { cam_a_hs_lane0_p }]
set_property -dict { PACKAGE_PIN K22 IOSTANDARD LVDS_25 DIFF_TERM TRUE } [get_ports { cam_a_hs_lane0_n }]
set_property -dict { PACKAGE_PIN N22 IOSTANDARD LVDS_25 DIFF_TERM TRUE } [get_ports { cam_a_hs_lane1_p }]
set_property -dict { PACKAGE_PIN M22 IOSTANDARD LVDS_25 DIFF_TERM TRUE } [get_ports { cam_a_hs_lane1_n }]

##----------------------------------------------------------------------------
## FMC Pcam Adapter port A LP signals
##----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN M15 IOSTANDARD LVCMOS25 } [get_ports { cam_a_lp_clk_p }]
set_property -dict { PACKAGE_PIN M16 IOSTANDARD LVCMOS25 } [get_ports { cam_a_lp_clk_n }]
set_property -dict { PACKAGE_PIN N18 IOSTANDARD LVCMOS25 } [get_ports { cam_a_lp_lane0_p }]
set_property -dict { PACKAGE_PIN N19 IOSTANDARD LVCMOS25 } [get_ports { cam_a_lp_lane0_n }]
set_property -dict { PACKAGE_PIN L19 IOSTANDARD LVCMOS25 } [get_ports { cam_a_lp_lane1_p }]
set_property -dict { PACKAGE_PIN L20 IOSTANDARD LVCMOS25 } [get_ports { cam_a_lp_lane1_n }]

##----------------------------------------------------------------------------
## Sonar 1 on JA
##   JA3 <- UART
##   JA4 <- PWM
##----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN AB20 IOSTANDARD LVCMOS33 PULLUP true } [get_ports { sonar1_uart_i }]
set_property -dict { PACKAGE_PIN AB18 IOSTANDARD LVCMOS33 PULLDOWN true } [get_ports { sonar1_pwm_i }]

##----------------------------------------------------------------------------
## CLS write-only UART debug on JB
##
## Corrected mapping:
##   cls_txd_o -> JB10 -> FPGA pin Y7
##
## NOTES
##   - This is the FPGA transmit signal going into CLS RXD
##   - The earlier comment claiming JB4 was inconsistent with the actual Y7 LOC
##----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN Y7 IOSTANDARD LVCMOS33 } [get_ports { cls_txd_o }]

##----------------------------------------------------------------------------
## Sonar 2 on JC
##   JC3 <- UART
##   JC4 <- PWM
##----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN AA8 IOSTANDARD LVCMOS33 PULLUP true } [get_ports { sonar2_uart_i }]
set_property -dict { PACKAGE_PIN AB8 IOSTANDARD LVCMOS33 PULLDOWN true } [get_ports { sonar2_pwm_i }]

##----------------------------------------------------------------------------
## Debug outputs on PMOD pins
## dbg_frame_tick is the camera frame pulse.
## Additional camera debug strobes use the freed legacy JA camera pins.
##----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN AA21 IOSTANDARD LVCMOS33 } [get_ports { dbg_frame_tick }]
set_property -dict { PACKAGE_PIN AA20 IOSTANDARD LVCMOS33 } [get_ports { dbg_sonar_update }]
set_property -dict { PACKAGE_PIN AA18 IOSTANDARD LVCMOS33 } [get_ports { dbg_heartbeat }]
set_property -dict { PACKAGE_PIN AB22 IOSTANDARD LVCMOS33 } [get_ports { dbg_cam_sccb_transaction }]
set_property -dict { PACKAGE_PIN AB21 IOSTANDARD LVCMOS33 } [get_ports { dbg_cam_init_done }]
set_property -dict { PACKAGE_PIN Y21  IOSTANDARD LVCMOS33 } [get_ports { dbg_cam_overflow_event }]

##----------------------------------------------------------------------------
## On-board OLED (SSD1306, 128x32)
##----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN U21 IOSTANDARD LVCMOS33 } [get_ports { oled_res_n  }]
set_property -dict { PACKAGE_PIN W22 IOSTANDARD LVCMOS33 } [get_ports { oled_dc     }]
set_property -dict { PACKAGE_PIN W21 IOSTANDARD LVCMOS33 } [get_ports { oled_sclk   }]
set_property -dict { PACKAGE_PIN Y22 IOSTANDARD LVCMOS33 } [get_ports { oled_sdin   }]
set_property -dict { PACKAGE_PIN P20 IOSTANDARD LVCMOS33 } [get_ports { oled_vbat_n }]
set_property -dict { PACKAGE_PIN V22 IOSTANDARD LVCMOS33 } [get_ports { oled_vdd_n  }]
