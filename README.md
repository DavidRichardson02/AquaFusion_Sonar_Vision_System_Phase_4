# AquaFusion Sonar Vision System Phase 4

Vivado 2023.2 FPGA project for the AquaFusion Nexys Video platform.

## Target

- Board: Digilent Nexys Video
- Toolchain: Xilinx Vivado 2023.2
- Primary top module: `aquafusion_nexys_video_top`
- Project file: `AquaFusion_Sonar_Vision_System_Phase_4.xpr`

## Major Subsystems

- HDMI/TMDS video output
- Pcam 5C camera bring-up and status publication
- Sonar UART/PWM acquisition and filtering
- SYS-to-VID telemetry snapshot crossings
- HUD/debug rendering
- Sonar occupancy map, painter, radar, and rich tile overlays
- OLED telemetry output
- CLS debug UART support

## Source Layout

- `AquaFusion_Sonar_Vision_System_Phase_4.srcs/sources_1/new/` - project RTL and HUD/rendering modules
- `AquaFusion_Sonar_Vision_System_Phase_4.srcs/constrs_1/new/` - Nexys Video constraints
- `rtl/camera/` - camera subsystem RTL
- `docs/` - project notes
- `tools/` - helper scripts

## Repository Policy

The repository should track source, constraints, scripts, docs, and IP configuration files. Vivado generated directories such as `.Xil`, `.runs`, `.cache`, `.gen`, `.hw`, `.sim`, and `.ip_user_files` are intentionally ignored.

## HDMI View Selection

The top-level uses `SW7..SW5` as the auxiliary HDMI view code. Press `BTND` to latch the selected view into the video domain.

- `000`: test pattern
- `001`: full-screen camera diagnostic
- `010`: camera HUD with bounded camera viewport
- `011`: camera HUD plus UART terminal
- `100`: sonar map debug isolation view
