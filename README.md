# AquaFusion Sonar Vision System Phase 4

FPGA-based sonar and camera fusion display for the Digilent Nexys Video platform.
The design integrates Pcam 5C camera bring-up, sonar acquisition, telemetry
publication, explicit clock-domain crossing, frame-committed HUD rendering,
HDMI/TMDS scanout, OLED telemetry, and debug-output plumbing.

Suggested GitHub repository description:

```text
Nexys Video FPGA sensor-fusion HUD integrating Pcam 5C camera input, sonar acquisition, CDC-safe telemetry snapshots, HDMI compositing, OLED telemetry, and debug outputs.
```

## Project Status

This repository captures the Phase 4 integration state of the AquaFusion sonar
vision system. The current engineering focus is the HDMI rendering path,
especially reliable composition of camera, sonar map, radar, painter, UART, and
debug HUD surfaces.

Recent work in this project/discussion established:

- A region-qualified HDMI compositor path that does not treat black pixels as
  transparency.
- A dedicated sonar map debug view to isolate map rendering from the rest of the
  HUD gates.
- A fallback visual shell for the sonar map panel so an empty map is still
  visible during bring-up.
- Corrected map-renderer read-latency handling for the dual-port map memory.
- Low-value map-cell palette handling so weak/nonzero occupancy data is visible.
- A bounded camera viewport widget for HUD modes, while retaining full-screen
  camera diagnostic mode.
- A post-reset publication fix for the auxiliary view selector so the default
  view is committed after reset rather than lost during reset.

## Target Platform

| Item | Value |
| --- | --- |
| FPGA board | Digilent Nexys Video |
| Camera | Digilent Pcam 5C through FMC Pcam Adapter, Port A |
| Sonar inputs | UART and PWM range sources |
| Primary display output | Nexys Video HDMI source port |
| Auxiliary display | SSD1306-style OLED over SPI |
| Toolchain | Xilinx Vivado 2023.2 |
| Primary top module | `aquafusion_nexys_video_top` |
| Project file | `AquaFusion_Sonar_Vision_System_Phase_4.xpr` |

## Design Doctrine

The top-level design follows a strict publication and rendering discipline:

```text
SYS-domain producers
    -> stable packed snapshots / telemetry buses
    -> explicit SYS-to-VID CDC
    -> VID-domain frame-boundary commit
    -> raster-derived rendering
    -> region-qualified HDMI composition
```

The important invariants are:

- No video-domain renderer directly samples moving system-domain state.
- Multi-bit SYS-to-VID transfers use explicit snapshot CDC modules.
- User-visible HUD state is committed at video frame boundaries.
- Physical outputs have single owners at the top level.
- Renderers behave as functions of pixel coordinate, active-video state, and
  frame-stable telemetry.

## Repository Layout

```text
AquaFusion_Sonar_Vision_System_Phase_4.xpr
    Vivado project file.

AquaFusion_Sonar_Vision_System_Phase_4.srcs/sources_1/new/
    Main integration RTL, HUD renderers, sonar modules, HDMI wrapper,
    OLED modules, CDC blocks, and utility renderers.

AquaFusion_Sonar_Vision_System_Phase_4.srcs/constrs_1/new/
    Nexys Video board constraints.

rtl/camera/
    Camera subsystem RTL: Pcam/FMC control, SCCB bring-up, CSI wrapper boundary,
    pixel normalization, frame synchronization, and status publication.

docs/
    Project notes and engineering documentation.

tools/
    Helper scripts and local tooling.
```

Generated Vivado directories such as `.Xil`, `.runs`, `.cache`, `.gen`, `.hw`,
`.sim`, `.ip_user_files`, and `xsim.dir` are intentionally ignored.

## Top-Level Architecture

The canonical top-level is:

```text
AquaFusion_Sonar_Vision_System_Phase_4.srcs/sources_1/new/aquafusion_nexys_video_top.v
```

It integrates these subsystems:

- Clock and reset management.
- Board switch/button conditioning.
- Video timing generation.
- Sonar UART/PWM acquisition and parsing.
- Sonar filtering, watchdog, snapshot packing, and bus48 telemetry.
- SYS-to-VID telemetry crossing.
- Camera control, SCCB initialization, pixel-frame synchronization, and status
  snapshot publication.
- Bearing/synthetic-angle publication for sonar painter and radar direction.
- Sonar occupancy painter and map memory.
- Sonar map, telemetry overlay, radar, rich tile, UART terminal, and debug HUD
  renderers.
- Explicit region-qualified HDMI compositor cascade.
- HDMI/TMDS wrapper.
- OLED telemetry packer and SSD1306 controller.
- Debug outputs and reserved-input sinks.

## Clock Domains

### SYS Domain

`clk_sys` / `rst_sys_local` owns:

- Sonar acquisition, parsing, filtering, watchdogs, and telemetry packing.
- Camera control-plane bring-up.
- Camera status publication.
- OLED telemetry generation.
- CLS/debug publication.
- SYS-side sonar map writes from the occupancy painter.

### VID Domain

`clk_vid` / `rst_vid` owns:

- Raster timing.
- Frame-committed snapshot consumption.
- HUD, map, radar, terminal, and viewport rendering.
- HDMI pixel scanout.

### TMDS Domain

`clk_tmds_5x` / `rst_tmds_5x` owns:

- High-speed TMDS serialization through the HDMI wrapper.

## HDMI Rendering and Composition

The HDMI path is intentionally explicit. Earlier compositor behavior used black
as an implicit transparent key, which is unsafe because many HUD panels
legitimately draw black interiors. The current top-level instead uses explicit
geometry masks for each layer.

Composition policy:

```text
base
    test pattern, full-screen camera diagnostic, or black

stage 0
    bounded camera viewport widget

stage 1
    global debug panel

stage 2
    camera debug tile

stage 3
    UART terminal

stage 4
    sonar rich tile

stage 5
    sonar map panel and telemetry overlay

stage 6
    sonar painter diagnostic tile

stage 7
    sonar radar overlay

stage 8
    active-video clamp before HDMI
```

This makes each layer's visibility a function of its view enable and its
geometric region, not its color value.

## HDMI View Controls

`SW7..SW5` select the auxiliary HDMI view code. Press `BTND` to latch the
selected view into the video domain. Button mapping comes from the Nexys Video
constraints:

| Board control | Signal |
| --- | --- |
| `BTNC` | `btn_in[0]` |
| `BTNU` | `btn_in[1]` |
| `BTNL` | `btn_in[2]` |
| `BTNR` | `btn_in[3]` |
| `BTND` | `btn_in[4]`, aux-view load |

View code mapping:

| `SW7..SW5` | View | HDMI behavior |
| --- | --- | --- |
| `000` | Test pattern | Raster test pattern, no HUD. |
| `001` | Camera only | Full-screen raw camera diagnostic. |
| `010` | Camera HUD | HUD surfaces plus bounded camera viewport. |
| `011` | UART | Camera HUD plus UART terminal. |
| `100` | Map debug | Black base with sonar map panel isolated. |

Other relevant switches:

| Switch | Meaning |
| --- | --- |
| `SW0` | OLED test mode. |
| `SW1` | Sonar 1 raw publication mode. Useful during bring-up. |
| `SW2` | Sonar 1 painter brush mode. |
| `SW4` | UART terminal display mode input. |

## Camera Rendering

The camera path has two visible modes:

1. Full-screen diagnostic mode
   - Select `SW7..SW5 = 001`, then press `BTND`.
   - The HDMI base is the camera RGB stream when valid.

2. Bounded HUD viewport
   - Select `SW7..SW5 = 010` or `011`, then press `BTND`.
   - The camera appears inside a framed viewport widget.
   - The viewport uses VID-domain camera RGB and committed camera status bits.

The viewport widget is a raster clip, not a scaler. It shows the camera pixels
for that screen region and adds a visible status frame:

| Frame color | Meaning |
| --- | --- |
| Blue | Camera boot/init not complete. |
| Amber | Camera initialized or waiting for live frame data. |
| Green | Camera ready/live. |
| Red | Camera initialization failure. |

True arbitrary camera placement or scaling would require an addressable
framebuffer read path or scaler upstream of the widget.

## Sonar Acquisition and Publication

Sonar 1 is the active live sonar path in this revision. Sonar 2 wiring is
reserved/tied off in the current top-level.

Sonar 1 data flow:

```text
UART RX / PWM capture
    -> frame parser / PWM distance sample
    -> optional sonar_filter
    -> selected raw-or-filtered publication path
    -> watchdog and age counters
    -> status snapshot
    -> rich diagnostic snapshot
    -> bus48 telemetry
    -> SYS-to-VID snapshot CDC
    -> HUD, map overlay, radar, debug panel
```

`SW1` controls the raw-vs-filtered publication policy unless the design is built
with `SONAR_FILTER_BYPASS`:

- `SW1 = 1`: publish raw sonar samples.
- `SW1 = 0`: publish filtered sonar samples.

During sensor bring-up, raw mode is useful because it removes filter latency and
filter rejection from the debug loop.

## Sonar Painter and Map Rendering

The sonar painter consumes the selected sonar distance, a direction vector, and
brush mode. It writes an occupancy-style map in the SYS domain. The map memory
is read in the VID domain by the map renderer.

Map path:

```text
sonar_occupancy_painter
    -> sonar_map_mem_tdp_dc
    -> sonar_map_renderer
    -> sonar_map_overlay_telem
    -> rgb444_to_rgb888
    -> region-qualified HDMI compositor
```

The current map panel is 128x128 pixels and is placed at:

```text
x = 8
y = 344
w = 128
h = 128
```

The map debug view is intended to isolate this path:

```text
Set SW7..SW5 = 100
Press BTND
```

Expected result:

- Mostly black HDMI output.
- A visible dark/bordered sonar map panel at the lower-left.
- If the panel appears but no rays/cells appear, the HDMI compositor gate is
  working and the likely fault is upstream: sonar valid pulse, painter map write
  enable, distance conversion, or direction generation.

## Camera Status and Debug Tile

Camera status is packed in the SYS domain, crossed into VID, and consumed by the
camera debug tile. The tile displays control-plane and frame-store state such as
camera mode, format, age, frame count, drop/overflow counters, step index,
retry count, frame dimensions, CSI error flags, and readiness bits.

The camera debug tile is a status renderer. It does not own camera bring-up,
CSI reception, frame synchronization, or HDMI composition.

## OLED Telemetry

The OLED path is SYS-domain only. It consumes sonar, camera, HDMI HPD,
heartbeat, and synchronized frame-counter state, then formats four fixed-width
text lines for the SSD1306 controller.

OLED and HDMI rendering are intentionally isolated. OLED state should not be
treated as an HDMI rendering dependency.

## Bring-Up Procedure

1. Open `AquaFusion_Sonar_Vision_System_Phase_4.xpr` in Vivado 2023.2.
2. Confirm `aquafusion_nexys_video_top` is the active top module.
3. Generate bitstream and program the Nexys Video.
4. Connect HDMI sink and verify HPD/clock status.
5. Start with `SW7..SW5 = 000`, press `BTND`, and confirm the test pattern.
6. Select `SW7..SW5 = 001`, press `BTND`, and confirm full-screen camera mode.
7. Select `SW7..SW5 = 010`, press `BTND`, and confirm HUD plus camera viewport.
8. Select `SW7..SW5 = 100`, press `BTND`, and confirm the isolated sonar map
   panel.
9. Use `SW1 = 1` during sonar bring-up to publish raw sonar samples.
10. Use `SW2` to compare painter brush behavior once samples are accepted.

## Verification Notes

Recent targeted Vivado checks used:

```powershell
xvlog --incr --relax <changed RTL files>
xelab --relax -L xpm -L unisims_ver -L unimacro_ver -L secureip work.aquafusion_nexys_video_top work.glbl
```

The edited HDMI/rendering path parsed and elaborated successfully. Existing
warnings remain in unrelated text/HUD width connections.

## Known Limitations

- The camera viewport is a clipped raster aperture, not a scaler.
- Sonar 2 is structurally present in the top-level naming scheme but disabled
  and tied off in this revision.
- The bearing-publication path is architecturally present, but the current
  source keeps camera/fusion bearing validity low and uses a synthetic fallback
  sweep for painter/radar direction.
- CLS debug bridge integration is present but commented in the current
  top-level.
- The repo should track source and constraints, not generated Vivado build
  products.

## Source Ownership Highlights

Important files:

| File | Purpose |
| --- | --- |
| `aquafusion_nexys_video_top.v` | Board-level integration, CDC wiring, view decode, final HDMI composition. |
| `camera_viewport_widget.v` | Bounded camera viewport surface for HUD modes. |
| `camera_top.v` | Camera subsystem integration and status publication. |
| `camera_frame_sync.v` | Camera frame storage/crossing into VID scanout. |
| `sonar_occupancy_painter.v` | SYS-side sonar occupancy map writer and painter telemetry publisher. |
| `sonar_map_mem_tdp_dc.v` | Dual-clock map memory. |
| `sonar_map_renderer.v` | VID-side map memory reader and map pixel renderer. |
| `sonar_map_overlay_telem.v` | Map panel fallback shell and telemetry overlay. |
| `hud_sonar_radar_overlay_pix.v` | Sonar radar renderer. |
| `hud_sonar_tile_rich.v` | Rich sonar diagnostic tile. |
| `vga_uart_terminal_overlay.v` | UART terminal renderer. |
| `hdmi_tx_wrapper_rgb2dvi.v` | HDMI/TMDS output wrapper. |
| `oled_telemetry_pack_sys.v` | SYS-domain OLED text telemetry packer. |

## GitHub Publishing Notes

The remote repository currently exists at:

```text
https://github.com/DavidRichardson02/AquaFusion_Sonar_Vision_System_Phase_4
```

If Git for Windows is installed locally, publish the source tree with:

```powershell
git init
git branch -M main
git remote add origin https://github.com/DavidRichardson02/AquaFusion_Sonar_Vision_System_Phase_4.git
git add README.md .gitignore AquaFusion_Sonar_Vision_System_Phase_4.xpr AquaFusion_Sonar_Vision_System_Phase_4.srcs/sources_1/new AquaFusion_Sonar_Vision_System_Phase_4.srcs/constrs_1/new rtl docs tools
git commit -m "Initial Vivado project import"
git push -u origin main
```

Do not add generated Vivado output directories unless there is a specific
release-artifact reason to do so.
