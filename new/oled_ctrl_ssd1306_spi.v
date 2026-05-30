`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// oled_ctrl_ssd1306_spi
//------------------------------------------------------------------------------
// ROLE
//   Deterministic SSD1306 controller for the on-board Nexys Video 128x32 OLED.
//
// HIGH-LEVEL PURPOSE
//   This module is the active controller for the SSD1306-based OLED display.
//   It is responsible for the complete bring-up and refresh lifecycle of the
//   panel, including:
//
//     1) power sequencing
//     2) reset timing
//     3) initialization-command playback
//     4) enabling panel power (VBAT)
//     5) writing all display RAM bytes
//     6) issuing the final Display ON command
//     7) refreshing the display on demand or at a periodic low rate
//
//   This is not merely an SPI shifter. It is the policy engine for the OLED.
//
// SYSTEM-LEVEL CONTRACT
//   - Entire controller runs in the single `clk` domain.
//   - Display contents are fetched from an external byte-addressed source using:
//
//         byte_addr -> byte_data
//
//   - Full-screen refresh is performed over all 4 pages x 128 columns = 512
//     bytes for a 128x32 monochrome display.
//
//   - Refresh is triggered by:
//       * `line_upd`
//       * or a periodic timeout determined by REFRESH_HZ
//
// BOARD / DEVICE ASSUMPTIONS
//   The design comments and implementation assume the Nexys Video onboard OLED
//   control style, including:
//
//     - CS# is hard-wired low on the board
//     - D/C# selects command/data
//     - SPI data is sent MSB-first
//     - data is prepared while SCLK is low and held through the rising edge
//
// POWER-UP / STARTUP POLICY
//   The implemented startup sequence is:
//
//     1) drive VDD# low, then wait POWERUP_WAIT_MS
//     2) drive RES# low for RESET_LOW_US
//     3) release RES# high and wait RESET_HIGH_US
//     4) send the initialization command ROM sequence
//     5) drive VBAT# low and wait VBAT_WAIT_MS
//     6) write all display RAM bytes
//     7) send 0xAF (Display ON)
//     8) enter idle refresh service
//
//   This policy intentionally delays visible display enable until after power,
//   initialization, and framebuffer write have completed.
//
// ADDRESSING MODEL
//   The display RAM is treated as:
//
//     4 pages x 128 columns = 512 bytes
//
//   The external fetch address is formed as:
//
//       byte_addr = {page[1:0], col[6:0]}
//
//   so:
//
//       page 0, col 0   -> byte_addr 0
//       ...
//       page 3, col 127 -> byte_addr 511
//
// WHY THIS MODULE EXISTS
//   OLED control involves both slow board-level timing and fast byte-level SPI.
//   Combining these carefully in a deterministic state machine ensures that the
//   display is always brought up and refreshed in a known, reviewable way.
//
// IMPORTANT IMPLEMENTATION NOTE
//   The framebuffer fetch path is intentionally split into two states:
//
//       ST_PAGE_DATA_ADDR
//       ST_PAGE_DATA_LAUNCH
//
//   so that `byte_addr` is presented first, and `byte_data` is sampled only on
//   the following clock edge. This avoids same-cycle address/data hazards.
//
// IMPORTANT DESIGN STYLE
//   This module is deliberately deterministic and single-threaded:
//   only one SPI byte is ever in flight at a time, and the higher-level FSM
//   advances only when the byte transmitter reports completion.
//------------------------------------------------------------------------------
module oled_ctrl_ssd1306_spi #(
    parameter integer CLK_HZ          = 100_000_000,
    parameter integer SPI_HZ          = 10_000_000,
    parameter integer POWERUP_WAIT_MS = 1,
    parameter integer RESET_LOW_US    = 10,
    parameter integer RESET_HIGH_US   = 10,
    parameter integer VBAT_WAIT_MS    = 100,
    parameter integer REFRESH_HZ      = 5
)(
    input  wire       clk,
    input  wire       rst,

    input  wire       line_upd,
    output reg  [8:0] byte_addr,
    input  wire [7:0] byte_data,

    output reg        oled_res_n,
    output reg        oled_dc,
    output reg        oled_sclk,
    output reg        oled_sdin,
    output reg        oled_vbat_n,
    output reg        oled_vdd_n
);

    //--------------------------------------------------------------------------
    // Safe parameter reduction
    //--------------------------------------------------------------------------
    localparam integer SPI_HZ_SAFE      = (SPI_HZ     < 1) ? 1 : SPI_HZ;
    localparam integer REFRESH_HZ_SAFE  = (REFRESH_HZ < 1) ? 1 : REFRESH_HZ;

    localparam integer SPI_DIV_RAW      = CLK_HZ / (SPI_HZ_SAFE * 2);
    localparam integer TICKS_PER_MS_RAW = CLK_HZ / 1000;
    localparam integer TICKS_PER_US_RAW = CLK_HZ / 1000000;
    localparam integer REFRESH_RAW      = CLK_HZ / REFRESH_HZ_SAFE;

    localparam integer SPI_DIV          = (SPI_DIV_RAW      < 1) ? 1 : SPI_DIV_RAW;
    localparam integer TICKS_PER_MS     = (TICKS_PER_MS_RAW < 1) ? 1 : TICKS_PER_MS_RAW;
    localparam integer TICKS_PER_US     = (TICKS_PER_US_RAW < 1) ? 1 : TICKS_PER_US_RAW;
    localparam integer PERIODIC_REFRESH = (REFRESH_RAW      < 1) ? 1 : REFRESH_RAW;

    //--------------------------------------------------------------------------
    // Controller states
    //--------------------------------------------------------------------------
    localparam [4:0]
        ST_BOOT_OFF          = 5'd0,
        ST_VDD_WAIT          = 5'd1,
        ST_RESET_LOW         = 5'd2,
        ST_RESET_HIGH        = 5'd3,
        ST_INIT_FETCH        = 5'd4,
        ST_INIT_SEND         = 5'd5,
        ST_INIT_ADV          = 5'd6,
        ST_VBAT_WAIT         = 5'd7,
        ST_REFRESH_BEGIN     = 5'd8,
        ST_PAGE_SET_CMD      = 5'd9,
        ST_PAGE_SET_LO       = 5'd10,
        ST_PAGE_SET_HI       = 5'd11,
        ST_PAGE_DATA_ADDR    = 5'd12,
        ST_PAGE_DATA_LAUNCH  = 5'd13,
        ST_PAGE_DATA_SEND    = 5'd14,
        ST_PAGE_DATA_ADV     = 5'd15,
        ST_DISPLAY_ON_START  = 5'd16,
        ST_DISPLAY_ON_WAIT   = 5'd17,
        ST_IDLE              = 5'd18;

    reg [4:0] state;

    //--------------------------------------------------------------------------
    // Long-timescale control
    //--------------------------------------------------------------------------
    reg [31:0] wait_ctr;
    reg [31:0] refresh_ctr;
    reg        refresh_pending;
    reg        display_on_sent;

    //--------------------------------------------------------------------------
    // Init ROM interface
    //--------------------------------------------------------------------------
    reg  [5:0] init_index;
    wire [7:0] init_data;
    wire       init_valid;
    wire       init_last;

    //--------------------------------------------------------------------------
    // SPI byte transmitter control
    //--------------------------------------------------------------------------
    reg  [7:0] tx_byte;
    reg  [2:0] tx_bit_idx;
    reg  [1:0] tx_phase;
    reg        tx_active;
    reg        tx_start;
    reg        tx_done_pulse;
    reg        tx_dc_next;

    //--------------------------------------------------------------------------
    // Page / column traversal
    //--------------------------------------------------------------------------
    reg [1:0] page_idx;
    reg [6:0] col_idx;

    //--------------------------------------------------------------------------
    // SPI tick generation
    //--------------------------------------------------------------------------
    reg [31:0] spi_div_ctr;
    reg        spi_tick;

    //--------------------------------------------------------------------------
    // Init ROM
    //--------------------------------------------------------------------------
    oled_init_rom u_oled_init_rom (
        .index (init_index),
        .data  (init_data),
        .valid (init_valid),
        .last  (init_last)
    );

    //==========================================================================
    // 1) SPI half-period tick generator
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            spi_div_ctr <= 32'd0;
            spi_tick    <= 1'b0;
        end else if (spi_div_ctr >= SPI_DIV-1) begin
            spi_div_ctr <= 32'd0;
            spi_tick    <= 1'b1;
        end else begin
            spi_div_ctr <= spi_div_ctr + 32'd1;
            spi_tick    <= 1'b0;
        end
    end

    //==========================================================================
    // 2) Single-byte SPI transmitter
    //--------------------------------------------------------------------------
    // SPI behavior:
    //   - MSB first
    //   - idle clock low
    //   - update data while clock low
    //   - receiving device samples on rising edge
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            tx_phase      <= 2'd0;
            tx_active     <= 1'b0;
            tx_done_pulse <= 1'b0;
            tx_bit_idx    <= 3'd7;

            oled_sclk     <= 1'b0;
            oled_sdin     <= 1'b0;
            oled_dc       <= 1'b0;
        end else begin
            tx_done_pulse <= 1'b0;

            if (tx_start && !tx_active) begin
                tx_active  <= 1'b1;
                tx_phase   <= 2'd0;
                tx_bit_idx <= 3'd7;

                oled_sclk  <= 1'b0;
                oled_dc    <= tx_dc_next;
                oled_sdin  <= tx_byte[7];
            end else if (tx_active && spi_tick) begin
                case (tx_phase)
                    2'd0: begin
                        oled_sclk <= 1'b1;
                        tx_phase  <= 2'd1;
                    end

                    2'd1: begin
                        oled_sclk <= 1'b0;
                        tx_phase  <= 2'd2;
                    end

                    default: begin
                        if (tx_bit_idx == 3'd0) begin
                            tx_active     <= 1'b0;
                            tx_done_pulse <= 1'b1;
                            tx_phase      <= 2'd0;
                        end else begin
                            tx_bit_idx <= tx_bit_idx - 3'd1;
                            oled_sdin  <= tx_byte[tx_bit_idx - 3'd1];
                            tx_phase   <= 2'd0;
                        end
                    end
                endcase
            end
        end
    end

    //==========================================================================
    // 3) Main controller FSM
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            state           <= ST_BOOT_OFF;
            wait_ctr        <= 32'd0;
            refresh_ctr     <= 32'd0;
            refresh_pending <= 1'b0;
            display_on_sent <= 1'b0;

            init_index      <= 6'd0;

            tx_byte         <= 8'd0;
            tx_dc_next      <= 1'b0;
            tx_start        <= 1'b0;

            page_idx        <= 2'd0;
            col_idx         <= 7'd0;
            byte_addr       <= 9'd0;

            oled_res_n      <= 1'b1;
            oled_vbat_n     <= 1'b1;
            oled_vdd_n      <= 1'b1;
        end else begin
            //------------------------------------------------------------------
            // Default one-cycle launch pulse clearing
            //------------------------------------------------------------------
            tx_start <= 1'b0;

            //------------------------------------------------------------------
            // Periodic refresh scheduler
            //------------------------------------------------------------------
            if (refresh_ctr >= PERIODIC_REFRESH-1)
                refresh_ctr <= 32'd0;
            else
                refresh_ctr <= refresh_ctr + 32'd1;

            if (line_upd)
                refresh_pending <= 1'b1;
            else if (refresh_ctr >= PERIODIC_REFRESH-1)
                refresh_pending <= 1'b1;

            case (state)

                //==============================================================
                // Startup sequencing
                //==============================================================
                ST_BOOT_OFF: begin
                    oled_res_n      <= 1'b1;
                    oled_vbat_n     <= 1'b1;
                    oled_vdd_n      <= 1'b1;

                    display_on_sent <= 1'b0;
                    init_index      <= 6'd0;
                    page_idx        <= 2'd0;
                    col_idx         <= 7'd0;
                    byte_addr       <= 9'd0;
                    wait_ctr        <= 32'd0;

                    state           <= ST_VDD_WAIT;
                end

                ST_VDD_WAIT: begin
                    oled_vdd_n <= 1'b0;

                    if (wait_ctr >= (POWERUP_WAIT_MS * TICKS_PER_MS) - 1) begin
                        wait_ctr   <= 32'd0;
                        oled_res_n <= 1'b0;
                        state      <= ST_RESET_LOW;
                    end else begin
                        wait_ctr <= wait_ctr + 32'd1;
                    end
                end

                ST_RESET_LOW: begin
                    if (wait_ctr >= (RESET_LOW_US * TICKS_PER_US) - 1) begin
                        wait_ctr   <= 32'd0;
                        oled_res_n <= 1'b1;
                        state      <= ST_RESET_HIGH;
                    end else begin
                        wait_ctr <= wait_ctr + 32'd1;
                    end
                end

                ST_RESET_HIGH: begin
                    if (wait_ctr >= (RESET_HIGH_US * TICKS_PER_US) - 1) begin
                        wait_ctr   <= 32'd0;
                        init_index <= 6'd0;
                        state      <= ST_INIT_FETCH;
                    end else begin
                        wait_ctr <= wait_ctr + 32'd1;
                    end
                end

                //==============================================================
                // Initialization ROM playback
                //==============================================================
                ST_INIT_FETCH: begin
                    if (!tx_active) begin
                        if (init_valid) begin
                            tx_byte    <= init_data;
                            tx_dc_next <= 1'b0;
                            tx_start   <= 1'b1;
                            state      <= ST_INIT_SEND;
                        end else begin
                            oled_vbat_n <= 1'b0;
                            wait_ctr    <= 32'd0;
                            state       <= ST_VBAT_WAIT;
                        end
                    end
                end

                ST_INIT_SEND: begin
                    if (tx_done_pulse)
                        state <= ST_INIT_ADV;
                end

                ST_INIT_ADV: begin
                    if (init_last || !init_valid) begin
                        oled_vbat_n <= 1'b0;
                        wait_ctr    <= 32'd0;
                        state       <= ST_VBAT_WAIT;
                    end else begin
                        init_index <= init_index + 6'd1;
                        state      <= ST_INIT_FETCH;
                    end
                end

                //==============================================================
                // Panel power stabilization
                //==============================================================
                ST_VBAT_WAIT: begin
                    if (wait_ctr >= (VBAT_WAIT_MS * TICKS_PER_MS) - 1) begin
                        wait_ctr  <= 32'd0;
                        page_idx  <= 2'd0;
                        col_idx   <= 7'd0;
                        byte_addr <= 9'd0;
                        state     <= ST_REFRESH_BEGIN;
                    end else begin
                        wait_ctr <= wait_ctr + 32'd1;
                    end
                end

                //==============================================================
                // Full-screen refresh sequencing
                //==============================================================
                ST_REFRESH_BEGIN: begin
                    page_idx  <= 2'd0;
                    col_idx   <= 7'd0;
                    byte_addr <= 9'd0;
                    state     <= ST_PAGE_SET_CMD;
                end

                ST_PAGE_SET_CMD: begin
                    if (!tx_active) begin
                        tx_byte    <= 8'hB0 | {6'd0, page_idx};
                        tx_dc_next <= 1'b0;
                        tx_start   <= 1'b1;
                        state      <= ST_PAGE_SET_LO;
                    end
                end

                ST_PAGE_SET_LO: begin
                    if (tx_done_pulse) begin
                        tx_byte    <= 8'h00;
                        tx_dc_next <= 1'b0;
                        tx_start   <= 1'b1;
                        state      <= ST_PAGE_SET_HI;
                    end
                end

                ST_PAGE_SET_HI: begin
                    if (tx_done_pulse) begin
                        tx_byte    <= 8'h10;
                        tx_dc_next <= 1'b0;
                        tx_start   <= 1'b1;
                        state      <= ST_PAGE_DATA_ADDR;
                    end
                end

                //--------------------------------------------------------------------------
                // Critical two-step fetch:
                //   ST_PAGE_DATA_ADDR   : drive byte_addr
                //   ST_PAGE_DATA_LAUNCH : sample byte_data and launch SPI
                //--------------------------------------------------------------------------
                ST_PAGE_DATA_ADDR: begin
                    byte_addr <= {page_idx, col_idx};
                    state     <= ST_PAGE_DATA_LAUNCH;
                end

                ST_PAGE_DATA_LAUNCH: begin
                    if (!tx_active) begin
                        tx_byte    <= byte_data;
                        tx_dc_next <= 1'b1;
                        tx_start   <= 1'b1;
                        state      <= ST_PAGE_DATA_SEND;
                    end
                end

                ST_PAGE_DATA_SEND: begin
                    if (tx_done_pulse)
                        state <= ST_PAGE_DATA_ADV;
                end

                ST_PAGE_DATA_ADV: begin
                    if (col_idx == 7'd127) begin
                        col_idx <= 7'd0;

                        if (page_idx == 2'd3) begin
                            if (!display_on_sent) begin
                                state <= ST_DISPLAY_ON_START;
                            end else begin
                                refresh_pending <= 1'b0;
                                state           <= ST_IDLE;
                            end
                        end else begin
                            page_idx <= page_idx + 2'd1;
                            state    <= ST_PAGE_SET_CMD;
                        end
                    end else begin
                        col_idx <= col_idx + 7'd1;
                        state   <= ST_PAGE_DATA_ADDR;
                    end
                end

                //==============================================================
                // Final display enable
                //==============================================================
                ST_DISPLAY_ON_START: begin
                    if (!tx_active) begin
                        tx_byte    <= 8'hAF;
                        tx_dc_next <= 1'b0;
                        tx_start   <= 1'b1;
                        state      <= ST_DISPLAY_ON_WAIT;
                    end
                end

                ST_DISPLAY_ON_WAIT: begin
                    if (tx_done_pulse) begin
                        display_on_sent <= 1'b1;
                        refresh_pending <= 1'b0;
                        state           <= ST_IDLE;
                    end
                end

                //==============================================================
                // Idle / refresh service
                //==============================================================
                ST_IDLE: begin
                    if (refresh_pending) begin
                        page_idx  <= 2'd0;
                        col_idx   <= 7'd0;
                        byte_addr <= 9'd0;
                        state     <= ST_REFRESH_BEGIN;
                    end
                end

                //==============================================================
                // Default recovery
                //==============================================================
                default: begin
                    state <= ST_BOOT_OFF;
                end
            endcase
        end
    end

endmodule

`default_nettype wire