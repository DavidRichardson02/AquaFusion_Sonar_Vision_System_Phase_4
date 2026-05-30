`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// tmds_oserdes_lane
//------------------------------------------------------------------------------
// PURPOSE
//   Serialize one 10-bit TMDS word onto one differential TMDS data lane using
//   a Xilinx 7-series OSERDESE2 master/slave cascade followed by an OBUFDS.
//
// HIGH-LEVEL ROLE
//   A TMDS encoder produces one 10-bit symbol per pixel clock for each output
//   lane. That 10-bit symbol must then be shifted out serially at 10 bits per
//   pixel interval.
//
//   In a common 7-series transmit architecture:
//
//     - PixelClk  = pixel-rate parallel clock
//     - SerialClk = 5x PixelClk
//     - DDR output is used at SerialClk
//
//   Because DDR launches two bits per SerialClk cycle, a 5x SerialClk can emit
//   10 serial bits during one PixelClk period:
//
//       5 SerialClk cycles * 2 bits/cycle = 10 bits
//
//   Therefore this module performs:
//
//       one 10-bit parallel TMDS word
//         --> OSERDES master/slave serialization
//         --> one single-ended serial stream
//         --> one differential TMDS pair
//
// CONTRACT
//   Inputs:
//     PixelClk
//       Pixel-rate clock. The 10-bit TMDS word is loaded in this domain.
//
//     SerialClk
//       5x high-speed serializer clock used by OSERDESE2 in DDR mode.
//
//     rst
//       Reset for the serializer primitives.
//
//     tmds_word[9:0]
//       Parallel 10-bit TMDS symbol to be serialized.
//       This implementation is wired so bit 0 is launched first, followed by
//       bit 1, ..., up to bit 9, subject to OSERDESE2 bit ordering conventions.
//
//   Outputs:
//     tmds_p, tmds_n
//       Differential TMDS lane outputs driven through OBUFDS.
//
// IMPORTANT DEVICE / TOOL ASSUMPTIONS
//   - This module is written for Xilinx 7-series style OSERDESE2 primitives.
//   - DATA_WIDTH=10 requires a MASTER/SLAVE cascade.
//   - PixelClk and SerialClk must have the correct frequency relationship:
//         SerialClk = 5 * PixelClk
//   - The surrounding design must ensure proper clocking, placement, reset
//     release, and output constraints appropriate for TMDS transmission.
//
// BIT MAPPING
//   In this architecture:
//
//     MASTER carries the lower 8 bits:
//       D1..D8 <= tmds_word[0]..tmds_word[7]
//
//     SLAVE carries the upper 2 bits:
//       D3     <= tmds_word[8]
//       D4     <= tmds_word[9]
//
//   The master/slave cascade then forms the full 10-bit serializer load.
//
// DESIGN NOTES
//   - OCE is held high permanently; a new word is loaded every PixelClk cycle.
//   - Tristate control is disabled; the lane always actively drives.
//   - The output of the master OSERDES is single-ended and then converted to
//     the differential pair using OBUFDS with TMDS_33 I/O standard.
//
//==============================================================================
module tmds_oserdes_lane (
    input  wire       PixelClk,
    input  wire       SerialClk,
    input  wire       rst,
    input  wire [9:0] tmds_word,
    output wire       tmds_p,
    output wire       tmds_n
);

    //--------------------------------------------------------------------------
    // Cascade interconnect between SLAVE and MASTER OSERDESE2 instances
    //--------------------------------------------------------------------------
    // The slave provides upper-bit serialization support to the master through
    // the SHIFTIN/SHIFTOUT cascade ports.
    //--------------------------------------------------------------------------
    wire shift1;
    wire shift2;

    //--------------------------------------------------------------------------
    // Single-ended serialized output from the MASTER OSERDES
    //--------------------------------------------------------------------------
    // This signal is later driven into a differential output buffer.
    //--------------------------------------------------------------------------
    wire tmds_se;

    //--------------------------------------------------------------------------
    // SLAVE OSERDES: supplies the upper serializer bits for DATA_WIDTH=10
    //--------------------------------------------------------------------------
    // For DATA_WIDTH=10 in DDR mode on 7-series, the MASTER handles the lower
    // 8 bits while the SLAVE contributes the remaining upper bits through the
    // internal shift chain.
    //
    // In this mapping:
    //   tmds_word[8] -> D3
    //   tmds_word[9] -> D4
    //
    // D1, D2, D5, D6, D7, D8 are unused for this 10-bit configuration and are
    // tied low.
    //--------------------------------------------------------------------------
    OSERDESE2 #(
        .DATA_RATE_OQ ("DDR"),
        .DATA_RATE_TQ ("BUF"),
        .DATA_WIDTH   (10),
        .SERDES_MODE  ("SLAVE"),
        .TRISTATE_WIDTH(1),
        .TBYTE_CTL    ("FALSE"),
        .TBYTE_SRC    ("FALSE")
    ) u_oserdes_slave (
        .OQ         (),
        .OFB        (),
        .TQ         (),
        .TFB        (),

        .SHIFTOUT1  (shift1),
        .SHIFTOUT2  (shift2),
        .SHIFTIN1   (1'b0),
        .SHIFTIN2   (1'b0),

        .CLK        (SerialClk),
        .CLKDIV     (PixelClk),

        .D1         (1'b0),
        .D2         (1'b0),
        .D3         (tmds_word[8]),
        .D4         (tmds_word[9]),
        .D5         (1'b0),
        .D6         (1'b0),
        .D7         (1'b0),
        .D8         (1'b0),

        .OCE        (1'b1),
        .RST        (rst),

        .T1         (1'b0),
        .T2         (1'b0),
        .T3         (1'b0),
        .T4         (1'b0),
        .TBYTEIN    (1'b0),
        .TCE        (1'b0)
    );

    //--------------------------------------------------------------------------
    // MASTER OSERDES: drives the lane output and accepts lower 8 bits directly
    //--------------------------------------------------------------------------
    // Bit mapping:
    //   D1 <= tmds_word[0]
    //   D2 <= tmds_word[1]
    //   D3 <= tmds_word[2]
    //   D4 <= tmds_word[3]
    //   D5 <= tmds_word[4]
    //   D6 <= tmds_word[5]
    //   D7 <= tmds_word[6]
    //   D8 <= tmds_word[7]
    //
    // The remaining two bits are received from the SLAVE through SHIFTIN1/2.
    //--------------------------------------------------------------------------
    OSERDESE2 #(
        .DATA_RATE_OQ ("DDR"),
        .DATA_RATE_TQ ("BUF"),
        .DATA_WIDTH   (10),
        .SERDES_MODE  ("MASTER"),
        .TRISTATE_WIDTH(1),
        .TBYTE_CTL    ("FALSE"),
        .TBYTE_SRC    ("FALSE")
    ) u_oserdes_master (
        .OQ         (tmds_se),
        .OFB        (),
        .TQ         (),
        .TFB        (),

        .SHIFTOUT1  (),
        .SHIFTOUT2  (),
        .SHIFTIN1   (shift1),
        .SHIFTIN2   (shift2),

        .CLK        (SerialClk),
        .CLKDIV     (PixelClk),

        .D1         (tmds_word[0]),
        .D2         (tmds_word[1]),
        .D3         (tmds_word[2]),
        .D4         (tmds_word[3]),
        .D5         (tmds_word[4]),
        .D6         (tmds_word[5]),
        .D7         (tmds_word[6]),
        .D8         (tmds_word[7]),

        .OCE        (1'b1),
        .RST        (rst),

        .T1         (1'b0),
        .T2         (1'b0),
        .T3         (1'b0),
        .T4         (1'b0),
        .TBYTEIN    (1'b0),
        .TCE        (1'b0)
    );

    //--------------------------------------------------------------------------
    // Differential output buffer
    //--------------------------------------------------------------------------
    // The serialized single-ended TMDS stream from the MASTER OSERDES is
    // converted into a differential TMDS pair for board-level output.
    //
    // TMDS_33 is the standard I/O setting used for simple HDMI/DVI transmitter
    // implementations on 3.3 V compatible FPGA banks, subject to board/device
    // support and implementation constraints.
    //--------------------------------------------------------------------------
    OBUFDS #(
        .IOSTANDARD("TMDS_33"),
        .SLEW      ("FAST")
    ) u_obufds_tmds_data (
        .I  (tmds_se),
        .O  (tmds_p),
        .OB (tmds_n)
    );

endmodule

`default_nettype wire