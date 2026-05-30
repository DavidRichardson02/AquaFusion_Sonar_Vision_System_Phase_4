`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// bram_1r1w_u8
//------------------------------------------------------------------------------
// ROLE
//   Simple synchronous 1-read / 1-write byte-wide block RAM wrapper.
//
// PURPOSE
//   Provide a compact, synthesizable memory primitive abstraction for a 2-D
//   phosphor plane stored as a 1-D linear array.
//
// MEMORY MODEL
//   - One shared clock.
//   - One write port.
//   - One synchronous read port.
//   - Read data updates on the active clock edge from the supplied raddr.
//
// ADDRESSING MODEL
//   The logical 2-D plane of size W x H is flattened into:
//
//       linear_addr = row * W + col
//
//   Address generation is expected to occur outside this wrapper.
//
// SYNTHESIS INTENT
//   The ram_style attribute requests block-RAM inference where supported.
//==============================================================================
module bram_1r1w_u8 #(
    parameter integer W  = 128,
    parameter integer H  = 128,
    parameter integer AW = 14   // 2^AW >= W*H
) (
    input  wire              clk,

    input  wire              we,
    input  wire [AW-1:0]     waddr,
    input  wire [7:0]        wdata,

    input  wire [AW-1:0]     raddr,
    output reg  [7:0]        rdata
);

    //--------------------------------------------------------------------------
    // DEPTH
    //--------------------------------------------------------------------------
    // Total number of addressable cells in the flattened 2-D memory.
    //--------------------------------------------------------------------------
    localparam integer DEPTH = W*H;

    //--------------------------------------------------------------------------
    // Storage array
    //--------------------------------------------------------------------------
    // Each element stores one 8-bit phosphor intensity value.
    //--------------------------------------------------------------------------
    (* ram_style = "block" *)
    reg [7:0] mem [0:DEPTH-1];

    //--------------------------------------------------------------------------
    // SEQUENTIAL MEMORY CONTRACT
    //
    // STEP-BY-STEP
    //   1) If we is asserted, write wdata into mem[waddr].
    //   2) Independently of write activity, update rdata from mem[raddr].
    //
    // READ-DURING-WRITE NOTE
    //   If waddr == raddr in the same cycle, the exact returned value may be
    //   implementation-dependent after synthesis, depending on target-device
    //   BRAM semantics. Upstream logic should not rely on unspecified same-
    //   address read-during-write behavior unless the target device contract is
    //   explicitly frozen.
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (we)
            mem[waddr] <= wdata;

        rdata <= mem[raddr];
    end

endmodule

`default_nettype wire