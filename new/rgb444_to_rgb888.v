`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// rgb444_to_rgb888
//------------------------------------------------------------------------------
// Expand RGB444 into RGB888 by nibble replication.
//   4'hA -> 8'hAA
//==============================================================================
module rgb444_to_rgb888 (
    input  wire [11:0] rgb444,
    output wire [23:0] rgb888
);
    assign rgb888 = {
        rgb444[11:8], rgb444[11:8],
        rgb444[7:4],  rgb444[7:4],
        rgb444[3:0],  rgb444[3:0]
    };
endmodule

`default_nettype wire