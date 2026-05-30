`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// camera_bearing_publisher_sys
//------------------------------------------------------------------------------
// ROLE
//   SYS-domain publisher for camera-registration bearing.
//
// PURPOSE
//   Accept a future camera-derived alignment/bearing update and publish:
//
//     1) cam_align_bearing_deg_sys
//        - integer degrees, intended for SYS-domain painter logic
//
//     2) bearing_snap_sys + bearing_snap_upd_sys
//        - compact snapshot bus intended for explicit SYS->VID transfer
//
// BEARING CONVENTION
//   - Integer degrees in [0,359]
//   - 0 degrees is a frozen project reference direction
//   - Positive angular direction is defined by the upstream registration block
//
// UPDATE POLICY
//   - On bearing_upd_sys, input bearing is clamped (if enabled) and published
//   - Output remains stable until next published update
//
// SNAPSHOT FORMAT
//   [8:0]   bearing_deg
//   [9]     valid
//   [17:10] seq
//   [31:18] reserved
//==============================================================================
module camera_bearing_publisher_sys #(
    parameter integer CLAMP_EN = 1
)(
    input  wire        clk_sys,
    input  wire        rst_sys,

    input  wire [8:0]  bearing_deg_in_sys,
    input  wire        bearing_valid_sys,
    input  wire        bearing_upd_sys,

    output reg  [8:0]  cam_align_bearing_deg_sys,

    output reg  [31:0] bearing_snap_sys,
    output reg         bearing_snap_upd_sys
);

    reg [7:0] seq_sys;
    reg [8:0] bearing_deg_eff_sys;

    always @(*) begin
        if (bearing_deg_in_sys < 9'd360)
            bearing_deg_eff_sys = bearing_deg_in_sys;
        else if (CLAMP_EN != 0)
            bearing_deg_eff_sys = 9'd359;
        else
            bearing_deg_eff_sys = 9'd0;
    end

    always @(posedge clk_sys) begin
        if (rst_sys) begin
            seq_sys                   <= 8'd0;
            cam_align_bearing_deg_sys <= 9'd0;
            bearing_snap_sys          <= 32'd0;
            bearing_snap_upd_sys      <= 1'b0;
        end else begin
            bearing_snap_upd_sys <= 1'b0;

            if (bearing_upd_sys && bearing_valid_sys) begin
                cam_align_bearing_deg_sys <= bearing_deg_eff_sys;

                bearing_snap_sys[8:0]    <= bearing_deg_eff_sys;
                bearing_snap_sys[9]      <= 1'b1;
                bearing_snap_sys[17:10]  <= seq_sys;
                bearing_snap_sys[31:18]  <= 14'd0;

                bearing_snap_upd_sys     <= 1'b1;
                seq_sys                  <= seq_sys + 8'd1;
            end
        end
    end

endmodule

`default_nettype wire