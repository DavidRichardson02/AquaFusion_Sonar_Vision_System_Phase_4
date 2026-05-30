`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// MODULE: csi_axis_to_pixel_stream
//------------------------------------------------------------------------------
// ROLE
//   Convert a vendor/Digilent CSI receiver AXI-stream-like pixel output into
//   the AquaFusion project-local camera pixel stream.
//
// SYSTEM CONTEXT
//   This module sits immediately after a real CSI/D-PHY backend:
//
//       Digilent/licensed CSI-DPHY backend
//           -> AXI-stream pixel bus
//           -> csi_axis_to_pixel_stream
//           -> pcam_csi_rx_external_ip
//           -> pcam_csi_rx_wrapper
//           -> camera_pixel_stream
//
// IMPORTANT BOUNDARY
//   This module does not decode MIPI CSI-2 packets.
//   It assumes a real backend has already recovered and packetized pixels.
//
// AXI-STREAM CONVENTION
//   axis_tvalid:
//     Valid qualifier for axis_tdata, axis_tuser, and axis_tlast.
//
//   axis_tready:
//     Backpressure output. This adapter currently accepts every valid pixel
//     while enabled, so axis_tready is high when enable is high.
//
//   axis_tdata:
//     Pixel payload. Current first-light convention uses RGB565 in [15:0].
//
//   axis_tuser:
//     Start-of-frame marker. Treated as frame_start.
//
//   axis_tlast:
//     End-of-line marker. This adapter uses the following-line pixel at x=0 to
//     emit line_start. The first pixel of a frame emits line_start with
//     frame_start.
//
// OUTPUT STREAM CONTRACT
//   rx_pixel_valid:
//     One-cycle valid qualifier.
//
//   rx_pixel_data:
//     16-bit pixel word, currently RGB565.
//
//   rx_frame_start:
//     Asserted with the first pixel of a frame.
//
//   rx_line_start:
//     Asserted with the first pixel of every line.
//
//   rx_frame_done:
//     Asserted with the final pixel of the expected frame rectangle.
//
// GEOMETRY CONTRACT
//   FRAME_W and FRAME_H define the expected output frame size.
//   The adapter counts accepted pixels and flags protocol errors when events
//   conflict with the expected geometry.
//
// ERROR FLAGS
//   rx_error_flags[0]:
//     Pixel arrived before a frame start.
//
//   rx_error_flags[1]:
//     axis_tlast occurred before expected end-of-line.
//
//   rx_error_flags[2]:
//     axis_tlast was missing at expected end-of-line.
//
//   rx_error_flags[3]:
//     frame_start occurred before the previous frame completed.
//
//   rx_error_flags[4]:
//     frame completed at an unexpected coordinate.
//
//   rx_error_flags[5]:
//     Reserved.
//
//   rx_error_flags[6]:
//     Reserved.
//
//   rx_error_flags[7]:
//     Backend absent/fail-closed flag. Not driven by this adapter.
//
// SYNTHESIS NOTES
//   - Verilog-2001 compatible.
//   - No SystemVerilog constructs.
//   - No CDC is performed; all signals are synchronous to axis_clk.
//==============================================================================

module csi_axis_to_pixel_stream #(
    parameter integer FRAME_W = 640,
    parameter integer FRAME_H = 480
)(
    input  wire        axis_clk,
    input  wire        rst,
    input  wire        enable,

    input  wire        axis_tvalid,
    output wire        axis_tready,
    input  wire [15:0] axis_tdata,
    input  wire        axis_tuser,
    input  wire        axis_tlast,

    output reg         rx_pixel_valid,
    output reg  [15:0] rx_pixel_data,
    output reg         rx_frame_start,
    output reg         rx_line_start,
    output reg         rx_frame_done,
    output reg         rx_locked,
    output reg  [7:0]  rx_error_flags,
    output reg         overflow_pulse
);

    //==========================================================================
    // FUNCTION: integer_at_least_one
    //--------------------------------------------------------------------------
    // PURPOSE
    //   Clamp a geometry parameter to at least one.
    //
    // STEP-BY-STEP
    //   1) Check whether value is zero or negative.
    //   2) Return one for invalid values.
    //   3) Return value unchanged for positive values.
    //==========================================================================
    function integer integer_at_least_one;
        input integer value;
        begin
            if (value <= 0)
                integer_at_least_one = 1;
            else
                integer_at_least_one = value;
        end
    endfunction

    localparam integer SAFE_FRAME_W_INT = integer_at_least_one(FRAME_W);
    localparam integer SAFE_FRAME_H_INT = integer_at_least_one(FRAME_H);

    localparam [31:0] SAFE_FRAME_W = SAFE_FRAME_W_INT;
    localparam [31:0] SAFE_FRAME_H = SAFE_FRAME_H_INT;

    localparam [31:0] FRAME_W_M1 = SAFE_FRAME_W - 32'd1;
    localparam [31:0] FRAME_H_M1 = SAFE_FRAME_H - 32'd1;

    reg [31:0] x_ctr;
    reg [31:0] y_ctr;
    reg        in_frame;
    reg        next_pixel_starts_line;

    wire accept_pixel;
    assign axis_tready  = enable;
    assign accept_pixel = enable && axis_tvalid && axis_tready;

    wire at_line_end;
    wire at_frame_end;

    assign at_line_end  = (x_ctr == FRAME_W_M1);
    assign at_frame_end = (x_ctr == FRAME_W_M1) && (y_ctr == FRAME_H_M1);

    //==========================================================================
    // SEQUENTIAL BLOCK: AXI-stream event adaptation
    //==========================================================================
    always @(posedge axis_clk) begin
        if (rst) begin
            x_ctr                 <= 32'd0;
            y_ctr                 <= 32'd0;
            in_frame              <= 1'b0;
            next_pixel_starts_line <= 1'b1;

            rx_pixel_valid        <= 1'b0;
            rx_pixel_data         <= 16'h0000;
            rx_frame_start        <= 1'b0;
            rx_line_start         <= 1'b0;
            rx_frame_done         <= 1'b0;
            rx_locked             <= 1'b0;
            rx_error_flags        <= 8'h00;
            overflow_pulse        <= 1'b0;
        end else begin
            //------------------------------------------------------------------
            // Default one-cycle strobes.
            //------------------------------------------------------------------
            rx_pixel_valid <= 1'b0;
            rx_frame_start <= 1'b0;
            rx_line_start  <= 1'b0;
            rx_frame_done  <= 1'b0;
            overflow_pulse <= 1'b0;

            if (!enable) begin
                x_ctr                 <= 32'd0;
                y_ctr                 <= 32'd0;
                in_frame              <= 1'b0;
                next_pixel_starts_line <= 1'b1;

                rx_locked             <= 1'b0;
                rx_error_flags        <= 8'h00;
            end else begin
                rx_locked <= 1'b1;

                if (accept_pixel) begin
                    //----------------------------------------------------------
                    // Publish pixel.
                    //----------------------------------------------------------
                    rx_pixel_valid <= 1'b1;
                    rx_pixel_data  <= axis_tdata;

                    rx_frame_start <= axis_tuser;
                    rx_line_start  <= axis_tuser || next_pixel_starts_line;
                    rx_frame_done  <= at_frame_end;

                    //----------------------------------------------------------
                    // Frame-start handling.
                    //----------------------------------------------------------
                    if (axis_tuser) begin
                        if (in_frame)
                            rx_error_flags[3] <= 1'b1;

                        x_ctr                 <= 32'd0;
                        y_ctr                 <= 32'd0;
                        in_frame              <= 1'b1;
                        next_pixel_starts_line <= 1'b0;
                    end else begin
                        if (!in_frame)
                            rx_error_flags[0] <= 1'b1;
                    end

                    //----------------------------------------------------------
                    // Line-end validation.
                    //----------------------------------------------------------
                    if (axis_tlast && !at_line_end)
                        rx_error_flags[1] <= 1'b1;

                    if (!axis_tlast && at_line_end)
                        rx_error_flags[2] <= 1'b1;

                    //----------------------------------------------------------
                    // Frame-end validation.
                    //----------------------------------------------------------
                    if (at_frame_end) begin
                        if (!axis_tlast)
                            rx_error_flags[4] <= 1'b1;

                        in_frame              <= 1'b0;
                        x_ctr                 <= 32'd0;
                        y_ctr                 <= 32'd0;
                        next_pixel_starts_line <= 1'b1;
                    end else if (at_line_end) begin
                        x_ctr                 <= 32'd0;
                        y_ctr                 <= y_ctr + 32'd1;
                        next_pixel_starts_line <= 1'b1;
                    end else begin
                        x_ctr <= x_ctr + 32'd1;

                        if (axis_tlast)
                            next_pixel_starts_line <= 1'b1;
                        else
                            next_pixel_starts_line <= 1'b0;
                    end
                end
            end
        end
    end

endmodule

`default_nettype wire