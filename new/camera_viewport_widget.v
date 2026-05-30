`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// camera_viewport_widget
//------------------------------------------------------------------------------
// ROLE
//   Render a bounded camera viewing aperture into the HDMI pixel stream.
//
// CONTRACT
//   This module does not fetch or rescale camera pixels. It clips the live
//   camera RGB stream at the current raster coordinate into a visible panel.
//   Full arbitrary placement with true scaling would require an addressable
//   camera framebuffer read path or scaler upstream.
//
// CDC
//   All inputs are already in the VID domain or are frame-committed VID-domain
//   status bits from the camera status snapshot.
//==============================================================================

module camera_viewport_widget #(
    parameter integer X0           = 456,
    parameter integer Y0           = 8,
    parameter integer W            = 176,
    parameter integer H            = 132,
    parameter integer BORDER       = 2,
    parameter integer STATUS_H     = 10
)(
    input  wire [11:0] pix_x,
    input  wire [11:0] pix_y,
    input  wire        de,

    input  wire [23:0] cam_rgb,
    input  wire        cam_rgb_valid,
    input  wire        cam_frame_valid,
    input  wire        cam_ready,
    input  wire        cam_init_done,
    input  wire        cam_init_fail,

    output reg  [23:0] rgb_out
);

    localparam [23:0] C_BLACK        = 24'h000000;
    localparam [23:0] C_BG0          = 24'h081014;
    localparam [23:0] C_BG1          = 24'h101820;
    localparam [23:0] C_NO_SIGNAL0   = 24'h180808;
    localparam [23:0] C_NO_SIGNAL1   = 24'h201010;
    localparam [23:0] C_BORDER_READY = 24'h20D060;
    localparam [23:0] C_BORDER_WAIT  = 24'hD8A020;
    localparam [23:0] C_BORDER_FAIL  = 24'hE03030;
    localparam [23:0] C_BORDER_BOOT  = 24'h3080E0;
    localparam [23:0] C_RETICLE      = 24'h80F0E0;

    localparam integer X1        = X0 + W;
    localparam integer Y1        = Y0 + H;
    localparam integer IMAGE_Y0  = BORDER + STATUS_H;
    localparam integer IMAGE_Y1  = H - BORDER;
    localparam integer CX        = W / 2;
    localparam integer CY        = IMAGE_Y0 + ((IMAGE_Y1 - IMAGE_Y0) / 2);

    localparam [11:0] X0_U12             = X0;
    localparam [11:0] Y0_U12             = Y0;
    localparam [11:0] X1_U12             = X1;
    localparam [11:0] Y1_U12             = Y1;
    localparam [11:0] BORDER_U12         = BORDER;
    localparam [11:0] W_MINUS_BORDER_U12 = W - BORDER;
    localparam [11:0] H_MINUS_BORDER_U12 = H - BORDER;
    localparam [11:0] STATUS_Y1_U12      = BORDER + STATUS_H;
    localparam [11:0] IMAGE_Y0_U12       = IMAGE_Y0;
    localparam [11:0] IMAGE_Y1_U12       = IMAGE_Y1;
    localparam [11:0] CX_U12             = CX;
    localparam [11:0] CY_U12             = CY;
    localparam [11:0] RETICLE_X0_U12     = CX - 12;
    localparam [11:0] RETICLE_X1_U12     = CX + 12;
    localparam [11:0] RETICLE_Y0_U12     = CY - 12;
    localparam [11:0] RETICLE_Y1_U12     = CY + 12;
    localparam [11:0] RETICLE_XC1_U12    = CX + 1;
    localparam [11:0] RETICLE_YC1_U12    = CY + 1;

    wire in_widget =
        de &&
        (pix_x >= X0_U12) && (pix_x < X1_U12) &&
        (pix_y >= Y0_U12) && (pix_y < Y1_U12);

    wire [11:0] rel_x = pix_x - X0_U12;
    wire [11:0] rel_y = pix_y - Y0_U12;

    wire border =
        in_widget &&
        ((rel_x < BORDER_U12) || (rel_x >= W_MINUS_BORDER_U12) ||
         (rel_y < BORDER_U12) || (rel_y >= H_MINUS_BORDER_U12));

    wire status_bar =
        in_widget &&
        (rel_x >= BORDER_U12) && (rel_x < W_MINUS_BORDER_U12) &&
        (rel_y >= BORDER_U12) && (rel_y < STATUS_Y1_U12);

    wire image_region =
        in_widget &&
        (rel_x >= BORDER_U12) && (rel_x < W_MINUS_BORDER_U12) &&
        (rel_y >= IMAGE_Y0_U12) && (rel_y < IMAGE_Y1_U12);

    wire checker = rel_x[4] ^ rel_y[4];
    wire cam_live = cam_rgb_valid;
    wire stream_ready = cam_frame_valid || cam_rgb_valid;

    wire reticle_x =
        image_region &&
        (rel_x >= RETICLE_X0_U12) && (rel_x <= RETICLE_X1_U12) &&
        ((rel_y == CY_U12) || (rel_y == RETICLE_YC1_U12));

    wire reticle_y =
        image_region &&
        (rel_y >= RETICLE_Y0_U12) && (rel_y <= RETICLE_Y1_U12) &&
        ((rel_x == CX_U12) || (rel_x == RETICLE_XC1_U12));

    wire [23:0] border_color =
        cam_init_fail ? C_BORDER_FAIL :
        (cam_ready && stream_ready) ? C_BORDER_READY :
        cam_init_done ? C_BORDER_WAIT :
                        C_BORDER_BOOT;

    always @(*) begin
        rgb_out = C_BLACK;

        if (in_widget)
            rgb_out = checker ? C_BG1 : C_BG0;

        if (image_region) begin
            if (cam_live)
                rgb_out = cam_rgb;
            else
                rgb_out = checker ? C_NO_SIGNAL1 : C_NO_SIGNAL0;
        end

        if (status_bar)
            rgb_out = border_color;

        if (border)
            rgb_out = border_color;

        if (cam_live && (reticle_x || reticle_y))
            rgb_out = C_RETICLE;
    end

endmodule

`default_nettype wire
