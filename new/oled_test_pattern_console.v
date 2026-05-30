`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// oled_test_pattern_console
//------------------------------------------------------------------------------
// ROLE
//   Static byte-addressed OLED test-page generator for 128x32 SSD1306 display.
//
// PURPOSE
//   Provide a known-good independent byte source for OLED bring-up.
//
// INTERFACE
//   byte_addr[8:7] = page index 0..3
//   byte_addr[6:0] = column index 0..127
//
// OUTPUT
//   byte_data is a deterministic test pattern independent of telemetry logic.
//==============================================================================

module oled_test_pattern_console (
    input  wire [8:0] byte_addr,
    output reg  [7:0] byte_data
);

    reg [1:0] page_sel;
    reg [6:0] col_sel;

    always @(*) begin
        page_sel = byte_addr[8:7];
        col_sel  = byte_addr[6:0];

        case (page_sel)
            //--------------------------------------------------------------------------
            // Page 0: alternating vertical bar groups
            //--------------------------------------------------------------------------
            2'd0: begin
                if (col_sel[4])
                    byte_data = 8'hFF;
                else
                    byte_data = 8'h00;
            end

            //--------------------------------------------------------------------------
            // Page 1: alternating bit patterns by column
            //--------------------------------------------------------------------------
            2'd1: begin
                if (col_sel[3])
                    byte_data = 8'hAA;
                else
                    byte_data = 8'h55;
            end

            //--------------------------------------------------------------------------
            // Page 2: simple border-like region
            //--------------------------------------------------------------------------
            2'd2: begin
                if ((col_sel < 7'd8) || (col_sel > 7'd119))
                    byte_data = 8'hFF;
                else
                    byte_data = 8'h81;
            end

            //--------------------------------------------------------------------------
            // Page 3: center band
            //--------------------------------------------------------------------------
            2'd3: begin
                if ((col_sel >= 7'd32) && (col_sel <= 7'd95))
                    byte_data = 8'h3C;
                else
                    byte_data = 8'h00;
            end

            default: begin
                byte_data = 8'h00;
            end
        endcase
    end

endmodule

`default_nettype wire