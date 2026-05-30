`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// uart_tx_8n1
//------------------------------------------------------------------------------
// ROLE
//   Minimal 8-N-1 UART transmitter.
//
// CONTRACT
//   - data_valid is a one-cycle strobe when ready=1.
//   - txd idles high.
//   - No FIFO is included.
//==============================================================================

module uart_tx_8n1 #(
    parameter integer CLK_HZ = 100_000_000,
    parameter integer BAUD   = 9600
)(
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] data_in,
    input  wire       data_valid,
    output wire       ready,
    output reg        txd
);

    localparam integer BAUD_DIV = (CLK_HZ + (BAUD/2)) / BAUD;

    reg [31:0] baud_ctr;
    reg [3:0]  bit_idx;
    reg [9:0]  shift_reg;
    reg        busy;

    assign ready = ~busy;

    always @(posedge clk) begin
        if (rst) begin
            baud_ctr  <= 32'd0;
            bit_idx   <= 4'd0;
            shift_reg <= 10'h3FF;
            busy      <= 1'b0;
            txd       <= 1'b1;
        end else begin
            if (!busy) begin
                txd <= 1'b1;

                if (data_valid) begin
                    // shift_reg = {stop, data[7:0], start}
                    shift_reg <= {1'b1, data_in, 1'b0};
                    baud_ctr  <= BAUD_DIV - 1;
                    bit_idx   <= 4'd0;
                    busy      <= 1'b1;
                    txd       <= 1'b0; // start bit
                end
            end else begin
                if (baud_ctr == 0) begin
                    baud_ctr  <= BAUD_DIV - 1;
                    shift_reg <= {1'b1, shift_reg[9:1]};

                    if (bit_idx == 4'd9) begin
                        busy    <= 1'b0;
                        txd     <= 1'b1;
                        bit_idx <= 4'd0;
                    end else begin
                        txd     <= shift_reg[1];
                        bit_idx <= bit_idx + 1'b1;
                    end
                end else begin
                    baud_ctr <= baud_ctr - 1'b1;
                end
            end
        end
    end

endmodule

`default_nettype wire