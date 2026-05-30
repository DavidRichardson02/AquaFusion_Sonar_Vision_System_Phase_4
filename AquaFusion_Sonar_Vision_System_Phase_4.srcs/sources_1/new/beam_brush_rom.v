`timescale 1ns/1ps
`default_nettype none

// ============================================================================
// beam_brush_rom.v
//------------------------------------------------------------------------------
// Purpose : Parameterized brush-offset ROM for endpoint "beam width" painting.
//
// Shapes
// ------
// 0: CROSS      (radius R)     => {(0,0), (±k,0), (0,±k)} k=1..R
// 1: SQUARE     (radius R)     => all dx,dy with |dx|<=R and |dy|<=R
// 2: DIAMOND    (radius R)     => all dx,dy with |dx|+|dy|<=R
//
// Interface
// ---------
// idx -> (dx,dy,valid,count)
// Where dx,dy are signed small integers.
//
// Notes
// -----
// - Combinational "ROM" enumerator. Synthesizes to LUT logic.
// - count is the number of valid footprint points.
// ============================================================================
module beam_brush_rom #(
    parameter integer RADIUS = 1,
    parameter integer SHAPE  = 1,  // 0=CROSS, 1=SQUARE, 2=DIAMOND
    parameter integer DXW    = 4   // signed width for dx/dy
)(
    input  wire [7:0] idx,
    output reg  signed [DXW-1:0] dx,
    output reg  signed [DXW-1:0] dy,
    output reg                   valid,
    output reg  [7:0]            count
);

    // ------------------------------------------------------------
    // Count computation
    // ------------------------------------------------------------
    function [7:0] count_square;
        input integer r;
        integer tmp;
        begin
            tmp = (2*r + 1) * (2*r + 1);
            count_square = tmp[7:0];
        end
    endfunction

    function [7:0] count_cross;
        input integer r;
        integer tmp;
        begin
            tmp = 1 + (4*r);
            count_cross = tmp[7:0];
        end
    endfunction

    function [7:0] count_diamond;
        input integer r;
        integer k;
        integer sum;
        begin
            sum = 0;
            for (k = 0; k <= r; k = k + 1) begin
                if (k == 0) sum = sum + 1;
                else        sum = sum + (4*k);
            end
            count_diamond = sum[7:0];
        end
    endfunction

    // ------------------------------------------------------------
    // idx -> (dx,dy) enumerators
    // ------------------------------------------------------------
    task square_at;
        input  [7:0] i;
        input  integer r;
        output signed [DXW-1:0] ox;
        output signed [DXW-1:0] oy;
        integer side;
        integer ix;
        integer x_i;
        integer y_i;
        begin
            side = (2*r + 1);
            ix   = i;

            x_i  = (ix % side) - r;
            y_i  = (ix / side) - r;

            ox = x_i;
            oy = y_i;
        end
    endtask

    task cross_at;
        input  [7:0] i;
        input  integer r;
        output signed [DXW-1:0] ox;
        output signed [DXW-1:0] oy;
        integer k;
        integer sel;
        begin
            // order: (0,0), then for k=1..r: +x, -x, +y, -y
            if (i == 0) begin
                ox = 0;
                oy = 0;
            end else begin
                k   = ((i-1) / 4) + 1; // 1..r
                sel = ((i-1) % 4);

                case (sel)
                    0: begin ox =  k; oy = 0; end
                    1: begin ox = -k; oy = 0; end
                    2: begin ox = 0;  oy =  k; end
                    default: begin ox = 0; oy = -k; end
                endcase
            end
        end
    endtask

    task diamond_at;
        input  [7:0] i;
        input  integer r;
        output signed [DXW-1:0] ox;
        output signed [DXW-1:0] oy;
        integer ring;
        integer ring_count;
        integer pos_global;
        integer pos;
        integer a;
        integer found;
        begin
            // ring 0: (0,0)
            // ring k>0: 4*k points around diamond
            ox = 0;
            oy = 0;

            if (i == 0) begin
                ox = 0;
                oy = 0;
            end else begin
                pos_global = i - 1; // 0..(count-2)
                found      = 0;

                for (ring = 1; ring <= r; ring = ring + 1) begin
                    ring_count = 4 * ring;

                    if (!found) begin
                        if (pos_global < ring_count) begin
                            pos   = pos_global;
                            found = 1;

                            if (pos < ring) begin
                                ox = (ring - pos);
                                oy = (pos);
                            end else if (pos < (2*ring)) begin
                                a  = (pos - ring);
                                ox = -a;
                                oy = (ring - a);
                            end else if (pos < (3*ring)) begin
                                a  = (pos - 2*ring);
                                ox = -(ring - a);
                                oy = -a;
                            end else begin
                                a  = (pos - 3*ring);
                                ox = a;
                                oy = -(ring - a);
                            end
                        end else begin
                            pos_global = pos_global - ring_count;
                        end
                    end
                end
            end
        end
    endtask

    // ------------------------------------------------------------
    // Combinational ROM
    // ------------------------------------------------------------
    always @* begin
        dx    = 0;
        dy    = 0;
        valid = 1'b0;
        count = 8'd0;

        if (SHAPE == 0) begin
            count = count_cross(RADIUS);
            valid = (idx < count);
            if (valid) cross_at(idx, RADIUS, dx, dy);
        end else if (SHAPE == 1) begin
            count = count_square(RADIUS);
            valid = (idx < count);
            if (valid) square_at(idx, RADIUS, dx, dy);
        end else begin
            count = count_diamond(RADIUS);
            valid = (idx < count);
            if (valid) diamond_at(idx, RADIUS, dx, dy);
        end
    end

endmodule

`default_nettype wire