`timescale 1ns/1ps
`default_nettype none

//==============================================================================
// tmds_encoder
//------------------------------------------------------------------------------
// PURPOSE
//   Encode one 8-bit video/control input symbol into one 10-bit TMDS symbol per
//   pixel clock.
//
// HIGH-LEVEL ROLE
//   TMDS encoding is used by DVI/HDMI-style links to transform parallel pixel
//   channel data into a serialized code that has two key properties:
//
//     1) Reduced transition count
//        The first stage of the encoder converts the raw 8-bit word into an
//        intermediate 9-bit word chosen to reduce bit transitions across the
//        symbol.
//
//     2) Controlled running disparity
//        The second stage chooses whether to invert part of the intermediate
//        code so that the long-term DC balance of the link remains bounded.
//
// ACTIVE VIDEO VS BLANKING
//   The encoder behaves differently depending on `de`:
//
//     - de = 1:
//         Active video period.
//         `din` is encoded into a data symbol and running disparity is updated.
//
//     - de = 0:
//         Blanking / control period.
//         `c0` and `c1` select one of the four TMDS control symbols.
//         Running disparity is reset to zero.
//
// INTERFACE CONTRACT
//   Inputs:
//     clk  : pixel-rate clock; one encoded symbol is produced per rising edge
//     rst  : synchronous reset
//     din  : 8-bit data byte to encode during active video
//     c0   : control bit 0 used only when de = 0
//     c1   : control bit 1 used only when de = 0
//     de   : data enable; selects active-video encoding vs control-symbol mode
//
//   Output:
//     dout : 10-bit TMDS symbol for the current pixel/control interval
//
// IMPORTANT ARCHITECTURAL NOTE
//   This module outputs one 10-bit TMDS word at the pixel rate. It does NOT
//   serialize that word onto a physical pin. A separate serializer stage
//   (typically OSERDES / DDR logic) must emit these 10 bits at the higher-rate
//   TMDS serial clock.
//
// INTERNAL ORGANIZATION
//   The encoder is divided into two conceptual stages:
//
//     Stage A: Transition minimization
//       Input:  din[7:0]
//       Output: q_m[8:0]
//
//     Stage B: Running disparity correction
//       Input:  q_m[8:0], current disparity
//       Output: dout[9:0], updated disparity
//
// ALGORITHM OVERVIEW
//   Let n1_d denote the number of ones in din[7:0].
//
//   First, a transition-minimized intermediate code q_m is formed. The encoder
//   chooses between XOR and XNOR recurrence based on n1_d and din[0]:
//
//     use_xnor = (n1_d > 4) || ((n1_d == 4) && (din[0] == 0))
//
//   Then:
//     q_m[0] = din[0]
//
//     For i = 1..7:
//       if use_xnor:
//         q_m[i] = ~(q_m[i-1] ^ din[i])
//       else:
//         q_m[i] =  (q_m[i-1] ^ din[i])
//
//   The ninth bit:
//     q_m[8] = ~use_xnor
//
//   Next, let n1_qm be the number of ones in q_m[7:0], and define:
//
//     balance_qm = n1_qm - 4
//
//   This quantity measures whether q_m[7:0] has more ones than zeros
//   (positive), more zeros than ones (negative), or is balanced (zero).
//
//   The final 10-bit symbol is then chosen based on:
//     - current running disparity
//     - balance_qm
//     - q_m[8]
//
// RUNNING DISPARITY
//   `disparity` is a signed state variable that tracks cumulative imbalance of
//   transmitted ones versus zeros over time.
//
//   Positive disparity:
//     More ones than zeros have been transmitted recently.
//
//   Negative disparity:
//     More zeros than ones have been transmitted recently.
//
//   Near-zero disparity is desirable because it reduces DC bias on the link.
//
// CONTROL SYMBOLS DURING BLANKING
//   When de = 0, the encoder does not encode din. Instead it emits one of four
//   fixed TMDS control symbols selected by {c1, c0}:
//
//     c1 c0 = 00 -> 1101010100
//     c1 c0 = 01 -> 0010101011
//     c1 c0 = 10 -> 0101010100
//     c1 c0 = 11 -> 1010101011
//
//   These symbols are defined by the TMDS scheme and also reset running
//   disparity to zero in this implementation.
//
// SYNTHESIS STYLE
//   - One combinational block computes the intermediate quantities
//   - One sequential block registers the final TMDS output and updates
//     disparity
//
//   This is a standard and appropriate partitioning for a pixel-rate encoder.
//
//==============================================================================
module tmds_encoder (
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] din,
    input  wire       c0,
    input  wire       c1,
    input  wire       de,
    output reg  [9:0] dout
);

    //--------------------------------------------------------------------------
    // Running disparity state
    //--------------------------------------------------------------------------
    // Signed accumulator tracking long-term ones-minus-zeros tendency.
    //
    // Width:
    //   5 bits signed is commonly sufficient for TMDS disparity bookkeeping in
    //   this style of implementation.
    //--------------------------------------------------------------------------
    reg signed [4:0] disparity;

    //--------------------------------------------------------------------------
    // Stage A intermediates: transition minimization
    //--------------------------------------------------------------------------
    // n1_d
    //   Number of ones in din[7:0].
    //
    // use_xnor
    //   Selects whether the recurrence uses XOR or XNOR.
    //
    // q_m
    //   9-bit intermediate code:
    //     q_m[7:0] = transition-minimized data bits
    //     q_m[8]   = flag indicating which recurrence family was used
    //--------------------------------------------------------------------------
    reg [3:0] n1_d;
    reg       use_xnor;
    reg [8:0] q_m;

    //--------------------------------------------------------------------------
    // Stage B intermediates: balance measurement of q_m
    //--------------------------------------------------------------------------
    // n1_qm
    //   Number of ones in q_m[7:0].
    //
    // balance_qm
    //   Signed measure of imbalance relative to 4 ones / 4 zeros:
    //
    //       balance_qm = n1_qm - 4
    //
    //   Possible values range from -4 to +4.
    //--------------------------------------------------------------------------
    reg [3:0]        n1_qm;
    reg signed [4:0] balance_qm;

    //--------------------------------------------------------------------------
    // Loop index for combinational recurrence construction of q_m[7:0].
    //--------------------------------------------------------------------------
    integer i;

    //--------------------------------------------------------------------------
    // Combinational pre-encoding stage
    //--------------------------------------------------------------------------
    // This block computes all purely combinational quantities derived from the
    // current input byte din:
    //
    //   1) count ones in din
    //   2) decide XOR vs XNOR recurrence
    //   3) construct q_m[7:0]
    //   4) assign q_m[8]
    //   5) count ones in q_m[7:0]
    //   6) compute balance_qm
    //
    // No state is updated here. This block merely prepares the intermediate
    // symbol that the sequential stage will use on the current clock edge.
    //--------------------------------------------------------------------------
    always @(*) begin
        //----------------------------------------------------------------------
        // Step 1: Count the number of logic-1 bits in din[7:0].
        //
        // This is the Hamming weight (population count) of the input byte.
        //
        // Example:
        //   din = 8'b10110010  ->  n1_d = 4
        //
        // This count is used to choose between XOR and XNOR recurrence so that
        // the intermediate code tends to reduce transition density.
        //----------------------------------------------------------------------
        n1_d = din[0] + din[1] + din[2] + din[3] +
               din[4] + din[5] + din[6] + din[7];

        //----------------------------------------------------------------------
        // Step 2: Select the TMDS transition-minimization rule.
        //
        // Standard rule:
        //
        //   use_xnor = (n1_d > 4) || ((n1_d == 4) && (din[0] == 0))
        //
        // Interpretation:
        //   - If the input has more ones than zeros, choose XNOR.
        //   - If exactly balanced (4 ones), use din[0] as the tie-breaker.
        //
        // This choice biases the recurrence toward a representation with fewer
        // transitions.
        //----------------------------------------------------------------------
        use_xnor = (n1_d > 4) || ((n1_d == 4) && (din[0] == 1'b0));

        //----------------------------------------------------------------------
        // Step 3: Build q_m[7:0].
        //
        // The recurrence is defined as:
        //
        //   q_m[0] = din[0]
        //
        //   for i = 1..7:
        //     if use_xnor:
        //         q_m[i] = ~(q_m[i-1] ^ din[i])
        //     else:
        //         q_m[i] =  (q_m[i-1] ^ din[i])
        //
        // This transforms the original byte into a transition-minimized form.
        //----------------------------------------------------------------------
        q_m[0] = din[0];
        for (i = 1; i < 8; i = i + 1) begin
            if (use_xnor)
                q_m[i] = ~(q_m[i-1] ^ din[i]);
            else
                q_m[i] =  (q_m[i-1] ^ din[i]);
        end

        //----------------------------------------------------------------------
        // Step 4: Record which recurrence family was chosen.
        //
        // By convention:
        //   q_m[8] = ~use_xnor
        //
        // This bit is later used in the final 10-bit symbol construction and
        // in running disparity update logic.
        //----------------------------------------------------------------------
        q_m[8] = ~use_xnor;

        //----------------------------------------------------------------------
        // Step 5: Count ones in q_m[7:0].
        //
        // This is used to determine the signed imbalance of the 8-bit
        // transition-minimized payload.
        //----------------------------------------------------------------------
        n1_qm = q_m[0] + q_m[1] + q_m[2] + q_m[3] +
                q_m[4] + q_m[5] + q_m[6] + q_m[7];

        //----------------------------------------------------------------------
        // Step 6: Compute balance relative to an 8-bit perfectly balanced word.
        //
        // If q_m[7:0] contains:
        //   4 ones -> balance_qm = 0
        //   5 ones -> balance_qm = +1
        //   3 ones -> balance_qm = -1
        //   etc.
        //
        // This signed quantity drives the second-stage disparity correction.
        //----------------------------------------------------------------------
        balance_qm = $signed({1'b0, n1_qm}) - 5'sd4;
    end

    //--------------------------------------------------------------------------
    // Sequential output encoding stage
    //--------------------------------------------------------------------------
    // This block performs the final TMDS symbol selection and updates running
    // disparity once per pixel clock.
    //
    // Step-by-step behavior:
    //
    //   A) On reset:
    //        - clear running disparity
    //        - drive a known control symbol on dout
    //
    //   B) During blanking (de = 0):
    //        - output one of four fixed control symbols from {c1, c0}
    //        - reset running disparity to zero
    //
    //   C) During active video (de = 1):
    //        - examine current disparity and balance_qm
    //        - choose one of the legal TMDS data-symbol forms
    //        - update disparity accordingly
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            //------------------------------------------------------------------
            // Reset behavior:
            //   - running disparity is re-initialized to zero
            //   - dout is driven to a legal control symbol
            //
            // The selected reset symbol here is the control code for {c1,c0}=00.
            //------------------------------------------------------------------
            disparity <= 5'sd0;
            dout      <= 10'b1101010100;
        end else begin
            if (!de) begin
                //----------------------------------------------------------------
                // Blanking / control interval behavior
                //
                // During blanking, data encoding is suspended. Instead one of the
                // four fixed TMDS control symbols is emitted according to {c1,c0}.
                //
                // Running disparity is reset to zero in this implementation.
                //
                // This is a common and correct design convention for TMDS control
                // periods.
                //----------------------------------------------------------------
                disparity <= 5'sd0;

                case ({c1, c0})
                    2'b00:  dout <= 10'b1101010100;
                    2'b01:  dout <= 10'b0010101011;
                    2'b10:  dout <= 10'b0101010100;
                    2'b11:  dout <= 10'b1010101011;
                    default:dout <= 10'b1101010100;
                endcase
            end else begin
                //----------------------------------------------------------------
                // Active video data encoding
                //
                // The final 10-bit data symbol depends on:
                //   - current running disparity
                //   - balance_qm
                //   - q_m[8]
                //
                // There are three major decision regions:
                //
                //   1) disparity == 0  OR  balance_qm == 0
                //   2) disparity and balance_qm have the same sign
                //   3) disparity and balance_qm have opposite signs
                //
                // These cases correspond to the standard TMDS balancing logic.
                //----------------------------------------------------------------
                if ((disparity == 0) || (balance_qm == 0)) begin
                    //----------------------------------------------------------------
                    // Case 1:
                    //   Current running disparity is neutral, or the intermediate
                    //   symbol itself is already balanced.
                    //
                    // In this regime, the symbol form is chosen primarily using
                    // q_m[8], and disparity is updated by adding or subtracting
                    // balance_qm accordingly.
                    //----------------------------------------------------------------
                    dout[9] <= ~q_m[8];
                    dout[8] <=  q_m[8];

                    if (q_m[8]) begin
                        //----------------------------------------------------------
                        // Transmit q_m[7:0] directly.
                        //----------------------------------------------------------
                        dout[7:0] <= q_m[7:0];
                        disparity <= disparity + balance_qm;
                    end else begin
                        //----------------------------------------------------------
                        // Transmit inverted q_m[7:0].
                        //----------------------------------------------------------
                        dout[7:0] <= ~q_m[7:0];
                        disparity <= disparity - balance_qm;
                    end
                end else if ((disparity[4] == 0 && balance_qm[4] == 0) ||
                             (disparity[4] == 1 && balance_qm[4] == 1)) begin
                    //----------------------------------------------------------------
                    // Case 2:
                    //   disparity and balance_qm have the same sign.
                    //
                    // Interpretation:
                    //   The current intermediate word would tend to push the
                    //   running disparity further away from zero if transmitted in
                    //   the wrong form. Therefore the encoder selects the symbol
                    //   form that counteracts this tendency.
                    //
                    // This branch emits:
                    //   dout[9]   = 1
                    //   dout[8]   = q_m[8]
                    //   dout[7:0] = ~q_m[7:0]
                    //
                    // and updates disparity with the corresponding arithmetic.
                    //----------------------------------------------------------------
                    dout[9]   <= 1'b1;
                    dout[8]   <= q_m[8];
                    dout[7:0] <= ~q_m[7:0];
                    disparity <= disparity + $signed({4'd0, q_m[8]}) - balance_qm;
                end else begin
                    //----------------------------------------------------------------
                    // Case 3:
                    //   disparity and balance_qm have opposite signs.
                    //
                    // Interpretation:
                    //   The current intermediate word naturally tends to pull the
                    //   running disparity back toward zero, so the encoder can use
                    //   the non-inverted q_m[7:0]-style form.
                    //----------------------------------------------------------------
                    dout[9]   <= 1'b0;
                    dout[8]   <= q_m[8];
                    dout[7:0] <= q_m[7:0];
                    disparity <= disparity - $signed({4'd0, ~q_m[8]}) + balance_qm;
                end
            end
        end
    end

endmodule

`default_nettype wire