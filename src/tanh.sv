/***************************************************/
/* Hyperbolic Tangent (Tanh) Circuit              */
/* Computes tanh(x) using pipelined Taylor series */
/***************************************************/

module tanh (
    input  clk,         // Input clock signal
    input  rst,         // Active-high synchronous reset
    // Input interface
    input  [13:0] i_x,  // Input value x
    input  i_valid,     // Input is valid
    output o_ready,     // Module is ready to accept a new input
    // Output interface 
    output [13:0] o_fx, // Output value f(x) = tanh(x)
    output o_valid,     // Output is valid
    input  i_ready      // Downstream module is ready to accept output
);

// Taylor series coefficients for tanh(x)
localparam signed [13:0] A0 = 14'b11101010101011; // a0 = -0.33349609375
localparam signed [13:0] A1 = 14'b00001000100010; // a1 =  0.13330078125
localparam signed [13:0] A2 = 14'b11111100100011; // a2 = -0.05419921875
localparam signed [13:0] A3 = 14'b00000001011001; // a3 =  0.021484375
localparam signed [13:0] A4 = 14'b11111111011100; // a4 = -0.0087890625

// Multiplies two 14-bit signed numbers and truncates result to 14 bits
function automatic logic signed [13:0] multiply (input logic signed [13:0] x, input logic signed [13:0] y);
    logic signed [27:0] z;
    z = x * y; 
    multiply = z[25:12];
endfunction

// Input stage
logic signed [13:0] r_x; 
logic r_valid;

// Pipeline registers for each stage
// Each stage computes a portion of the Taylor series using Horner's method
logic signed [13:0] r_fx1, r_fx2, r_fx3, r_fx4, r_fx5, r_fx6, r_fx7, r_fx8, r_fx9, r_fx10, r_fx11;
logic signed [13:0] r_x1,  r_x2,  r_x3,  r_x4,  r_x5,  r_x6,  r_x7,  r_x8,  r_x9,  r_x10,  r_x11;
logic signed [13:0] r_xsq2, r_xsq3, r_xsq4, r_xsq5, r_xsq6, r_xsq7, r_xsq8, r_xsq9;
logic r_valid1, r_valid2, r_valid3, r_valid4, r_valid5, r_valid6, r_valid7, r_valid8, r_valid9, r_valid10, r_valid11;

// Output stage
logic signed [13:0] r_fx; 
logic r_o_valid;

// Pipelined processing: 11 stages computing polynomial in Horner's form
always_ff @ (posedge clk) begin
    if (rst) begin
        // Reset all pipeline registers
        r_x <= 0; r_valid <= 0;
        r_fx1 <= 0; r_x1 <= 0; r_valid1 <= 0;
        r_fx2 <= 0; r_x2 <= 0; r_valid2 <= 0; r_xsq2 <= 0;
        r_fx3 <= 0; r_x3 <= 0; r_valid3 <= 0; r_xsq3 <= 0;
        r_fx4 <= 0; r_x4 <= 0; r_valid4 <= 0; r_xsq4 <= 0;
        r_fx5 <= 0; r_x5 <= 0; r_valid5 <= 0; r_xsq5 <= 0;
        r_fx6 <= 0; r_x6 <= 0; r_valid6 <= 0; r_xsq6 <= 0;
        r_fx7 <= 0; r_x7 <= 0; r_valid7 <= 0; r_xsq7 <= 0;
        r_fx8 <= 0; r_x8 <= 0; r_valid8 <= 0; r_xsq8 <= 0;
        r_fx9 <= 0; r_x9 <= 0; r_valid9 <= 0; r_xsq9 <= 0;
        r_fx10 <= 0; r_x10 <= 0; r_valid10 <= 0;
        r_fx11 <= 0; r_x11 <= 0; r_valid11 <= 0;
        r_fx <= 0; r_o_valid <= 0;
    end else if (i_ready) begin
        // Stage 0: latch input
        r_x <= i_x;
        r_valid <= i_valid;

        // Stage 1: x^2
        r_x1 <= r_x;
        r_valid1 <= r_valid;
        r_fx1 <= multiply(r_x, r_x);

        // Stage 2: fx = x^2 * A4
        r_x2 <= r_x1;
        r_valid2 <= r_valid1;
        r_xsq2 <= r_fx1;
        r_fx2 <= multiply(r_fx1, A4);

        // Stage 3: fx += A3
        r_x3 <= r_x2;
        r_valid3 <= r_valid2;
        r_xsq3 <= r_xsq2;
        r_fx3 <= r_fx2 + A3;

        // Stage 4: fx *= x^2
        r_x4 <= r_x3;
        r_valid4 <= r_valid3;
        r_xsq4 <= r_xsq3;
        r_fx4 <= multiply(r_fx3, r_xsq3);

        // Stage 5: fx += A2
        r_x5 <= r_x4;
        r_valid5 <= r_valid4;
        r_xsq5 <= r_xsq4;
        r_fx5 <= r_fx4 + A2;

        // Stage 6: fx *= x^2
        r_x6 <= r_x5;
        r_valid6 <= r_valid5;
        r_xsq6 <= r_xsq5;
        r_fx6 <= multiply(r_fx5, r_xsq5);

        // Stage 7: fx += A1
        r_x7 <= r_x6;
        r_valid7 <= r_valid6;
        r_xsq7 <= r_xsq6;
        r_fx7 <= r_fx6 + A1;

        // Stage 8: fx *= x^2
        r_x8 <= r_x7;
        r_valid8 <= r_valid7;
        r_xsq8 <= r_xsq7;
        r_fx8 <= multiply(r_fx7, r_xsq7);

        // Stage 9: fx += A0
        r_x9 <= r_x8;
        r_valid9 <= r_valid8;
        r_xsq9 <= r_xsq8;
        r_fx9 <= r_fx8 + A0;

        // Stage 10: fx *= x^2
        r_x10 <= r_x9;
        r_valid10 <= r_valid9;
        r_fx10 <= multiply(r_fx9, r_xsq9);

        // Stage 11: fx *= x
        r_x11 <= r_x10;
        r_valid11 <= r_valid10;
        r_fx11 <= multiply(r_fx10, r_x10);

        // Final output: fx + x
        r_fx <= r_fx11 + r_x11;
        r_o_valid <= r_valid11;
    end
end

// Output assignments
assign o_ready = i_ready;
assign o_valid = r_o_valid;
assign o_fx = r_fx;

endmodule