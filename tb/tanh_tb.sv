/***************************************************/
/* Hyperbolic Tangent (Tanh) Testbench            */
/* Verifies pipelined tanh approximation circuit  */
/***************************************************/

`timescale 1 ns / 1 ps

module tanh_tb();

// Simulation parameters
localparam CLK_PERIOD = 4;              // 4 ns clock period
localparam NUM_TESTS = 50;              // Number of test inputs
localparam SCALE_FACTOR = 2.0**-12.0;   // Q2.12 fixed-point scaling factor
localparam ERROR_TOLERANCE = 0.001;     // Acceptable output error tolerance

// DUT interface signals
logic clk, rst;
logic [13:0] i_x, o_fx;
logic i_valid, o_valid, i_ready, o_ready;

// Instantiate DUT

// Connect testbench signals to DUT ports
// DUT approximates tanh(x) using Taylor expansion
// Inputs are in Q2.12 fixed-point format

// Device Under Test


// Clock generation: 50% duty cycle clock
initial begin
    clk = 1'b0;
    forever #(CLK_PERIOD/2) clk = ~clk;
end

// Taylor series-based software model of tanh(x) for expected output
function real tanh_taylor(input real x);
    real a0, a1, a2, a3, a4;
begin
    a0 = -0.33333;
    a1 =  0.13333;
    a2 = -0.05397;
    a3 =  0.02187;
    a4 = -0.00886;
    tanh_taylor = x + (a0 * (x**3)) + (a1 * (x**5)) + (a2 * (x**7)) + (a3 * (x**9)) + (a4 * (x**11));
end
endfunction

// Computes absolute error between two real values
function real error(input real x0, input real x1);
begin
    error = (x0 > x1) ? (x0 - x1) : (x1 - x0);
end
endfunction

// Converts fixed-point Q2.12 input to real value
function real real_value(input logic [13:0] x);
begin
    if (x[13]) real_value = -1.0 * $itor(-x) * SCALE_FACTOR;
    else       real_value = $itor(x) * SCALE_FACTOR;
end
endfunction

// Input test vectors
integer test_id;
integer num_valid_inputs;
logic [13:0] generated_x [0:NUM_TESTS-1];
logic        generated_valid [0:NUM_TESTS-1];
real         golden_results_taylor_tanh [0:NUM_TESTS-1];
real         golden_results_real_tanh   [0:NUM_TESTS-1];
integer      golden_results_input_id    [0:NUM_TESTS-1];

// Generate test inputs and expected outputs
initial begin
    num_valid_inputs = 0;
    for (test_id = 0; test_id < NUM_TESTS; test_id = test_id + 1) begin
        i_x = $random;
        i_x[13] = {(2){$random % 2}};  // Force Q2.12 range [-1, 1]
        i_x[12] = i_x[13];
        i_valid = ($random % 8 != 'd0);
        generated_x[test_id] = i_x;
        generated_valid[test_id] = i_valid;
        if (i_valid) begin
            golden_results_real_tanh[num_valid_inputs] = $tanh(real_value(i_x));
            golden_results_taylor_tanh[num_valid_inputs] = tanh_taylor(real_value(i_x));
            golden_results_input_id[num_valid_inputs] = test_id;
            num_valid_inputs = num_valid_inputs + 1;
        end
    end
end

// Input driving logic
integer input_id;
integer i_ready_low_duration, i_ready_low_cycle_count;

initial begin
    rst = 1'b1;
    i_valid = 1'b0;
    i_x = 'd0;
    i_ready = 1'b1;
    i_ready_low_duration = 20;
    i_ready_low_cycle_count = 0;
    #(25 * CLK_PERIOD);
    rst = 1'b0;

    for (input_id = 0; input_id < NUM_TESTS; input_id = input_id + 1) begin
        i_x = generated_x[input_id];
        i_valid = generated_valid[input_id];
        if (input_id == NUM_TESTS/2) begin
            i_ready = 1'b0; // Simulate backpressure
        end

        // Wait until DUT is ready
        do begin
            #(CLK_PERIOD);
            if (!i_ready) begin
                if (i_ready_low_cycle_count == i_ready_low_duration) begin
                    i_ready = 1'b1;
                end else begin
                    i_ready_low_cycle_count = i_ready_low_cycle_count + 1;
                end
            end
        end while (!o_ready);
    end
end

// Output checking logic
integer num_received_outputs, mismatched_results, matched_results;
real output_error;
logic sim_failed;

initial begin
    $timeformat(-9, 2, " ns");
    num_received_outputs = 0;
    mismatched_results = 0;
    matched_results = 0;
    sim_failed = 1'b0;

    while (num_received_outputs < num_valid_inputs) begin
        if (o_valid && o_ready) begin
            output_error = error(golden_results_taylor_tanh[num_received_outputs], real_value(o_fx));
            if (output_error < ERROR_TOLERANCE) begin
                $display("[%0t] SUCCESS\t x= %9.6f\t Real Tanh(x)= %9.6f\t Taylor Tanh(x)= %9.6f\t o_fx= %9.6f\t Error: %9.6f\t < %9.6f",
                    $time,
                    real_value(generated_x[golden_results_input_id[num_received_outputs]]),
                    golden_results_real_tanh[num_received_outputs],
                    golden_results_taylor_tanh[num_received_outputs],
                    real_value(o_fx),
                    output_error, ERROR_TOLERANCE);
                matched_results = matched_results + 1;
            end else begin
                $display("[%0t] FAILURE\t x= %9.6f\t Real Tanh(x)= %9.6f\t Taylor Tanh(x)= %9.6f\t o_fx= %9.6f\t Error: %9.6f\t > %9.6f",
                    $time,
                    real_value(generated_x[golden_results_input_id[num_received_outputs]]),
                    golden_results_real_tanh[num_received_outputs],
                    golden_results_taylor_tanh[num_received_outputs],
                    real_value(o_fx),
                    output_error, ERROR_TOLERANCE);
                sim_failed = 1'b1;
                mismatched_results = mismatched_results + 1;
            end
            num_received_outputs = num_received_outputs + 1;
        end
        #(CLK_PERIOD);
    end

    // Final test report
    if (sim_failed) begin
        $display("TEST FAILED! %d results (out of %d) exceed tolerance.", mismatched_results, num_received_outputs);
    end else begin
        $display("TEST PASSED! %d results (out of %d) are within tolerance.", matched_results, num_received_outputs);
    end
    $stop;
end

endmodule