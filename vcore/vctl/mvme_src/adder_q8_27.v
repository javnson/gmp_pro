`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/11/29 18:44:54
// Design Name: 
// Module Name: adder_q8_27
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////



`timescale 1ns / 1ps

module adder_q8_27 (
    input  wire                  clk,
    input  wire                  rst,        // Active High Reset
    input  wire signed [34:0]    a,          // Q8.27 Input
    input  wire signed [34:0]    b,          // Q8.27 Input
    output reg  signed [34:0]    s_out,      // Q8.27 Output (Saturated)
    output reg                   overload    // Overflow Warning
);

    //==========================================================================
    // 1. Input Sign Extension (35-bit -> 48-bit)
    //    We must extend the sign bit (bit 34) to fill the upper 13 bits.
    //==========================================================================
    wire signed [47:0] a_ext;
    wire signed [47:0] b_ext;

    assign a_ext = { {13{a[34]}}, a };
    assign b_ext = { {13{b[34]}}, b };

    //==========================================================================
    // 2. Instantiate Xilinx Adder IP (48-bit)
    //    Component Name: adder_47 (Based on your screenshot)
    //    Latency: 1 cycle
    //==========================================================================
    wire signed [47:0] raw_sum;

    adder_48 adder_core (
        .CLK  (clk),
        .A    (a_ext),
        .B    (b_ext),
        .C_IN (1'b0),     // Tied to 0 (No rounding needed for Q8.27 + Q8.27)
        .SCLR (rst),      // Synchronous Clear
        .S    (raw_sum)   // 48-bit Result
    );

    //==========================================================================
    // 3. Overflow Detection Logic
    //    The output is valid 35-bit if the upper bits [47:34] are all identical.
    //    (i.e., all 0s for positive, all 1s for negative).
    //==========================================================================
    wire sign_bit;
    wire is_overflow;

    assign sign_bit = raw_sum[47]; // The true sign of the result
    
    // Check if bits [46:34] match the sign bit [47]
    // If they don't match, it means the value spilled into the upper bits.
    assign is_overflow = (raw_sum[47:34] != {14{sign_bit}});

    // Define Clamping Limits for 35-bit Signed Q8.27
    localparam signed [34:0] MAX_POS = {1'b0, {34{1'b1}}}; // +127.99...
    localparam signed [34:0] MAX_NEG = {1'b1, {34{1'b0}}}; // -128.00...

    //==========================================================================
    // 4. Output Pipeline & Saturation (Clamping)
    //    Adds 1 cycle latency. Total Latency = 1 (IP) + 1 (Logic) = 2 cycles.
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            s_out    <= 35'd0;
            overload <= 1'b0;
        end else begin
            overload <= is_overflow;

            if (is_overflow) begin
                // Saturation Logic
                if (sign_bit == 1'b0)
                    s_out <= MAX_POS; // Positive Overflow -> Clamp to Max
                else
                    s_out <= MAX_NEG; // Negative Overflow -> Clamp to Min
            end else begin
                // Normal Operation: Truncate upper 13 bits (which are just sign copies)
                s_out <= raw_sum[34:0];
            end
        end
    end

endmodule

