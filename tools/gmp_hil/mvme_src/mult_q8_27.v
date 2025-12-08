`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/11/29 18:43:28
// Design Name: 
// Module Name: mult_q8_27
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
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/27/2025 07:08:12 PM
// Design Name: 
// Module Name: mult_35s
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

module mult_q8_27 (
    input  wire                  clk,
    input  wire                  sclr,       // Synchronous Clear
    input  wire signed [34:0]    a,          // Input A (Q8.27)
    input  wire signed [34:0]    b,          // Input B (Q8.27)
    output reg  signed [34:0]    p_out,      // Output Product (Q8.27)
    output reg                   overload    // Overflow/Saturation Flag
);

    //==========================================================================
    // 1. Instantiate Xilinx Multiplier IP (35x35=70)
    //    Format: Q8.27 * Q8.27 = Q16.54 (1 Sign + 15 Int + 54 Frac)
    //    Latency: 6 cycles (Defined in IP Customization)
    //==========================================================================
    wire signed [69:0] raw_product;

    mult_gen_35_35_70 u_mult_core (
        .CLK  (clk),
        .A    (a),
        .B    (b),
        .SCLR (sclr),
        .P    (raw_product)
    );

    //==========================================================================
    // 2. Rounding Logic (Round-Half-Up) 
    //    We need to keep bits [61:27] for Q8.27 format.
    //    We discard bits [26:0].
    //    Strategy: Add 1 to the MSB of the discarded part (bit 26) 
    //              before truncation.
    //==========================================================================
    wire signed [69:0] p_rounded;
    
    // 70'sd67108864 is (1 << 26)
    assign p_rounded = raw_product + 70'sd67108864;

    //==========================================================================
    // 3. Overflow Detection & Clamping 
    // --------------------------------------------------------
    //    Target MSB (Sign) is at bit 61.
    //    Original Sign is at bit 69.
    //    Valid Range: Bits [69:61] must all be identical (all 0s or all 1s).
    //    If any bit in [68:61] differs from bit [69], overflow occurred.
    //==========================================================================
    
    wire sign_bit;
    wire is_overflow;

    assign sign_bit = p_rounded[69];
    
    // Check if the top 9 bits are identical
    assign is_overflow = (p_rounded[69:61] != {9{sign_bit}});

    // Define Clamping Limits for 35-bit Signed Q8.27
    // Max Positive: 0111...1 (+127.999...)
    localparam signed [34:0] MAX_POS = {1'b0, {34{1'b1}}}; 
    // Max Negative: 1000...0 (-128.000...)
    localparam signed [34:0] MAX_NEG = {1'b1, {34{1'b0}}};

    //==========================================================================
    // 4. Output Pipeline Stage
    //    Adds 1 cycle latency. Total Latency = 6 (IP) + 1 (Logic) = 7 cycles.
    //==========================================================================
    always @(posedge clk) begin
        if (sclr) begin
            p_out    <= 35'd0;
            overload <= 1'b0;
        end else begin
            overload <= is_overflow;
            
            if (is_overflow) begin
                // Clamping: Force to physical limits upon divergence [cite: 15]
                if (sign_bit == 1'b0) 
                    p_out <= MAX_POS;
                else
                    p_out <= MAX_NEG;
            end else begin
                // Normal Operation: Truncate to [61:27]
                p_out <= p_rounded[61:27];
            end
        end
    end

endmodule

