`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/11/29 18:40:25
// Design Name: 
// Module Name: mvme_4ch_std
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


module mvme_35_4ch_std (
    input  wire                  clk,
    input  wire                  rst,       // Active High Reset
    
    // Inputs: 4 Pairs of Multiplication Factors (Q8.27)
    input  wire signed [34:0]    a, b,      // Pair 1
    input  wire signed [34:0]    c, d,      // Pair 2
    input  wire signed [34:0]    e, f,      // Pair 3
    input  wire signed [34:0]    g, h,      // Pair 4
    
    // ========================================================================
    // Outputs (All Aligned to Latency = 10 Cycles)
    // ========================================================================
    
    // 1. Individual Product Outputs (Direct A*B, C*D...)
    output wire signed [34:0]    out_p0,    
    output wire signed [34:0]    out_p1,    
    output wire signed [34:0]    out_p2,    
    output wire signed [34:0]    out_p3,    

    // 2. Split Outputs (Intermediate Sums)
    output wire signed [34:0]    out_ab_cd, // (A*B + C*D)
    output wire signed [34:0]    out_ef_gh, // (E*F + G*H)
    
    // 3. Total Output (Final Sum)
    output wire signed [34:0]    out_total, // Sum of all 4
    
    // Global Overload Flag (Aligned to Cycle 10)
    output wire                  overload
);

    //==========================================================================
    // Stage 1: Parallel Multiplication (Latency = 6)
    // Timeline: Input T=0 -> Output T=6
    //==========================================================================
    wire signed [34:0] prod_0, prod_1, prod_2, prod_3;
    wire ov_m0, ov_m1, ov_m2, ov_m3;

    mult_q8_27 u_mult_0 (.clk(clk), .sclr(rst), .a(a), .b(b), .p_out(prod_0), .overload(ov_m0));
    mult_q8_27 u_mult_1 (.clk(clk), .sclr(rst), .a(c), .b(d), .p_out(prod_1), .overload(ov_m1));
    mult_q8_27 u_mult_2 (.clk(clk), .sclr(rst), .a(e), .b(f), .p_out(prod_2), .overload(ov_m2));
    mult_q8_27 u_mult_3 (.clk(clk), .sclr(rst), .a(g), .b(h), .p_out(prod_3), .overload(ov_m3));

    //==========================================================================
    // Stage 2: Intermediate Addition (Latency = 2)
    // Timeline: Input T=6 -> Output T=8
    //==========================================================================
    wire signed [34:0] sum_front, sum_back;
    wire ov_a0, ov_a1;

    // Sum Front: (A*B) + (C*D)
    adder_q8_27 u_add_layer1_front (
        .clk(clk), .rst(rst),
        .a(prod_0), .b(prod_1),
        .s_out(sum_front), .overload(ov_a0)
    );

    // Sum Back: (E*F) + (G*H)
    adder_q8_27 u_add_layer1_back (
        .clk(clk), .rst(rst),
        .a(prod_2), .b(prod_3),
        .s_out(sum_back), .overload(ov_a1)
    );

    //==========================================================================
    // Stage 3: Final Addition (Latency = 2)
    // Timeline: Input T=8 -> Output T=10 (Natural Alignment)
    //==========================================================================
    wire signed [34:0] sum_total;
    wire ov_a2;

    adder_q8_27 u_add_layer2_final (
        .clk(clk), .rst(rst),
        .a(sum_front), .b(sum_back),
        .s_out(sum_total), .overload(ov_a2)
    );

    //==========================================================================
    // Delay Alignment Logic (Compensating specific paths to T=10)
    //==========================================================================

    // --- Path 1: Individual Products ---
    // Current valid at T=6. Need T=10.
    // Delay required = 10 - 6 = 4 cycles.
    reg signed [34:0] dly_p0 [0:3]; // Depth 4
    reg signed [34:0] dly_p1 [0:3];
    reg signed [34:0] dly_p2 [0:3];
    reg signed [34:0] dly_p3 [0:3];
    reg [3:0]         dly_ov_m0, dly_ov_m1, dly_ov_m2, dly_ov_m3; // Overload delays

    always @(posedge clk) begin
        if (rst) begin
             {dly_ov_m0, dly_ov_m1, dly_ov_m2, dly_ov_m3} <= 0;
        end else begin
            // Shift Data
            dly_p0[0] <= prod_0; dly_p0[1] <= dly_p0[0]; dly_p0[2] <= dly_p0[1]; dly_p0[3] <= dly_p0[2];
            dly_p1[0] <= prod_1; dly_p1[1] <= dly_p1[0]; dly_p1[2] <= dly_p1[1]; dly_p1[3] <= dly_p1[2];
            dly_p2[0] <= prod_2; dly_p2[1] <= dly_p2[0]; dly_p2[2] <= dly_p2[1]; dly_p2[3] <= dly_p2[2];
            dly_p3[0] <= prod_3; dly_p3[1] <= dly_p3[0]; dly_p3[2] <= dly_p3[1]; dly_p3[3] <= dly_p3[2];
            
            // Shift Overload Flags
            dly_ov_m0 <= {dly_ov_m0[2:0], ov_m0};
            dly_ov_m1 <= {dly_ov_m1[2:0], ov_m1};
            dly_ov_m2 <= {dly_ov_m2[2:0], ov_m2};
            dly_ov_m3 <= {dly_ov_m3[2:0], ov_m3};
        end
    end

    // --- Path 2: Split Sums ---
    // Current valid at T=8. Need T=10.
    // Delay required = 10 - 8 = 2 cycles.
    reg signed [34:0] dly_sum_f [0:1]; // Depth 2
    reg signed [34:0] dly_sum_b [0:1];
    reg [1:0]         dly_ov_a0, dly_ov_a1;

    always @(posedge clk) begin
        if (rst) begin
            dly_ov_a0 <= 0;
            dly_ov_a1 <= 0;
        end else begin
            // Shift Data
            dly_sum_f[0] <= sum_front; dly_sum_f[1] <= dly_sum_f[0];
            dly_sum_b[0] <= sum_back;  dly_sum_b[1] <= dly_sum_b[0];

            // Shift Overload Flags
            dly_ov_a0 <= {dly_ov_a0[0], ov_a0};
            dly_ov_a1 <= {dly_ov_a1[0], ov_a1};
        end
    end

    //==========================================================================
    // Final Output Assignments
    //==========================================================================
    
    // 1. Individual Products (Delayed 4 cycles)
    assign out_p0 = dly_p0[3];
    assign out_p1 = dly_p1[3];
    assign out_p2 = dly_p2[3];
    assign out_p3 = dly_p3[3];

    // 2. Split Sums (Delayed 2 cycles)
    assign out_ab_cd = dly_sum_f[1];
    assign out_ef_gh = dly_sum_b[1];

    // 3. Total Sum (Natural path 6+2+2 = 10 cycles)
    assign out_total = sum_total;

    // 4. Unified Overload (All aligned at T=10)
    assign overload = dly_ov_m0[3] | dly_ov_m1[3] | dly_ov_m2[3] | dly_ov_m3[3] | // Delayed Mult Ov
                      dly_ov_a0[1] | dly_ov_a1[1] |                               // Delayed Layer1 Ov
                      ov_a2;                                                      // Current Layer2 Ov

endmodule
