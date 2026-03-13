`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/11/29 18:52:51
// Design Name: 
// Module Name: mvme_std_tb
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

module top_mvme_verify (
    input  wire sys_clk_in,  // 板载晶振时钟输入
    input  wire sys_rst_in_n   // 板载物理复位按键 (Active High/Low视板子而定)
);

    wire sys_rst_in;

    assign sys_rst_in = !sys_rst_in_n;

    //==========================================================================
    // 1. Clock Management (Generate 200MHz)
    //==========================================================================
    wire clk_200m;
    wire locked;
    
    clk_wiz_0 u_clk_wiz (
        .clk_out1 (clk_200m),   // 200MHz for MVME
        .reset    (sys_rst_in), // Physical reset
        .locked   (locked),
        .clk_in1  (sys_clk_in)
    );

    //==========================================================================
    // 2. VIO Control (Virtual Input/Output)
    //==========================================================================
    // 通过 Vivado 硬件管理器控制这些信号
    wire vio_rst;
    wire vio_mode; // 0: 固定值模式, 1: 计数器/斜坡模式
    
    vio_0 u_vio (
        .clk        (clk_200m),
        .probe_out0 (vio_rst),
        .probe_out1 (vio_mode)
    );

    // System Reset = Physical Unlock OR VIO Reset
    wire sys_rst = !locked || vio_rst;

    //==========================================================================
    // 3. Stimulus Generator (激励生成器)
    //==========================================================================
    // 我们需要在硬件里造数据。这里设计两种模式：
    // Mode 0: 输入固定值 (验证 1.0 * 1.0)
    // Mode 1: 输入动态斜坡 (验证波形连续性)
    
    reg signed [34:0] a, b, c, d, e, f, g, h;
    reg [34:0] counter;

    // Q8.27 常量定义
    localparam signed [34:0] VAL_ONE = 35'd134217728; // 1.0 in Q8.27
    localparam signed [34:0] VAL_TWO = 35'd268435456; // 2.0 in Q8.27

    always @(posedge clk_200m) begin
        if (sys_rst) begin
            counter <= 0;
            {a,b,c,d,e,f,g,h} <= 0;
        end else begin
            // 计数器用于生成斜坡数据
            counter <= counter + 1;

            if (vio_mode == 0) begin
                // --- 静态模式 ---
                // 所有通道做 1.0 * 1.0，方便检查 out_total 是否为 4.0
                a <= VAL_ONE; b <= VAL_ONE;
                c <= VAL_ONE; d <= VAL_ONE;
                e <= VAL_ONE; f <= VAL_ONE;
                g <= VAL_ONE; h <= VAL_ONE;
            end else begin
                // // --- 动态模式 ---
                // // 让输入变成正弦波类似的三角波，或者简单的计数器
                // // 这里让 A 为斜坡，B 为常数 1.0。Out 应跟随 A 变化。
                // a <= {3'b0, counter[31:0]}; // 随时间增加
                // b <= VAL_ONE;               // 保持 1.0
                
                // // C 为反向斜坡
                // c <= -{3'b0, counter[31:0]};
                // d <= VAL_ONE;
                
                // // 其他保持 0，方便观察 A和C 的抵消效果
                // e <= 0; f <= 0;
                // g <= 0; h <= 0;

                // 通道 1: 正向计数
                a <= {3'b0, counter[31:0]}; 
                b <= VAL_ONE;               // Result = +Counter
                
                // 通道 2: 也是正向计数 (之前是负的，现在改为正的)
                c <= {3'b0, counter[31:0]}; 
                d <= VAL_ONE;               // Result = +Counter
                
                // 通道 3: 输入稍微小一点的数
                e <= {5'b0, counter[29:0]}; // Counter / 4
                f <= VAL_ONE;
                
                // 通道 4: 固定常数测试
                g <= VAL_ONE;               // 1.0
                h <= VAL_TWO;               // 2.0 -> Result = 2.0
            end
        end
    end

    //==========================================================================
    // 4. DUT Instantiation (Device Under Test)
    //==========================================================================
    wire signed [34:0] out_p0, out_p1, out_p2, out_p3;
    wire signed [34:0] out_ab_cd, out_ef_gh;
    wire signed [34:0] out_total;
    wire overload;

    mvme_35_4ch_std u_dut (
        .clk        (clk_200m),
        .rst        (sys_rst),
        
        .a(a), .b(b), 
        .c(c), .d(d), 
        .e(e), .f(f), 
        .g(g), .h(h),
        
        .out_p0     (out_p0),
        .out_p1     (out_p1),
        .out_p2     (out_p2),
        .out_p3     (out_p3),
        
        .out_ab_cd  (out_ab_cd),
        .out_ef_gh  (out_ef_gh),
        
        .out_total  (out_total),
        .overload   (overload)
    );

    //==========================================================================
    // 5. ILA Instantiation (集成逻辑分析仪)
    //==========================================================================
    // 捕捉所有关键信号，确保位宽匹配 IP 核设置
    
    ila_0 u_ila (
        .clk    (clk_200m), // 必须采样时钟
        
        // Inputs (Probes 0-7)
        .probe0 (a),
        .probe1 (b),
        .probe2 (c),
        .probe3 (d),
        .probe4 (e),
        .probe5 (f),
        .probe6 (g),
        .probe7 (h),
        
        // Outputs - Individual (Probes 8-11)
        .probe8 (out_p0),
        .probe9 (out_p1),
        .probe10(out_p2),
        .probe11(out_p3),
        
        // Outputs - Split (Probes 12-13)
        .probe12(out_ab_cd),
        .probe13(out_ef_gh),
        
        // Output - Total & Flag (Probe 14-15)
        .probe14(out_total),
        .probe15(overload)
    );

endmodule
