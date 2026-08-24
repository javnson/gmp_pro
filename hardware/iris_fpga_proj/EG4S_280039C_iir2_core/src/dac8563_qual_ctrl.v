`timescale 1ns / 1ps

module dac8563_quad_ctrl (
    input  wire        clk,        // FPGA系统时钟 (100MHz)
    input  wire        rst_n,      // 异步复位，低电平有效

    input  wire        flush,      // 触发信号
    input  wire [15:0] dac_ch1,    
    input  wire [15:0] dac_ch2,    
    input  wire [15:0] dac_ch3,    
    input  wire [15:0] dac_ch4,    

    output reg         dac_sync_n, 
    output wire        dac_sclk_1, 
    output wire        dac_sclk_2, 
    output reg         dac_din_1,  
    output reg         dac_din_2,  
    output reg         dac_clr_n,  
    output wire        dac_ldac_1_n,
    output wire        dac_ldac_2_n
);

    reg         dac_ldac_n;
    assign dac_ldac_1_n = dac_ldac_n;
    assign dac_ldac_2_n = dac_ldac_n;

    // =========================================================
    // 1. Flush 边缘检测 & "挂起更新" 标志 (修复漏接 Bug)
    // =========================================================
    reg flush_d1;
    wire flush_pulse = (flush && !flush_d1);
    
    reg        pending_update; // 新增：缓冲标志
    reg [15:0] latched_ch1, latched_ch2, latched_ch3, latched_ch4;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            flush_d1 <= 1'b0;
            pending_update <= 1'b0;
            latched_ch1 <= 16'd0;
            latched_ch2 <= 16'd0;
            latched_ch3 <= 16'd0;
            latched_ch4 <= 16'd0;
        end else begin
            flush_d1 <= flush;
            
            // 只要检测到脉冲，就立刻锁存数据，并拉高更新标志
            if (flush_pulse) begin
                latched_ch1 <= dac_ch1;
                latched_ch2 <= dac_ch2;
                latched_ch3 <= dac_ch3;
                latched_ch4 <= dac_ch4;
                pending_update <= 1'b1;
            end
            
            // 当状态机处于 IDLE，并且已经开始处理该更新时，清除标志
            else if (state == ST_IDLE && pending_update) begin
                pending_update <= 1'b0;
            end
        end
    end

    // =========================================================
    // 2. SPI 发送引擎 (保持不变，工作良好)
    // =========================================================
    reg         req_send;
    reg         tx_busy;
    reg [23:0]  tx_data_1;
    reg [23:0]  tx_data_2;
    reg [23:0]  shift_reg_1;
    reg [23:0]  shift_reg_2;
    reg [5:0]   tx_bit_cnt;
    reg [1:0]   tx_clk_div;
    reg         sclk_int;
    
    assign dac_sclk_1 = sclk_int;
    assign dac_sclk_2 = sclk_int;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_busy    <= 1'b0;
            dac_sync_n <= 1'b1;
            sclk_int   <= 1'b0;
            dac_din_1  <= 1'b0;
            dac_din_2  <= 1'b0;
            tx_bit_cnt <= 6'd0;
            tx_clk_div <= 2'd0;
        end else begin
            if (req_send && !tx_busy) begin
                tx_busy     <= 1'b1;
                tx_bit_cnt  <= 6'd0;
                tx_clk_div  <= 2'd0;
                shift_reg_1 <= tx_data_1;
                shift_reg_2 <= tx_data_2;
                dac_sync_n  <= 1'b0;  
                sclk_int    <= 1'b0;
            end else if (tx_busy) begin
                tx_clk_div <= tx_clk_div + 1'b1;
                if (tx_clk_div == 2'd0) begin
                    if (tx_bit_cnt < 6'd24) begin
                        sclk_int    <= 1'b1;
                        dac_din_1   <= shift_reg_1[23];
                        dac_din_2   <= shift_reg_2[23];
                        shift_reg_1 <= {shift_reg_1[22:0], 1'b0};
                        shift_reg_2 <= {shift_reg_2[22:0], 1'b0};
                    end
                end else if (tx_clk_div == 2'd2) begin
                    sclk_int <= 1'b0;
                    if (tx_bit_cnt < 6'd24) begin
                        tx_bit_cnt <= tx_bit_cnt + 1'b1;
                    end else begin
                        tx_busy    <= 1'b0;
                        dac_sync_n <= 1'b1; 
                    end
                end
            end
        end
    end

    // =========================================================
    // 3. 业务状态机
    // =========================================================
    localparam ST_POWER_ON   = 3'd7; 
    localparam ST_INIT       = 3'd0;
    localparam ST_IDLE       = 3'd1;
    localparam ST_SEND_A     = 3'd2;
    localparam ST_WAIT_SYNC  = 3'd3;
    localparam ST_SEND_B     = 3'd4;
    localparam ST_LDAC_PULSE = 3'd5;
    localparam ST_WAIT_TX    = 3'd6;

    reg [2:0]  state;
    reg [2:0]  next_state;
    reg [19:0] delay_cnt; 

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_POWER_ON; 
            req_send   <= 1'b0;
            dac_clr_n  <= 1'b1; 
            dac_ldac_n <= 1'b1;
            delay_cnt  <= 20'd0;
        end else begin
            case (state)
                ST_POWER_ON: begin
                    if (delay_cnt < 20'd1_000_000) begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end else begin
                        delay_cnt <= 20'd0;
                        state     <= ST_INIT;
                    end
                end

                ST_INIT: begin
                    // 修正：0x380000 禁用内部参考。
                    // 严禁在此处写保留位！DAC8563 硬件增益强制为 2，不可更改。
                    tx_data_1 <= 24'h380000; 
                    tx_data_2 <= 24'h380000;
                    req_send  <= 1'b1;
                    next_state <= ST_IDLE;
                    state      <= ST_WAIT_TX;
                end
                
                ST_IDLE: begin
                    // 修正：使用缓冲的更新请求
                    if (pending_update) begin
                        state <= ST_SEND_A;
                    end
                end
                
                ST_SEND_A: begin
                    tx_data_1 <= {8'h00, latched_ch1}; 
                    tx_data_2 <= {8'h00, latched_ch3};
                    req_send  <= 1'b1;
                    next_state <= ST_WAIT_SYNC;
                    state      <= ST_WAIT_TX;
                end
                
                ST_WAIT_SYNC: begin
                    delay_cnt <= delay_cnt + 1'b1;
                    if (delay_cnt == 20'd20) begin 
                        delay_cnt <= 20'd0;
                        state     <= ST_SEND_B;
                    end
                end
                
                ST_SEND_B: begin
                    tx_data_1 <= {8'h01, latched_ch2}; 
                    tx_data_2 <= {8'h01, latched_ch4};
                    req_send  <= 1'b1;
                    next_state <= ST_LDAC_PULSE;
                    state      <= ST_WAIT_TX;
                end
                
                ST_LDAC_PULSE: begin
                    dac_ldac_n <= 1'b0;
                    delay_cnt  <= delay_cnt + 1'b1;
                    if (delay_cnt == 20'd20) begin
                        dac_ldac_n <= 1'b1;
                        delay_cnt  <= 20'd0;
                        state      <= ST_IDLE; 
                    end
                end
                
                ST_WAIT_TX: begin
                    if (req_send && tx_busy) begin
                        req_send <= 1'b0; 
                    end
                    if (!tx_busy && !req_send) begin
                        delay_cnt <= delay_cnt + 1'b1;
                        if (delay_cnt == 20'd5) begin
                            delay_cnt <= 20'd0;
                            state <= next_state; 
                        end
                    end
                end
                
                default: state <= ST_POWER_ON;
            endcase
        end
    end

endmodule