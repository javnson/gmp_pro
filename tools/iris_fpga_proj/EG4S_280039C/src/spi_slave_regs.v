module spi_slave_regs_easy (
    input  wire         clk,        // FPGA系统时钟
    input  wire         rst_n,      // 异步复位，低电平有效
    
    // SPI Slave 接口 (连接至 280039C)
    input  wire         spi_cs_n,
    input  wire         spi_sclk,
    input  wire         spi_mosi,
    output wire         spi_miso,
    
    // 内部控制与数据接口
    input  wire         flush,      // 接口保留，但内部不再依赖
    input  wire [255:0] in_wires,   // 16个16位输入 (地址 0x00 - 0x0F)
    output reg  [255:0] out_regs,   // 16个16位输出 (地址 0x00 - 0x0F)
    
    output reg          write_pulse // 成功写入 out_regs 后，产生一个clk周期的高电平脉冲
);

    //-------------------------------------------------------
    // 1. SPI 信号跨时钟域同步
    //-------------------------------------------------------
    reg [2:0] sclk_sync;
    reg [2:0] cs_sync;
    reg [1:0] mosi_sync;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sclk_sync <= 3'b000;
            cs_sync   <= 3'b111;
            mosi_sync <= 2'b00;
        end else begin
            sclk_sync <= {sclk_sync[1:0], spi_sclk};
            cs_sync   <= {cs_sync[1:0], spi_cs_n};
            mosi_sync <= {mosi_sync[0], spi_mosi};
        end
    end
    
    wire sclk_rise = (sclk_sync[2:1] == 2'b01);
    wire sclk_fall = (sclk_sync[2:1] == 2'b10);
    wire cs_fall   = (cs_sync[2:1]   == 2'b10); // CS 下降沿检测
    wire cs_active = ~cs_sync[1]; 

    //-------------------------------------------------------
    // 2. 自动快照逻辑 (每次 CS 拉低时自动锁存最新状态)
    //-------------------------------------------------------
    reg [255:0] in_wires_snapshot;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_wires_snapshot <= 256'd0;
        end else begin
            if (cs_fall) begin
                // 确保 DSP 读取时，总线数据是同一时刻的完整切片
                in_wires_snapshot <= in_wires; 
            end
        end
    end

    //-------------------------------------------------------
    // 3. SPI 状态机与收发逻辑
    //-------------------------------------------------------
    reg [4:0]  bit_cnt;       
    reg [15:0] rx_shift_reg;
    reg [15:0] tx_shift_reg;
    
    reg        is_data_phase; 
    reg        is_write;      
    reg [6:0]  cmd_addr;      

    assign spi_miso = (cs_active) ? tx_shift_reg[15] : 1'bz;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_cnt       <= 5'd0;
            is_data_phase <= 1'b0;
            rx_shift_reg  <= 16'd0;
            tx_shift_reg  <= 16'd0;
            out_regs      <= 256'd0;
            is_write      <= 1'b0;
            cmd_addr      <= 7'd0;
            write_pulse   <= 1'b0;
        end else begin
            write_pulse <= 1'b0; // 默认拉低
            
            if (!cs_active) begin
                bit_cnt       <= 5'd0;
                is_data_phase <= 1'b0;
            end else begin
                // --- 采样 MOSI (SCLK 上升沿) ---
                if (sclk_rise) begin
                    rx_shift_reg <= {rx_shift_reg[14:0], mosi_sync[1]};
                    bit_cnt <= bit_cnt + 1'b1;
                    
                    if (bit_cnt == 5'd15) begin
                        bit_cnt <= 5'd0;
                        
                        if (!is_data_phase) begin
                            // 【命令帧解析完毕】
                            is_data_phase <= 1'b1;
                            // 规则：最高位(Bit 15)为 1 是读，0 是写
                            is_write      <= ~rx_shift_reg[14];    
                            cmd_addr      <= rx_shift_reg[13:7];   
                        end else begin
                            // 【数据帧解析完毕】
                            is_data_phase <= 1'b0; 
                            
                            // 只有判定为写操作，才更新 out_regs
                            if (is_write && (cmd_addr < 7'h10)) begin
                                out_regs[(cmd_addr * 16) +: 16] <= {rx_shift_reg[14:0], mosi_sync[1]};
                                write_pulse <= 1'b1; 
                            end
                        end
                    end
                end
                
                // --- 准备 MISO (SCLK 下降沿) ---
                if (sclk_fall) begin
                    if (!is_data_phase && bit_cnt == 5'd0) begin
                        // 命令帧接收后的第一个下降沿，准备数据帧的回读数据
                        tx_shift_reg <= 16'h0000;
                    end 
                    else if (is_data_phase && bit_cnt == 5'd0) begin
                        // 如果判定为读操作，且地址合法，从快照中取出数据送往 DSP
                        if (!is_write && (cmd_addr < 7'h10)) begin
                            tx_shift_reg <= in_wires_snapshot[(cmd_addr * 16) +: 16];
                        end else begin
                            tx_shift_reg <= 16'h0000; 
                        end
                    end 
                    else begin
                        // 常规移位输出
                        tx_shift_reg <= {tx_shift_reg[14:0], 1'b0};
                    end
                end
            end
        end
    end

endmodule