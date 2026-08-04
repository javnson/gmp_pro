module ads8688_masked_ctrl (
    input  wire        clk,          // FPGA系统时钟 (例如 100MHz)
    input  wire        rst_n,        // 异步复位，低电平有效

    // 控制与数据接口
    input  wire        trigger,      // 触发信号
    input  wire [7:0]  channel_mask, // 掩码：每一位对应一个通道 (1=采集, 0=跳过)
    
    // ADC 数据输出 (只有掩码对应的通道才会被更新，其他保持原值)
    output reg  [15:0] adc_ch0,
    output reg  [15:0] adc_ch1,
    output reg  [15:0] adc_ch2,
    output reg  [15:0] adc_ch3,
    output reg  [15:0] adc_ch4,
    output reg  [15:0] adc_ch5,
    output reg  [15:0] adc_ch6,
    output reg  [15:0] adc_ch7,
    output reg         update_pulse, // 一次完整的掩码扫描完成后产生一个高电平脉冲

    // ADS8688 硬件 SPI 接口
    output reg         spi_cs_n,
    output wire        spi_sclk,
    output reg         spi_mosi,
    input  wire        spi_miso,
    output wire        spi_rst_n
);

    // ADS8688 硬件复位直接绑定系统复位
    assign spi_rst_n = rst_n;

    // =========================================================
    // 1. 触发边缘检测与挂起缓冲 (防止漏接)
    // =========================================================
    reg trigger_d1;
    wire trigger_pulse = (trigger && !trigger_d1);
    reg pending_trigger;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            trigger_d1 <= 1'b0;
            pending_trigger <= 1'b0;
        end else begin
            trigger_d1 <= trigger;
            if (trigger_pulse) begin
                pending_trigger <= 1'b1;
            end 
            else if (state == ST_IDLE && pending_trigger) begin
                pending_trigger <= 1'b0; // 状态机已响应，清除标记
            end
        end
    end

    // =========================================================
    // 2. 状态机定义
    // =========================================================
    localparam ST_POWER_ON  = 3'd6; // 上电等待
    localparam ST_IDLE      = 3'd0;
    localparam ST_FIND_NEXT = 3'd1;
    localparam ST_CS_LOW    = 3'd2;
    localparam ST_SPI_TXRX  = 3'd3;
    localparam ST_CS_HIGH   = 3'd4;
    localparam ST_DONE      = 3'd5;

    reg [2:0]  state;
    reg [7:0]  mask_reg;
    reg [19:0] delay_cnt; // 延时计数器

    // SPI 引擎与流水线追踪
    reg [31:0] tx_shift_reg;
    reg [31:0] rx_shift_reg;
    reg [5:0]  bit_cnt;
    reg [2:0]  sclk_div; 
    reg        sclk_int;

    reg        reading_valid;      
    reg [2:0]  reading_ch;         
    reg        nxt_reading_valid;
    reg [2:0]  nxt_reading_ch;

    assign spi_sclk = sclk_int;

    // =========================================================
    // 3. 掩码优先编码器
    // =========================================================
    wire [2:0] next_ch_idx;
    wire       has_next = (mask_reg != 8'd0);
    
    assign next_ch_idx = mask_reg[0] ? 3'd0 :
                         mask_reg[1] ? 3'd1 :
                         mask_reg[2] ? 3'd2 :
                         mask_reg[3] ? 3'd3 :
                         mask_reg[4] ? 3'd4 :
                         mask_reg[5] ? 3'd5 :
                         mask_reg[6] ? 3'd6 :
                         mask_reg[7] ? 3'd7 : 3'd0;

    // =========================================================
    // 4. 主控与 SPI 状态机
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= ST_POWER_ON;
            mask_reg      <= 8'd0;
            delay_cnt     <= 20'd0;
            spi_cs_n      <= 1'b1;
            spi_mosi      <= 1'b0;
            sclk_int      <= 1'b0;
            update_pulse  <= 1'b0;
            reading_valid <= 1'b0;
            reading_ch    <= 3'd0;
            adc_ch0       <= 16'd0; adc_ch1 <= 16'd0; adc_ch2 <= 16'd0; adc_ch3 <= 16'd0;
            adc_ch4       <= 16'd0; adc_ch5 <= 16'd0; adc_ch6 <= 16'd0; adc_ch7 <= 16'd0;
        end else begin
            case(state)
                ST_POWER_ON: begin
                    // 10ms 上电延时，等待 ADC 内部初始化完成 (基于100MHz时钟)
                    if (delay_cnt < 20'd1_000_000) begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end else begin
                        delay_cnt <= 20'd0;
                        state     <= ST_IDLE;
                    end
                end

                ST_IDLE: begin
                    spi_cs_n     <= 1'b1;
                    sclk_int     <= 1'b0;
                    spi_mosi     <= 1'b0;
                    update_pulse <= 1'b0;
                    reading_valid<= 1'b0;
                    delay_cnt    <= 20'd0;
                    
                    if (pending_trigger && channel_mask != 8'd0) begin
                        mask_reg <= channel_mask;
                        state    <= ST_FIND_NEXT;
                    end
                end

                ST_FIND_NEXT: begin
                    if (has_next) begin
                        tx_shift_reg      <= {16'hC000 | ({13'd0, next_ch_idx} << 10), 16'h0000};
                        nxt_reading_valid <= 1'b1;
                        nxt_reading_ch    <= next_ch_idx;
                        mask_reg[next_ch_idx] <= 1'b0; 
                        state             <= ST_CS_LOW;
                    end 
                    else if (reading_valid) begin
                        tx_shift_reg      <= 32'h0000_0000; // NOP 空操作，用于挤出最后一次流水线数据
                        nxt_reading_valid <= 1'b0;
                        state             <= ST_CS_LOW;
                    end 
                    else begin
                        update_pulse <= 1'b1;
                        state        <= ST_DONE;
                    end
                end

                ST_CS_LOW: begin
                    spi_cs_n     <= 1'b0;
                    spi_mosi     <= tx_shift_reg[31];
                    tx_shift_reg <= {tx_shift_reg[30:0], 1'b0};
                    sclk_div     <= 3'd0;
                    bit_cnt      <= 6'd0;
                    sclk_int     <= 1'b0;
                    state        <= ST_SPI_TXRX;
                end

                ST_SPI_TXRX: begin
                    sclk_div <= sclk_div + 1'b1;
                    if (sclk_div == 3'd3) begin
                        sclk_int     <= 1'b1;
                        rx_shift_reg <= {rx_shift_reg[30:0], spi_miso};
                    end 
                    else if (sclk_div == 3'd7) begin
                        sclk_int <= 1'b0;
                        if (bit_cnt == 6'd31) begin
                            state <= ST_CS_HIGH; 
                        end else begin
                            bit_cnt      <= bit_cnt + 1'b1;
                            spi_mosi     <= tx_shift_reg[31];
                            tx_shift_reg <= {tx_shift_reg[30:0], 1'b0};
                        end
                    end
                end

                ST_CS_HIGH: begin
                    spi_cs_n <= 1'b1;
                    
                    // 仅在刚进入本状态时执行一次数据保存
                    if (delay_cnt == 20'd0) begin
                        if (reading_valid) begin
                            case (reading_ch)
                                3'd0: adc_ch0 <= rx_shift_reg[15:0];
                                3'd1: adc_ch1 <= rx_shift_reg[15:0];
                                3'd2: adc_ch2 <= rx_shift_reg[15:0];
                                3'd3: adc_ch3 <= rx_shift_reg[15:0];
                                3'd4: adc_ch4 <= rx_shift_reg[15:0];
                                3'd5: adc_ch5 <= rx_shift_reg[15:0];
                                3'd6: adc_ch6 <= rx_shift_reg[15:0];
                                3'd7: adc_ch7 <= rx_shift_reg[15:0];
                            endcase
                        end
                        reading_valid <= nxt_reading_valid;
                        reading_ch    <= nxt_reading_ch;
                    end

                    // --- 核心修复：保持 CS 高电平 100 个系统时钟周期 (1000ns) ---
                    // 确保 ADS8688 内部采样保持电容充分充电
                    delay_cnt <= delay_cnt + 1'b1;
                    if (delay_cnt == 20'd10) begin
                        delay_cnt <= 20'd0;
                        state <= ST_FIND_NEXT;
                    end
                end

                ST_DONE: begin
                    update_pulse <= 1'b0; 
                    state        <= ST_IDLE;
                end
                
                default: state <= ST_POWER_ON;
            endcase
        end
    end

endmodule