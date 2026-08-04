
module top_soc (
    input  wire        clk_in,       // 系统时钟
    input  wire        rst_n,        // 系统复位 (低电平有效)

    input  wire [3:0]  encoder,      // 拨码开关输入
    output wire [3:0]  led,          // LED 输出

    // 外部 SPI 接口 (连接到 IO 引脚)
    input  wire        spi_sclk,
    input  wire        spi_csn,
    input  wire        spi_mosi,  
    output wire        spi_miso,  

    // DAC 接口 (连接到 IO 引脚)
    output wire [1:0]  dac_din,
    output wire [1:0]  dac_sclk,
    output wire        dac_sync, 
    output wire        dac_clr,
    output wire [1:0]  dac_ldac,

    // ADC 接口 (连接到 IO 引脚)
    output wire        adc_sdi,
    input  wire        adc_sdo,
    output wire        adc_sclk,
    output wire        adc_cs, 
    output wire        adc_rst,

    // GPIO interface
    output wire [3:0]  gpio
);

    // 内部复位信号处理
    wire rst; 
    assign rst = ~rst_n; 
    
    wire clk;

    // 2. PLL 实例化
    clock_manager u_pll_main(
        .refclk   (clk_in),
        .reset    (~rst_n),
        .clk0_out (clk)
    );

    // ---------------------------------------------------------
    // 内部信号与数据总线定义
    // ---------------------------------------------------------
    wire [255:0] spi_in_wires;  // 传输给 DSP 读取的数据
    wire [255:0] spi_out_regs;  // DSP 写入的数据
    wire         flush_pulse;   // DSP 每次写操作触发的同步脉冲

    wire [15:0]  adc_data [0:7]; // ADC 8个通道的数据缓存

    // ---------------------------------------------------------
    // 3. 内存映射 (Memory Mapping) 核心路由
    // ---------------------------------------------------------
    
    // R0 (0x00): 保留配置寄存器 (读写回环)
    assign spi_in_wires[15:0]   = spi_out_regs[15:0];
    
    // R1 (0x01): GPIO 输出寄存器 (读写回环，低4位接LED)
    assign spi_in_wires[31:16]  = spi_out_regs[31:16];
    assign led = spi_out_regs[19:16]; // 取 R1 的低 4 位控制 LED
    
    // R2 (0x02): GPIO 输入寄存器 (只读，低4位接Encoder)
    // 高12位补0，低4位实时读取外部引脚电平
    assign spi_in_wires[47:32]  = {12'h000, encoder[3:0]};
    
    // R3 (0x03): ADC 控制寄存器 (读写回环，低8位为通道掩码)
    assign spi_in_wires[63:48]  = spi_out_regs[63:48];

    // R4-R7 (0x04-0x07): DAC 数据寄存器 (读写回环)
    assign spi_in_wires[127:64] = spi_out_regs[127:64];

    // R8-R15 (0x08-0x0F): ADC 采集结果寄存器 (只读)
    assign spi_in_wires[143:128] = adc_data[0];
    assign spi_in_wires[159:144] = adc_data[1];
    assign spi_in_wires[175:160] = adc_data[2];
    assign spi_in_wires[191:176] = adc_data[3];
    assign spi_in_wires[207:192] = adc_data[4];
    assign spi_in_wires[223:208] = adc_data[5];
    assign spi_in_wires[239:224] = adc_data[6];
    assign spi_in_wires[255:240] = adc_data[7];


    // ---------------------------------------------------------
    // 4. 子模块实例化
    // ---------------------------------------------------------
    
    // SPI 从机模块
    spi_slave_regs_easy u_spi_slave_regs (
        .clk         (clk),
        .rst_n       (rst_n),
        .spi_cs_n    (spi_csn),
        .spi_sclk    (spi_sclk),
        .spi_mosi    (spi_mosi),
        .spi_miso    (spi_miso),
        .flush       (flush_pulse),  
        .in_wires    (spi_in_wires), 
        .out_regs    (spi_out_regs), 
        .write_pulse (flush_pulse)   
    );

    // 四通道 DAC 控制器
    dac8563_quad_ctrl u_quad_dac(
        .clk          (clk),
        .rst_n        (rst_n),
        .flush        (flush_pulse), 
        // 从 R4~R7 (bit 64 到 127) 中提取目标值
        .dac_ch1      (spi_out_regs[79:64]),  
        .dac_ch2      (spi_out_regs[95:80]),  
        .dac_ch3      (spi_out_regs[111:96]), 
        .dac_ch4      (spi_out_regs[127:112]),
        .dac_sync_n   (dac_sync),
        .dac_sclk_1   (dac_sclk[0]),
        .dac_sclk_2   (dac_sclk[1]),
        .dac_din_1    (dac_din[0]),
        .dac_din_2    (dac_din[1]),
        .dac_clr_n    (dac_clr),
        .dac_ldac_1_n (dac_ldac[0]),
        .dac_ldac_2_n (dac_ldac[1])
    );

    // 八通道 ADC 控制器
    ads8688_masked_ctrl u_ads8688_adc(
        .clk          (clk),
        .rst_n        (rst_n),
        .trigger      (flush_pulse), 
        // 从 R3 (bit 48 到 63) 的低 8 位提取掩码配置
        .channel_mask (spi_out_regs[55:48]), 
        .adc_ch0      (adc_data[0]), 
        .adc_ch1      (adc_data[1]), 
        .adc_ch2      (adc_data[2]), 
        .adc_ch3      (adc_data[3]),
        .adc_ch4      (adc_data[4]), 
        .adc_ch5      (adc_data[5]), 
        .adc_ch6      (adc_data[6]), 
        .adc_ch7      (adc_data[7]),
        .update_pulse (), 
        .spi_cs_n     (adc_cs),
        .spi_sclk     (adc_sclk),
        .spi_mosi     (adc_sdi),
        .spi_miso     (adc_sdo),
        .spi_rst_n    (adc_rst)
    );

endmodule
