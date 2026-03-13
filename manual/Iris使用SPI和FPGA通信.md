备份使用SPI和FPGA通信



``` C

//    // 确保发送前接收 FIFO 是空的
//        while(SPI_getRxFIFOStatus(IRIS_SPI_FPGA_BRIDGE_BASE) != SPI_FIFO_RXEMPTY) {
//            SPI_readDataNonBlocking(IRIS_SPI_FPGA_BRIDGE_BASE);
//        }
//
//    // 1. 发送数据到 FPGA
//            // 注意：SPI 是交换协议，发送的同时也会接收
//            SPI_writeDataNonBlocking(IRIS_SPI_FPGA_BRIDGE_BASE, sData);
//
////            // 2. 严格等待传输完成标志 (非 FIFO 模式通常检查 SPISTS 寄存器的 INT_FLAG)
////                while(!(HWREGH(IRIS_SPI_FPGA_BRIDGE_BASE + SPI_O_STS) & SPI_STS_INT_FLAG)) {
////                    // 等待硬件完成 16-bit 移位
////                }
//
//            // 2. 阻塞等待：直到接收 FIFO 收到至少一个数据
//                // 这样保证了 DSP 的 SPI 模块已经完成了 16 个时钟周期的物理移位
////                while(SPI_getRxFIFOStatus(IRIS_SPI_FPGA_BRIDGE_BASE) == SPI_FIFO_RXEMPTY);
//
//                // 等待接收 FIFO 至少有一个数据 (水位线检测)
//                    while(SPI_getRxFIFOStatus(IRIS_SPI_FPGA_BRIDGE_BASE) < SPI_FIFO_RX1);
//
//            // 3. 读取数据
//            rData = SPI_readDataNonBlocking(IRIS_SPI_FPGA_BRIDGE_BASE);
////            rData = SPI_readDataBlockingNonFIFO(IRIS_SPI_FPGA_BRIDGE_BASE);
////            rData = SPI_readDataBlockingNonFIFO(IRIS_SPI_FPGA_BRIDGE_BASE);
//
//
//            // 4. 验证数据 (第一次发送可能收到空数据，取决于 FPGA 复位状态)
//            // 这里的逻辑是：如果 FPGA 是原样返回，rData 应该等于上一次发送的 sData
//            if(rData == sData) {
//                passCount++;
//            } else {
//                failCount++;
//            }
//
//            // 5. 更新数据准备下一次测试
//            sData++;
//
//            DEVICE_DELAY_US(100); // 延时 1ms，方便示波器抓包


//        SPI_writeDataNonBlocking(IRIS_SPI_FPGA_BRIDGE_BASE, sData);
//        DEVICE_DELAY_US(2);
//        rData = SPI_readDataNonBlocking(IRIS_SPI_FPGA_BRIDGE_BASE);


        // 步骤 A: 确保接收 FIFO 是空的（排除任何意外干扰）
            while(SPI_getRxFIFOStatus(IRIS_SPI_FPGA_BRIDGE_BASE) != SPI_FIFO_RXEMPTY)
            {
                SPI_readDataNonBlocking(IRIS_SPI_FPGA_BRIDGE_BASE);
            }

            // 步骤 B: 发送数据
            SPI_writeDataNonBlocking(IRIS_SPI_FPGA_BRIDGE_BASE, sData);
            SPI_writeDataNonBlocking(IRIS_SPI_FPGA_BRIDGE_BASE, sData);

            // 步骤 C: 关键！等待硬件完成移位 (等待 RX FIFO 变为非空)
            // 此时硬件已物理完成了 16bit 的交换
            while(SPI_getRxFIFOStatus(IRIS_SPI_FPGA_BRIDGE_BASE) == SPI_FIFO_RXEMPTY)
            {
                // 阻塞等待，此处时间由 SPI 速率决定 (10MHz 约为 1.6us)
            }

            // 步骤 D: 读取本次交换回来的数据
            rData = SPI_readDataNonBlocking(IRIS_SPI_FPGA_BRIDGE_BASE);

        sData++;
        DEVICE_DELAY_US(100);
```

可能最稳妥的方法就是阻塞通信，基本上1.1us能够发送16bit。

上面的测试代码说明了32bit（两次发送完成之后）NCS才会被再次拉高，没有问题，可以使用。



下面备份一下使用的SPI外设。

``` verilog
///////////////////////////////////////////////////////////////////////////////
// Description: Optimized 16-bit SPI Slave for TI DSP Communication
//              - Fixed HDL-8007 assignment error
//              - Supports 16-bit word length
//              - Tri-state MISO when CS_n is high
///////////////////////////////////////////////////////////////////////////////

module SPI_Slave
  #(parameter SPI_MODE = 0)
  (
   // FPGA Logic Signals
   input            i_Rst_L,    // FPGA Reset, active low
   input            i_Clk,      // FPGA System Clock
   output reg       o_RX_DV,    // Data Valid pulse
   output reg [15:0] o_RX_Word,  // 16-bit Word received
   input            i_TX_DV,    // TX Data Valid pulse
   input  [15:0]    i_TX_Word,  // 16-bit Word to send

   // SPI Interface Pins
   input            i_SPI_Clk,
   output           o_SPI_MISO, // Fixed: Changed to wire for assign
   input            i_SPI_MOSI,
   input            i_SPI_CS_n
   );

  // Control Signals
  wire w_CPHA;
  wire w_SPI_Clk;
  wire w_MISO_Mux;
  
  // Registers
  reg [3:0] r_RX_Bit_Count;     // 4-bit count for 0-15
  reg [3:0] r_TX_Bit_Count;
  reg [15:0] r_Temp_RX_Word;
  reg [15:0] r_RX_Word_Full;
  reg r_RX_Done, r2_RX_Done, r3_RX_Done;
  reg [15:0] r_TX_Word_Reg;
  reg r_SPI_MISO_Bit, r_Preload_MISO;

  // SPI Mode Configuration
  assign w_CPHA = (SPI_MODE == 1) | (SPI_MODE == 3);
  assign w_SPI_Clk = w_CPHA ? ~i_SPI_Clk : i_SPI_Clk;

  // --- Receiver Logic (MOSI) ---
  always @(posedge w_SPI_Clk or posedge i_SPI_CS_n)
  begin
    if (i_SPI_CS_n) begin
      r_RX_Bit_Count <= 4'b0000;
      r_RX_Done      <= 1'b0;
    end else begin
      r_RX_Bit_Count <= r_RX_Bit_Count + 1'b1;
      r_Temp_RX_Word <= {r_Temp_RX_Word[14:0], i_SPI_MOSI};
    
      if (r_RX_Bit_Count == 4'b1111) begin
        r_RX_Done      <= 1'b1;
        r_RX_Word_Full <= {r_Temp_RX_Word[14:0], i_SPI_MOSI};
      end else begin
        r_RX_Done      <= 1'b0;
      end
    end
  end

  // --- Clock Domain Crossing (SPI -> FPGA) ---
  always @(posedge i_Clk or negedge i_Rst_L)
  begin
    if (~i_Rst_L) begin
      {r2_RX_Done, r3_RX_Done} <= 2'b00;
      o_RX_DV   <= 1'b0;
      o_RX_Word <= 16'h0000;
    end else begin
      r2_RX_Done <= r_RX_Done;
      r3_RX_Done <= r2_RX_Done;

      if (r3_RX_Done == 1'b0 && r2_RX_Done == 1'b1) begin // Edge Detect
        o_RX_DV   <= 1'b1;
        o_RX_Word <= r_RX_Word_Full;
      end else begin
        o_RX_DV   <= 1'b0;
      end
    end
  end

  // --- Transmitter Logic (MISO) ---
  // Preload MSB when CS_n goes low
  always @(posedge w_SPI_Clk or posedge i_SPI_CS_n)
  begin
    if (i_SPI_CS_n)
      r_Preload_MISO <= 1'b1;
    else
      r_Preload_MISO <= 1'b0;
  end

  always @(posedge w_SPI_Clk or posedge i_SPI_CS_n)
  begin
    if (i_SPI_CS_n) begin
      r_TX_Bit_Count <= 4'b1111;
      r_SPI_MISO_Bit <= r_TX_Word_Reg[15];
    end else begin
      r_TX_Bit_Count <= r_TX_Bit_Count - 1'b1;
      r_SPI_MISO_Bit <= r_TX_Word_Reg[r_TX_Bit_Count];
    end
  end

  // Register TX Word from FPGA Logic
  always @(posedge i_Clk or negedge i_Rst_L)
  begin
    if (~i_Rst_L)
      r_TX_Word_Reg <= 16'h0000;
    else if (i_TX_DV)
      r_TX_Word_Reg <= i_TX_Word;
  end

  // --- Output Drivers ---
  // Fixes HDL-8007 by using intermediate wire for MISO mux
  assign w_MISO_Mux = r_Preload_MISO ? r_TX_Word_Reg[15] : r_SPI_MISO_Bit;
  assign o_SPI_MISO = i_SPI_CS_n ? 1'bZ : w_MISO_Mux;

endmodule

```

