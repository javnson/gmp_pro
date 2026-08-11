`timescale 1ns/1ps
module tb_spi_axi_bridge;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0,spi_csn=1,spi_sclk=0,spi_mosi=0;
    wire spi_miso;
    wire [11:0] awaddr,araddr; wire awvalid,awready,wvalid,wready;
    wire [31:0] wdata,rdata; wire [3:0] wstrb; wire [1:0] bresp,rresp;
    wire bvalid,arvalid,arready,rvalid; wire write_pulse,bridge_error;
    wire signed [31:0] out_data; wire out_valid,in_ready,busy;
    integer errors=0;

    spi_axi_lite_bridge bridge(.clk(clk),.rst_n(rst_n),.spi_cs_n(spi_csn),
        .spi_sclk(spi_sclk),.spi_mosi(spi_mosi),.spi_miso(spi_miso),
        .m_axi_awaddr(awaddr),.m_axi_awvalid(awvalid),.m_axi_awready(awready),
        .m_axi_wdata(wdata),.m_axi_wstrb(wstrb),.m_axi_wvalid(wvalid),
        .m_axi_wready(wready),.m_axi_bresp(bresp),.m_axi_bvalid(bvalid),
        .m_axi_bready(),.m_axi_araddr(araddr),.m_axi_arvalid(arvalid),
        .m_axi_arready(arready),.m_axi_rdata(rdata),.m_axi_rresp(rresp),
        .m_axi_rvalid(rvalid),.m_axi_rready(),.write_complete_pulse(write_pulse),
        .bridge_error(bridge_error));

    iir2_control_core #(.NUM_SECTIONS(1)) core(.clk(clk),.rst_n(rst_n),
        .s_axi_awaddr(awaddr),.s_axi_awvalid(awvalid),.s_axi_awready(awready),
        .s_axi_wdata(wdata),.s_axi_wstrb(wstrb),.s_axi_wvalid(wvalid),
        .s_axi_wready(wready),.s_axi_bresp(bresp),.s_axi_bvalid(bvalid),
        .s_axi_bready(1'b1),.s_axi_araddr(araddr),.s_axi_arvalid(arvalid),
        .s_axi_arready(arready),.s_axi_rdata(rdata),.s_axi_rresp(rresp),
        .s_axi_rvalid(rvalid),.s_axi_rready(1'b1),.s_axis_tdata(32'd0),
        .s_axis_tvalid(1'b0),.s_axis_tready(in_ready),.s_axis_tlast(1'b0),
        .m_axis_tdata(out_data),.m_axis_tvalid(out_valid),.m_axis_tready(1'b1),
        .m_axis_tlast(),.adc_samples_flat(128'd0),.pwm_trip_latched(1'b0),
        .dma_read_busy(1'b0),.dma_write_busy(1'b0),.dma_done(1'b0),
        .dma_error(1'b0),.pipeline_busy(busy));

    task spi_transfer(input [15:0] command,input [15:0] write_data,output [15:0] read_data);
        integer bit_index;
        reg [15:0] captured;
        begin
            captured=0; spi_csn=0; #60;
            for(bit_index=15;bit_index>=0;bit_index=bit_index-1) begin
                spi_mosi=command[bit_index]; #50; spi_sclk=1; #50;
                spi_sclk=0; #50;
            end
            for(bit_index=15;bit_index>=0;bit_index=bit_index-1) begin
                spi_mosi=write_data[bit_index]; #50; spi_sclk=1; #25;
                captured[bit_index]=spi_miso; #25; spi_sclk=0; #50;
            end
            spi_csn=1; spi_mosi=0; #100; read_data=captured;
        end
    endtask

    reg [15:0] value;
    initial begin
        repeat(4) @(posedge clk); rst_n=1; #100;
        spi_transfer(16'h8000,16'h0000,value);
        if(value!==16'h5232) begin errors=errors+1; $display("ERROR SPI ID low=%h",value); end
        spi_transfer(16'h8100,16'h0000,value);
        if(value!==16'h4932) begin errors=errors+1; $display("ERROR SPI ID high=%h",value); end
        spi_transfer(16'h0400,16'h0001,value); // CONTROL low half
        #200;
        spi_transfer(16'h8400,16'h0000,value);
        if(value!==16'h0001) begin errors=errors+1; $display("ERROR SPI CONTROL=%h",value); end
        if(bridge_error) begin errors=errors+1; $display("ERROR bridge_error set"); end
        if(errors==0) $display("PASS tb_spi_axi_bridge");
        else $fatal(1,"FAIL tb_spi_axi_bridge errors=%0d",errors);
        #20 $finish;
    end
endmodule
