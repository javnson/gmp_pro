`timescale 1ns/1ps
module tb_control_core_axi;
    localparam W=32; localparam F=28; localparam N=2;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0;
    reg [11:0] awaddr; reg awvalid=0; wire awready;
    reg [31:0] wdata; reg [3:0] wstrb; reg wvalid=0; wire wready;
    wire [1:0] bresp; wire bvalid; reg bready=1;
    reg [11:0] araddr; reg arvalid=0; wire arready;
    wire [31:0] rdata; wire [1:0] rresp; wire rvalid; reg rready=1;
    reg signed [W-1:0] sdata=0; reg svalid=0, slast=0; wire sready;
    wire signed [W-1:0] mdata; wire mvalid, mlast; reg mready=1;
    wire [7:0] adc_mask; wire [2:0] adc_channel;
    wire auto_adc,dac_auto; wire [15:0] dm1,dm2,dm3,dm4;
    wire pe,pca,pce,ptc; wire [31:0] pp,pc,ph; wire [15:0] pd;
    wire busy;
    integer errors=0;

    iir2_control_core #(.DATA_WIDTH(W),.FRAC_WIDTH(F),.NUM_SECTIONS(N)) dut (
        .clk(clk),.rst_n(rst_n),.s_axi_awaddr(awaddr),.s_axi_awvalid(awvalid),
        .s_axi_awready(awready),.s_axi_wdata(wdata),.s_axi_wstrb(wstrb),
        .s_axi_wvalid(wvalid),.s_axi_wready(wready),.s_axi_bresp(bresp),
        .s_axi_bvalid(bvalid),.s_axi_bready(bready),.s_axi_araddr(araddr),
        .s_axi_arvalid(arvalid),.s_axi_arready(arready),.s_axi_rdata(rdata),
        .s_axi_rresp(rresp),.s_axi_rvalid(rvalid),.s_axi_rready(rready),
        .s_axis_tdata(sdata),.s_axis_tvalid(svalid),.s_axis_tready(sready),
        .s_axis_tlast(slast),.m_axis_tdata(mdata),.m_axis_tvalid(mvalid),
        .m_axis_tready(mready),.m_axis_tlast(mlast),.adc_samples_flat(128'd0),
        .pwm_trip_latched(1'b0),.dma_read_busy(1'b0),.dma_write_busy(1'b0),
        .dma_done(1'b0),.dma_error(1'b0),.adc_channel_mask(adc_mask),
        .adc_control_channel(adc_channel),.auto_adc_enable(auto_adc),
        .dac_auto_enable(dac_auto),.dac_manual_ch1(dm1),.dac_manual_ch2(dm2),
        .dac_manual_ch3(dm3),.dac_manual_ch4(dm4),.pwm_enable(pe),
        .pwm_center_aligned(pca),.pwm_control_enable(pce),.pwm_trip_clear(ptc),
        .pwm_period(pp),.pwm_compare_shadow(pc),.pwm_deadtime(pd),
        .pwm_phase(ph),.pipeline_busy(busy)
    );

    task axi_write(input [11:0] addr,input [31:0] data);
        begin
            @(negedge clk); awaddr=addr; wdata=data; wstrb=4'hf;
            awvalid=1; wvalid=1;
            while(!(awready && wready)) @(negedge clk);
            @(negedge clk); awvalid=0; wvalid=0;
            while(!bvalid) @(posedge clk);
            if(bresp!=0) begin errors=errors+1; $display("ERROR AXI BRESP"); end
            @(posedge clk);
        end
    endtask

    task axi_read(input [11:0] addr,input [31:0] expected);
        begin
            @(negedge clk); araddr=addr; arvalid=1;
            while(!arready) @(negedge clk);
            @(negedge clk); arvalid=0;
            while(!rvalid) @(posedge clk);
            if(rdata!==expected) begin
                errors=errors+1; $display("ERROR AXI read %h got=%h expected=%h",addr,rdata,expected);
            end
            @(posedge clk);
        end
    endtask

    task stream_sample(input signed [W-1:0] value,input signed [W-1:0] expected);
        begin
            @(negedge clk); sdata=value; slast=1; svalid=1;
            while(!sready) @(negedge clk);
            @(negedge clk); svalid=0; slast=0;
            while(!mvalid) @(posedge clk);
            if($signed(mdata)!==expected || !mlast) begin
                errors=errors+1; $display("ERROR stream got=%0d expected=%0d",$signed(mdata),expected);
            end
            @(posedge clk);
        end
    endtask

    initial begin
        awaddr=0; wdata=0; wstrb=0; araddr=0;
        repeat(4) @(posedge clk); rst_n=1;
        axi_read(12'h000,32'h4932_5232);
        axi_write(12'h008,32'h0000_0001); // run
        stream_sample(32'sh1000_0000,32'sh1000_0000); // reset identity

        axi_write(12'h040,32'h0800_0000); // b0 = 0.5
        axi_write(12'h044,32'h0000_0000);
        axi_write(12'h048,32'h0000_0000);
        axi_write(12'h04c,32'h0000_0000);
        axi_write(12'h050,32'h0000_0000);
        axi_write(12'h024,32'h0000_0001); // atomic commit
        stream_sample(32'sh1000_0000,32'sh0800_0000);
        axi_read(12'h018,32'd2);

        if(errors==0) $display("PASS tb_control_core_axi");
        else $fatal(1,"FAIL tb_control_core_axi errors=%0d",errors);
        #20 $finish;
    end
endmodule
