`timescale 1ns/1ps
module tb_external_memory_system;
    localparam COUNT=20;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0;
    reg [11:0] awaddr=0; reg awvalid=0; wire awready;
    reg [31:0] wdata=0; reg [3:0] wstrb=0; reg wvalid=0; wire wready;
    wire [1:0] bresp; wire bvalid; reg bready=1;
    reg [11:0] araddr=0; reg arvalid=0; wire arready;
    wire [31:0] rdata; wire [1:0] rresp; wire rvalid; reg rready=1;
    wire [0:0] arid,awid; wire [31:0] mem_araddr,mem_awaddr;
    wire [7:0] arlen,awlen; wire [2:0] arsize,awsize;
    wire [1:0] arburst,awburst; wire arlock,awlock;
    wire [3:0] arcache,awcache,arqos,awqos; wire [2:0] arprot,awprot;
    wire mem_arvalid,mem_rready,mem_awvalid,mem_wlast,mem_wvalid,mem_bready;
    reg mem_arready,mem_rvalid,mem_rlast,mem_awready,mem_wready,mem_bvalid;
    reg [31:0] mem_rdata; reg [1:0] mem_rresp=0,mem_bresp=0;
    wire [31:0] mem_wdata; wire [3:0] mem_wstrb;
    wire dma_busy,dma_done,dma_error;
    wire [31:0] injection_data;
    wire injection_valid,injection_ready,injection_last;
    reg [31:0] memory [0:255];
    reg read_active,write_active;
    integer read_word,read_left,write_word,write_left;
    integer i,errors=0;

    iir2_external_memory_system #(.NUM_SECTIONS(2),.MAX_BURST_LEN(16)) dut (
        .clk(clk),.rst_n(rst_n),.s_axi_ctrl_awaddr(awaddr),
        .s_axi_ctrl_awvalid(awvalid),.s_axi_ctrl_awready(awready),
        .s_axi_ctrl_wdata(wdata),.s_axi_ctrl_wstrb(wstrb),
        .s_axi_ctrl_wvalid(wvalid),.s_axi_ctrl_wready(wready),
        .s_axi_ctrl_bresp(bresp),.s_axi_ctrl_bvalid(bvalid),
        .s_axi_ctrl_bready(bready),.s_axi_ctrl_araddr(araddr),
        .s_axi_ctrl_arvalid(arvalid),.s_axi_ctrl_arready(arready),
        .s_axi_ctrl_rdata(rdata),.s_axi_ctrl_rresp(rresp),
        .s_axi_ctrl_rvalid(rvalid),.s_axi_ctrl_rready(rready),
        .s_axis_live_tdata(32'd0),.s_axis_live_tvalid(1'b0),.s_axis_live_tready(),
        .s_axis_live_tlast(1'b0),.m_axis_live_tdata(),.m_axis_live_tvalid(),
        .m_axis_live_tready(1'b1),.m_axis_live_tlast(),.result_tap_data(),
        .result_tap_valid(),.m_axis_injection_tdata(injection_data),
        .m_axis_injection_tvalid(injection_valid),
        .m_axis_injection_tready(injection_ready),
        .m_axis_injection_tlast(injection_last),
        .s_axis_measurement_tdata(injection_data),
        .s_axis_measurement_tvalid(injection_valid),
        .s_axis_measurement_tready(injection_ready),
        .s_axis_measurement_tlast(injection_last),
        .m_axi_mem_arid(arid),.m_axi_mem_araddr(mem_araddr),
        .m_axi_mem_arlen(arlen),.m_axi_mem_arsize(arsize),
        .m_axi_mem_arburst(arburst),.m_axi_mem_arlock(arlock),
        .m_axi_mem_arcache(arcache),.m_axi_mem_arprot(arprot),
        .m_axi_mem_arqos(arqos),.m_axi_mem_arvalid(mem_arvalid),
        .m_axi_mem_arready(mem_arready),.m_axi_mem_rid(1'b0),
        .m_axi_mem_rdata(mem_rdata),.m_axi_mem_rresp(mem_rresp),
        .m_axi_mem_rlast(mem_rlast),.m_axi_mem_rvalid(mem_rvalid),
        .m_axi_mem_rready(mem_rready),.m_axi_mem_awid(awid),
        .m_axi_mem_awaddr(mem_awaddr),.m_axi_mem_awlen(awlen),
        .m_axi_mem_awsize(awsize),.m_axi_mem_awburst(awburst),
        .m_axi_mem_awlock(awlock),.m_axi_mem_awcache(awcache),
        .m_axi_mem_awprot(awprot),.m_axi_mem_awqos(awqos),
        .m_axi_mem_awvalid(mem_awvalid),.m_axi_mem_awready(mem_awready),
        .m_axi_mem_wdata(mem_wdata),.m_axi_mem_wstrb(mem_wstrb),
        .m_axi_mem_wlast(mem_wlast),.m_axi_mem_wvalid(mem_wvalid),
        .m_axi_mem_wready(mem_wready),.m_axi_mem_bid(1'b0),
        .m_axi_mem_bresp(mem_bresp),.m_axi_mem_bvalid(mem_bvalid),
        .m_axi_mem_bready(mem_bready),.dma_busy(dma_busy),
        .dma_done(dma_done),.dma_error(dma_error));

    task axi_write(input [11:0] addr,input [31:0] data);
        begin
            @(negedge clk); awaddr=addr; wdata=data; wstrb=4'hf; awvalid=1; wvalid=1;
            while(!(awready&&wready)) @(negedge clk);
            @(negedge clk); awvalid=0; wvalid=0;
            while(!bvalid) @(posedge clk);
            @(posedge clk);
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            mem_arready<=1; mem_rvalid<=0; mem_rlast<=0; read_active<=0;
            mem_awready<=1; mem_wready<=0; mem_bvalid<=0; write_active<=0;
            read_word<=0; read_left<=0; write_word<=0; write_left<=0;
        end else begin
            if(mem_arvalid && mem_arready) begin
                read_active<=1; read_word<=mem_araddr>>2; read_left<=arlen+1;
                mem_rdata<=memory[mem_araddr>>2]; mem_rvalid<=1;
                mem_rlast<=(arlen==0); mem_arready<=0;
            end else if(mem_rvalid && mem_rready) begin
                if(read_left==1) begin
                    mem_rvalid<=0; mem_rlast<=0; read_active<=0; mem_arready<=1;
                end else begin
                    read_word<=read_word+1; read_left<=read_left-1;
                    mem_rdata<=memory[read_word+1]; mem_rlast<=(read_left==2);
                end
            end
            if(mem_awvalid && mem_awready) begin
                write_active<=1; write_word<=mem_awaddr>>2; write_left<=awlen+1;
                mem_awready<=0; mem_wready<=1;
            end
            if(mem_wvalid && mem_wready) begin
                memory[write_word]<=mem_wdata;
                if(write_left==1) begin
                    if(!mem_wlast) errors<=errors+1;
                    write_active<=0; mem_wready<=0; mem_bvalid<=1;
                end else begin
                    write_word<=write_word+1; write_left<=write_left-1;
                end
            end
            if(mem_bvalid && mem_bready) begin
                mem_bvalid<=0; mem_awready<=1;
            end
        end
    end

    initial begin
        for(i=0;i<256;i=i+1) memory[i]=0;
        for(i=0;i<COUNT;i=i+1) memory[i]=32'h0100_0000+i;
        repeat(4) @(posedge clk); rst_n=1;
        axi_write(12'h008,32'h1);       // run enable
        axi_write(12'h0E4,32'h0000_0000);
        axi_write(12'h0E8,32'h0000_0100);
        axi_write(12'h0EC,COUNT);
        axi_write(12'h0E0,32'h3);       // memory mode + start
        wait(dma_done); @(posedge clk);
        if(dma_error) begin errors=errors+1; $display("ERROR DMA status"); end
        for(i=0;i<COUNT;i=i+1) begin
            if(memory[64+i] !== (32'h0100_0000+i)) begin
                errors=errors+1;
                $display("ERROR memory[%0d]=%h expected=%h",64+i,memory[64+i],32'h0100_0000+i);
            end
        end
        // Identification mode bypasses IIR: DDR injection stream is exposed
        // to the plant and the independent measurement stream is captured.
        for(i=0;i<5;i=i+1) memory[128+i]=32'h2000_0000+i;
        axi_write(12'h0E4,32'h0000_0200);
        axi_write(12'h0E8,32'h0000_0300);
        axi_write(12'h0EC,5);
        axi_write(12'h0E0,32'h7);       // memory + start + identification
        wait(dma_done); @(posedge clk);
        for(i=0;i<5;i=i+1) begin
            if(memory[192+i] !== (32'h2000_0000+i)) begin
                errors=errors+1;
                $display("ERROR identification memory[%0d]=%h",192+i,memory[192+i]);
            end
        end
        axi_write(12'h0E4,32'h0000_0001); // reject unaligned source
        axi_write(12'h0E8,32'h0000_0300);
        axi_write(12'h0EC,1);
        axi_write(12'h0E0,32'h3);
        wait(dma_done); @(posedge clk);
        if(!dma_error || dma_busy) begin
            errors=errors+1; $display("ERROR unaligned DMA request was not rejected");
        end
        if(errors==0) $display("PASS tb_external_memory_system");
        else $fatal(1,"FAIL tb_external_memory_system errors=%0d",errors);
        #20 $finish;
    end
endmodule
