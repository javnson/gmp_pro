`timescale 1ns / 1ps

// Xilinx/TD-portable batch controller. AXI4-Lite configures the controller and
// DMA; AXI4 accesses external DDR/SDRAM; live AXI4-Stream remains available
// when memory_mode (register 0xE0 bit 0) is clear.
module iir2_external_memory_system #(
    parameter integer ADDR_WIDTH = 32,
    parameter integer ID_WIDTH = 1,
    parameter integer DATA_WIDTH = 32,
    parameter integer FRAC_WIDTH = 28,
    parameter integer NUM_SECTIONS = 4,
    parameter integer MAX_BURST_LEN = 16
) (
    input  wire                         clk,
    input  wire                         rst_n,

    input  wire [11:0]                  s_axi_ctrl_awaddr,
    input  wire                         s_axi_ctrl_awvalid,
    output wire                         s_axi_ctrl_awready,
    input  wire [31:0]                  s_axi_ctrl_wdata,
    input  wire [3:0]                   s_axi_ctrl_wstrb,
    input  wire                         s_axi_ctrl_wvalid,
    output wire                         s_axi_ctrl_wready,
    output wire [1:0]                   s_axi_ctrl_bresp,
    output wire                         s_axi_ctrl_bvalid,
    input  wire                         s_axi_ctrl_bready,
    input  wire [11:0]                  s_axi_ctrl_araddr,
    input  wire                         s_axi_ctrl_arvalid,
    output wire                         s_axi_ctrl_arready,
    output wire [31:0]                  s_axi_ctrl_rdata,
    output wire [1:0]                   s_axi_ctrl_rresp,
    output wire                         s_axi_ctrl_rvalid,
    input  wire                         s_axi_ctrl_rready,

    input  wire signed [DATA_WIDTH-1:0] s_axis_live_tdata,
    input  wire                         s_axis_live_tvalid,
    output wire                         s_axis_live_tready,
    input  wire                         s_axis_live_tlast,
    output wire signed [DATA_WIDTH-1:0] m_axis_live_tdata,
    output wire                         m_axis_live_tvalid,
    input  wire                         m_axis_live_tready,
    output wire                         m_axis_live_tlast,
    output wire signed [DATA_WIDTH-1:0] result_tap_data,
    output wire                         result_tap_valid,
    output wire [DATA_WIDTH-1:0]        m_axis_injection_tdata,
    output wire                         m_axis_injection_tvalid,
    input  wire                         m_axis_injection_tready,
    output wire                         m_axis_injection_tlast,
    input  wire [DATA_WIDTH-1:0]        s_axis_measurement_tdata,
    input  wire                         s_axis_measurement_tvalid,
    output wire                         s_axis_measurement_tready,
    input  wire                         s_axis_measurement_tlast,

    output wire [ID_WIDTH-1:0]          m_axi_mem_arid,
    output wire [ADDR_WIDTH-1:0]        m_axi_mem_araddr,
    output wire [7:0]                   m_axi_mem_arlen,
    output wire [2:0]                   m_axi_mem_arsize,
    output wire [1:0]                   m_axi_mem_arburst,
    output wire                         m_axi_mem_arlock,
    output wire [3:0]                   m_axi_mem_arcache,
    output wire [2:0]                   m_axi_mem_arprot,
    output wire [3:0]                   m_axi_mem_arqos,
    output wire                         m_axi_mem_arvalid,
    input  wire                         m_axi_mem_arready,
    input  wire [ID_WIDTH-1:0]          m_axi_mem_rid,
    input  wire [DATA_WIDTH-1:0]        m_axi_mem_rdata,
    input  wire [1:0]                   m_axi_mem_rresp,
    input  wire                         m_axi_mem_rlast,
    input  wire                         m_axi_mem_rvalid,
    output wire                         m_axi_mem_rready,
    output wire [ID_WIDTH-1:0]          m_axi_mem_awid,
    output wire [ADDR_WIDTH-1:0]        m_axi_mem_awaddr,
    output wire [7:0]                   m_axi_mem_awlen,
    output wire [2:0]                   m_axi_mem_awsize,
    output wire [1:0]                   m_axi_mem_awburst,
    output wire                         m_axi_mem_awlock,
    output wire [3:0]                   m_axi_mem_awcache,
    output wire [2:0]                   m_axi_mem_awprot,
    output wire [3:0]                   m_axi_mem_awqos,
    output wire                         m_axi_mem_awvalid,
    input  wire                         m_axi_mem_awready,
    output wire [DATA_WIDTH-1:0]        m_axi_mem_wdata,
    output wire [DATA_WIDTH/8-1:0]      m_axi_mem_wstrb,
    output wire                         m_axi_mem_wlast,
    output wire                         m_axi_mem_wvalid,
    input  wire                         m_axi_mem_wready,
    input  wire [ID_WIDTH-1:0]          m_axi_mem_bid,
    input  wire [1:0]                   m_axi_mem_bresp,
    input  wire                         m_axi_mem_bvalid,
    output wire                         m_axi_mem_bready,
    output wire                         dma_busy,
    output wire                         dma_done,
    output wire                         dma_error
);
    wire memory_mode, identification_mode, dma_start;
    wire [31:0] dma_read_base, dma_write_base, dma_sample_count;
    wire dma_read_busy, dma_write_busy;
    wire [DATA_WIDTH-1:0] dma_read_data;
    wire dma_read_valid, dma_read_ready, dma_read_last;
    wire [DATA_WIDTH-1:0] dma_write_data;
    wire dma_write_valid, dma_write_ready, dma_write_last;
    wire signed [DATA_WIDTH-1:0] core_input_data;
    wire core_input_valid, core_input_ready, core_input_last;
    wire signed [DATA_WIDTH-1:0] core_output_data;
    wire core_output_valid, core_output_ready, core_output_last;
    wire pipeline_busy;

    assign core_input_data  = memory_mode ? dma_read_data : s_axis_live_tdata;
    assign core_input_valid = memory_mode ?
                              (identification_mode ? 1'b0 : dma_read_valid) :
                              s_axis_live_tvalid;
    assign core_input_last  = memory_mode ? dma_read_last : s_axis_live_tlast;
    assign dma_read_ready   = memory_mode &&
                              (identification_mode ? m_axis_injection_tready :
                                                     core_input_ready);
    assign s_axis_live_tready = !memory_mode && core_input_ready;
    assign m_axis_injection_tdata  = dma_read_data;
    assign m_axis_injection_tvalid = memory_mode && identification_mode && dma_read_valid;
    assign m_axis_injection_tlast  = dma_read_last;

    assign core_output_ready = memory_mode ?
                               (identification_mode ? 1'b0 : dma_write_ready) :
                               m_axis_live_tready;
    assign dma_write_data    = identification_mode ? s_axis_measurement_tdata :
                                                     core_output_data;
    assign dma_write_valid   = memory_mode &&
                               (identification_mode ? s_axis_measurement_tvalid :
                                                      core_output_valid);
    assign dma_write_last    = identification_mode ? s_axis_measurement_tlast :
                                                     core_output_last;
    assign s_axis_measurement_tready = memory_mode && identification_mode &&
                                       dma_write_ready;
    assign m_axis_live_tdata  = core_output_data;
    assign m_axis_live_tvalid = !memory_mode && core_output_valid;
    assign m_axis_live_tlast  = core_output_last;
    assign result_tap_data    = core_output_data;
    assign result_tap_valid   = core_output_valid && core_output_ready;
    assign dma_busy           = dma_read_busy || dma_write_busy;

    axi4_stream_dma #(
        .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH), .ID_WIDTH(ID_WIDTH),
        .MAX_BURST_LEN(MAX_BURST_LEN)
    ) u_dma (
        .clk(clk), .rst_n(rst_n), .start(dma_start && memory_mode),
        .read_base_addr(dma_read_base), .write_base_addr(dma_write_base),
        .sample_count(dma_sample_count), .read_busy(dma_read_busy),
        .write_busy(dma_write_busy), .done(dma_done), .error(dma_error),
        .m_axis_tdata(dma_read_data), .m_axis_tvalid(dma_read_valid),
        .m_axis_tready(dma_read_ready), .m_axis_tlast(dma_read_last),
        .s_axis_tdata(dma_write_data), .s_axis_tvalid(dma_write_valid),
        .s_axis_tready(dma_write_ready), .s_axis_tlast(dma_write_last),
        .m_axi_arid(m_axi_mem_arid), .m_axi_araddr(m_axi_mem_araddr),
        .m_axi_arlen(m_axi_mem_arlen), .m_axi_arsize(m_axi_mem_arsize),
        .m_axi_arburst(m_axi_mem_arburst), .m_axi_arlock(m_axi_mem_arlock),
        .m_axi_arcache(m_axi_mem_arcache), .m_axi_arprot(m_axi_mem_arprot),
        .m_axi_arqos(m_axi_mem_arqos), .m_axi_arvalid(m_axi_mem_arvalid),
        .m_axi_arready(m_axi_mem_arready), .m_axi_rid(m_axi_mem_rid),
        .m_axi_rdata(m_axi_mem_rdata), .m_axi_rresp(m_axi_mem_rresp),
        .m_axi_rlast(m_axi_mem_rlast), .m_axi_rvalid(m_axi_mem_rvalid),
        .m_axi_rready(m_axi_mem_rready), .m_axi_awid(m_axi_mem_awid),
        .m_axi_awaddr(m_axi_mem_awaddr), .m_axi_awlen(m_axi_mem_awlen),
        .m_axi_awsize(m_axi_mem_awsize), .m_axi_awburst(m_axi_mem_awburst),
        .m_axi_awlock(m_axi_mem_awlock), .m_axi_awcache(m_axi_mem_awcache),
        .m_axi_awprot(m_axi_mem_awprot), .m_axi_awqos(m_axi_mem_awqos),
        .m_axi_awvalid(m_axi_mem_awvalid), .m_axi_awready(m_axi_mem_awready),
        .m_axi_wdata(m_axi_mem_wdata), .m_axi_wstrb(m_axi_mem_wstrb),
        .m_axi_wlast(m_axi_mem_wlast), .m_axi_wvalid(m_axi_mem_wvalid),
        .m_axi_wready(m_axi_mem_wready), .m_axi_bid(m_axi_mem_bid),
        .m_axi_bresp(m_axi_mem_bresp), .m_axi_bvalid(m_axi_mem_bvalid),
        .m_axi_bready(m_axi_mem_bready)
    );

    // Board-related register outputs are intentionally left at this wrapper
    // boundary; the TD top uses iir2_control_core directly for ADC/DAC/PWM.
    iir2_control_core #(
        .AXI_ADDR_WIDTH(12), .DATA_WIDTH(DATA_WIDTH),
        .FRAC_WIDTH(FRAC_WIDTH), .NUM_SECTIONS(NUM_SECTIONS)
    ) u_core (
        .clk(clk), .rst_n(rst_n),
        .s_axi_awaddr(s_axi_ctrl_awaddr), .s_axi_awvalid(s_axi_ctrl_awvalid),
        .s_axi_awready(s_axi_ctrl_awready), .s_axi_wdata(s_axi_ctrl_wdata),
        .s_axi_wstrb(s_axi_ctrl_wstrb), .s_axi_wvalid(s_axi_ctrl_wvalid),
        .s_axi_wready(s_axi_ctrl_wready), .s_axi_bresp(s_axi_ctrl_bresp),
        .s_axi_bvalid(s_axi_ctrl_bvalid), .s_axi_bready(s_axi_ctrl_bready),
        .s_axi_araddr(s_axi_ctrl_araddr), .s_axi_arvalid(s_axi_ctrl_arvalid),
        .s_axi_arready(s_axi_ctrl_arready), .s_axi_rdata(s_axi_ctrl_rdata),
        .s_axi_rresp(s_axi_ctrl_rresp), .s_axi_rvalid(s_axi_ctrl_rvalid),
        .s_axi_rready(s_axi_ctrl_rready),
        .s_axis_tdata(core_input_data), .s_axis_tvalid(core_input_valid),
        .s_axis_tready(core_input_ready), .s_axis_tlast(core_input_last),
        .m_axis_tdata(core_output_data), .m_axis_tvalid(core_output_valid),
        .m_axis_tready(core_output_ready), .m_axis_tlast(core_output_last),
        .adc_samples_flat(128'd0), .pwm_trip_latched(1'b0),
        .dma_read_busy(dma_read_busy), .dma_write_busy(dma_write_busy),
        .dma_done(dma_done), .dma_error(dma_error),
        .adc_channel_mask(), .adc_control_channel(), .auto_adc_enable(),
        .dac_auto_enable(), .dac_manual_ch1(), .dac_manual_ch2(),
        .dac_manual_ch3(), .dac_manual_ch4(), .pwm_enable(),
        .pwm_center_aligned(), .pwm_control_enable(), .pwm_trip_clear(),
        .pwm_period(), .pwm_compare_shadow(), .pwm_deadtime(), .pwm_phase(),
        .pipeline_busy(pipeline_busy), .memory_mode(memory_mode),
        .identification_mode(identification_mode),
        .dma_start(dma_start), .dma_read_base(dma_read_base),
        .dma_write_base(dma_write_base), .dma_sample_count(dma_sample_count)
    );
endmodule
