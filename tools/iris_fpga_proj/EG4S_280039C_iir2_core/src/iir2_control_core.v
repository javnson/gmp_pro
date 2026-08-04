`timescale 1ns / 1ps

// Complete portable AXI subsystem: AXI4-Lite configuration/register mapping
// plus AXI4-Stream samples through a cascade of IIR2 sections.
module iir2_control_core #(
    parameter integer AXI_ADDR_WIDTH = 12,
    parameter integer DATA_WIDTH     = 32,
    parameter integer FRAC_WIDTH     = 28,
    parameter integer NUM_SECTIONS   = 4
) (
    input  wire                         clk,
    input  wire                         rst_n,

    input  wire [AXI_ADDR_WIDTH-1:0]    s_axi_awaddr,
    input  wire                         s_axi_awvalid,
    output wire                         s_axi_awready,
    input  wire [31:0]                  s_axi_wdata,
    input  wire [3:0]                   s_axi_wstrb,
    input  wire                         s_axi_wvalid,
    output wire                         s_axi_wready,
    output wire [1:0]                   s_axi_bresp,
    output wire                         s_axi_bvalid,
    input  wire                         s_axi_bready,
    input  wire [AXI_ADDR_WIDTH-1:0]    s_axi_araddr,
    input  wire                         s_axi_arvalid,
    output wire                         s_axi_arready,
    output wire [31:0]                  s_axi_rdata,
    output wire [1:0]                   s_axi_rresp,
    output wire                         s_axi_rvalid,
    input  wire                         s_axi_rready,

    input  wire signed [DATA_WIDTH-1:0] s_axis_tdata,
    input  wire                         s_axis_tvalid,
    output wire                         s_axis_tready,
    input  wire                         s_axis_tlast,
    output wire signed [DATA_WIDTH-1:0] m_axis_tdata,
    output wire                         m_axis_tvalid,
    input  wire                         m_axis_tready,
    output wire                         m_axis_tlast,

    input  wire [8*16-1:0]              adc_samples_flat,
    input  wire                         pwm_trip_latched,
    input  wire                         dma_read_busy,
    input  wire                         dma_write_busy,
    input  wire                         dma_done,
    input  wire                         dma_error,
    output wire [7:0]                   adc_channel_mask,
    output wire [2:0]                   adc_control_channel,
    output wire                         auto_adc_enable,
    output wire                         dac_auto_enable,
    output wire [15:0]                  dac_manual_ch1,
    output wire [15:0]                  dac_manual_ch2,
    output wire [15:0]                  dac_manual_ch3,
    output wire [15:0]                  dac_manual_ch4,
    output wire                         pwm_enable,
    output wire                         pwm_center_aligned,
    output wire                         pwm_control_enable,
    output wire                         pwm_trip_clear,
    output wire [31:0]                  pwm_period,
    output wire [31:0]                  pwm_compare_shadow,
    output wire [15:0]                  pwm_deadtime,
    output wire [31:0]                  pwm_phase,
    output wire                         pipeline_busy,
    output wire                         memory_mode,
    output wire                         identification_mode,
    output wire                         dma_start,
    output wire [31:0]                  dma_read_base,
    output wire [31:0]                  dma_write_base,
    output wire [31:0]                  dma_sample_count
);
    wire run_enable;
    wire clear_state;
    wire [7:0] active_sections;
    wire [NUM_SECTIONS*DATA_WIDTH-1:0] coeff_b0_flat;
    wire [NUM_SECTIONS*DATA_WIDTH-1:0] coeff_b1_flat;
    wire [NUM_SECTIONS*DATA_WIDTH-1:0] coeff_b2_flat;
    wire [NUM_SECTIONS*DATA_WIDTH-1:0] coeff_a1_flat;
    wire [NUM_SECTIONS*DATA_WIDTH-1:0] coeff_a2_flat;
    wire signed [DATA_WIDTH-1:0] manual_sample;
    wire manual_valid;
    wire manual_ready;
    wire selected_valid;
    wire selected_ready;
    wire signed [DATA_WIDTH-1:0] selected_data;
    wire selected_last;
    wire saturation_event;
    wire result_fire;

    assign selected_valid = manual_valid ? 1'b1 :
                            (run_enable ? s_axis_tvalid : 1'b0);
    assign selected_data  = manual_valid ? manual_sample : s_axis_tdata;
    assign selected_last  = manual_valid ? 1'b1 : s_axis_tlast;
    assign manual_ready   = manual_valid && selected_ready;
    assign s_axis_tready  = run_enable && !manual_valid && selected_ready;
    assign result_fire    = m_axis_tvalid && m_axis_tready;

    iir2_pipeline_container #(
        .DATA_WIDTH(DATA_WIDTH),
        .FRAC_WIDTH(FRAC_WIDTH),
        .NUM_SECTIONS(NUM_SECTIONS)
    ) u_pipeline (
        .clk(clk), .rst_n(rst_n), .clear_state(clear_state),
        .active_sections(active_sections),
        .coeff_b0_flat(coeff_b0_flat), .coeff_b1_flat(coeff_b1_flat),
        .coeff_b2_flat(coeff_b2_flat), .coeff_a1_flat(coeff_a1_flat),
        .coeff_a2_flat(coeff_a2_flat),
        .s_axis_tdata(selected_data), .s_axis_tvalid(selected_valid),
        .s_axis_tready(selected_ready), .s_axis_tlast(selected_last),
        .m_axis_tdata(m_axis_tdata), .m_axis_tvalid(m_axis_tvalid),
        .m_axis_tready(m_axis_tready), .m_axis_tlast(m_axis_tlast),
        .busy(pipeline_busy), .saturated(saturation_event)
    );

    iir2_axi_lite_regs #(
        .ADDR_WIDTH(AXI_ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH),
        .FRAC_WIDTH(FRAC_WIDTH), .NUM_SECTIONS(NUM_SECTIONS)
    ) u_regs (
        .clk(clk), .rst_n(rst_n),
        .s_axi_awaddr(s_axi_awaddr), .s_axi_awvalid(s_axi_awvalid),
        .s_axi_awready(s_axi_awready), .s_axi_wdata(s_axi_wdata),
        .s_axi_wstrb(s_axi_wstrb), .s_axi_wvalid(s_axi_wvalid),
        .s_axi_wready(s_axi_wready), .s_axi_bresp(s_axi_bresp),
        .s_axi_bvalid(s_axi_bvalid), .s_axi_bready(s_axi_bready),
        .s_axi_araddr(s_axi_araddr), .s_axi_arvalid(s_axi_arvalid),
        .s_axi_arready(s_axi_arready), .s_axi_rdata(s_axi_rdata),
        .s_axi_rresp(s_axi_rresp), .s_axi_rvalid(s_axi_rvalid),
        .s_axi_rready(s_axi_rready),
        .pipeline_busy(pipeline_busy), .result_data(m_axis_tdata),
        .result_valid(result_fire), .saturation_event(saturation_event),
        .adc_samples_flat(adc_samples_flat), .pwm_trip_latched(pwm_trip_latched),
        .dma_read_busy(dma_read_busy), .dma_write_busy(dma_write_busy),
        .dma_done(dma_done), .dma_error(dma_error),
        .run_enable(run_enable), .clear_state(clear_state),
        .active_sections(active_sections),
        .coeff_b0_flat(coeff_b0_flat), .coeff_b1_flat(coeff_b1_flat),
        .coeff_b2_flat(coeff_b2_flat), .coeff_a1_flat(coeff_a1_flat),
        .coeff_a2_flat(coeff_a2_flat),
        .manual_sample(manual_sample), .manual_valid(manual_valid),
        .manual_ready(manual_ready),
        .adc_channel_mask(adc_channel_mask),
        .adc_control_channel(adc_control_channel),
        .auto_adc_enable(auto_adc_enable), .dac_auto_enable(dac_auto_enable),
        .dac_manual_ch1(dac_manual_ch1), .dac_manual_ch2(dac_manual_ch2),
        .dac_manual_ch3(dac_manual_ch3), .dac_manual_ch4(dac_manual_ch4),
        .pwm_enable(pwm_enable), .pwm_center_aligned(pwm_center_aligned),
        .pwm_control_enable(pwm_control_enable), .pwm_trip_clear(pwm_trip_clear),
        .pwm_period(pwm_period), .pwm_compare_shadow(pwm_compare_shadow),
        .pwm_deadtime(pwm_deadtime), .pwm_phase(pwm_phase),
        .memory_mode(memory_mode), .identification_mode(identification_mode),
        .dma_start(dma_start),
        .dma_read_base(dma_read_base), .dma_write_base(dma_write_base),
        .dma_sample_count(dma_sample_count)
    );
endmodule
