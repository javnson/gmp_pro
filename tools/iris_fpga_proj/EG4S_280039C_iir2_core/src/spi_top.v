`timescale 1ns / 1ps

// Tang Dynasty EG4S20 board integration.  All control RTL below the PLL and
// physical ADC/DAC drivers is vendor-neutral and can be instantiated unchanged
// in an AMD Xilinx block design.
module td_iir2_control_top #(
    parameter integer CONTROL_PERIOD_CLKS = 10000
) (
    input  wire       clk_in,
    input  wire       rst_n,
    input  wire [3:0] encoder,
    output wire [3:0] led,
    input  wire       spi_sclk,
    input  wire       spi_csn,
    input  wire       spi_mosi,
    output wire       spi_miso,
    output wire [1:0] dac_din,
    output wire [1:0] dac_sclk,
    output wire       dac_sync,
    output wire       dac_clr,
    output wire [1:0] dac_ldac,
    output wire       adc_sdi,
    input  wire       adc_sdo,
    output wire       adc_sclk,
    output wire       adc_cs,
    output wire       adc_rst,
    output wire [3:0] gpio
);
    localparam integer DATA_WIDTH = 32;
    localparam integer FRAC_WIDTH = 28;
    localparam integer NUM_SECTIONS = 4;

    wire clk;
    clock_manager u_pll_main (
        .refclk(clk_in), .reset(~rst_n), .clk0_out(clk)
    );

    // AXI4-Lite wires from the board SPI bridge to the portable core.
    wire [11:0] axi_awaddr;
    wire axi_awvalid, axi_awready;
    wire [31:0] axi_wdata;
    wire [3:0] axi_wstrb;
    wire axi_wvalid, axi_wready;
    wire [1:0] axi_bresp;
    wire axi_bvalid;
    wire [11:0] axi_araddr;
    wire axi_arvalid, axi_arready;
    wire [31:0] axi_rdata;
    wire [1:0] axi_rresp;
    wire axi_rvalid;
    wire spi_write_complete;
    wire bridge_error;

    spi_axi_lite_bridge #(.AXI_ADDR_WIDTH(12)) u_spi_axi_bridge (
        .clk(clk), .rst_n(rst_n),
        .spi_cs_n(spi_csn), .spi_sclk(spi_sclk),
        .spi_mosi(spi_mosi), .spi_miso(spi_miso),
        .m_axi_awaddr(axi_awaddr), .m_axi_awvalid(axi_awvalid),
        .m_axi_awready(axi_awready), .m_axi_wdata(axi_wdata),
        .m_axi_wstrb(axi_wstrb), .m_axi_wvalid(axi_wvalid),
        .m_axi_wready(axi_wready), .m_axi_bresp(axi_bresp),
        .m_axi_bvalid(axi_bvalid), .m_axi_bready(),
        .m_axi_araddr(axi_araddr), .m_axi_arvalid(axi_arvalid),
        .m_axi_arready(axi_arready), .m_axi_rdata(axi_rdata),
        .m_axi_rresp(axi_rresp), .m_axi_rvalid(axi_rvalid),
        .m_axi_rready(), .write_complete_pulse(spi_write_complete),
        .bridge_error(bridge_error)
    );

    wire [15:0] adc_ch0, adc_ch1, adc_ch2, adc_ch3;
    wire [15:0] adc_ch4, adc_ch5, adc_ch6, adc_ch7;
    wire [127:0] adc_samples_flat = {adc_ch7, adc_ch6, adc_ch5, adc_ch4,
                                     adc_ch3, adc_ch2, adc_ch1, adc_ch0};
    wire adc_update;
    wire [7:0] adc_channel_mask;
    wire [2:0] adc_control_channel;
    wire auto_adc_enable;
    reg [15:0] selected_adc;

    always @* begin
        case (adc_control_channel)
            3'd0: selected_adc = adc_ch0;
            3'd1: selected_adc = adc_ch1;
            3'd2: selected_adc = adc_ch2;
            3'd3: selected_adc = adc_ch3;
            3'd4: selected_adc = adc_ch4;
            3'd5: selected_adc = adc_ch5;
            3'd6: selected_adc = adc_ch6;
            default: selected_adc = adc_ch7;
        endcase
    end

    reg [31:0] control_tick_counter;
    reg control_tick;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            control_tick_counter <= 32'd0;
            control_tick <= 1'b0;
        end else begin
            control_tick <= 1'b0;
            if (control_tick_counter >= CONTROL_PERIOD_CLKS-1) begin
                control_tick_counter <= 32'd0;
                control_tick <= 1'b1;
            end else begin
                control_tick_counter <= control_tick_counter + 1'b1;
            end
        end
    end

    ads8688_masked_ctrl u_ads8688 (
        .clk(clk), .rst_n(rst_n),
        .trigger(auto_adc_enable ? control_tick : spi_write_complete),
        .channel_mask(adc_channel_mask),
        .adc_ch0(adc_ch0), .adc_ch1(adc_ch1), .adc_ch2(adc_ch2),
        .adc_ch3(adc_ch3), .adc_ch4(adc_ch4), .adc_ch5(adc_ch5),
        .adc_ch6(adc_ch6), .adc_ch7(adc_ch7),
        .update_pulse(adc_update), .spi_cs_n(adc_cs),
        .spi_sclk(adc_sclk), .spi_mosi(adc_sdi),
        .spi_miso(adc_sdo), .spi_rst_n(adc_rst)
    );

    wire signed [DATA_WIDTH-1:0] adc_axis_data;
    wire adc_axis_valid, adc_axis_ready;
    wire adc_overflow;
    adc_axis_adapter #(
        .DATA_WIDTH(DATA_WIDTH), .FRAC_WIDTH(FRAC_WIDTH)
    ) u_adc_axis_adapter (
        .clk(clk), .rst_n(rst_n), .adc_sample(selected_adc),
        .sample_valid(adc_update && auto_adc_enable),
        .m_axis_tdata(adc_axis_data), .m_axis_tvalid(adc_axis_valid),
        .m_axis_tready(adc_axis_ready), .overflow(adc_overflow)
    );

    wire signed [DATA_WIDTH-1:0] control_result;
    wire control_result_valid;
    wire pipeline_busy;
    wire dac_auto_enable;
    wire [15:0] dac_manual_ch1, dac_manual_ch2, dac_manual_ch3, dac_manual_ch4;
    wire pwm_enable, pwm_center_aligned, pwm_control_enable, pwm_trip_clear;
    wire [31:0] pwm_period, pwm_compare_shadow, pwm_phase;
    wire [15:0] pwm_deadtime;
    wire pwm_trip_latched;

    iir2_control_core #(
        .AXI_ADDR_WIDTH(12), .DATA_WIDTH(DATA_WIDTH),
        .FRAC_WIDTH(FRAC_WIDTH), .NUM_SECTIONS(NUM_SECTIONS)
    ) u_control_core (
        .clk(clk), .rst_n(rst_n),
        .s_axi_awaddr(axi_awaddr), .s_axi_awvalid(axi_awvalid),
        .s_axi_awready(axi_awready), .s_axi_wdata(axi_wdata),
        .s_axi_wstrb(axi_wstrb), .s_axi_wvalid(axi_wvalid),
        .s_axi_wready(axi_wready), .s_axi_bresp(axi_bresp),
        .s_axi_bvalid(axi_bvalid), .s_axi_bready(1'b1),
        .s_axi_araddr(axi_araddr), .s_axi_arvalid(axi_arvalid),
        .s_axi_arready(axi_arready), .s_axi_rdata(axi_rdata),
        .s_axi_rresp(axi_rresp), .s_axi_rvalid(axi_rvalid),
        .s_axi_rready(1'b1),
        .s_axis_tdata(adc_axis_data), .s_axis_tvalid(adc_axis_valid),
        .s_axis_tready(adc_axis_ready), .s_axis_tlast(1'b1),
        .m_axis_tdata(control_result), .m_axis_tvalid(control_result_valid),
        .m_axis_tready(1'b1), .m_axis_tlast(),
        .adc_samples_flat(adc_samples_flat),
        .pwm_trip_latched(pwm_trip_latched),
        .dma_read_busy(1'b0), .dma_write_busy(1'b0),
        .dma_done(1'b0), .dma_error(1'b0),
        .adc_channel_mask(adc_channel_mask),
        .adc_control_channel(adc_control_channel),
        .auto_adc_enable(auto_adc_enable), .dac_auto_enable(dac_auto_enable),
        .dac_manual_ch1(dac_manual_ch1), .dac_manual_ch2(dac_manual_ch2),
        .dac_manual_ch3(dac_manual_ch3), .dac_manual_ch4(dac_manual_ch4),
        .pwm_enable(pwm_enable), .pwm_center_aligned(pwm_center_aligned),
        .pwm_control_enable(pwm_control_enable), .pwm_trip_clear(pwm_trip_clear),
        .pwm_period(pwm_period), .pwm_compare_shadow(pwm_compare_shadow),
        .pwm_deadtime(pwm_deadtime), .pwm_phase(pwm_phase),
        .pipeline_busy(pipeline_busy),
        .memory_mode(), .identification_mode(), .dma_start(), .dma_read_base(),
        .dma_write_base(), .dma_sample_count()
    );

    wire [15:0] automatic_dac_code;
    fixed_to_dac #(
        .DATA_WIDTH(DATA_WIDTH), .FRAC_WIDTH(FRAC_WIDTH)
    ) u_fixed_to_dac (
        .fixed_sample(control_result), .dac_code(automatic_dac_code)
    );

    dac8563_quad_ctrl u_dac8563 (
        .clk(clk), .rst_n(rst_n),
        .flush(dac_auto_enable ? control_result_valid : spi_write_complete),
        .dac_ch1(dac_auto_enable ? automatic_dac_code : dac_manual_ch1),
        .dac_ch2(dac_manual_ch2), .dac_ch3(dac_manual_ch3),
        .dac_ch4(dac_manual_ch4), .dac_sync_n(dac_sync),
        .dac_sclk_1(dac_sclk[0]), .dac_sclk_2(dac_sclk[1]),
        .dac_din_1(dac_din[0]), .dac_din_2(dac_din[1]),
        .dac_clr_n(dac_clr), .dac_ldac_1_n(dac_ldac[0]),
        .dac_ldac_2_n(dac_ldac[1])
    );

    wire pwm_a, pwm_b, pwm_period_tick;
    epwm_modulator #(
        .DATA_WIDTH(DATA_WIDTH), .FRAC_WIDTH(FRAC_WIDTH)
    ) u_epwm (
        .clk(clk), .rst_n(rst_n), .enable(pwm_enable),
        .center_aligned(pwm_center_aligned), .period(pwm_period),
        .compare_shadow(pwm_compare_shadow), .deadtime_cycles(pwm_deadtime),
        .phase(pwm_phase), .sync_in(1'b0), .trip_in(encoder[3]),
        .trip_clear(pwm_trip_clear), .control_enable(pwm_control_enable),
        .control_data(control_result), .control_valid(control_result_valid),
        .pwm_a(pwm_a), .pwm_b(pwm_b), .period_tick(pwm_period_tick),
        .trip_latched(pwm_trip_latched), .counter_value(),
        .compare_active_value()
    );

    assign gpio = {pipeline_busy, pwm_period_tick, pwm_b, pwm_a};
    assign led  = {pwm_trip_latched, (bridge_error | adc_overflow),
                   pipeline_busy, auto_adc_enable};
endmodule
