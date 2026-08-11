`timescale 1ns / 1ps

// AXI4-Lite register bank for the IIR2 controller and board-level controls.
// One outstanding read and one outstanding write are supported. AW and W may
// arrive in either order, as required by AXI4-Lite.
module iir2_axi_lite_regs #(
    parameter integer ADDR_WIDTH   = 12,
    parameter integer DATA_WIDTH   = 32,
    parameter integer FRAC_WIDTH   = 28,
    parameter integer NUM_SECTIONS = 4
) (
    input  wire                               clk,
    input  wire                               rst_n,

    input  wire [ADDR_WIDTH-1:0]              s_axi_awaddr,
    input  wire                               s_axi_awvalid,
    output wire                               s_axi_awready,
    input  wire [31:0]                        s_axi_wdata,
    input  wire [3:0]                         s_axi_wstrb,
    input  wire                               s_axi_wvalid,
    output wire                               s_axi_wready,
    output reg  [1:0]                         s_axi_bresp,
    output reg                                s_axi_bvalid,
    input  wire                               s_axi_bready,
    input  wire [ADDR_WIDTH-1:0]              s_axi_araddr,
    input  wire                               s_axi_arvalid,
    output wire                               s_axi_arready,
    output reg  [31:0]                        s_axi_rdata,
    output reg  [1:0]                         s_axi_rresp,
    output reg                                s_axi_rvalid,
    input  wire                               s_axi_rready,

    input  wire                               pipeline_busy,
    input  wire signed [DATA_WIDTH-1:0]       result_data,
    input  wire                               result_valid,
    input  wire                               saturation_event,
    input  wire [8*16-1:0]                    adc_samples_flat,
    input  wire                               pwm_trip_latched,
    input  wire                               dma_read_busy,
    input  wire                               dma_write_busy,
    input  wire                               dma_done,
    input  wire                               dma_error,

    output reg                                run_enable,
    output reg                                clear_state,
    output reg  [7:0]                         active_sections,
    output reg  [NUM_SECTIONS*DATA_WIDTH-1:0] coeff_b0_flat,
    output reg  [NUM_SECTIONS*DATA_WIDTH-1:0] coeff_b1_flat,
    output reg  [NUM_SECTIONS*DATA_WIDTH-1:0] coeff_b2_flat,
    output reg  [NUM_SECTIONS*DATA_WIDTH-1:0] coeff_a1_flat,
    output reg  [NUM_SECTIONS*DATA_WIDTH-1:0] coeff_a2_flat,

    output reg signed [DATA_WIDTH-1:0]        manual_sample,
    output reg                                manual_valid,
    input  wire                               manual_ready,

    output reg  [7:0]                         adc_channel_mask,
    output reg  [2:0]                         adc_control_channel,
    output reg                                auto_adc_enable,
    output reg                                dac_auto_enable,
    output reg  [15:0]                        dac_manual_ch1,
    output reg  [15:0]                        dac_manual_ch2,
    output reg  [15:0]                        dac_manual_ch3,
    output reg  [15:0]                        dac_manual_ch4,

    output reg                                pwm_enable,
    output reg                                pwm_center_aligned,
    output reg                                pwm_control_enable,
    output reg                                pwm_trip_clear,
    output reg  [31:0]                        pwm_period,
    output reg  [31:0]                        pwm_compare_shadow,
    output reg  [15:0]                        pwm_deadtime,
    output reg  [31:0]                        pwm_phase,
    output reg                                memory_mode,
    output reg                                identification_mode,
    output reg                                dma_start,
    output reg  [31:0]                        dma_read_base,
    output reg  [31:0]                        dma_write_base,
    output reg  [31:0]                        dma_sample_count
);
    reg [ADDR_WIDTH-1:0] awaddr_hold;
    reg [31:0]           wdata_hold;
    reg [3:0]            wstrb_hold;
    reg                  aw_pending;
    reg                  w_pending;

    reg [NUM_SECTIONS*DATA_WIDTH-1:0] shadow_b0_flat;
    reg [NUM_SECTIONS*DATA_WIDTH-1:0] shadow_b1_flat;
    reg [NUM_SECTIONS*DATA_WIDTH-1:0] shadow_b2_flat;
    reg [NUM_SECTIONS*DATA_WIDTH-1:0] shadow_a1_flat;
    reg [NUM_SECTIONS*DATA_WIDTH-1:0] shadow_a2_flat;
    reg [7:0]                            shadow_active_sections;

    reg [DATA_WIDTH-1:0] latest_result;
    reg [31:0] sample_count;
    reg [31:0] saturation_count;
    reg        result_seen;
    reg        config_error;
    reg        dma_done_seen;
    reg [31:0] read_mux;
    reg [15:0] selected_adc;
    integer section_index;
    localparam [7:0] DATA_WIDTH_U8 = DATA_WIDTH;
    localparam [7:0] FRAC_WIDTH_U8 = FRAC_WIDTH;

    function [31:0] merge_wstrb;
        input [31:0] old_value;
        input [31:0] new_value;
        input [3:0]  strobes;
        integer byte_index;
        begin
            merge_wstrb = old_value;
            for (byte_index = 0; byte_index < 4; byte_index = byte_index + 1)
                if (strobes[byte_index])
                    merge_wstrb[byte_index*8 +: 8] = new_value[byte_index*8 +: 8];
        end
    endfunction

    assign s_axi_awready = !aw_pending && !s_axi_bvalid;
    assign s_axi_wready  = !w_pending  && !s_axi_bvalid;
    assign s_axi_arready = !s_axi_rvalid;

    always @* begin
        case (adc_control_channel)
            3'd0: selected_adc = adc_samples_flat[15:0];
            3'd1: selected_adc = adc_samples_flat[31:16];
            3'd2: selected_adc = adc_samples_flat[47:32];
            3'd3: selected_adc = adc_samples_flat[63:48];
            3'd4: selected_adc = adc_samples_flat[79:64];
            3'd5: selected_adc = adc_samples_flat[95:80];
            3'd6: selected_adc = adc_samples_flat[111:96];
            default: selected_adc = adc_samples_flat[127:112];
        endcase
    end

    always @* begin
        read_mux = 32'h0000_0000;
        case (s_axi_araddr[11:0])
            12'h000: read_mux = 32'h4932_5232; // "I2R2"
            12'h004: read_mux = 32'h0001_0000;
            12'h008: read_mux = {28'd0, dac_auto_enable,
                                 auto_adc_enable, 1'b0, run_enable};
            12'h00C: read_mux = {28'd0, result_seen, config_error,
                                 manual_valid, pipeline_busy};
            12'h010: read_mux = manual_sample;
            12'h014: read_mux = latest_result;
            12'h018: read_mux = sample_count;
            12'h01C: read_mux = saturation_count;
            12'h020: read_mux = {24'd0, shadow_active_sections};
            12'h024: read_mux = 32'd0;
            12'h028: read_mux = {DATA_WIDTH_U8, FRAC_WIDTH_U8, 16'd0};
            12'h02C: read_mux = NUM_SECTIONS;
            12'h030: read_mux = {13'd0, dac_auto_enable, auto_adc_enable,
                                 adc_control_channel, 8'd0, adc_channel_mask};
            12'h034: read_mux = {16'd0, selected_adc};
            12'h038: read_mux = {31'd0, pwm_trip_latched};
            12'h0A0: read_mux = {28'd0, pwm_trip_latched,
                                 pwm_control_enable, pwm_center_aligned, pwm_enable};
            12'h0A4: read_mux = pwm_period;
            12'h0A8: read_mux = pwm_compare_shadow;
            12'h0AC: read_mux = {16'd0, pwm_deadtime};
            12'h0B0: read_mux = pwm_phase;
            12'h0B4: read_mux = {dac_manual_ch2, dac_manual_ch1};
            12'h0B8: read_mux = {dac_manual_ch4, dac_manual_ch3};
            12'h0C0: read_mux = {16'd0, adc_samples_flat[15:0]};
            12'h0C4: read_mux = {16'd0, adc_samples_flat[31:16]};
            12'h0C8: read_mux = {16'd0, adc_samples_flat[47:32]};
            12'h0CC: read_mux = {16'd0, adc_samples_flat[63:48]};
            12'h0D0: read_mux = {16'd0, adc_samples_flat[79:64]};
            12'h0D4: read_mux = {16'd0, adc_samples_flat[95:80]};
            12'h0D8: read_mux = {16'd0, adc_samples_flat[111:96]};
            12'h0DC: read_mux = {16'd0, adc_samples_flat[127:112]};
            12'h0E0: read_mux = {29'd0, identification_mode, 1'b0, memory_mode};
            12'h0E4: read_mux = dma_read_base;
            12'h0E8: read_mux = dma_write_base;
            12'h0EC: read_mux = dma_sample_count;
            12'h0F0: read_mux = {28'd0, dma_error, dma_done_seen,
                                 dma_write_busy, dma_read_busy};
            default: begin
                read_mux = 32'h0000_0000;
                for (section_index = 0; section_index < NUM_SECTIONS;
                     section_index = section_index + 1) begin
                    if (s_axi_araddr[11:0] == (12'h040 + section_index*20 + 0))
                        read_mux = shadow_b0_flat[section_index*DATA_WIDTH +: DATA_WIDTH];
                    if (s_axi_araddr[11:0] == (12'h040 + section_index*20 + 4))
                        read_mux = shadow_b1_flat[section_index*DATA_WIDTH +: DATA_WIDTH];
                    if (s_axi_araddr[11:0] == (12'h040 + section_index*20 + 8))
                        read_mux = shadow_b2_flat[section_index*DATA_WIDTH +: DATA_WIDTH];
                    if (s_axi_araddr[11:0] == (12'h040 + section_index*20 + 12))
                        read_mux = shadow_a1_flat[section_index*DATA_WIDTH +: DATA_WIDTH];
                    if (s_axi_araddr[11:0] == (12'h040 + section_index*20 + 16))
                        read_mux = shadow_a2_flat[section_index*DATA_WIDTH +: DATA_WIDTH];
                end
            end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            aw_pending <= 1'b0;
            w_pending  <= 1'b0;
            s_axi_bvalid <= 1'b0;
            s_axi_bresp  <= 2'b00;
            s_axi_rvalid <= 1'b0;
            s_axi_rresp  <= 2'b00;
            s_axi_rdata  <= 32'd0;
        end else begin
            if (s_axi_awvalid && s_axi_awready) begin
                awaddr_hold <= s_axi_awaddr;
                aw_pending  <= 1'b1;
            end
            if (s_axi_wvalid && s_axi_wready) begin
                wdata_hold <= s_axi_wdata;
                wstrb_hold <= s_axi_wstrb;
                w_pending  <= 1'b1;
            end
            if (aw_pending && w_pending && !s_axi_bvalid) begin
                aw_pending   <= 1'b0;
                w_pending    <= 1'b0;
                s_axi_bvalid <= 1'b1;
                s_axi_bresp  <= 2'b00;
            end
            if (s_axi_bvalid && s_axi_bready)
                s_axi_bvalid <= 1'b0;

            if (s_axi_arvalid && s_axi_arready) begin
                s_axi_rdata  <= read_mux;
                s_axi_rresp  <= 2'b00;
                s_axi_rvalid <= 1'b1;
            end else if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            run_enable            <= 1'b0;
            clear_state           <= 1'b0;
            manual_sample         <= {DATA_WIDTH{1'b0}};
            manual_valid          <= 1'b0;
            active_sections       <= 8'd1;
            shadow_active_sections<= 8'd1;
            coeff_b0_flat         <= {NUM_SECTIONS*DATA_WIDTH{1'b0}};
            coeff_b1_flat         <= {NUM_SECTIONS*DATA_WIDTH{1'b0}};
            coeff_b2_flat         <= {NUM_SECTIONS*DATA_WIDTH{1'b0}};
            coeff_a1_flat         <= {NUM_SECTIONS*DATA_WIDTH{1'b0}};
            coeff_a2_flat         <= {NUM_SECTIONS*DATA_WIDTH{1'b0}};
            shadow_b0_flat        <= {NUM_SECTIONS*DATA_WIDTH{1'b0}};
            shadow_b1_flat        <= {NUM_SECTIONS*DATA_WIDTH{1'b0}};
            shadow_b2_flat        <= {NUM_SECTIONS*DATA_WIDTH{1'b0}};
            shadow_a1_flat        <= {NUM_SECTIONS*DATA_WIDTH{1'b0}};
            shadow_a2_flat        <= {NUM_SECTIONS*DATA_WIDTH{1'b0}};
            adc_channel_mask      <= 8'h01;
            adc_control_channel   <= 3'd0;
            auto_adc_enable       <= 1'b0;
            dac_auto_enable       <= 1'b0;
            dac_manual_ch1        <= 16'h8000;
            dac_manual_ch2        <= 16'h8000;
            dac_manual_ch3        <= 16'h8000;
            dac_manual_ch4        <= 16'h8000;
            pwm_enable            <= 1'b0;
            pwm_center_aligned    <= 1'b1;
            pwm_control_enable    <= 1'b0;
            pwm_trip_clear        <= 1'b0;
            pwm_period            <= 32'd5000;
            pwm_compare_shadow    <= 32'd2500;
            pwm_deadtime          <= 16'd20;
            pwm_phase             <= 32'd0;
            memory_mode           <= 1'b0;
            identification_mode   <= 1'b0;
            dma_start             <= 1'b0;
            dma_read_base         <= 32'd0;
            dma_write_base        <= 32'd0;
            dma_sample_count      <= 32'd0;
            latest_result         <= {DATA_WIDTH{1'b0}};
            sample_count          <= 32'd0;
            saturation_count      <= 32'd0;
            result_seen           <= 1'b0;
            config_error          <= 1'b0;
            dma_done_seen         <= 1'b0;
            for (section_index = 0; section_index < NUM_SECTIONS;
                 section_index = section_index + 1) begin
                coeff_b0_flat[section_index*DATA_WIDTH +: DATA_WIDTH]
                    <= ({{(DATA_WIDTH-1){1'b0}},1'b1} << FRAC_WIDTH);
                shadow_b0_flat[section_index*DATA_WIDTH +: DATA_WIDTH]
                    <= ({{(DATA_WIDTH-1){1'b0}},1'b1} << FRAC_WIDTH);
            end
        end else begin
            clear_state    <= 1'b0;
            pwm_trip_clear <= 1'b0;
            dma_start      <= 1'b0;

            if (manual_valid && manual_ready)
                manual_valid <= 1'b0;

            if (result_valid) begin
                latest_result <= result_data;
                sample_count  <= sample_count + 1'b1;
                result_seen   <= 1'b1;
            end
            if (saturation_event)
                saturation_count <= saturation_count + 1'b1;
            if (dma_done)
                dma_done_seen <= 1'b1;

            if (aw_pending && w_pending && !s_axi_bvalid) begin
                case (awaddr_hold[11:0])
                    12'h008: begin
                        run_enable      <= wdata_hold[0];
                        auto_adc_enable <= wdata_hold[2];
                        dac_auto_enable <= wdata_hold[3];
                        if (wdata_hold[1]) begin
                            if (!pipeline_busy && !dma_read_busy && !dma_write_busy)
                                clear_state <= 1'b1;
                            else
                                config_error <= 1'b1;
                        end
                        if (wdata_hold[8]) begin
                            config_error     <= 1'b0;
                            result_seen      <= 1'b0;
                            sample_count     <= 32'd0;
                            saturation_count <= 32'd0;
                        end
                    end
                    12'h010: begin
                        if (!manual_valid) begin
                            manual_sample <= merge_wstrb(manual_sample,
                                                         wdata_hold, wstrb_hold);
                            manual_valid  <= 1'b1;
                        end else begin
                            config_error <= 1'b1;
                        end
                    end
                    12'h020: begin
                        if (wdata_hold[7:0] <= NUM_SECTIONS)
                            shadow_active_sections <= wdata_hold[7:0];
                        else
                            config_error <= 1'b1;
                    end
                    12'h024: begin
                        if (wdata_hold[0]) begin
                            if (!pipeline_busy && !dma_read_busy && !dma_write_busy) begin
                                coeff_b0_flat   <= shadow_b0_flat;
                                coeff_b1_flat   <= shadow_b1_flat;
                                coeff_b2_flat   <= shadow_b2_flat;
                                coeff_a1_flat   <= shadow_a1_flat;
                                coeff_a2_flat   <= shadow_a2_flat;
                                active_sections <= shadow_active_sections;
                                clear_state     <= 1'b1;
                            end else begin
                                config_error <= 1'b1;
                            end
                        end
                    end
                    12'h030: begin
                        adc_channel_mask    <= wdata_hold[7:0];
                        adc_control_channel <= wdata_hold[18:16];
                        auto_adc_enable     <= wdata_hold[19];
                        dac_auto_enable     <= wdata_hold[20];
                    end
                    12'h0A0: begin
                        pwm_enable         <= wdata_hold[0];
                        pwm_center_aligned <= wdata_hold[1];
                        pwm_control_enable <= wdata_hold[2];
                        if (wdata_hold[8])
                            pwm_trip_clear <= 1'b1;
                    end
                    12'h0A4: pwm_period <= merge_wstrb(pwm_period, wdata_hold, wstrb_hold);
                    12'h0A8: pwm_compare_shadow <= merge_wstrb(pwm_compare_shadow, wdata_hold, wstrb_hold);
                    12'h0AC: pwm_deadtime <= merge_wstrb({16'd0,pwm_deadtime}, wdata_hold, wstrb_hold);
                    12'h0B0: pwm_phase <= merge_wstrb(pwm_phase, wdata_hold, wstrb_hold);
                    12'h0B4: begin
                        if (wstrb_hold[0]) dac_manual_ch1[7:0]  <= wdata_hold[7:0];
                        if (wstrb_hold[1]) dac_manual_ch1[15:8] <= wdata_hold[15:8];
                        if (wstrb_hold[2]) dac_manual_ch2[7:0]  <= wdata_hold[23:16];
                        if (wstrb_hold[3]) dac_manual_ch2[15:8] <= wdata_hold[31:24];
                    end
                    12'h0B8: begin
                        if (wstrb_hold[0]) dac_manual_ch3[7:0]  <= wdata_hold[7:0];
                        if (wstrb_hold[1]) dac_manual_ch3[15:8] <= wdata_hold[15:8];
                        if (wstrb_hold[2]) dac_manual_ch4[7:0]  <= wdata_hold[23:16];
                        if (wstrb_hold[3]) dac_manual_ch4[15:8] <= wdata_hold[31:24];
                    end
                    12'h0E0: begin
                        if (!dma_read_busy && !dma_write_busy) begin
                            memory_mode <= wdata_hold[0];
                            identification_mode <= wdata_hold[2];
                            if (wdata_hold[1]) begin
                                dma_start <= 1'b1;
                                dma_done_seen <= 1'b0;
                            end
                        end else begin
                            config_error <= 1'b1;
                        end
                    end
                    12'h0E4: dma_read_base <= merge_wstrb(dma_read_base,wdata_hold,wstrb_hold);
                    12'h0E8: dma_write_base <= merge_wstrb(dma_write_base,wdata_hold,wstrb_hold);
                    12'h0EC: dma_sample_count <= merge_wstrb(dma_sample_count,wdata_hold,wstrb_hold);
                    default: begin
                        for (section_index = 0; section_index < NUM_SECTIONS;
                             section_index = section_index + 1) begin
                            if (awaddr_hold[11:0] == (12'h040 + section_index*20 + 0))
                                shadow_b0_flat[section_index*DATA_WIDTH +: DATA_WIDTH]
                                    <= merge_wstrb(shadow_b0_flat[section_index*DATA_WIDTH +: DATA_WIDTH],wdata_hold,wstrb_hold);
                            if (awaddr_hold[11:0] == (12'h040 + section_index*20 + 4))
                                shadow_b1_flat[section_index*DATA_WIDTH +: DATA_WIDTH]
                                    <= merge_wstrb(shadow_b1_flat[section_index*DATA_WIDTH +: DATA_WIDTH],wdata_hold,wstrb_hold);
                            if (awaddr_hold[11:0] == (12'h040 + section_index*20 + 8))
                                shadow_b2_flat[section_index*DATA_WIDTH +: DATA_WIDTH]
                                    <= merge_wstrb(shadow_b2_flat[section_index*DATA_WIDTH +: DATA_WIDTH],wdata_hold,wstrb_hold);
                            if (awaddr_hold[11:0] == (12'h040 + section_index*20 + 12))
                                shadow_a1_flat[section_index*DATA_WIDTH +: DATA_WIDTH]
                                    <= merge_wstrb(shadow_a1_flat[section_index*DATA_WIDTH +: DATA_WIDTH],wdata_hold,wstrb_hold);
                            if (awaddr_hold[11:0] == (12'h040 + section_index*20 + 16))
                                shadow_a2_flat[section_index*DATA_WIDTH +: DATA_WIDTH]
                                    <= merge_wstrb(shadow_a2_flat[section_index*DATA_WIDTH +: DATA_WIDTH],wdata_hold,wstrb_hold);
                        end
                    end
                endcase
            end
        end
    end
endmodule
