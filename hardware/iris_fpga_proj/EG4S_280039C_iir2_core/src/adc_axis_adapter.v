`timescale 1ns / 1ps

// Converts an ADS8688 16-bit offset-binary sample into a normalized signed
// fixed-point AXI4-Stream sample.  A one-entry elastic buffer absorbs the ADC
// update pulse while the controller applies backpressure.
module adc_axis_adapter #(
    parameter integer DATA_WIDTH = 32,
    parameter integer FRAC_WIDTH = 28,
    parameter integer OFFSET_BINARY = 1
) (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire [15:0]                  adc_sample,
    input  wire                         sample_valid,
    output wire signed [DATA_WIDTH-1:0] m_axis_tdata,
    output wire                         m_axis_tvalid,
    input  wire                         m_axis_tready,
    output reg                          overflow
);
    reg signed [DATA_WIDTH-1:0] sample_reg;
    reg                         valid_reg;
    wire [15:0] centered_code = (OFFSET_BINARY != 0) ?
                                {~adc_sample[15], adc_sample[14:0]} : adc_sample;
    wire signed [DATA_WIDTH-1:0] centered_extended =
        {{(DATA_WIDTH-16){centered_code[15]}}, centered_code};

    assign m_axis_tdata  = sample_reg;
    assign m_axis_tvalid = valid_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_reg <= {DATA_WIDTH{1'b0}};
            valid_reg  <= 1'b0;
            overflow   <= 1'b0;
        end else begin
            if (valid_reg && m_axis_tready)
                valid_reg <= 1'b0;

            if (sample_valid) begin
                if (!valid_reg || m_axis_tready) begin
                    if (FRAC_WIDTH >= 15)
                        sample_reg <= centered_extended <<< (FRAC_WIDTH - 15);
                    else
                        sample_reg <= centered_extended >>> (15 - FRAC_WIDTH);
                    valid_reg <= 1'b1;
                end else begin
                    overflow <= 1'b1;
                end
            end
        end
    end
endmodule
