`timescale 1ns / 1ps

// AXI4-Stream container for a cascade of second-order sections (SOS).
// Each section is an independent pipeline; ready/valid provides the staggered
// read/compute/write schedule and lossless downstream backpressure.
module iir2_pipeline_container #(
    parameter integer DATA_WIDTH   = 32,
    parameter integer FRAC_WIDTH   = 28,
    parameter integer NUM_SECTIONS = 4
) (
    input  wire                               clk,
    input  wire                               rst_n,
    input  wire                               clear_state,
    input  wire [7:0]                         active_sections,

    input  wire [NUM_SECTIONS*DATA_WIDTH-1:0] coeff_b0_flat,
    input  wire [NUM_SECTIONS*DATA_WIDTH-1:0] coeff_b1_flat,
    input  wire [NUM_SECTIONS*DATA_WIDTH-1:0] coeff_b2_flat,
    input  wire [NUM_SECTIONS*DATA_WIDTH-1:0] coeff_a1_flat,
    input  wire [NUM_SECTIONS*DATA_WIDTH-1:0] coeff_a2_flat,

    input  wire signed [DATA_WIDTH-1:0]       s_axis_tdata,
    input  wire                               s_axis_tvalid,
    output wire                               s_axis_tready,
    input  wire                               s_axis_tlast,

    output wire signed [DATA_WIDTH-1:0]       m_axis_tdata,
    output wire                               m_axis_tvalid,
    input  wire                               m_axis_tready,
    output wire                               m_axis_tlast,

    output wire                               busy,
    output wire                               saturated
);
    wire [(NUM_SECTIONS+1)*DATA_WIDTH-1:0] stage_data;
    wire [NUM_SECTIONS:0]                  stage_valid;
    wire [NUM_SECTIONS:0]                  stage_ready;
    wire [NUM_SECTIONS:0]                  stage_last;
    wire [NUM_SECTIONS-1:0]                section_busy;
    wire [NUM_SECTIONS-1:0]                section_sat;

    assign stage_data[DATA_WIDTH-1:0] = s_axis_tdata;
    assign stage_valid[0]             = s_axis_tvalid;
    assign stage_last[0]              = s_axis_tlast;
    assign s_axis_tready              = stage_ready[0];

    assign m_axis_tdata  = stage_data[NUM_SECTIONS*DATA_WIDTH +: DATA_WIDTH];
    assign m_axis_tvalid = stage_valid[NUM_SECTIONS];
    assign m_axis_tlast  = stage_last[NUM_SECTIONS];
    assign stage_ready[NUM_SECTIONS] = m_axis_tready;

    assign busy      = |section_busy;
    assign saturated = |section_sat;

    genvar section_index;
    generate
        for (section_index = 0; section_index < NUM_SECTIONS;
             section_index = section_index + 1) begin : g_iir2_section
            iir2_operator #(
                .DATA_WIDTH(DATA_WIDTH),
                .FRAC_WIDTH(FRAC_WIDTH)
            ) u_iir2_operator (
                .clk            (clk),
                .rst_n          (rst_n),
                .clear_state    (clear_state),
                .section_enable (active_sections > section_index),
                .coeff_b0       (coeff_b0_flat[section_index*DATA_WIDTH +: DATA_WIDTH]),
                .coeff_b1       (coeff_b1_flat[section_index*DATA_WIDTH +: DATA_WIDTH]),
                .coeff_b2       (coeff_b2_flat[section_index*DATA_WIDTH +: DATA_WIDTH]),
                .coeff_a1       (coeff_a1_flat[section_index*DATA_WIDTH +: DATA_WIDTH]),
                .coeff_a2       (coeff_a2_flat[section_index*DATA_WIDTH +: DATA_WIDTH]),
                .s_axis_tdata   (stage_data[section_index*DATA_WIDTH +: DATA_WIDTH]),
                .s_axis_tvalid  (stage_valid[section_index]),
                .s_axis_tready  (stage_ready[section_index]),
                .s_axis_tlast   (stage_last[section_index]),
                .m_axis_tdata   (stage_data[(section_index+1)*DATA_WIDTH +: DATA_WIDTH]),
                .m_axis_tvalid  (stage_valid[section_index+1]),
                .m_axis_tready  (stage_ready[section_index+1]),
                .m_axis_tlast   (stage_last[section_index+1]),
                .busy           (section_busy[section_index]),
                .saturated      (section_sat[section_index])
            );
        end
    endgenerate
endmodule
