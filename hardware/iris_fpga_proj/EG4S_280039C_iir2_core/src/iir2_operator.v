`timescale 1ns / 1ps

// Portable, vendor-neutral fixed-point second-order IIR section.
//
// Difference equation:
//   y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2]
//                    - a1*y[n-1] - a2*y[n-2]
//
// Samples and coefficients use the same signed Q format.  Five products are
// evaluated in parallel, followed by a registered accumulator and a
// saturating scale stage.  A section accepts a new sample every three clocks;
// independent sections in iir2_pipeline_container operate concurrently.
module iir2_operator #(
    parameter integer DATA_WIDTH = 32,
    parameter integer FRAC_WIDTH = 28
) (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         clear_state,
    input  wire                         section_enable,

    input  wire signed [DATA_WIDTH-1:0] coeff_b0,
    input  wire signed [DATA_WIDTH-1:0] coeff_b1,
    input  wire signed [DATA_WIDTH-1:0] coeff_b2,
    input  wire signed [DATA_WIDTH-1:0] coeff_a1,
    input  wire signed [DATA_WIDTH-1:0] coeff_a2,

    input  wire signed [DATA_WIDTH-1:0] s_axis_tdata,
    input  wire                         s_axis_tvalid,
    output wire                         s_axis_tready,
    input  wire                         s_axis_tlast,

    output wire signed [DATA_WIDTH-1:0] m_axis_tdata,
    output wire                         m_axis_tvalid,
    input  wire                         m_axis_tready,
    output wire                         m_axis_tlast,

    output wire                         busy,
    output wire                         saturated
);
    localparam integer PROD_WIDTH = 2 * DATA_WIDTH;
    localparam integer ACC_WIDTH  = PROD_WIDTH + 3;

    reg signed [DATA_WIDTH-1:0] x_z1;
    reg signed [DATA_WIDTH-1:0] x_z2;
    reg signed [DATA_WIDTH-1:0] y_z1;
    reg signed [DATA_WIDTH-1:0] y_z2;

    reg signed [DATA_WIDTH-1:0] p0_x;
    reg signed [PROD_WIDTH-1:0] p0_b0;
    reg signed [PROD_WIDTH-1:0] p0_b1;
    reg signed [PROD_WIDTH-1:0] p0_b2;
    reg signed [PROD_WIDTH-1:0] p0_a1;
    reg signed [PROD_WIDTH-1:0] p0_a2;
    reg                         p0_valid;
    reg                         p0_last;
    reg                         p0_bypass;

    reg signed [DATA_WIDTH-1:0] p1_x;
    reg signed [ACC_WIDTH-1:0]  p1_acc;
    reg                         p1_valid;
    reg                         p1_last;
    reg                         p1_bypass;

    reg signed [DATA_WIDTH-1:0] out_data;
    reg                         out_valid;
    reg                         out_last;
    reg                         out_sat;

    function signed [DATA_WIDTH-1:0] saturate_scaled;
        input signed [ACC_WIDTH-1:0] value;
        reg signed [ACC_WIDTH-1:0] scaled;
        reg signed [ACC_WIDTH-1:0] max_value;
        reg signed [ACC_WIDTH-1:0] min_value;
        begin
            scaled = value >>> FRAC_WIDTH;
            max_value = {{(ACC_WIDTH-DATA_WIDTH){1'b0}}, 1'b0,
                         {(DATA_WIDTH-1){1'b1}}};
            min_value = {{(ACC_WIDTH-DATA_WIDTH){1'b1}}, 1'b1,
                         {(DATA_WIDTH-1){1'b0}}};
            if (scaled > max_value)
                saturate_scaled = {1'b0, {(DATA_WIDTH-1){1'b1}}};
            else if (scaled < min_value)
                saturate_scaled = {1'b1, {(DATA_WIDTH-1){1'b0}}};
            else
                saturate_scaled = scaled[DATA_WIDTH-1:0];
        end
    endfunction

    function is_saturated;
        input signed [ACC_WIDTH-1:0] value;
        reg signed [ACC_WIDTH-1:0] scaled;
        reg signed [ACC_WIDTH-1:0] max_value;
        reg signed [ACC_WIDTH-1:0] min_value;
        begin
            scaled = value >>> FRAC_WIDTH;
            max_value = {{(ACC_WIDTH-DATA_WIDTH){1'b0}}, 1'b0,
                         {(DATA_WIDTH-1){1'b1}}};
            min_value = {{(ACC_WIDTH-DATA_WIDTH){1'b1}}, 1'b1,
                         {(DATA_WIDTH-1){1'b0}}};
            is_saturated = (scaled > max_value) || (scaled < min_value);
        end
    endfunction

    assign s_axis_tready = !p0_valid && !p1_valid &&
                           (!out_valid || m_axis_tready);
    assign m_axis_tdata  = out_data;
    assign m_axis_tvalid = out_valid;
    assign m_axis_tlast  = out_last;
    assign busy          = p0_valid || p1_valid || out_valid;
    assign saturated     = out_valid && out_sat && m_axis_tready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_z1      <= {DATA_WIDTH{1'b0}};
            x_z2      <= {DATA_WIDTH{1'b0}};
            y_z1      <= {DATA_WIDTH{1'b0}};
            y_z2      <= {DATA_WIDTH{1'b0}};
            p0_valid  <= 1'b0;
            p1_valid  <= 1'b0;
            out_valid <= 1'b0;
            out_data  <= {DATA_WIDTH{1'b0}};
            out_last  <= 1'b0;
            out_sat   <= 1'b0;
        end else if (clear_state) begin
            x_z1      <= {DATA_WIDTH{1'b0}};
            x_z2      <= {DATA_WIDTH{1'b0}};
            y_z1      <= {DATA_WIDTH{1'b0}};
            y_z2      <= {DATA_WIDTH{1'b0}};
            p0_valid  <= 1'b0;
            p1_valid  <= 1'b0;
            out_valid <= 1'b0;
            out_sat   <= 1'b0;
        end else begin
            if (out_valid && m_axis_tready)
                out_valid <= 1'b0;

            if (p1_valid && (!out_valid || m_axis_tready)) begin
                out_data  <= p1_bypass ? p1_x : saturate_scaled(p1_acc);
                out_last  <= p1_last;
                out_sat   <= p1_bypass ? 1'b0 : is_saturated(p1_acc);
                out_valid <= 1'b1;
                p1_valid  <= 1'b0;

                // State changes exactly when the calculated result becomes
                // visible. Backpressure can therefore never duplicate state.
                if (!p1_bypass) begin
                    x_z2 <= x_z1;
                    x_z1 <= p1_x;
                    y_z2 <= y_z1;
                    y_z1 <= saturate_scaled(p1_acc);
                end
            end

            if (p0_valid && !p1_valid) begin
                p1_x   <= p0_x;
                p1_acc <= {{3{p0_b0[PROD_WIDTH-1]}}, p0_b0}
                        + {{3{p0_b1[PROD_WIDTH-1]}}, p0_b1}
                        + {{3{p0_b2[PROD_WIDTH-1]}}, p0_b2}
                        - {{3{p0_a1[PROD_WIDTH-1]}}, p0_a1}
                        - {{3{p0_a2[PROD_WIDTH-1]}}, p0_a2};
                p1_last   <= p0_last;
                p1_bypass <= p0_bypass;
                p1_valid  <= 1'b1;
                p0_valid  <= 1'b0;
            end

            if (s_axis_tvalid && s_axis_tready) begin
                p0_x      <= s_axis_tdata;
                p0_b0     <= $signed(s_axis_tdata) * $signed(coeff_b0);
                p0_b1     <= $signed(x_z1) * $signed(coeff_b1);
                p0_b2     <= $signed(x_z2) * $signed(coeff_b2);
                p0_a1     <= $signed(y_z1) * $signed(coeff_a1);
                p0_a2     <= $signed(y_z2) * $signed(coeff_a2);
                p0_last   <= s_axis_tlast;
                p0_bypass <= !section_enable;
                p0_valid  <= 1'b1;
            end
        end
    end
endmodule
