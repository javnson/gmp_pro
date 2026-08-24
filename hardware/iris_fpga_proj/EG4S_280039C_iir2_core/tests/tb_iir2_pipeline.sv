`timescale 1ns/1ps
module tb_iir2_pipeline;
    localparam W = 16;
    localparam F = 12;
    localparam N = 2;
    reg clk = 0; always #5 clk = ~clk;
    reg rst_n = 0, clear_state = 0;
    reg [7:0] active_sections = 2;
    reg [N*W-1:0] b0, b1, b2, a1, a2;
    reg signed [W-1:0] s_data;
    reg s_valid = 0, s_last = 0;
    wire s_ready;
    wire signed [W-1:0] m_data;
    wire m_valid, m_last;
    reg m_ready = 1;
    wire busy, saturated;
    integer errors = 0;

    iir2_pipeline_container #(.DATA_WIDTH(W),.FRAC_WIDTH(F),.NUM_SECTIONS(N)) dut (
        .clk(clk),.rst_n(rst_n),.clear_state(clear_state),
        .active_sections(active_sections),.coeff_b0_flat(b0),
        .coeff_b1_flat(b1),.coeff_b2_flat(b2),.coeff_a1_flat(a1),
        .coeff_a2_flat(a2),.s_axis_tdata(s_data),.s_axis_tvalid(s_valid),
        .s_axis_tready(s_ready),.s_axis_tlast(s_last),.m_axis_tdata(m_data),
        .m_axis_tvalid(m_valid),.m_axis_tready(m_ready),.m_axis_tlast(m_last),
        .busy(busy),.saturated(saturated)
    );

    initial begin
        b0 = 0; b1 = 0; b2 = 0; a1 = 0; a2 = 0;
        b0[0 +: W] = 16'sd2048;
        b0[W +: W] = 16'sd2048;
        s_data = 0;
        repeat(3) @(posedge clk); rst_n = 1;
        @(negedge clk); s_data = 16'sd4096; s_last = 1; s_valid = 1;
        while(!s_ready) @(negedge clk);
        @(negedge clk); s_valid = 0; s_last = 0;

        // Hold output for several clocks to verify AXI backpressure stability.
        while(!m_valid) @(posedge clk);
        m_ready = 0;
        repeat(3) begin
            @(posedge clk);
            if (m_data !== 16'sd1024 || !m_last) begin
                $display("ERROR pipeline output changed under backpressure");
                errors = errors + 1;
            end
        end
        @(negedge clk); m_ready = 1;
        @(posedge clk);

        if (errors == 0) $display("PASS tb_iir2_pipeline");
        else $fatal(1,"FAIL tb_iir2_pipeline errors=%0d",errors);
        #20 $finish;
    end
endmodule
