`timescale 1ns/1ps
module tb_iir2_operator;
    localparam W = 16;
    localparam F = 12;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst_n = 0;
    reg clear_state = 0;
    reg section_enable = 1;
    reg signed [W-1:0] b0, b1, b2, a1, a2;
    reg signed [W-1:0] s_data;
    reg s_valid, s_last;
    wire s_ready;
    wire signed [W-1:0] m_data;
    wire m_valid, m_last;
    reg m_ready = 1;
    wire busy, saturated;
    integer errors = 0;

    iir2_operator #(.DATA_WIDTH(W), .FRAC_WIDTH(F)) dut (
        .clk(clk), .rst_n(rst_n), .clear_state(clear_state),
        .section_enable(section_enable), .coeff_b0(b0), .coeff_b1(b1),
        .coeff_b2(b2), .coeff_a1(a1), .coeff_a2(a2),
        .s_axis_tdata(s_data), .s_axis_tvalid(s_valid),
        .s_axis_tready(s_ready), .s_axis_tlast(s_last),
        .m_axis_tdata(m_data), .m_axis_tvalid(m_valid),
        .m_axis_tready(m_ready), .m_axis_tlast(m_last),
        .busy(busy), .saturated(saturated)
    );

    task send_sample(input signed [W-1:0] value, input last);
        begin
            @(negedge clk); s_data = value; s_last = last; s_valid = 1;
            while (!s_ready) @(negedge clk);
            @(negedge clk); s_valid = 0; s_last = 0;
        end
    endtask

    task expect_sample(input signed [W-1:0] expected, input expected_last);
        begin
            while (!m_valid) @(posedge clk);
            if ($signed(m_data) !== expected) begin
                $display("ERROR iir output got=%0d expected=%0d", $signed(m_data), expected);
                errors = errors + 1;
            end
            if (m_last !== expected_last) begin
                $display("ERROR tlast got=%b expected=%b", m_last, expected_last);
                errors = errors + 1;
            end
            @(posedge clk);
        end
    endtask

    initial begin
        s_valid = 0; s_last = 0; s_data = 0;
        b0 = 16'sd2048; b1 = 0; b2 = 0; a1 = -16'sd2048; a2 = 0;
        repeat (3) @(posedge clk); rst_n = 1;

        send_sample(16'sd4096, 0); expect_sample(16'sd2048, 0);
        send_sample(16'sd0,    0); expect_sample(16'sd1024, 0);
        send_sample(16'sd0,    1); expect_sample(16'sd512,  1);

        @(negedge clk); clear_state = 1;
        @(negedge clk); clear_state = 0;
        b0 = 16'sd8192; a1 = 0;
        send_sample(16'sd32767, 1);
        while (!m_valid) @(posedge clk);
        if (m_data !== 16'sh7fff || !saturated) begin
            $display("ERROR saturation data=%h flag=%b", m_data, saturated);
            errors = errors + 1;
        end

        if (errors == 0) $display("PASS tb_iir2_operator");
        else $fatal(1, "FAIL tb_iir2_operator errors=%0d", errors);
        #20 $finish;
    end
endmodule
