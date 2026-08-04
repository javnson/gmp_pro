`timescale 1ns/1ps
module tb_data_adapters;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0; reg [15:0] adc_sample=0; reg sample_valid=0;
    wire signed [31:0] fixed_data; wire fixed_valid; reg fixed_ready=1;
    wire overflow; wire [15:0] dac_code;
    integer errors=0;
    adc_axis_adapter adc(.clk(clk),.rst_n(rst_n),.adc_sample(adc_sample),
        .sample_valid(sample_valid),.m_axis_tdata(fixed_data),
        .m_axis_tvalid(fixed_valid),.m_axis_tready(fixed_ready),.overflow(overflow));
    fixed_to_dac dac(.fixed_sample(fixed_data),.dac_code(dac_code));

    task check_code(input [15:0] raw,input signed [31:0] expected_fixed,input [15:0] expected_dac);
        begin
            @(negedge clk); adc_sample=raw; sample_valid=1;
            @(negedge clk); sample_valid=0;
            while(!fixed_valid) @(posedge clk);
            if(fixed_data!==expected_fixed || dac_code!==expected_dac) begin
                errors=errors+1;
                $display("ERROR adapter raw=%h fixed=%h dac=%h ext=%h",
                         raw,fixed_data,dac_code,adc.centered_extended);
            end
            @(posedge clk);
        end
    endtask

    initial begin
        repeat(3) @(posedge clk); rst_n=1;
        check_code(16'h0000,32'shf000_0000,16'h0000);
        check_code(16'h8000,32'sh0000_0000,16'h8000);
        check_code(16'hffff,32'sh0fff_e000,16'hffff);
        if(errors==0) $display("PASS tb_data_adapters");
        else $fatal(1,"FAIL tb_data_adapters errors=%0d",errors);
        #20 $finish;
    end
endmodule
