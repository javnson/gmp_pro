`timescale 1ns/1ps
module tb_epwm_modulator;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0, enable=0, center=0, sync_in=0, trip=0, trip_clear=0;
    reg control_enable=0, control_valid=0;
    reg signed [31:0] control_data=0;
    reg [31:0] period=8, compare=4, phase=0;
    reg [15:0] deadtime=1;
    wire pwm_a,pwm_b,tick,trip_latched; wire [31:0] counter,active_compare;
    integer errors=0; integer ticks=0; integer k;

    epwm_modulator dut(.clk(clk),.rst_n(rst_n),.enable(enable),
        .center_aligned(center),.period(period),.compare_shadow(compare),
        .deadtime_cycles(deadtime),.phase(phase),.sync_in(sync_in),
        .trip_in(trip),.trip_clear(trip_clear),.control_enable(control_enable),
        .control_data(control_data),.control_valid(control_valid),.pwm_a(pwm_a),
        .pwm_b(pwm_b),.period_tick(tick),.trip_latched(trip_latched),
        .counter_value(counter),.compare_active_value(active_compare));

    always @(posedge clk) begin
        if(pwm_a && pwm_b) begin errors=errors+1; $display("ERROR shoot-through"); end
        if(tick) ticks=ticks+1;
    end

    initial begin
        repeat(3) @(posedge clk); rst_n=1;
        repeat(2) @(posedge clk); enable=1;
        for(k=0;k<40;k=k+1) @(posedge clk);
        if(ticks<3) begin errors=errors+1; $display("ERROR missing PWM ticks %0d",ticks); end
        trip=1; @(posedge clk); @(negedge clk); trip=0;
        repeat(2) @(posedge clk);
        if(pwm_a||pwm_b||!trip_latched) begin errors=errors+1; $display("ERROR trip shutdown"); end
        trip_clear=1; @(posedge clk); @(negedge clk); trip_clear=0;
        repeat(3) @(posedge clk);
        if(trip_latched) begin errors=errors+1; $display("ERROR trip clear"); end
        control_enable=1; control_data=32'sh1000_0000;
        control_valid=1; @(posedge clk); @(negedge clk); control_valid=0;
        wait(tick); @(posedge clk);
        if(active_compare!==period) begin
            errors=errors+1; $display("ERROR control-to-compare got=%0d",active_compare);
        end
        if(errors==0) $display("PASS tb_epwm_modulator");
        else $fatal(1,"FAIL tb_epwm_modulator errors=%0d",errors);
        #20 $finish;
    end
endmodule
