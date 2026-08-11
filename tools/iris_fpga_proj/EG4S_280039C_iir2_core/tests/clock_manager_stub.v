`timescale 1ns/1ps
module clock_manager(input wire refclk,input wire reset,output wire clk0_out);
    assign clk0_out = refclk;
endmodule
