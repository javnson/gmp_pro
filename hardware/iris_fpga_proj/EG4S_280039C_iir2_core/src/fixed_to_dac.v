`timescale 1ns / 1ps

// Maps normalized signed fixed point (-1.0 .. +1.0) to a 16-bit
// offset-binary DAC code with saturation.
module fixed_to_dac #(
    parameter integer DATA_WIDTH = 32,
    parameter integer FRAC_WIDTH = 28
) (
    input  wire signed [DATA_WIDTH-1:0] fixed_sample,
    output reg  [15:0]                  dac_code
);
    reg signed [DATA_WIDTH:0] scaled;
    always @* begin
        if (FRAC_WIDTH >= 15)
            scaled = (fixed_sample >>> (FRAC_WIDTH - 15)) + 17'sd32768;
        else
            scaled = (fixed_sample <<< (15 - FRAC_WIDTH)) + 17'sd32768;

        if (scaled < 0)
            dac_code = 16'h0000;
        else if (scaled > 17'sd65535)
            dac_code = 16'hFFFF;
        else
            dac_code = scaled[15:0];
    end
endmodule
