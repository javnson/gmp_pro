`timescale 1ns / 1ps

// Vendor-neutral ePWM-style modulator. Configuration is expected to come from
// the AXI4-Lite register bank. Fast controller results use a direct valid/data
// path and become active only at a PWM zero event (shadow-load semantics).
module epwm_modulator #(
    parameter integer DATA_WIDTH = 32,
    parameter integer FRAC_WIDTH = 28
) (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         enable,
    input  wire                         center_aligned,
    input  wire [31:0]                  period,
    input  wire [31:0]                  compare_shadow,
    input  wire [15:0]                  deadtime_cycles,
    input  wire [31:0]                  phase,
    input  wire                         sync_in,
    input  wire                         trip_in,
    input  wire                         trip_clear,
    input  wire                         control_enable,
    input  wire signed [DATA_WIDTH-1:0] control_data,
    input  wire                         control_valid,
    output reg                          pwm_a,
    output reg                          pwm_b,
    output reg                          period_tick,
    output reg                          trip_latched,
    output wire [31:0]                  counter_value,
    output wire [31:0]                  compare_active_value
);
    reg [31:0] counter;
    reg [31:0] active_period;
    reg [31:0] active_compare;
    reg [31:0] control_compare_shadow;
    reg        count_down;
    reg [15:0] delay_a;
    reg [15:0] delay_b;
    reg        raw_a;

    reg signed [63:0] control_product;
    reg signed [63:0] control_compare_calc;
    reg [31:0] mapped_compare;

    assign counter_value        = counter;
    assign compare_active_value = active_compare;

    always @* begin
        control_product = $signed(control_data) * $signed({1'b0, active_period});
        control_compare_calc = $signed({1'b0, active_period}) / 2
                             + (control_product >>> (FRAC_WIDTH + 1));
        if (control_compare_calc < 0)
            mapped_compare = 32'd0;
        else if (control_compare_calc > $signed({1'b0, active_period}))
            mapped_compare = active_period;
        else
            mapped_compare = control_compare_calc[31:0];
        raw_a = enable && !trip_latched && (counter < active_compare);
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter                <= 32'd0;
            active_period          <= 32'd1;
            active_compare         <= 32'd0;
            control_compare_shadow <= 32'd0;
            count_down             <= 1'b0;
            period_tick            <= 1'b0;
            trip_latched           <= 1'b0;
        end else begin
            period_tick <= 1'b0;
            if (trip_in)
                trip_latched <= 1'b1;
            else if (trip_clear)
                trip_latched <= 1'b0;

            if (control_valid)
                control_compare_shadow <= mapped_compare;

            if (!enable) begin
                counter    <= 32'd0;
                count_down <= 1'b0;
                active_period  <= (period == 0) ? 32'd1 : period;
                active_compare <= (compare_shadow > period) ? period : compare_shadow;
            end else if (sync_in) begin
                counter <= (phase > active_period) ? active_period : phase;
            end else if (center_aligned) begin
                if (!count_down) begin
                    if (counter >= active_period) begin
                        count_down <= 1'b1;
                        if (counter != 0)
                            counter <= counter - 1'b1;
                    end else begin
                        counter <= counter + 1'b1;
                    end
                end else begin
                    if (counter == 0) begin
                        count_down    <= 1'b0;
                        period_tick   <= 1'b1;
                        active_period <= (period == 0) ? 32'd1 : period;
                        if (control_enable)
                            active_compare <= control_compare_shadow;
                        else
                            active_compare <= (compare_shadow > period) ? period : compare_shadow;
                        counter <= 32'd1;
                    end else begin
                        counter <= counter - 1'b1;
                    end
                end
            end else begin
                if (counter >= active_period) begin
                    counter       <= 32'd0;
                    period_tick   <= 1'b1;
                    active_period <= (period == 0) ? 32'd1 : period;
                    if (control_enable)
                        active_compare <= control_compare_shadow;
                    else
                        active_compare <= (compare_shadow > period) ? period : compare_shadow;
                end else begin
                    counter <= counter + 1'b1;
                end
            end
        end
    end

    // Complementary output dead-band: falling edges are immediate and rising
    // edges are delayed until the opposite switch is confirmed off.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_a   <= 1'b0;
            pwm_b   <= 1'b0;
            delay_a <= 16'd0;
            delay_b <= 16'd0;
        end else if (!enable || trip_latched || trip_in) begin
            pwm_a   <= 1'b0;
            pwm_b   <= 1'b0;
            delay_a <= deadtime_cycles;
            delay_b <= deadtime_cycles;
        end else begin
            if (!raw_a) begin
                pwm_a   <= 1'b0;
                delay_a <= deadtime_cycles;
            end else if (!pwm_b) begin
                if (deadtime_cycles == 0 || delay_a <= 1) begin
                    pwm_a <= 1'b1;
                    delay_a <= 16'd0;
                end
                else
                    delay_a <= delay_a - 1'b1;
            end

            if (raw_a) begin
                pwm_b   <= 1'b0;
                delay_b <= deadtime_cycles;
            end else if (!pwm_a) begin
                if (deadtime_cycles == 0 || delay_b <= 1) begin
                    pwm_b <= 1'b1;
                    delay_b <= 16'd0;
                end
                else
                    delay_b <= delay_b - 1'b1;
            end
        end
    end
endmodule
