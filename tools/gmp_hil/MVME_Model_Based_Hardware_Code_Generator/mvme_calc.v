`timescale 1ns / 1ps

module pmsm_solver_v2 (
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 enable,
    input  wire                 trigger,
    input  wire                 load,
    input  wire                 update_param,
    output reg                  param_ready,
    output reg                  valid,

    // --- Parameters ---
    input  wire signed [34:0] K_ID_SELF,
    input  wire signed [34:0] K_ID_CROSS,
    input  wire signed [34:0] K_ID_VOLT,
    input  wire signed [34:0] K_IQ_SELF,
    input  wire signed [34:0] K_IQ_CROSS,
    input  wire signed [34:0] K_IQ_EMF,
    input  wire signed [34:0] K_IQ_VOLT,
    input  wire signed [34:0] K_THETA,
    input  wire signed [34:0] ONE_CONST,

    // --- State Inputs (Load values) ---
    input  wire signed [34:0] id_init,
    input  wire signed [34:0] iq_init,
    input  wire signed [34:0] omega_init,
    input  wire signed [34:0] theta_init,

    // --- State Outputs ---
    output reg  signed [34:0] id,
    output reg  signed [34:0] iq,
    output reg  signed [34:0] omega,
    output reg  signed [34:0] theta
);

    // --- Internal Signals ---
    reg signed [34:0] K_ID_SELF_shadow;
    reg signed [34:0] K_ID_SELF_active;
    reg signed [34:0] K_ID_CROSS_shadow;
    reg signed [34:0] K_ID_CROSS_active;
    reg signed [34:0] K_ID_VOLT_shadow;
    reg signed [34:0] K_ID_VOLT_active;
    reg signed [34:0] K_IQ_SELF_shadow;
    reg signed [34:0] K_IQ_SELF_active;
    reg signed [34:0] K_IQ_CROSS_shadow;
    reg signed [34:0] K_IQ_CROSS_active;
    reg signed [34:0] K_IQ_EMF_shadow;
    reg signed [34:0] K_IQ_EMF_active;
    reg signed [34:0] K_IQ_VOLT_shadow;
    reg signed [34:0] K_IQ_VOLT_active;
    reg signed [34:0] K_THETA_shadow;
    reg signed [34:0] K_THETA_active;
    reg signed [34:0] ONE_CONST_shadow;
    reg signed [34:0] ONE_CONST_active;
    reg signed [34:0] ud;
    reg signed [34:0] uq;
    reg signed [34:0] w_iq;
    reg signed [34:0] w_id;
    reg signed [34:0] temp_mac_res;

    // MVME_0 Interface
    reg signed [34:0] mvme0_a, mvme0_b;
    reg signed [34:0] mvme0_c, mvme0_d;
    reg signed [34:0] mvme0_e, mvme0_f;
    reg signed [34:0] mvme0_g, mvme0_h;
    wire signed [34:0] mvme0_out_total;
    wire signed [34:0] mvme0_out_ab_cd, mvme0_out_ef_gh;
    wire signed [34:0] mvme0_out_p0, mvme0_out_p1;
    wire signed [34:0] mvme0_out_p2, mvme0_out_p3;
    wire               mvme0_overload;

    // MVME_1 Interface
    reg signed [34:0] mvme1_a, mvme1_b;
    reg signed [34:0] mvme1_c, mvme1_d;
    reg signed [34:0] mvme1_e, mvme1_f;
    reg signed [34:0] mvme1_g, mvme1_h;
    wire signed [34:0] mvme1_out_total;
    wire signed [34:0] mvme1_out_ab_cd, mvme1_out_ef_gh;
    wire signed [34:0] mvme1_out_p0, mvme1_out_p1;
    wire signed [34:0] mvme1_out_p2, mvme1_out_p3;
    wire               mvme1_overload;

    // Pipeline Control
    reg [15:0] pc_cnt; // Pipeline Counter
    reg        running;
    reg        trigger_d;
    wire       trigger_pulse = trigger & ~trigger_d;
    localparam MAX_PC = 23;

    // --- Parameter Shadow Logic ---
    always @(posedge clk) begin
        if (rst) begin
            param_ready <= 0;
            K_ID_SELF_shadow <= 0; K_ID_SELF_active <= 0;
            K_ID_CROSS_shadow <= 0; K_ID_CROSS_active <= 0;
            K_ID_VOLT_shadow <= 0; K_ID_VOLT_active <= 0;
            K_IQ_SELF_shadow <= 0; K_IQ_SELF_active <= 0;
            K_IQ_CROSS_shadow <= 0; K_IQ_CROSS_active <= 0;
            K_IQ_EMF_shadow <= 0; K_IQ_EMF_active <= 0;
            K_IQ_VOLT_shadow <= 0; K_IQ_VOLT_active <= 0;
            K_THETA_shadow <= 0; K_THETA_active <= 0;
            ONE_CONST_shadow <= 0; ONE_CONST_active <= 0;
        end else begin
            param_ready <= 0;
            K_ID_SELF_shadow <= K_ID_SELF;
            K_ID_CROSS_shadow <= K_ID_CROSS;
            K_ID_VOLT_shadow <= K_ID_VOLT;
            K_IQ_SELF_shadow <= K_IQ_SELF;
            K_IQ_CROSS_shadow <= K_IQ_CROSS;
            K_IQ_EMF_shadow <= K_IQ_EMF;
            K_IQ_VOLT_shadow <= K_IQ_VOLT;
            K_THETA_shadow <= K_THETA;
            ONE_CONST_shadow <= ONE_CONST;
            if (update_param && !running) begin
                K_ID_SELF_active <= K_ID_SELF_shadow;
                K_ID_CROSS_active <= K_ID_CROSS_shadow;
                K_ID_VOLT_active <= K_ID_VOLT_shadow;
                K_IQ_SELF_active <= K_IQ_SELF_shadow;
                K_IQ_CROSS_active <= K_IQ_CROSS_shadow;
                K_IQ_EMF_active <= K_IQ_EMF_shadow;
                K_IQ_VOLT_active <= K_IQ_VOLT_shadow;
                K_THETA_active <= K_THETA_shadow;
                ONE_CONST_active <= ONE_CONST_shadow;
                param_ready <= 1;
            end
        end
    end

    // --- MVME Instantiation (mvme_35_4ch_std) ---
    mvme_35_4ch_std u_mvme_0 (
        .clk(clk), .rst(rst),
        .a(mvme0_a), .b(mvme0_b),
        .c(mvme0_c), .d(mvme0_d),
        .e(mvme0_e), .f(mvme0_f),
        .g(mvme0_g), .h(mvme0_h),
        .out_total(mvme0_out_total),
        .out_ab_cd(mvme0_out_ab_cd),
        .out_ef_gh(mvme0_out_ef_gh),
        .out_p0(mvme0_out_p0), .out_p1(mvme0_out_p1),
        .out_p2(mvme0_out_p2), .out_p3(mvme0_out_p3),
        .overload(mvme0_overload)
    );

    mvme_35_4ch_std u_mvme_1 (
        .clk(clk), .rst(rst),
        .a(mvme1_a), .b(mvme1_b),
        .c(mvme1_c), .d(mvme1_d),
        .e(mvme1_e), .f(mvme1_f),
        .g(mvme1_g), .h(mvme1_h),
        .out_total(mvme1_out_total),
        .out_ab_cd(mvme1_out_ab_cd),
        .out_ef_gh(mvme1_out_ef_gh),
        .out_p0(mvme1_out_p0), .out_p1(mvme1_out_p1),
        .out_p2(mvme1_out_p2), .out_p3(mvme1_out_p3),
        .overload(mvme1_overload)
    );

    // --- Pipeline Controller ---
    always @(posedge clk) begin
        trigger_d <= trigger;
        if (rst || !enable) begin
            pc_cnt <= 0;
            running <= 0;
            valid <= 0;
        end else begin
            valid <= 0;
            
            if (!running) begin
                pc_cnt <= 0;
                if (trigger || trigger_pulse) begin
                    running <= 1;
                end
            end else begin
                if (pc_cnt < MAX_PC) begin
                    pc_cnt <= pc_cnt + 1;
                end else begin
                    valid <= 1;
                    if (trigger) pc_cnt <= 0;
                    else running <= 0;
                end
            end
        end
    end

    // --- MUX Logic (Input Scheduling) ---
    always @(*) begin
        {mvme0_a, mvme0_b, mvme0_c, mvme0_d} = 0;
        {mvme0_e, mvme0_f, mvme0_g, mvme0_h} = 0;
        {mvme1_a, mvme1_b, mvme1_c, mvme1_d} = 0;
        {mvme1_e, mvme1_f, mvme1_g, mvme1_h} = 0;

        case (pc_cnt)
            0: begin
                // Tick 0: Park Transform (Ud, Uq)
                mvme0_a = id;
                mvme0_b = K_ID_SELF_active;
                mvme0_c = omega;
                mvme0_d = iq;
                mvme0_e = ud;
                mvme0_f = K_ID_VOLT_active;
                mvme0_g = 0;
                mvme0_h = 0;
                // Tick 0: Park Transform (Ud, Uq)
                mvme1_a = id;
                mvme1_b = K_ID_SELF_active;
                mvme1_c = omega;
                mvme1_d = iq;
                mvme1_e = ud;
                mvme1_f = K_ID_VOLT_active;
                mvme1_g = 0;
                mvme1_h = 0;
            end
            1: begin
                // Tick 1: Theta Update
                mvme0_a = theta;
                mvme0_b = ONE_CONST_active;
                mvme0_c = omega;
                mvme0_d = K_THETA_active;
                mvme0_e = 0;
                mvme0_f = 0;
                mvme0_g = 0;
                mvme0_h = 0;
                // Tick 1: Theta Update
                mvme1_a = theta;
                mvme1_b = ONE_CONST_active;
                mvme1_c = omega;
                mvme1_d = K_THETA_active;
                mvme1_e = 0;
                mvme1_f = 0;
                mvme1_g = 0;
                mvme1_h = 0;
            end
            12: begin
                // Tick 12: Current Update (Depends on T=0 result)
                mvme0_a = ud;
                mvme0_b = K_ID_VOLT_active;
                mvme0_c = 0;
                mvme0_d = 0;
                mvme0_e = 0;
                mvme0_f = 0;
                mvme0_g = 0;
                mvme0_h = 0;
                // Tick 12: Current Update (Depends on T=0 result)
                mvme1_a = ud;
                mvme1_b = K_ID_VOLT_active;
                mvme1_c = 0;
                mvme1_d = 0;
                mvme1_e = 0;
                mvme1_f = 0;
                mvme1_g = 0;
                mvme1_h = 0;
            end
        endcase
    end

    // --- Latch Logic (Output Capturing) ---
    always @(posedge clk) begin
        if (rst) begin
            id <= 0;
            iq <= 0;
            omega <= 0;
            theta <= 0;
            ud <= 0;
            uq <= 0;
            w_iq <= 0;
            w_id <= 0;
            temp_mac_res <= 0;
        end else if (enable && !running && load && !trigger) begin
            id <= id_init;
            iq <= iq_init;
            omega <= omega_init;
            theta <= theta_init;
        end else if (running) begin
            case (pc_cnt)
                10: begin
                    // Capture for: Tick 0: Park Transform (Ud, Uq)
                    ud <= mvme0_out_ab_cd;
                    uq <= mvme0_out_ef_gh;
                    // Capture for: Tick 0: Park Transform (Ud, Uq)
                    ud <= mvme1_out_ab_cd;
                    uq <= mvme1_out_ef_gh;
                end
                11: begin
                    // Capture for: Tick 1: Theta Update
                    theta <= mvme0_out_total;
                    // Capture for: Tick 1: Theta Update
                    theta <= mvme1_out_total;
                end
                22: begin
                    // Capture for: Tick 12: Current Update (Depends on T=0 result)
                    id <= mvme0_out_total;
                    // Capture for: Tick 12: Current Update (Depends on T=0 result)
                    id <= mvme1_out_total;
                end
            endcase
        end
    end

endmodule
