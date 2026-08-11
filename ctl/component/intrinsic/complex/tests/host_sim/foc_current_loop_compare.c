#include <math.h>
#include <stdio.h>

#include <gmp_core.h>
#include <ctl/component/intrinsic/complex/dq_ladrc1.h>
#include <ctl/component/motor_control/current_loop/foc_core.h>

typedef struct _tag_response_metric
{
    float rise_10_s;
    float rise_90_s;
    float settling_s;
    float overshoot_pct;
    float iae;
    float final_value;
} response_metric_t;

static void update_metric(response_metric_t* metric, float time_s, float value, float target, float dt)
{
    const float error = target - value;
    if (metric->rise_10_s < 0.0f && value >= 0.1f * target)
        metric->rise_10_s = time_s;
    if (metric->rise_90_s < 0.0f && value >= 0.9f * target)
        metric->rise_90_s = time_s;
    if (value > metric->final_value)
        metric->final_value = value;
    if (fabsf(error) > 0.02f * fabsf(target))
        metric->settling_s = time_s;
    metric->iae += fabsf(error) * dt;
}

int main(void)
{
    const float fs = 20000.0f;
    const float dt = 1.0f / fs;
    const float duration_s = 0.03f;
    const float step_time_s = 0.001f;
    const float target_q = 0.3f;
    const float voltage_base = 24.0f / 1.73205080757f;
    const float current_base = 10.0f;
    const float inductance = 50.0e-6f;
    const float resistance = 0.13f;
    mc_foc_init_t pi_init = {0};
    mc_foc_init_t ladrc_init = {0};
    ctl_dq_pi_t pi;
    ctl_dq_ladrc1_t ladrc;
    ctl_vector2_t target = {{0}};
    ctl_vector2_t pi_feedback = {{0}};
    ctl_vector2_t ladrc_feedback = {{0}};
    ctl_vector2_t zero_ff = {{0}};
    ctl_vector2_t pi_output;
    ctl_vector2_t ladrc_output;
    response_metric_t pi_metric = {-1.0f, -1.0f, 0.0f, 0.0f, 0.0f, -1.0e30f};
    response_metric_t ladrc_metric = {-1.0f, -1.0f, 0.0f, 0.0f, 0.0f, -1.0e30f};
    const float plant_b0 = voltage_base / (current_base * inductance);
    const float plant_pole = resistance / inductance;
    int sample;

    pi_init.fs = ladrc_init.fs = fs;
    pi_init.v_base = ladrc_init.v_base = voltage_base;
    pi_init.i_base = ladrc_init.i_base = current_base;
    pi_init.mtr_Ld = pi_init.mtr_Lq = ladrc_init.mtr_Ld = ladrc_init.mtr_Lq = inductance;
    pi_init.mtr_Rs = ladrc_init.mtr_Rs = resistance;
    ctl_auto_tuning_foc_core_pi(&pi_init);
    ctl_auto_tuning_foc_core_ladrc1(&ladrc_init);

    ctl_init_dq_pi(&pi, pi_init.kpd, pi_init.kid, pi_init.kpq, pi_init.kiq, fs);
    ctl_init_dq_ladrc1(&ladrc, ladrc_init.ladrc_b0d, ladrc_init.ladrc_fcd, ladrc_init.ladrc_fod,
                       ladrc_init.ladrc_b0q, ladrc_init.ladrc_fcq, ladrc_init.ladrc_foq, fs);
    ctl_set_dq_pi_circle_limit(&pi, float2ctrl(0.9f));
    ctl_set_dq_ladrc1_circle_limit(&ladrc, float2ctrl(0.9f));
    ctl_enable_dq_pi_circle_limit(&pi);
    ctl_enable_dq_ladrc1_circle_limit(&ladrc);

    for (sample = 0; sample < (int)(duration_s * fs); ++sample)
    {
        const float time_s = sample * dt;
        float pi_iq;
        float ladrc_iq;
        if (time_s >= step_time_s)
            target.dat[1] = float2ctrl(target_q);

        ctl_step_dq_pi(&pi, &target, &pi_feedback, &zero_ff, &pi_output);
        ctl_step_dq_ladrc1(&ladrc, &target, &ladrc_feedback, &zero_ff, &ladrc_output);

        pi_iq = ctrl2float(pi_feedback.dat[1]);
        ladrc_iq = ctrl2float(ladrc_feedback.dat[1]);
        pi_iq += dt * (plant_b0 * ctrl2float(pi_output.dat[1]) - plant_pole * pi_iq);
        ladrc_iq += dt * (plant_b0 * ctrl2float(ladrc_output.dat[1]) - plant_pole * ladrc_iq);
        pi_feedback.dat[1] = float2ctrl(pi_iq);
        ladrc_feedback.dat[1] = float2ctrl(ladrc_iq);

        if (time_s >= step_time_s)
        {
            update_metric(&pi_metric, time_s - step_time_s, pi_iq, target_q, dt);
            update_metric(&ladrc_metric, time_s - step_time_s, ladrc_iq, target_q, dt);
        }
    }

    pi_metric.overshoot_pct = 100.0f * (pi_metric.final_value - target_q) / target_q;
    ladrc_metric.overshoot_pct = 100.0f * (ladrc_metric.final_value - target_q) / target_q;
    pi_metric.final_value = ctrl2float(pi_feedback.dat[1]);
    ladrc_metric.final_value = ctrl2float(ladrc_feedback.dat[1]);

    printf("bandwidth_hz,controller,rise_10_90_ms,settling_2pct_ms,overshoot_pct,iae,final_iq_pu\n");
    printf("%.6f,PI,%.6f,%.6f,%.6f,%.9f,%.6f\n", pi_init.current_loop_bw,
           1000.0f * (pi_metric.rise_90_s - pi_metric.rise_10_s),
           1000.0f * pi_metric.settling_s, pi_metric.overshoot_pct, pi_metric.iae, pi_metric.final_value);
    printf("%.6f,LADRC1,%.6f,%.6f,%.6f,%.9f,%.6f\n", ladrc_init.ladrc_fcd,
           1000.0f * (ladrc_metric.rise_90_s - ladrc_metric.rise_10_s),
           1000.0f * ladrc_metric.settling_s, ladrc_metric.overshoot_pct, ladrc_metric.iae,
           ladrc_metric.final_value);

    if (!isfinite(pi_metric.final_value) || !isfinite(ladrc_metric.final_value) ||
        fabsf(pi_metric.final_value - target_q) > 0.005f ||
        fabsf(ladrc_metric.final_value - target_q) > 0.005f)
        return 1;
    return 0;
}
