#include <gmp_core.h>

#include <math.h>
#include <stdio.h>

#include <ctl/component/digital_power/inv/inv_voltage_ctrl.h>
#include <ctl/component/digital_power/inv/inv_zero_ctrl.h>
#include <ctl/math_block/coordinate/svpwm_3d.h>

static int check_svpwm_3d(void)
{
    const int samples = 20000;
    ctrl_gt max_error = 0.0f;
    ctrl_gt duty_min = 1.0f;
    ctrl_gt duty_max = 0.0f;
    int k;

    for (k = 0; k < samples; ++k)
    {
        parameter_gt theta = CTL_PARAM_CONST_2PI * (parameter_gt)k / (parameter_gt)samples;
        ctl_vector3_t reference;
        ctl_vector3_t reconstructed_abc;
        ctl_vector3_t reconstructed_ab0;
        ctl_vector4_t duty;
        int leg;

        reference.dat[phase_alpha] = float2ctrl(0.35f * cosf(theta));
        reference.dat[phase_beta] = float2ctrl(0.35f * sinf(theta));
        reference.dat[phase_0] = float2ctrl(0.08f * sinf(3.0f * theta));

        ctl_ct_svpwm_3d_calc(&reference, &duty);

        for (leg = 0; leg < 4; ++leg)
        {
            if (duty.dat[leg] < duty_min)
                duty_min = duty.dat[leg];
            if (duty.dat[leg] > duty_max)
                duty_max = duty.dat[leg];
        }

        reconstructed_abc.dat[phase_A] = duty.dat[phase_A] - duty.dat[phase_N];
        reconstructed_abc.dat[phase_B] = duty.dat[phase_B] - duty.dat[phase_N];
        reconstructed_abc.dat[phase_C] = duty.dat[phase_C] - duty.dat[phase_N];
        ctl_ct_clarke(&reconstructed_abc, &reconstructed_ab0);

        for (leg = 0; leg < 3; ++leg)
        {
            ctrl_gt error = ctl_abs(reconstructed_ab0.dat[leg] - reference.dat[leg]);
            if (error > max_error)
                max_error = error;
        }
    }

    printf("svpwm_3d: max_reconstruction_error=%.9g duty_range=[%.6f, %.6f]\n",
           ctrl2float(max_error), ctrl2float(duty_min), ctrl2float(duty_max));
    return !((max_error < float2ctrl(2.0e-6f)) && (duty_min >= 0.0f) && (duty_max <= 1.0f));
}

typedef struct _tag_voltage_metrics
{
    parameter_gt vd_error;
    parameter_gt vq_error;
    parameter_gt q_rms;
} voltage_metrics_t;

static voltage_metrics_t simulate_voltage_loop(fast_gt decouple)
{
    const parameter_gt fs = 20000.0f;
    const parameter_gt dt = 1.0f / fs;
    const parameter_gt omega = CTL_PARAM_CONST_2PI * 50.0f;
    const parameter_gt v_base = 325.0f;
    const parameter_gt i_base = 20.0f;
    const parameter_gt capacitance = 20.0e-6f;
    const parameter_gt load_resistance = 32.5f;
    const parameter_gt cap_gain = i_base / (capacitance * v_base);
    const parameter_gt load_gain = v_base / (load_resistance * i_base);
    const parameter_gt current_alpha = dt * CTL_PARAM_CONST_2PI * 1500.0f;
    const int total = (int)(0.30f * fs);
    const int rms_start = (int)(0.20f * fs);
    inv_voltage_ctrl_init_t init;
    inv_voltage_ctrl_t controller;
    ctl_vector2_t vab;
    ctl_vector2_t phasor;
    ctl_vector2_t idq_ref;
    parameter_gt vd = 0.0f;
    parameter_gt vq = 0.0f;
    parameter_gt id = 0.0f;
    parameter_gt iq = 0.0f;
    parameter_gt q_sq = 0.0f;
    int q_count = 0;
    int k;
    voltage_metrics_t metrics;

    init.fs = fs;
    init.freq_base = 50.0f;
    init.v_base = v_base;
    init.i_base = i_base;
    init.filter_C = capacitance;
    init.voltage_loop_bw = 200.0f;
    init.voltage_loop_zero = 40.0f;
    init.current_limit_max = 2.0f;
    init.current_limit_min = -2.0f;

    ctl_init_voltage_inv(&controller, &init);
    ctl_attach_voltage_inv(&controller, &vab, &phasor, &idq_ref);
    ctl_set_voltage_inv_reference(&controller, float2ctrl(1.0f), float2ctrl(0.0f));
    controller.flag_enable_decouple = decouple;
    ctl_enable_voltage_inv(&controller);

    for (k = 0; k < total; ++k)
    {
        parameter_gt theta = omega * dt * (parameter_gt)k;
        ctl_vector2_t vdq;

        phasor.dat[phasor_sin] = float2ctrl(sinf(theta));
        phasor.dat[phasor_cos] = float2ctrl(cosf(theta));
        vdq.dat[phase_d] = float2ctrl(vd);
        vdq.dat[phase_q] = float2ctrl(vq);
        ctl_ct_ipark2(&vdq, &phasor, &vab);

        ctl_step_voltage_inv_ctrl(&controller);

        id += current_alpha * (ctrl2float(idq_ref.dat[phase_d]) - id);
        iq += current_alpha * (ctrl2float(idq_ref.dat[phase_q]) - iq);

        {
            parameter_gt dvd = cap_gain * (id - load_gain * vd) + omega * vq;
            parameter_gt dvq = cap_gain * (iq - load_gain * vq) - omega * vd;
            vd += dt * dvd;
            vq += dt * dvq;
        }

        if (k >= rms_start)
        {
            q_sq += vq * vq;
            ++q_count;
        }
    }

    metrics.vd_error = fabsf(1.0f - vd);
    metrics.vq_error = fabsf(vq);
    metrics.q_rms = sqrtf(q_sq / (parameter_gt)q_count);
    return metrics;
}

static int check_voltage_loop(void)
{
    voltage_metrics_t coupled = simulate_voltage_loop(0);
    voltage_metrics_t decoupled = simulate_voltage_loop(1);

    printf("voltage_loop: decoupled vd_err=%.6g vq_err=%.6g q_rms=%.6g; "
           "disabled_ff q_rms=%.6g\n",
           decoupled.vd_error, decoupled.vq_error, decoupled.q_rms, coupled.q_rms);

    return !((decoupled.vd_error < 0.015f) && (decoupled.vq_error < 0.015f) &&
             (decoupled.q_rms < coupled.q_rms));
}

static parameter_gt simulate_zero_loop(fast_gt enabled)
{
    const parameter_gt fs = 20000.0f;
    const parameter_gt dt = 1.0f / fs;
    const parameter_gt omega = CTL_PARAM_CONST_2PI * 50.0f;
    const parameter_gt v_base = 325.0f;
    const parameter_gt i_base = 20.0f;
    const parameter_gt inductance = 2.0e-3f;
    const parameter_gt resistance = 0.5f;
    const parameter_gt plant_gain = v_base / (inductance * i_base);
    const parameter_gt damping = resistance / inductance;
    const int total = (int)(0.50f * fs);
    const int rms_start = (int)(0.30f * fs);
    inv_zero_ctrl_init_t init;
    inv_zero_ctrl_t controller;
    ctrl_gt i0_port = 0.0f;
    ctrl_gt v0_port = 0.0f;
    parameter_gt current = 0.0f;
    parameter_gt sum_sq = 0.0f;
    int count = 0;
    int k;

    init.fs = fs;
    init.kp = 0.62f;
    init.kr = 3.10f;
    init.freq_resonant = 50.0f;
    init.freq_cut = 5.0f;
    init.tune_mode = CTL_TUNE_QR_PREWARPED;
    init.output_limit_max = 0.5f;
    init.output_limit_min = -0.5f;

    ctl_init_zero_inv(&controller, &init);
    ctl_attach_zero_inv(&controller, &i0_port, &v0_port);
    if (enabled)
        ctl_enable_zero_inv(&controller);

    for (k = 0; k < total; ++k)
    {
        parameter_gt time = dt * (parameter_gt)k;
        parameter_gt disturbance_pu = 0.06f * sinf(omega * time);

        i0_port = float2ctrl(current);
        ctl_step_zero_inv_ctrl(&controller);
        current += dt * (plant_gain * (ctrl2float(v0_port) + disturbance_pu) - damping * current);

        if (k >= rms_start)
        {
            sum_sq += current * current;
            ++count;
        }
    }

    if (enabled)
    {
        ctl_tune_zero_inv_ctrl(&controller, 0.70f, 3.50f, 50.0f, 6.0f,
                               CTL_TUNE_QR_PREWARPED, fs);
        if (!controller.flag_tune_pending)
            return 1.0e6f;
        ctl_step_zero_inv_ctrl(&controller);
        if (controller.flag_tune_pending || fabsf(ctrl2float(controller.qpr.kp) - 0.70f) > 1.0e-6f)
            return 1.0e6f;
    }

    return sqrtf(sum_sq / (parameter_gt)count);
}

static int check_zero_loop(void)
{
    parameter_gt disabled_rms = simulate_zero_loop(0);
    parameter_gt enabled_rms = simulate_zero_loop(1);
    parameter_gt attenuation = disabled_rms / enabled_rms;

    printf("zero_sequence: disabled_rms=%.6g enabled_rms=%.6g attenuation=%.3fx\n",
           disabled_rms, enabled_rms, attenuation);
    return !(attenuation > 8.0f);
}

int main(void)
{
    int failures = 0;

    failures += check_svpwm_3d();
    failures += check_voltage_loop();
    failures += check_zero_loop();

    if (failures != 0)
    {
        fprintf(stderr, "FAILED: %d inverter host simulation check(s)\n", failures);
        return 1;
    }

    printf("PASS: all inverter host simulation checks passed\n");
    return 0;
}
