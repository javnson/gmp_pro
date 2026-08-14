#include <math.h>
#include <stdio.h>

#include <ctl/math_block/gmp_math.h>
#include <ctl/component/motor_control/consultant/acim_consultant.h>
#include <ctl/component/motor_control/consultant/pu_consultant.h>
#include <ctl/component/motor_control/observer/acim_fo.h>
#include <ctl/component/motor_control/observer/acim_pos_calc.h>
#include <ctl/component/motor_control/interface/sensorless_handover.h>
#include <ctl/component/motor_control/observer/pmsm_esmo.h>

#define TEST_FS (20000.0f)
#define TEST_W_BASE (CTL_PARAM_CONST_2PI * 50.0f)
#define TEST_V_BASE (48.0f / 1.73205080757f)
#define TEST_I_BASE (10.0f)

static float wrapped_error(float actual, float expected)
{
    float error = actual - expected;
    while (error >= 0.5f) error -= 1.0f;
    while (error < -0.5f) error += 1.0f;
    return error;
}

static void init_models(ctl_consultant_im_t* motor, ctl_consultant_pu_im_t* pu)
{
    ctl_consultant_im_init(motor, 2, 0.329f, 0.44f, 0.0015f, 0.0017f, 0.0012f);
    ctl_consultant_pu_im_init(pu, TEST_V_BASE, TEST_I_BASE, TEST_W_BASE, 2, 1.0f);
}

static int test_position_calculator(void)
{
    ctl_consultant_im_t motor;
    ctl_consultant_pu_im_t pu;
    ctl_im_pos_calc_t calc;
    const float id = 0.30f;
    const float iq = 0.10f;
    const float rotor_speed = 0.20f;
    float expected_slip;
    int k;

    init_models(&motor, &pu);
    ctl_init_im_pos_calc_consultant(&calc, &motor, &pu, TEST_FS);
    ctl_enable_im_pos_calc(&calc);
    for (k = 0; k < (int)TEST_FS; ++k)
        ctl_step_im_pos_calc(&calc, real2ctrl(id), real2ctrl(iq), real2ctrl(rotor_speed));

    expected_slip = iq / (id * motor.tau_r * pu.W_base);
    if (fabsf(ctrl2param(calc.i_md_pu) - id) > 2e-4f) return 1;
    if (fabsf(ctrl2param(calc.w_slip_pu) - expected_slip) > 2e-4f) return 2;
    if (fabsf(ctrl2param(calc.w_sync_pu) - rotor_speed - expected_slip) > 2e-4f) return 3;
    return 0;
}

static int test_ato(void)
{
    ctl_ato_pll_t ato;
    const float target_speed = 0.40f;
    float target_angle = 0.0f;
    int k;

    ctl_init_ato_pll(&ato, 20.0f, 1.0f, TEST_W_BASE, TEST_FS, 2.0f, -2.0f);
    for (k = 0; k < (int)(0.5f * TEST_FS); ++k)
    {
        float angle_err = wrapped_error(target_angle, ctrl2param(ato.elec_angle_pu));
        ctl_step_ato_pll(&ato, real2ctrl(sinf(CTL_PARAM_CONST_2PI * angle_err) / CTL_PARAM_CONST_2PI));
        target_angle += target_speed * TEST_W_BASE / (CTL_PARAM_CONST_2PI * TEST_FS);
        target_angle -= floorf(target_angle);
    }

    if (fabsf(ctrl2param(ato.elec_speed_pu) - target_speed) > 2e-3f) return 1;
    if (fabsf(wrapped_error(ctrl2param(ato.elec_angle_pu), target_angle)) > 2e-3f) return 2;
    return 0;
}

/* Reproduces the normalized PMSM ESMO/FO back-EMF phase detector. The
 * estimated angle and all target angles remain in electrical-turn PU. */
static int test_ato_pmsm_bemf_detector(void)
{
    ctl_ato_pll_t ato;
    const float target_speed = 0.35f;
    float target_angle = 0.0f;
    int k;

    ctl_init_ato_pll(&ato, 20.0f, 1.0f, TEST_W_BASE, TEST_FS, 2.0f, -2.0f);
    for (k = 0; k < (int)(0.6f * TEST_FS); ++k)
    {
        float theta = CTL_PARAM_CONST_2PI * target_angle;
        float theta_hat = CTL_PARAM_CONST_2PI * ctrl2param(ato.elec_angle_pu);
        float emf_mag = 0.05f + 0.75f * ((float)k / (0.6f * TEST_FS));
        float e_alpha = -emf_mag * sinf(theta);
        float e_beta = emf_mag * cosf(theta);
        float phase_err_pu = (-e_alpha * cosf(theta_hat) - e_beta * sinf(theta_hat)) /
                             (emf_mag * CTL_PARAM_CONST_2PI);

        ctl_step_ato_pll(&ato, real2ctrl(phase_err_pu));
        target_angle += target_speed * TEST_W_BASE / (CTL_PARAM_CONST_2PI * TEST_FS);
        target_angle -= floorf(target_angle);
    }

    if (fabsf(ctrl2param(ato.elec_speed_pu) - target_speed) > 2e-3f) return 1;
    if (fabsf(wrapped_error(ctrl2param(ato.elec_angle_pu), target_angle)) > 2e-3f) return 2;
    return 0;
}

static int test_flux_observer_steady_state(void)
{
    ctl_consultant_im_t motor;
    ctl_consultant_pu_im_t pu;
    ctl_im_fo_t fo;
    const float id_pu = 0.8f;
    const float iq_pu = 0.1f;
    const float rotor_speed_pu = 0.2f;
    float psi_r;
    float slip;
    float sync_speed_pu;
    float psi_sd;
    float psi_sq;
    float angle = 0.0f;
    int k;

    init_models(&motor, &pu);
    ctl_init_im_fo_consultant(&fo, &motor, &pu, TEST_FS, 10.0f, 20.0f, 20.0f);
    ctl_enable_im_fo(&fo);

    psi_r = motor.Lm * (id_pu * pu.I_s_base);
    slip = (iq_pu / id_pu) / motor.tau_r;
    sync_speed_pu = rotor_speed_pu + slip / pu.W_base;
    psi_sd = (motor.Lm / motor.Lr) * psi_r + motor.sigma_Ls * id_pu * pu.I_s_base;
    psi_sq = motor.sigma_Ls * iq_pu * pu.I_s_base;

    for (k = 0; k < (int)(1.0f * TEST_FS); ++k)
    {
        float c = cosf(CTL_PARAM_CONST_2PI * angle);
        float s = sinf(CTL_PARAM_CONST_2PI * angle);
        float ia = id_pu * c - iq_pu * s;
        float ib = id_pu * s + iq_pu * c;
        float psi_sa = psi_sd * c - psi_sq * s;
        float psi_sb = psi_sd * s + psi_sq * c;
        float omega_sync = sync_speed_pu * pu.W_base;
        float va = (motor.Rs * ia * pu.I_s_base - omega_sync * psi_sb) / pu.V_s_base;
        float vb = (motor.Rs * ib * pu.I_s_base + omega_sync * psi_sa) / pu.V_s_base;

        ctl_step_im_fo(&fo, real2ctrl(va), real2ctrl(vb), real2ctrl(ia), real2ctrl(ib));
        angle += sync_speed_pu * pu.W_base / (CTL_PARAM_CONST_2PI * TEST_FS);
        angle -= floorf(angle);
    }

    if (!fo.flag_observer_locked) return 1;
    if (fabsf(ctrl2param(fo.psi_r_mag) - psi_r / pu.Flux_s_base) > 0.02f) return 2;
    if (fabsf(ctrl2param(fo.spd_out.speed) - rotor_speed_pu) > 0.03f)
    {
        printf("FO speed=%g expected=%g sync=%g flux=%g expected_flux=%g angle_err=%g\n",
               ctrl2param(fo.spd_out.speed), rotor_speed_pu, ctrl2param(fo.ato_pll.elec_speed_pu),
               ctrl2param(fo.psi_r_mag), psi_r / pu.Flux_s_base,
               wrapped_error(ctrl2param(fo.pos_out.elec_position), angle));
        return 3;
    }
    if (fabsf(wrapped_error(ctrl2param(fo.pos_out.elec_position), angle)) > 0.02f) return 4;
    return 0;
}

static int test_sensorless_handover(void)
{
    ctl_sensorless_handover_t handover;
    rotation_ift pos_a = {CTL_CTRL_CONST_ZERO, real2ctrl(0.99f), 0};
    rotation_ift pos_b = {CTL_CTRL_CONST_ZERO, real2ctrl(0.01f), 0};
    velocity_ift spd_a = {real2ctrl(0.30f)};
    velocity_ift spd_b = {real2ctrl(0.36f)};
    float previous_angle;
    int k;

    ctl_init_sensorless_handover(&handover, 0.1f, 1000.0f,
                                 real2ctrl(0.4f), real2ctrl(0.3f),
                                 real2ctrl(0.1f), real2ctrl(0.1f),
                                 real2ctrl(0.3f));
    ctl_attach_sensorless_handover(&handover, &pos_a, &spd_a, &pos_b, &spd_b);
    ctl_configure_sensorless_handover_speed_qualification(
        &handover, 0.01f, 0.02f, 0.005f, 1000.0f);
    ctl_request_sensorless_handover(&handover, 1);

    for (k = 0; k < 10; ++k)
        ctl_step_sensorless_handover(&handover, spd_a.speed);
    if (handover.angle.state != ANGLE_SWITCH_IDLE_A) return 1;

    spd_b.speed = real2ctrl(0.305f);
    for (k = 0; k < 3; ++k)
        ctl_step_sensorless_handover(&handover, spd_a.speed);
    spd_b.speed = real2ctrl(0.315f); /* Hysteresis band: retain count. */
    ctl_step_sensorless_handover(&handover, spd_a.speed);
    spd_b.speed = real2ctrl(0.305f);
    previous_angle = ctrl2param(handover.angle.out_enc.elec_position);
    ctl_step_sensorless_handover(&handover, spd_a.speed);
    ctl_step_sensorless_handover(&handover, spd_a.speed);
    if (handover.angle.state != ANGLE_SWITCH_TRANS_A2B) return 2;
    if (fabsf(wrapped_error(ctrl2param(handover.angle.out_enc.elec_position),
                            previous_angle)) > 0.001f) return 3;

    for (k = 0; k < 110; ++k)
    {
        pos_b.elec_position = real2ctrl(fmodf(ctrl2param(pos_b.elec_position) + 0.0003f, 1.0f));
        ctl_step_sensorless_handover(&handover, spd_a.speed);
    }
    if (handover.angle.state != ANGLE_SWITCH_IDLE_B) return 4;
    if (fabsf(ctrl2param(handover.id_ref_out) - 0.1f) > 1e-5f) return 5;
    return 0;
}

static int test_esmo_normalization_flag(void)
{
    ctl_pmsm_esmo_t esmo = {0};
    ctl_enable_pmsm_esmo_emf_normalization(&esmo);
    if (!esmo.flag_emf_normalize) return 1;
    ctl_disable_pmsm_esmo_emf_normalization(&esmo);
    return esmo.flag_emf_normalize ? 2 : 0;
}

int main(void)
{
    int result = test_position_calculator();
    if (result) { printf("ACIM position calculator test failed: %d\n", result); return 10 + result; }

    result = test_ato();
    if (result) { printf("ATO test failed: %d\n", result); return 20 + result; }

    result = test_ato_pmsm_bemf_detector();
    if (result) { printf("PMSM BEMF/ATO test failed: %d\n", result); return 25 + result; }

    result = test_flux_observer_steady_state();
    if (result) { printf("ACIM flux observer test failed: %d\n", result); return 30 + result; }

    result = test_sensorless_handover();
    if (result) { printf("Sensorless handover test failed: %d\n", result); return 40 + result; }

    result = test_esmo_normalization_flag();
    if (result) { printf("ESMO normalization flag test failed: %d\n", result); return 50 + result; }

    puts("ACIM observer host tests passed");
    return 0;
}
