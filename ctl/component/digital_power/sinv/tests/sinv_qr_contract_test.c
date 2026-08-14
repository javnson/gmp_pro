/**
 * @file sinv_qr_contract_test.c
 * @brief Verifies the memory and numeric contract of the SINV harmonic QR branches.
 */

#include <stdio.h>

#include <ctl/component/intrinsic/discrete/proportional_resonant.h>

_Static_assert(sizeof(qr_ctrl_t) < sizeof(qpr_ctrl_t),
               "A harmonic QR branch must not store the unused QPR proportional gain");

/**
 * @brief Compares a QR branch with the equivalent zero-Kp QPR branch.
 * @return Zero when the two resonant responses match.
 */
int main(void)
{
    qr_ctrl_t qr;
    qpr_ctrl_t qpr;
    const parameter_gt kr = real2param(0.75);
    const parameter_gt resonant_frequency = real2param(250.0);
    const parameter_gt cutoff_frequency = real2param(2.0);
    const parameter_gt sampling_frequency = real2param(20000.0);
    const parameter_gt tolerance = real2param(1.0e-6);
    unsigned int sample;

    ctl_init_qr_controller(&qr, kr, resonant_frequency, cutoff_frequency, sampling_frequency);
    ctl_init_qpr_controller(&qpr, CTL_PARAM_CONST_ZERO, kr, resonant_frequency, cutoff_frequency,
                            sampling_frequency);

    for (sample = 0U; sample < 64U; ++sample)
    {
        const ctrl_gt input = (sample < 8U) ? CTL_CTRL_CONST_1 : CTL_CTRL_CONST_ZERO;
        const ctrl_gt qr_output = ctl_step_qr_controller(&qr, input);
        const ctrl_gt qpr_output = ctl_step_qpr_controller(&qpr, input);

        if (param_abs(ctrl2param(qr_output) - ctrl2param(qpr_output)) > tolerance)
        {
            printf("FAIL: QR and zero-Kp QPR differ at sample %u\n", sample);
            return 1;
        }
    }

    printf("PASS: harmonic QR contract; QR=%zu bytes, QPR=%zu bytes\n", sizeof(qr_ctrl_t), sizeof(qpr_ctrl_t));
    return 0;
}
