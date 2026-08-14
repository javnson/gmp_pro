/**
 * @file interface_gtests.cpp
 * @brief Portable Google Test coverage for CTL interface components.
 */

#include <gtest/gtest.h>

#include <ctl/portable/gmp_ctl_portable.h>
#include <ctl/component/interface/adc_channel.h>
#include <ctl/component/interface/bias_model.h>
#include <ctl/component/interface/dac_channel.h>
#include <ctl/component/interface/gain_model.h>
#include <ctl/component/interface/pwm_channel.h>
#include <ctl/component/interface/spwm_modulator.h>

namespace
{
constexpr double tolerance = 2.0e-5;

/** @brief Convert a control value to the hosted comparison type. */
double value_of(ctrl_gt value)
{
    return static_cast<double>(ctrl2param(value));
}
} // namespace

/** @brief Return a deterministic portable tick for hosted tests. */
time_gt gmp_ctl_portable_get_tick(void)
{
    return 0U;
}

/** @brief Verify interface gain and bias models. */
TEST(InterfaceModels, GainAndBiasEquationsAreCorrect)
{
    EXPECT_NEAR(ctl_gain_calc_generic(3.3, 0.01, 330.0), 1.0, 1.0e-12);
    EXPECT_NEAR(ctl_gain_calc_volt_divider(3.3, 330.0, 9000.0, 1000.0), 0.1, 1.0e-12);
    EXPECT_NEAR(ctl_bias_calc_via_Vref_Vbias(3.3, 1.65), 0.5, 1.0e-12);
    EXPECT_NEAR(ctl_bias_calc_via_res_divider(1000.0, 1000.0), 0.5, 1.0e-12);
}

/** @brief Verify ADC and DAC conversion channels. */
TEST(ConversionChannels, AdcAndDacScalingAreCorrect)
{
    adc_channel_t adc{};
    ctl_init_adc_channel(&adc, real2ctrl(2.0), real2ctrl(0.25), 12, 24);
    EXPECT_NEAR(value_of(ctl_step_adc_channel(&adc, static_cast<adc_gt>(2048))), 0.5, tolerance);

    dac_channel_t dac{};
    ctl_init_dac_channel(&dac, real2ctrl(0.5), real2ctrl(0.25), 12, 24);
    ctl_set_dac_channel_input(&dac, real2ctrl(0.5));
    EXPECT_NEAR(static_cast<double>(ctl_step_dac_channel(&dac)), 2047.5, 0.5);
}

/** @brief Verify single-channel PWM and SPWM mappings. */
TEST(PwmInterfaces, SingleAndThreePhaseZeroVectorScalingAreCorrect)
{
    pwm_channel_t pwm{};
    ctl_init_pwm_channel(&pwm, 0, 1000);
    EXPECT_EQ(ctl_step_pwm_channel(&pwm, real2ctrl(0.25)), 250);
    EXPECT_EQ(ctl_step_pwm_channel(&pwm, real2ctrl(1.5)), 1000);

    ctl_vector3_t currents{};
    spwm_modulator_t modulator{};
    ctl_init_spwm_modulator(&modulator, 1000, 20, &currents, real2ctrl(0.05), real2ctrl(0.01));
    ctl_step_spwm_modulator(&modulator);
    EXPECT_EQ(modulator.pwm_out[0], 500);
    EXPECT_EQ(modulator.pwm_out[1], 500);
    EXPECT_EQ(modulator.pwm_out[2], 500);
}
