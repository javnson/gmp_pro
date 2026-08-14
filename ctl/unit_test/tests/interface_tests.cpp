/**
 * @file interface_tests.cpp
 * @brief Visual Studio tests for CTL hardware-interface adapters and models.
 */

#include "vs_test_support.h"

#include <ctl/portable/gmp_ctl_portable.h>
#include <ctl/component/interface/adc_channel.h>
#include <ctl/component/interface/bias_model.h>
#include <ctl/component/interface/dac_channel.h>
#include <ctl/component/interface/gain_model.h>
#include <ctl/component/interface/interface_base.h>
#include <ctl/component/interface/pwm_channel.h>
#include <ctl/component/interface/spwm_modulator.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace gmp_ctl_unit_test
{
namespace
{
constexpr double interface_tolerance = 2.0e-5;

double interface_value(ctrl_gt value)
{
    return static_cast<double>(ctrl2param(value));
}
} // namespace

TEST_CLASS(InterfaceModelTests)
{
  public:
    /** @brief Verify the supported sensor gain equations. */
    TEST_METHOD(GainModelsMatchTheirCircuitEquations)
    {
        expect_near(static_cast<double>(ctl_gain_calc_generic(3.3, 0.01, 330.0)), 1.0, 1.0e-12,
                    L"generic gain model failed");
        expect_near(static_cast<double>(ctl_gain_calc_volt_divider(3.3, 330.0, 9000.0, 1000.0)), 0.1, 1.0e-12,
                    L"voltage-divider gain model failed");
        expect_near(static_cast<double>(ctl_gain_calc_shunt_amp(3.3, 33.0, 0.01, 10.0)), 1.0, 1.0e-12,
                    L"shunt-amplifier gain model failed");
    }

    /** @brief Verify voltage and resistor-divider bias equations. */
    TEST_METHOD(BiasModelsReturnPerUnitOffsets)
    {
        expect_near(static_cast<double>(ctl_bias_calc_via_Vref_Vbias(3.3, 1.65)), 0.5, 1.0e-12,
                    L"voltage-reference bias model failed");
        expect_near(static_cast<double>(ctl_bias_calc_via_res_divider(1000.0, 1000.0)), 0.5, 1.0e-12,
                    L"resistor-divider bias model failed");
    }
};

TEST_CLASS(ConversionChannelTests)
{
  public:
    /** @brief Verify ADC normalization, bias removal, and gain. */
    TEST_METHOD(AdcChannelAppliesResolutionBiasAndGain)
    {
        adc_channel_t adc{};
        ctl_init_adc_channel(&adc, real2ctrl(2.0), real2ctrl(0.25), 12, 24);
        expect_near(interface_value(ctl_step_adc_channel(&adc, static_cast<adc_gt>(2048))), 0.5,
                    interface_tolerance, L"ADC scaling failed");
        Assert::IsTrue(ctl_get_adc_channel_ctrl_port(&adc) == &adc.control_port);
    }

    /** @brief Verify DAC gain, bias, and quantization. */
    TEST_METHOD(DacChannelAppliesGainBiasAndResolution)
    {
        dac_channel_t dac{};
        ctl_init_dac_channel(&dac, real2ctrl(0.5), real2ctrl(0.25), 12, 24);
        ctl_set_dac_channel_input(&dac, real2ctrl(0.5));
        const dac_gt output = ctl_step_dac_channel(&dac);
        Assert::IsTrue(output >= static_cast<dac_gt>(2047) && output <= static_cast<dac_gt>(2048),
                       L"DAC scaling failed");
        Assert::AreEqual(output, ctl_get_dac_channel_value(&dac));
    }
};

TEST_CLASS(PwmInterfaceTests)
{
  public:
    /** @brief Verify single-channel PWM scaling and saturation. */
    TEST_METHOD(PwmChannelScalesAndSaturatesDutyCycles)
    {
        pwm_channel_t pwm{};
        ctl_init_pwm_channel(&pwm, 0, 1000);
        Assert::AreEqual<pwm_gt>(250, ctl_step_pwm_channel(&pwm, real2ctrl(0.25)));
        Assert::AreEqual<pwm_gt>(1000, ctl_step_pwm_channel(&pwm, real2ctrl(1.5)));
        Assert::AreEqual<pwm_gt>(0, ctl_step_pwm_channel(&pwm, real2ctrl(-0.5)));
        Assert::AreEqual<pwm_gt>(750, ctl_step_pwm_channel_inv(&pwm, real2ctrl(0.25)));
    }

    /** @brief Verify the SPWM zero-vector mapping. */
    TEST_METHOD(SpwmZeroVectorProducesHalfScaleOutputs)
    {
        ctl_vector3_t currents{};
        spwm_modulator_t modulator{};
        ctl_init_spwm_modulator(&modulator, 1000, 20, &currents, real2ctrl(0.05), real2ctrl(0.01));
        modulator.vab0_out.dat[0] = CTL_CTRL_CONST_ZERO;
        modulator.vab0_out.dat[1] = CTL_CTRL_CONST_ZERO;
        modulator.vab0_out.dat[2] = CTL_CTRL_CONST_ZERO;
        ctl_step_spwm_modulator(&modulator);
        Assert::AreEqual<pwm_gt>(500, modulator.pwm_out[0]);
        Assert::AreEqual<pwm_gt>(500, modulator.pwm_out[1]);
        Assert::AreEqual<pwm_gt>(500, modulator.pwm_out[2]);
    }
};
} // namespace gmp_ctl_unit_test
