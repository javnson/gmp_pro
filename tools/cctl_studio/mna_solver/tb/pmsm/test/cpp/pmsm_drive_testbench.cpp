#include "pmsmcircuit.hpp"

// Handwritten system testbench. cpp_codegen.py regenerates the circuit header and archive.

#include <cctl/circuit_model/pmsm_cs.hpp>
#include <ctl/hardware_preset/pmsm_motor/sm060r20b30mnad.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>

namespace
{

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDcBusVoltageV = 48.0;
constexpr double kCarrierFrequencyHz = 10.0e3;
constexpr double kTargetElectricalFrequencyHz = 20.0;
constexpr double kTargetModulation = 0.10;
constexpr double kFrequencyRampS = 500.0e-3;
constexpr double kDeadTimeS = 1.0e-6;
constexpr double kDurationS = 800.0e-3;
#if defined(CCTL_FIXED_MATH_USE_AVX) && CCTL_FIXED_MATH_USE_AVX
constexpr const char *kFixedKernel = "AVX2";
#else
constexpr const char *kFixedKernel = "scalar";
#endif

struct LegSignals
{
    std::uint32_t upper;
    std::uint32_t lower;
};

struct GateSignals
{
    std::uint32_t pwm1;
    std::uint32_t pwm2;
    std::uint32_t pwm3;
    std::uint32_t pwm4;
    std::uint32_t pwm5;
    std::uint32_t pwm6;
    double electrical_frequency_hz;
    double modulation;
};

double command_frequency(double time_s)
{
    return kTargetElectricalFrequencyHz * std::min(time_s / kFrequencyRampS, 1.0);
}

double command_angle(double time_s)
{
    if (time_s < kFrequencyRampS)
        return kPi * kTargetElectricalFrequencyHz * time_s * time_s / kFrequencyRampS;
    return 2.0 * kPi * kTargetElectricalFrequencyHz * (time_s - 0.5 * kFrequencyRampS);
}

LegSignals leg_signals(double carrier_phase_s, double duty)
{
    constexpr double carrier_period_s = 1.0 / kCarrierFrequencyHz;
    const double rising_edge_s = 0.5 * (1.0 - duty) * carrier_period_s;
    const double falling_edge_s = 0.5 * (1.0 + duty) * carrier_period_s;
    const bool upper = carrier_phase_s >= rising_edge_s + 0.5 * kDeadTimeS &&
                       carrier_phase_s < falling_edge_s - 0.5 * kDeadTimeS;
    const bool lower = carrier_phase_s < rising_edge_s - 0.5 * kDeadTimeS ||
                       carrier_phase_s >= falling_edge_s + 0.5 * kDeadTimeS;
    return {upper ? 1U : 0U, lower ? 1U : 0U};
}

GateSignals gate_signals(double time_s)
{
    constexpr double carrier_period_s = 1.0 / kCarrierFrequencyHz;
    double carrier_phase_s = std::fmod(time_s, carrier_period_s);
    if (carrier_phase_s < 0.0)
        carrier_phase_s += carrier_period_s;
    const double reference_time_s = time_s - carrier_phase_s + 0.5 * carrier_period_s;
    const double frequency = command_frequency(reference_time_s);
    const double modulation = kTargetModulation * std::min(reference_time_s / kFrequencyRampS, 1.0);
    const double angle = command_angle(reference_time_s);
    const LegSignals phase_a = leg_signals(
        carrier_phase_s, 0.5 + 0.5 * modulation * std::cos(angle));
    const LegSignals phase_b = leg_signals(
        carrier_phase_s, 0.5 + 0.5 * modulation * std::cos(angle - 2.0 * kPi / 3.0));
    const LegSignals phase_c = leg_signals(
        carrier_phase_s, 0.5 + 0.5 * modulation * std::cos(angle + 2.0 * kPi / 3.0));

    // PMSM.CIR maps motor A to PWM5/6, B to PWM3/4, and C to PWM1/2.
    return {phase_c.upper, phase_c.lower, phase_b.upper, phase_b.lower,
            phase_a.upper, phase_a.lower, frequency, modulation};
}

cctl::pmsm_cs_parameters<double> motor_parameters()
{
    cctl::pmsm_cs_parameters<double> parameters;
    parameters.sample_period_s = PmsmCircuit::normal_step_s;
    parameters.stator_resistance_ohm = SM060R20B30MNAD_RS;
    parameters.d_axis_inductance_h = SM060R20B30MNAD_LD;
    parameters.q_axis_inductance_h = SM060R20B30MNAD_LQ;
    parameters.permanent_magnet_flux_wb = SM060R20B30MNAD_FLUX;
    parameters.pole_pairs = static_cast<std::uint16_t>(SM060R20B30MNAD_POLE_PAIRS);
    // The preset stores g*cm^2 and uN*m*s/rad; pmsm_cs uses SI units.
    parameters.inertia_kg_m2 = SM060R20B30MNAD_INERTIA * 1.0e-7;
    parameters.viscous_friction_nm_s = SM060R20B30MNAD_FRICTION * 1.0e-6;
    return parameters;
}

bool verify_motor_electrical_model()
{
    const cctl::pmsm_cs_parameters<double> parameters = motor_parameters();
    cctl::pmsm_cs<double> motor(parameters);
    constexpr double test_voltage_v = 1.0;
    constexpr double test_duration_s = 5.0e-3;
    const std::size_t steps =
        static_cast<std::size_t>(test_duration_s / parameters.sample_period_s + 0.5);
    for (std::size_t index = 0U; index < steps; ++index)
        motor.step(test_voltage_v, -0.5 * test_voltage_v, -0.5 * test_voltage_v);

    const double expected_id = test_voltage_v / parameters.stator_resistance_ohm *
        (1.0 - std::exp(-parameters.stator_resistance_ohm * test_duration_s /
                        parameters.d_axis_inductance_h));
    const double current_sum = motor.output.phase_current_a[0] +
                               motor.output.phase_current_a[1] +
                               motor.output.phase_current_a[2];
    return std::abs(motor.output.d_axis_current_a - expected_id) < 1.0e-8 &&
           std::abs(motor.output.q_axis_current_a) < 1.0e-12 &&
           std::abs(motor.output.mechanical_speed_rad_s) < 1.0e-12 &&
           std::abs(current_sum) < 1.0e-12;
}

} // namespace

int main()
{
    if (!verify_motor_electrical_model())
    {
        std::cerr << "PMSM current-source model failed its locked-rotor RL check\n";
        return 1;
    }

    PmsmCircuit inverter;
    cctl::pmsm_cs<double> motor;
    motor.initialize(motor_parameters());

    constexpr std::size_t startup_short_steps = 200U;
    for (std::size_t index = 0U; index < startup_short_steps; ++index)
    {
        const GateSignals gate = gate_signals(double(index) * PmsmCircuit::short_step_s);
        PmsmCircuit::Inputs input;
        input.PWM1 = gate.pwm1;
        input.PWM2 = gate.pwm2;
        input.PWM3 = gate.pwm3;
        input.PWM4 = gate.pwm4;
        input.PWM5 = gate.pwm5;
        input.PWM6 = gate.pwm6;
        input.VS1 = kDcBusVoltageV;
        inverter.step_short(input);
    }

    std::ofstream csv("pmsm_cpp_open_loop_20hz.csv");
    if (!csv)
    {
        std::cerr << "cannot create pmsm_cpp_open_loop_20hz.csv\n";
        return 2;
    }
    csv << "time_s,command_frequency_hz,modulation,va_v,vb_v,vc_v,ia_a,ib_a,ic_a,"
           "id_a,iq_a,torque_nm,speed_rpm,electrical_frequency_hz,topology_index\n";
    csv << std::setprecision(17);

    const std::size_t normal_steps =
        static_cast<std::size_t>(kDurationS / PmsmCircuit::normal_step_s + 0.5);
    const std::size_t analysis_steps =
        static_cast<std::size_t>(50.0e-3 / PmsmCircuit::normal_step_s + 0.5);
    double electrical_frequency_sum = 0.0;
    double speed_rpm_sum = 0.0;
    double maximum_phase_current = 0.0;
    double maximum_current_sum = 0.0;
    std::size_t analysis_count = 0U;

    for (std::size_t index = 0U; index < normal_steps; ++index)
    {
        const double time_s = double(index) * PmsmCircuit::normal_step_s;
        const GateSignals gate = gate_signals(time_s);
        if ((gate.pwm1 && gate.pwm2) || (gate.pwm3 && gate.pwm4) ||
            (gate.pwm5 && gate.pwm6))
        {
            std::cerr << "complementary gate overlap at t=" << time_s << " s\n";
            return 3;
        }

        PmsmCircuit::Inputs inverter_input;
        inverter_input.PWM1 = gate.pwm1;
        inverter_input.PWM2 = gate.pwm2;
        inverter_input.PWM3 = gate.pwm3;
        inverter_input.PWM4 = gate.pwm4;
        inverter_input.PWM5 = gate.pwm5;
        inverter_input.PWM6 = gate.pwm6;
        inverter_input.IPMSM1_A = motor.output.phase_current_a[0];
        inverter_input.IPMSM1_B = motor.output.phase_current_a[1];
        inverter_input.IPMSM1_C = motor.output.phase_current_a[2];
        inverter_input.VS1 = kDcBusVoltageV;
        const PmsmCircuit::Outputs &inverter_output = inverter.step_normal(inverter_input);

        const cctl::pmsm_cs_output<double> &motor_output = motor.step(
            inverter_output.VPMSM1_A, inverter_output.VPMSM1_B,
            inverter_output.VPMSM1_C);
        const std::array<double, 3U> voltage = {
            inverter_output.VPMSM1_A, inverter_output.VPMSM1_B, inverter_output.VPMSM1_C};
        const std::array<double, 3U> current = {
            motor_output.phase_current_a[0], motor_output.phase_current_a[1],
            motor_output.phase_current_a[2]};

        for (std::size_t phase = 0U; phase < 3U; ++phase)
        {
            if (!std::isfinite(voltage[phase]) || !std::isfinite(current[phase]))
            {
                std::cerr << "non-finite coupled PMSM value at t=" << time_s << " s\n";
                return 4;
            }
            maximum_phase_current = std::max(maximum_phase_current, std::abs(current[phase]));
        }
        maximum_current_sum = std::max(
            maximum_current_sum, std::abs(current[0] + current[1] + current[2]));

        if (index % 1000U == 0U || index + 1U == normal_steps)
        {
            csv << time_s + PmsmCircuit::normal_step_s << ',' << gate.electrical_frequency_hz
                << ',' << gate.modulation << ',' << voltage[0] << ',' << voltage[1] << ','
                << voltage[2] << ',' << current[0] << ',' << current[1] << ',' << current[2]
                << ',' << motor_output.d_axis_current_a << ',' << motor_output.q_axis_current_a
                << ',' << motor_output.electromagnetic_torque_nm << ','
                << motor_output.mechanical_speed_rpm << ','
                << motor_output.electrical_frequency_hz << ','
                << inverter.last_topology_index() << '\n';
        }

        if (index + analysis_steps >= normal_steps)
        {
            electrical_frequency_sum += motor_output.electrical_frequency_hz;
            speed_rpm_sum += motor_output.mechanical_speed_rpm;
            ++analysis_count;
        }
    }

    const double mean_electrical_frequency =
        electrical_frequency_sum / static_cast<double>(analysis_count);
    const double mean_speed_rpm = speed_rpm_sum / static_cast<double>(analysis_count);
    std::cout << std::setprecision(12)
              << "PMSM current-source drive: Vdc=48 V, carrier=10 kHz, rotating voltage=20 Hz, "
                 "deadtime=1 us\n"
              << "matrix backend=" << PmsmCircuit::matrix_backend
              << ", CCTL fixed-vector kernel=" << kFixedKernel << '\n'
              << "mean final electrical frequency=" << mean_electrical_frequency
              << " Hz, mean speed=" << mean_speed_rpm << " rpm\n"
              << "final id/iq=" << motor.output.d_axis_current_a << "/"
              << motor.output.q_axis_current_a << " A, torque="
              << motor.output.electromagnetic_torque_nm << " N*m\n"
              << "maximum phase current=" << maximum_phase_current
              << " A, maximum ia+ib+ic=" << maximum_current_sum << " A\n";

    if (mean_electrical_frequency < 15.0 || mean_electrical_frequency > 25.0 ||
        mean_speed_rpm < 220.0 || mean_speed_rpm > 380.0)
    {
        std::cerr << "motor did not synchronize with the 20 Hz rotating voltage\n";
        return 5;
    }
    if (maximum_phase_current > 25.0 || maximum_current_sum > 1.0e-9)
    {
        std::cerr << "motor current is outside the expected three-wire region\n";
        return 6;
    }
    return 0;
}
