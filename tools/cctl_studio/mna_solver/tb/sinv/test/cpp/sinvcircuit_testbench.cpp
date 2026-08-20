#include "sinvcircuit.hpp"

// Handwritten case-specific testbench; cpp_codegen.py only regenerates the header.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kCarrierFrequencyHz = 10.0e3;
constexpr double kFundamentalFrequencyHz = 50.0;
constexpr double kModulationIndex = 0.8;
constexpr double kDeadTimeS = 1.0e-6;

struct GateSignals {
    std::uint32_t pwm1;
    std::uint32_t pwm2;
    std::uint32_t pwm3;
    std::uint32_t pwm4;
    double duty;
};

GateSignals gate_signals(double time_s) {
    constexpr double carrier_period_s = 1.0 / kCarrierFrequencyHz;
    double phase_s = std::fmod(time_s, carrier_period_s);
    if (phase_s < 0.0) {
        phase_s += carrier_period_s;
    }

    // Sample the 50 Hz reference at the centre of each carrier period. This gives
    // centre-aligned bipolar SPWM without moving either edge during that period.
    const double carrier_start_s = time_s - phase_s;
    const double reference_time_s = carrier_start_s + carrier_period_s / 2.0;
    const double duty = 0.5 + 0.5 * kModulationIndex *
                                  std::sin(2.0 * kPi * kFundamentalFrequencyHz * reference_time_s);
    const double rising_edge_s = 0.5 * (1.0 - duty) * carrier_period_s;
    const double falling_edge_s = 0.5 * (1.0 + duty) * carrier_period_s;

    // Each ideal commutation is replaced by a 1 us both-off interval. PWM1/PWM3
    // form one diagonal, while PWM2/PWM4 form the opposite diagonal.
    const bool diagonal_13 =
        phase_s >= rising_edge_s + kDeadTimeS / 2.0 &&
        phase_s < falling_edge_s - kDeadTimeS / 2.0;
    const bool diagonal_24 =
        phase_s < rising_edge_s - kDeadTimeS / 2.0 ||
        phase_s >= falling_edge_s + kDeadTimeS / 2.0;
    return {
        diagonal_13 ? 1U : 0U,
        diagonal_24 ? 1U : 0U,
        diagonal_13 ? 1U : 0U,
        diagonal_24 ? 1U : 0U,
        duty,
    };
}

}  // namespace

int main() {
    SinvCircuit circuit;
    constexpr double dc_bus_v = 30.0;
    constexpr double duration_s = 100.0e-3;
    constexpr double normal_dt_s = SinvCircuit::normal_step_s;
    constexpr double short_dt_s = SinvCircuit::short_step_s;
    constexpr std::size_t startup_short_steps = 200;
    constexpr std::size_t normal_steps =
        static_cast<std::size_t>(duration_s / normal_dt_s + 0.5);
    constexpr std::size_t carrier_samples =
        static_cast<std::size_t>((1.0 / kCarrierFrequencyHz) / normal_dt_s + 0.5);
    constexpr std::size_t analysis_samples =
        static_cast<std::size_t>((2.0 / kFundamentalFrequencyHz) / normal_dt_s + 0.5);

    double startup_time_s = 0.0;
    for (std::size_t index = 0; index < startup_short_steps; ++index) {
        const auto gate = gate_signals(startup_time_s);
        circuit.step_short(gate.pwm1, gate.pwm2, gate.pwm3, gate.pwm4, dc_bus_v);
        startup_time_s += short_dt_s;
    }

    std::ofstream csv("sinv_cpp_spwm.csv");
    if (!csv) {
        std::cerr << "cannot create sinv_cpp_spwm.csv\n";
        return 2;
    }
    csv << "time_s,PWM1,PWM2,PWM3,PWM4,duty,V(2,1),I(VAM1),topology_index\n";
    csv << std::setprecision(17);

    double voltage_sum = 0.0;
    double voltage_square_sum = 0.0;
    double voltage_sine_sum = 0.0;
    double voltage_cosine_sum = 0.0;
    double current_square_sum = 0.0;
    double current_sine_sum = 0.0;
    double current_cosine_sum = 0.0;
    double voltage_min = std::numeric_limits<double>::infinity();
    double voltage_max = -std::numeric_limits<double>::infinity();
    std::size_t analysis_count = 0;
    std::size_t dead_samples = 0;

    for (std::size_t index = 0; index < normal_steps; ++index) {
        const double time_s = static_cast<double>(index) * normal_dt_s;
        const auto gate = gate_signals(time_s);
        if (gate.pwm1 != gate.pwm3 || gate.pwm2 != gate.pwm4) {
            std::cerr << "diagonal gate mismatch at t=" << time_s << " s\n";
            return 3;
        }
        if ((gate.pwm1 != 0U && gate.pwm2 != 0U) ||
            (gate.pwm4 != 0U && gate.pwm3 != 0U)) {
            std::cerr << "bridge-leg gate overlap at t=" << time_s << " s\n";
            return 4;
        }

        const auto& output =
            circuit(gate.pwm1, gate.pwm2, gate.pwm3, gate.pwm4, dc_bus_v);
        const double voltage = output["V(2,1)"];
        const double current = output["I(VAM1)"];
        if (!std::isfinite(voltage) || !std::isfinite(current)) {
            std::cerr << "non-finite circuit output at t=" << time_s << " s\n";
            return 5;
        }
        if (index % 100U == 0U || index + 1U == normal_steps) {
            csv << time_s + normal_dt_s << ',' << gate.pwm1 << ',' << gate.pwm2 << ','
                << gate.pwm3 << ',' << gate.pwm4 << ',' << gate.duty << ',' << voltage
                << ',' << current << ',' << circuit.last_topology_index() << '\n';
        }

        if (index + carrier_samples >= normal_steps && gate.pwm1 == 0U && gate.pwm2 == 0U) {
            ++dead_samples;
        }
        if (index + analysis_samples >= normal_steps) {
            const double sample_time_s = time_s + normal_dt_s;
            const double angle = 2.0 * kPi * kFundamentalFrequencyHz * sample_time_s;
            voltage_sum += voltage;
            voltage_square_sum += voltage * voltage;
            voltage_sine_sum += voltage * std::sin(angle);
            voltage_cosine_sum += voltage * std::cos(angle);
            current_square_sum += current * current;
            current_sine_sum += current * std::sin(angle);
            current_cosine_sum += current * std::cos(angle);
            voltage_min = std::min(voltage_min, voltage);
            voltage_max = std::max(voltage_max, voltage);
            ++analysis_count;
        }
    }

    const double count = static_cast<double>(analysis_count);
    const double voltage_mean = voltage_sum / count;
    const double voltage_rms = std::sqrt(voltage_square_sum / count);
    const double current_rms = std::sqrt(current_square_sum / count);
    const double voltage_fundamental_peak =
        2.0 * std::hypot(voltage_sine_sum, voltage_cosine_sum) / count;
    const double current_fundamental_peak =
        2.0 * std::hypot(current_sine_sum, current_cosine_sum) / count;

    std::cout << std::setprecision(12)
              << "single-phase inverter: Vdc=30 V, carrier=10 kHz, sine=50 Hz, m=0.8, "
                 "deadtime=1 us\n"
              << "V(2,1): mean=" << voltage_mean << " V, rms=" << voltage_rms
              << " V, fundamental_peak=" << voltage_fundamental_peak << " V, min="
              << voltage_min << " V, max=" << voltage_max << " V\n"
              << "I(VAM1): rms=" << current_rms
              << " A, fundamental_peak=" << current_fundamental_peak << " A\n"
              << "last-carrier both-off samples=" << dead_samples << '\n';

    if (std::abs(voltage_mean) > 0.5 || voltage_fundamental_peak < 20.0 ||
        voltage_fundamental_peak > 26.0 || current_fundamental_peak < 1.5 ||
        current_fundamental_peak > 3.5) {
        std::cerr << "inverter did not reach the expected 50 Hz operating region\n";
        return 6;
    }
    if (dead_samples < 18U) {
        std::cerr << "the last carrier period does not contain the requested deadtime\n";
        return 7;
    }
    return 0;
}
