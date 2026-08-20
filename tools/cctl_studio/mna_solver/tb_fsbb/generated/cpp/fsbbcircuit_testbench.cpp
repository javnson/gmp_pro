#include "fsbbcircuit.hpp"

// Handwritten circuit-specific testbench; cpp_codegen.py only regenerates the header.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>

namespace {

struct GateSignals {
    std::uint32_t pwm1;
    std::uint32_t pwm2;
    std::uint32_t pwm3;
    std::uint32_t pwm4;
};

GateSignals gate_signals(double time_s) {
    constexpr double period_s = 100.0e-6;  // 10 kHz
    constexpr double half_period_s = period_s / 2.0;
    constexpr double dead_time_s = 1.0e-6;
    const double phase = std::fmod(time_s, period_s);

    // A 1 us both-off interval is centred on each commutation boundary.
    const bool first_half =
        phase >= dead_time_s / 2.0 && phase < half_period_s - dead_time_s / 2.0;
    const bool second_half =
        phase >= half_period_s + dead_time_s / 2.0 && phase < period_s - dead_time_s / 2.0;

    // PWM1/PWM2 and PWM3/PWM4 are the two complementary bridge-leg pairs.
    return {
        first_half ? 1U : 0U,
        second_half ? 1U : 0U,
        first_half ? 1U : 0U,
        second_half ? 1U : 0U,
    };
}

}  // namespace

int main() {
    FsbbCircuit circuit;
    constexpr double supply = 5.0;
    constexpr double duration_s = 50.0e-3;
    constexpr double normal_dt_s = FsbbCircuit::normal_step_s;
    constexpr double short_dt_s = FsbbCircuit::short_step_s;
    constexpr std::size_t startup_short_steps = 200;
    constexpr std::size_t normal_steps = static_cast<std::size_t>(duration_s / normal_dt_s);
    constexpr std::size_t samples_per_period = static_cast<std::size_t>(100.0e-6 / normal_dt_s);

    double startup_time_s = 0.0;
    for (std::size_t index = 0; index < startup_short_steps; ++index) {
        const auto gate = gate_signals(startup_time_s);
        circuit.step_short(gate.pwm1, gate.pwm2, gate.pwm3, gate.pwm4, supply);
        startup_time_s += short_dt_s;
    }

    std::ofstream csv("fsbb_cpp_50pct_deadtime.csv");
    if (!csv) {
        std::cerr << "cannot create fsbb_cpp_50pct_deadtime.csv\n";
        return 2;
    }
    csv << "time_s,PWM1,PWM2,PWM3,PWM4,V(VF1),I(VAM1),topology_index\n";
    csv << std::setprecision(17);

    double tail_sum = 0.0;
    double tail_min = std::numeric_limits<double>::infinity();
    double tail_max = -std::numeric_limits<double>::infinity();
    std::size_t tail_count = 0;
    std::size_t pair12_dead_samples = 0;
    std::size_t pair34_dead_samples = 0;

    for (std::size_t index = 0; index < normal_steps; ++index) {
        const double time_s = static_cast<double>(index) * normal_dt_s;
        const auto gate = gate_signals(time_s);
        if ((gate.pwm1 != 0U && gate.pwm2 != 0U) || (gate.pwm3 != 0U && gate.pwm4 != 0U)) {
            std::cerr << "complementary gate overlap at t=" << time_s << " s\n";
            return 3;
        }

        const auto& output = circuit(gate.pwm1, gate.pwm2, gate.pwm3, gate.pwm4, supply);
        if (index % 100U == 0U || index + 1U == normal_steps) {
            csv << time_s + normal_dt_s << ',' << gate.pwm1 << ',' << gate.pwm2 << ','
                << gate.pwm3 << ',' << gate.pwm4 << ',' << output.VF1 << ',' << output.VAM1
                << ',' << circuit.last_topology_index() << '\n';
        }

        if (index + samples_per_period >= normal_steps) {
            if (gate.pwm1 == 0U && gate.pwm2 == 0U) {
                ++pair12_dead_samples;
            }
            if (gate.pwm3 == 0U && gate.pwm4 == 0U) {
                ++pair34_dead_samples;
            }
        }
        if (index >= normal_steps * 9U / 10U) {
            tail_sum += output.VF1;
            tail_min = std::min(tail_min, output.VF1);
            tail_max = std::max(tail_max, output.VF1);
            ++tail_count;
        }
    }

    const double tail_mean = tail_sum / static_cast<double>(tail_count);
    std::cout << std::setprecision(12)
              << "FSBB 10 kHz, 50% complementary PWM, 1 us deadtime: V(VF1) tail mean="
              << tail_mean << ", min=" << tail_min << ", max=" << tail_max << '\n'
              << "last-period dead samples: pair12=" << pair12_dead_samples
              << ", pair34=" << pair34_dead_samples << '\n'
              << "string access V(VF1)=" << circuit["V(VF1)"] << '\n';

    if (!std::isfinite(tail_mean) || tail_mean < 3.5 || tail_mean > 4.2 || tail_max - tail_min > 0.3) {
        std::cerr << "FSBB output did not reach the expected steady operating region\n";
        return 4;
    }
    if (pair12_dead_samples < 18U || pair34_dead_samples < 18U) {
        std::cerr << "the last PWM period does not contain the requested deadtime\n";
        return 5;
    }
    return 0;
}
