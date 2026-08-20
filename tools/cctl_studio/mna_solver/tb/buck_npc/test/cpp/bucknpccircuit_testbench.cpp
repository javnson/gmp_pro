#include "bucknpccircuit.hpp"

// Handwritten case-specific testbench; cpp_codegen.py only regenerates the header.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

namespace {

constexpr double kCarrierFrequencyHz = 10.0e3;
constexpr double kDeadTimeS = 1.0e-6;
constexpr double kSourceVoltageV = 30.0;

struct GateSignals {
    std::uint32_t pwm1;
    std::uint32_t pwm2;
    std::uint32_t pwm3;
    std::uint32_t pwm4;
};

struct ComplementarySignals {
    std::uint32_t active;
    std::uint32_t complement;
};

ComplementarySignals centered_pair(double time_s, double active_duty) {
    constexpr double period_s = 1.0 / kCarrierFrequencyHz;
    double phase_s = std::fmod(time_s, period_s);
    if (phase_s < 0.0) phase_s += period_s;
    const double rising_s = 0.5 * (1.0 - active_duty) * period_s;
    const double falling_s = 0.5 * (1.0 + active_duty) * period_s;
    const bool active =
        phase_s >= rising_s + kDeadTimeS / 2.0 &&
        phase_s < falling_s - kDeadTimeS / 2.0;
    const bool complement =
        phase_s < rising_s - kDeadTimeS / 2.0 ||
        phase_s >= falling_s + kDeadTimeS / 2.0;
    return {active ? 1U : 0U, complement ? 1U : 0U};
}

GateSignals gate_signals(double time_s, double target_pu) {
    if (target_pu <= 0.5) {
        // N (0 V): PWM3/PWM4. O (30 V): PWM2/PWM3.
        const auto level_o = centered_pair(time_s, 2.0 * target_pu);
        return {0U, level_o.active, 1U, level_o.complement};
    }
    // O (30 V): PWM2/PWM3. P (60 V): PWM1/PWM2.
    const auto level_p = centered_pair(time_s, 2.0 * target_pu - 1.0);
    return {level_p.active, 1U, level_p.complement, 0U};
}

struct ScenarioResult {
    double mean_output_v{0.0};
    double ripple_v{0.0};
    double minimum_switch_v{std::numeric_limits<double>::infinity()};
    double maximum_switch_v{-std::numeric_limits<double>::infinity()};
    std::size_t low_level_samples{0};
    std::size_t middle_level_samples{0};
    std::size_t high_level_samples{0};
    std::size_t deadtime_samples{0};
};

ScenarioResult run_scenario(double target_pu, const std::string& csv_name) {
    BuckNpcCircuit circuit;
    constexpr double duration_s = 20.0e-3;
    constexpr double analysis_duration_s = 5.0e-3;
    constexpr double normal_dt_s = BuckNpcCircuit::normal_step_s;
    constexpr double short_dt_s = BuckNpcCircuit::short_step_s;
    constexpr std::size_t startup_short_steps = 200;
    constexpr std::size_t normal_steps =
        static_cast<std::size_t>(duration_s / normal_dt_s + 0.5);
    constexpr std::size_t analysis_samples =
        static_cast<std::size_t>(analysis_duration_s / normal_dt_s + 0.5);
    constexpr std::size_t carrier_samples =
        static_cast<std::size_t>((1.0 / kCarrierFrequencyHz) / normal_dt_s + 0.5);

    double startup_time_s = 0.0;
    for (std::size_t index = 0; index < startup_short_steps; ++index) {
        const auto gate = gate_signals(startup_time_s, target_pu);
        circuit.step_short(
            gate.pwm1, gate.pwm2, gate.pwm3, gate.pwm4,
            kSourceVoltageV, kSourceVoltageV);
        startup_time_s += short_dt_s;
    }

    std::ofstream csv(csv_name);
    if (!csv) throw std::runtime_error("cannot create " + csv_name);
    csv << "time_s,PWM1,PWM2,PWM3,PWM4,V(4),I(VAM1),V(VF1),topology_index\n";
    csv << std::setprecision(17);

    double output_sum = 0.0;
    double output_min = std::numeric_limits<double>::infinity();
    double output_max = -std::numeric_limits<double>::infinity();
    ScenarioResult result;
    for (std::size_t index = 0; index < normal_steps; ++index) {
        const double time_s = static_cast<double>(index) * normal_dt_s;
        const auto gate = gate_signals(time_s, target_pu);
        if ((gate.pwm1 != 0U && gate.pwm3 != 0U) ||
            (gate.pwm2 != 0U && gate.pwm4 != 0U)) {
            throw std::runtime_error("NPC complementary devices overlap");
        }

        const auto& output = circuit(
            gate.pwm1, gate.pwm2, gate.pwm3, gate.pwm4,
            kSourceVoltageV, kSourceVoltageV);
        const double switch_v = output["V(4)"];
        const double current_a = output["I(VAM1)"];
        const double filtered_v = output["V(VF1)"];
        if (!std::isfinite(switch_v) || !std::isfinite(current_a) || !std::isfinite(filtered_v)) {
            throw std::runtime_error("non-finite NPC output");
        }
        if (index % 100U == 0U || index + 1U == normal_steps) {
            csv << time_s + normal_dt_s << ',' << gate.pwm1 << ',' << gate.pwm2 << ','
                << gate.pwm3 << ',' << gate.pwm4 << ',' << switch_v << ',' << current_a
                << ',' << filtered_v << ',' << circuit.last_topology_index() << '\n';
        }

        if (index + analysis_samples >= normal_steps) {
            output_sum += filtered_v;
            output_min = std::min(output_min, filtered_v);
            output_max = std::max(output_max, filtered_v);
        }
        if (index + carrier_samples >= normal_steps) {
            result.minimum_switch_v = std::min(result.minimum_switch_v, switch_v);
            result.maximum_switch_v = std::max(result.maximum_switch_v, switch_v);
            if (switch_v < 15.0) ++result.low_level_samples;
            else if (switch_v < 45.0) ++result.middle_level_samples;
            else ++result.high_level_samples;
            if (target_pu <= 0.5) {
                if (gate.pwm2 == 0U && gate.pwm4 == 0U) ++result.deadtime_samples;
            } else if (gate.pwm1 == 0U && gate.pwm3 == 0U) {
                ++result.deadtime_samples;
            }
        }
    }
    result.mean_output_v = output_sum / static_cast<double>(analysis_samples);
    result.ripple_v = output_max - output_min;
    return result;
}

}  // namespace

int main() {
    try {
        const auto low = run_scenario(0.2, "buck_npc_cpp_02pu.csv");
        const auto high = run_scenario(0.8, "buck_npc_cpp_08pu.csv");
        std::cout << std::setprecision(12)
                  << "NPC Buck: VS1=VS2=30 V, carrier=10 kHz, deadtime=1 us\n"
                  << "0.2 pu: mean=" << low.mean_output_v << " V, ripple=" << low.ripple_v
                  << " V, switch range=" << low.minimum_switch_v << ".."
                  << low.maximum_switch_v << " V, level samples L/M/H="
                  << low.low_level_samples << '/' << low.middle_level_samples << '/'
                  << low.high_level_samples << ", deadtime samples=" << low.deadtime_samples
                  << "\n0.8 pu: mean=" << high.mean_output_v << " V, ripple=" << high.ripple_v
                  << " V, switch range=" << high.minimum_switch_v << ".."
                  << high.maximum_switch_v << " V, level samples L/M/H="
                  << high.low_level_samples << '/' << high.middle_level_samples << '/'
                  << high.high_level_samples << ", deadtime samples=" << high.deadtime_samples
                  << '\n';

        if (low.mean_output_v < 10.0 || low.mean_output_v > 14.0 || low.ripple_v > 2.0 ||
            low.low_level_samples == 0U || low.middle_level_samples == 0U ||
            low.high_level_samples > 20U || low.deadtime_samples < 15U) {
            std::cerr << "0.2 pu scenario did not use the expected N/O levels\n";
            return 3;
        }
        if (high.mean_output_v < 44.0 || high.mean_output_v > 50.0 || high.ripple_v > 2.0 ||
            high.low_level_samples > 20U || high.middle_level_samples == 0U ||
            high.high_level_samples == 0U || high.deadtime_samples < 15U) {
            std::cerr << "0.8 pu scenario did not use the expected O/P levels\n";
            return 4;
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 2;
    }
}
