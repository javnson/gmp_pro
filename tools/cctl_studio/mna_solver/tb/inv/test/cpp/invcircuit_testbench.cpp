#include "invcircuit.hpp"

// Handwritten case-specific testbench; cpp_codegen.py only regenerates the header.

#include <algorithm>
#include <array>
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

struct LegSignals {
    std::uint32_t upper;
    std::uint32_t lower;
    double duty;
};

struct GateSignals {
    std::uint32_t pwm1;
    std::uint32_t pwm2;
    std::uint32_t pwm3;
    std::uint32_t pwm4;
    std::uint32_t pwm5;
    std::uint32_t pwm6;
    std::array<double, 3> duty;
};

LegSignals leg_signals(double carrier_phase_s, double reference_time_s, double phase_offset_rad) {
    constexpr double carrier_period_s = 1.0 / kCarrierFrequencyHz;
    const double reference =
        std::sin(2.0 * kPi * kFundamentalFrequencyHz * reference_time_s + phase_offset_rad);
    const double duty = 0.5 + 0.5 * kModulationIndex * reference;
    const double rising_edge_s = 0.5 * (1.0 - duty) * carrier_period_s;
    const double falling_edge_s = 0.5 * (1.0 + duty) * carrier_period_s;

    const bool upper =
        carrier_phase_s >= rising_edge_s + kDeadTimeS / 2.0 &&
        carrier_phase_s < falling_edge_s - kDeadTimeS / 2.0;
    const bool lower =
        carrier_phase_s < rising_edge_s - kDeadTimeS / 2.0 ||
        carrier_phase_s >= falling_edge_s + kDeadTimeS / 2.0;
    return {upper ? 1U : 0U, lower ? 1U : 0U, duty};
}

GateSignals gate_signals(double time_s) {
    constexpr double carrier_period_s = 1.0 / kCarrierFrequencyHz;
    double carrier_phase_s = std::fmod(time_s, carrier_period_s);
    if (carrier_phase_s < 0.0) {
        carrier_phase_s += carrier_period_s;
    }

    const double carrier_start_s = time_s - carrier_phase_s;
    const double reference_time_s = carrier_start_s + carrier_period_s / 2.0;
    const auto phase_a = leg_signals(carrier_phase_s, reference_time_s, 0.0);
    const auto phase_b = leg_signals(carrier_phase_s, reference_time_s, -2.0 * kPi / 3.0);
    const auto phase_c = leg_signals(carrier_phase_s, reference_time_s, 2.0 * kPi / 3.0);

    // INV.CIR maps phase A to PWM1/PWM2, phase B to PWM3/PWM4, and phase C
    // to PWM5/PWM6. Each pair is complementary with a 1 us both-off interval.
    return {
        phase_a.upper,
        phase_a.lower,
        phase_b.upper,
        phase_b.lower,
        phase_c.upper,
        phase_c.lower,
        {phase_a.duty, phase_b.duty, phase_c.duty},
    };
}

struct PhaseStatistics {
    double sum{0.0};
    double square_sum{0.0};
    double sine_sum{0.0};
    double cosine_sum{0.0};
    double minimum{std::numeric_limits<double>::infinity()};
    double maximum{-std::numeric_limits<double>::infinity()};

    void add(double value, double angle) {
        sum += value;
        square_sum += value * value;
        sine_sum += value * std::sin(angle);
        cosine_sum += value * std::cos(angle);
        minimum = std::min(minimum, value);
        maximum = std::max(maximum, value);
    }

    double mean(double count) const { return sum / count; }
    double rms(double count) const { return std::sqrt(square_sum / count); }
    double fundamental_peak(double count) const {
        return 2.0 * std::hypot(sine_sum, cosine_sum) / count;
    }
};

double phasor_cosine(const PhaseStatistics& lhs, const PhaseStatistics& rhs) {
    const double numerator = lhs.sine_sum * rhs.sine_sum + lhs.cosine_sum * rhs.cosine_sum;
    const double denominator = std::hypot(lhs.sine_sum, lhs.cosine_sum) *
                               std::hypot(rhs.sine_sum, rhs.cosine_sum);
    return numerator / denominator;
}

}  // namespace

int main() {
    InvCircuit circuit;
    constexpr double dc_bus_v = 5.0;
    constexpr double duration_s = 100.0e-3;
    constexpr double normal_dt_s = InvCircuit::normal_step_s;
    constexpr double short_dt_s = InvCircuit::short_step_s;
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
        circuit.step_short(
            gate.pwm1, gate.pwm2, gate.pwm3, gate.pwm4, gate.pwm5, gate.pwm6, dc_bus_v);
        startup_time_s += short_dt_s;
    }

    std::ofstream csv("inv_cpp_three_phase_spwm.csv");
    if (!csv) {
        std::cerr << "cannot create inv_cpp_three_phase_spwm.csv\n";
        return 2;
    }
    csv << "time_s,PWM1,PWM2,PWM3,PWM4,PWM5,PWM6,duty_a,duty_b,duty_c,"
           "V(3,1),V(4,1),V(5,1),I(VAM1),I(VAM2),I(VAM3),topology_index\n";
    csv << std::setprecision(17);

    std::array<PhaseStatistics, 3> voltage_stats;
    std::array<PhaseStatistics, 3> current_stats;
    std::array<std::size_t, 3> dead_samples{};
    double maximum_phase_voltage_sum = 0.0;
    std::size_t analysis_count = 0;

    for (std::size_t index = 0; index < normal_steps; ++index) {
        const double time_s = static_cast<double>(index) * normal_dt_s;
        const auto gate = gate_signals(time_s);
        if ((gate.pwm1 != 0U && gate.pwm2 != 0U) ||
            (gate.pwm3 != 0U && gate.pwm4 != 0U) ||
            (gate.pwm5 != 0U && gate.pwm6 != 0U)) {
            std::cerr << "complementary gate overlap at t=" << time_s << " s\n";
            return 3;
        }

        const auto& output = circuit(
            gate.pwm1, gate.pwm2, gate.pwm3, gate.pwm4, gate.pwm5, gate.pwm6, dc_bus_v);
        const std::array<double, 3> voltage{
            output["V(3,1)"], output["V(4,1)"], output["V(5,1)"]};
        const std::array<double, 3> current{
            output["I(VAM1)"], output["I(VAM2)"], output["I(VAM3)"]};
        for (std::size_t phase = 0; phase < 3; ++phase) {
            if (!std::isfinite(voltage[phase]) || !std::isfinite(current[phase])) {
                std::cerr << "non-finite phase output at t=" << time_s << " s\n";
                return 4;
            }
        }

        if (index % 100U == 0U || index + 1U == normal_steps) {
            csv << time_s + normal_dt_s << ',' << gate.pwm1 << ',' << gate.pwm2 << ','
                << gate.pwm3 << ',' << gate.pwm4 << ',' << gate.pwm5 << ',' << gate.pwm6
                << ',' << gate.duty[0] << ',' << gate.duty[1] << ',' << gate.duty[2] << ','
                << voltage[0] << ',' << voltage[1] << ',' << voltage[2] << ',' << current[0]
                << ',' << current[1] << ',' << current[2] << ','
                << circuit.last_topology_index() << '\n';
        }

        if (index + carrier_samples >= normal_steps) {
            if (gate.pwm1 == 0U && gate.pwm2 == 0U) ++dead_samples[0];
            if (gate.pwm3 == 0U && gate.pwm4 == 0U) ++dead_samples[1];
            if (gate.pwm5 == 0U && gate.pwm6 == 0U) ++dead_samples[2];
        }
        if (index + analysis_samples >= normal_steps) {
            const double sample_time_s = time_s + normal_dt_s;
            const double angle = 2.0 * kPi * kFundamentalFrequencyHz * sample_time_s;
            for (std::size_t phase = 0; phase < 3; ++phase) {
                voltage_stats[phase].add(voltage[phase], angle);
                current_stats[phase].add(current[phase], angle);
            }
            maximum_phase_voltage_sum = std::max(
                maximum_phase_voltage_sum, std::abs(voltage[0] + voltage[1] + voltage[2]));
            ++analysis_count;
        }
    }

    const double count = static_cast<double>(analysis_count);
    std::array<double, 3> voltage_peak{};
    std::array<double, 3> current_peak{};
    for (std::size_t phase = 0; phase < 3; ++phase) {
        voltage_peak[phase] = voltage_stats[phase].fundamental_peak(count);
        current_peak[phase] = current_stats[phase].fundamental_peak(count);
    }
    const auto voltage_peak_range = std::minmax_element(voltage_peak.begin(), voltage_peak.end());
    const auto current_peak_range = std::minmax_element(current_peak.begin(), current_peak.end());
    const std::array<double, 3> phase_cosine{
        phasor_cosine(voltage_stats[0], voltage_stats[1]),
        phasor_cosine(voltage_stats[1], voltage_stats[2]),
        phasor_cosine(voltage_stats[2], voltage_stats[0]),
    };

    std::cout << std::setprecision(12)
              << "three-phase inverter: Vdc=5 V, carrier=10 kHz, sine=50 Hz, m=0.8, "
                 "deadtime=1 us\n";
    for (std::size_t phase = 0; phase < 3; ++phase) {
        const char name = static_cast<char>('A' + phase);
        std::cout << "phase " << name << ": voltage mean=" << voltage_stats[phase].mean(count)
                  << " V, rms=" << voltage_stats[phase].rms(count)
                  << " V, fundamental_peak=" << voltage_peak[phase]
                  << " V; current rms=" << current_stats[phase].rms(count)
                  << " A, fundamental_peak=" << current_peak[phase] << " A\n";
    }
    std::cout << "voltage phasor cosines AB/BC/CA=" << phase_cosine[0] << "/"
              << phase_cosine[1] << "/" << phase_cosine[2]
              << ", maximum Va+Vb+Vc=" << maximum_phase_voltage_sum << " V\n"
              << "last-carrier both-off samples A/B/C=" << dead_samples[0] << "/"
              << dead_samples[1] << "/" << dead_samples[2] << '\n';

    for (std::size_t phase = 0; phase < 3; ++phase) {
        if (std::abs(voltage_stats[phase].mean(count)) > 0.1 ||
            voltage_peak[phase] < 1.5 || voltage_peak[phase] > 2.4 ||
            current_peak[phase] < 0.1 || current_peak[phase] > 0.3) {
            std::cerr << "phase fundamental is outside the expected open-loop region\n";
            return 5;
        }
        if (phase_cosine[phase] < -0.65 || phase_cosine[phase] > -0.35) {
            std::cerr << "phase voltage separation is not approximately 120 degrees\n";
            return 6;
        }
        if (dead_samples[phase] < 18U) {
            std::cerr << "a bridge leg does not contain the requested deadtime\n";
            return 7;
        }
    }
    if (*voltage_peak_range.second - *voltage_peak_range.first > 0.08 ||
        *current_peak_range.second - *current_peak_range.first > 0.02 ||
        maximum_phase_voltage_sum > 1.0e-3) {
        std::cerr << "three-phase output is not sufficiently balanced\n";
        return 8;
    }
    return 0;
}
