#include "b2binvcircuit.hpp"

// Handwritten case-specific testbench; cpp_codegen.py regenerates the header and archive.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kCarrierFrequencyHz = 10.0e3;
constexpr double kGridFrequencyHz = 50.0;
constexpr double kGridLineRmsV = 32.0;
constexpr double kGridPhasePeakV = kGridLineRmsV * 0.81649658092772603273;
constexpr double kDcBusTargetV = 60.0;
constexpr double kGridModulation = 2.0 * kGridPhasePeakV / kDcBusTargetV;
constexpr double kOutputFrequencyHz = 30.0;
constexpr double kOutputModulation = 0.8;

struct LegSignals {
    std::uint32_t upper;
    std::uint32_t lower;
    double duty;
};

LegSignals leg_signals(double carrier_phase_s,
                       double reference_time_s,
                       double frequency_hz,
                       double modulation,
                       double phase_offset_rad) {
    constexpr double carrier_period_s = 1.0 / kCarrierFrequencyHz;
    const double reference =
        std::sin(2.0 * kPi * frequency_hz * reference_time_s + phase_offset_rad);
    const double duty = 0.5 + 0.5 * modulation * reference;
    const double rising_edge_s = 0.5 * (1.0 - duty) * carrier_period_s;
    const double falling_edge_s = 0.5 * (1.0 + duty) * carrier_period_s;
    const bool upper = carrier_phase_s >= rising_edge_s && carrier_phase_s < falling_edge_s;
    return {upper ? 1U : 0U, upper ? 0U : 1U, duty};
}

std::array<LegSignals, 3> three_phase_pwm(double time_s,
                                          double frequency_hz,
                                          double modulation) {
    constexpr double carrier_period_s = 1.0 / kCarrierFrequencyHz;
    double carrier_phase_s = std::fmod(time_s, carrier_period_s);
    if (carrier_phase_s < 0.0) carrier_phase_s += carrier_period_s;
    const double reference_time_s = time_s - carrier_phase_s + 0.5 * carrier_period_s;
    return {
        leg_signals(carrier_phase_s, reference_time_s, frequency_hz, modulation, 0.0),
        leg_signals(
            carrier_phase_s, reference_time_s, frequency_hz, modulation, -2.0 * kPi / 3.0),
        leg_signals(
            carrier_phase_s, reference_time_s, frequency_hz, modulation, 2.0 * kPi / 3.0),
    };
}

struct PhasorStatistics {
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

double phasor_cosine(const PhasorStatistics& lhs, const PhasorStatistics& rhs) {
    const double numerator = lhs.sine_sum * rhs.sine_sum + lhs.cosine_sum * rhs.cosine_sum;
    const double denominator = std::hypot(lhs.sine_sum, lhs.cosine_sum) *
                               std::hypot(rhs.sine_sum, rhs.cosine_sum);
    return numerator / denominator;
}

}  // namespace

int main() {
    const auto archive_load_start = std::chrono::steady_clock::now();
    B2bInvCircuit circuit;
    const double archive_load_s = std::chrono::duration<double>(
                                      std::chrono::steady_clock::now() - archive_load_start)
                                      .count();
    constexpr double duration_s = 2.0;
    constexpr double analysis_duration_s = 0.10;
    constexpr double dt_s = B2bInvCircuit::normal_step_s;
    constexpr std::size_t normal_steps =
        static_cast<std::size_t>(duration_s / dt_s + 0.5);
    constexpr std::size_t startup_short_steps = 200;
    constexpr std::size_t csv_decimation = 1000;

    B2bInvCircuit::Inputs input;
    for (std::size_t index = 0; index < startup_short_steps; ++index) {
        circuit.step_short(input);
    }

    std::ofstream csv("b2b_3ph_open_loop.csv");
    if (!csv) {
        std::cerr << "cannot create b2b_3ph_open_loop.csv\n";
        return 2;
    }
    csv << "time_s,grid_va_v,grid_vb_v,grid_vc_v,PWM1,PWM2,PWM3,PWM4,PWM5,PWM6,"
           "PWM7,PWM8,PWM9,PWM10,PWM11,PWM12,ubus_v,ubus2_v,grid_ia_a,grid_ib_a,"
           "grid_ic_a,out_va_v,out_vb_v,out_vc_v,out_ia_a,out_ib_a,out_ic_a,"
           "topology_index,calculation_state_index\n";
    csv << std::setprecision(17);

    std::array<PhasorStatistics, 3> output_voltage_stats;
    std::array<PhasorStatistics, 3> output_current_stats;
    std::array<PhasorStatistics, 3> grid_current_stats;
    PhasorStatistics bus_stats;
    double grid_power_sum = 0.0;
    double output_power_sum = 0.0;
    double maximum_bus_split_v = 0.0;
    std::size_t analysis_count = 0;
    std::vector<bool> visited(B2bInvCircuit::topology_count, false);
    std::size_t visited_count = 0;

    const auto wall_start = std::chrono::steady_clock::now();
    for (std::size_t index = 0; index < normal_steps; ++index) {
        const double time_s = static_cast<double>(index) * dt_s;
        const double grid_angle = 2.0 * kPi * kGridFrequencyHz * time_s;
        const std::array<double, 3> grid_voltage{
            kGridPhasePeakV * std::sin(grid_angle),
            kGridPhasePeakV * std::sin(grid_angle - 2.0 * kPi / 3.0),
            kGridPhasePeakV * std::sin(grid_angle + 2.0 * kPi / 3.0),
        };
        const auto rectifier = three_phase_pwm(time_s, kGridFrequencyHz, kGridModulation);
        const auto inverter = three_phase_pwm(time_s, kOutputFrequencyHz, kOutputModulation);

        input.PWM1 = rectifier[0].upper;
        input.PWM2 = rectifier[0].lower;
        input.PWM3 = rectifier[1].upper;
        input.PWM4 = rectifier[1].lower;
        input.PWM5 = rectifier[2].upper;
        input.PWM6 = rectifier[2].lower;
        input.PWM7 = inverter[0].upper;
        input.PWM8 = inverter[0].lower;
        input.PWM9 = inverter[1].upper;
        input.PWM10 = inverter[1].lower;
        input.PWM11 = inverter[2].upper;
        input.PWM12 = inverter[2].lower;
        input.VS2 = grid_voltage[0];
        input.VS3 = grid_voltage[1];
        input.VS4 = grid_voltage[2];

        const auto& output = circuit.step_normal(input);
        const std::array<double, 3> grid_current{
            output.LGRID_A, output.LGRID_B, output.LGRID_C};
        const std::array<double, 3> output_voltage{
            output.n_30, output.n_31, output.n_32};
        const std::array<double, 3> output_current{
            output.VAM1, output.VAM2, output.VAM3};
        for (std::size_t phase = 0; phase < 3; ++phase) {
            if (!std::isfinite(grid_current[phase]) ||
                !std::isfinite(output_voltage[phase]) ||
                !std::isfinite(output_current[phase])) {
                std::cerr << "non-finite phase result at t=" << time_s << " s\n";
                return 3;
            }
        }
        if (!std::isfinite(output.UBUS) || !std::isfinite(output.n_20)) {
            std::cerr << "non-finite DC bus result at t=" << time_s << " s\n";
            return 4;
        }

        const std::size_t topology = circuit.last_topology_index();
        if (!visited[topology]) {
            visited[topology] = true;
            ++visited_count;
        }
        if (index % csv_decimation == 0U || index + 1U == normal_steps) {
            csv << time_s + dt_s << ',' << grid_voltage[0] << ',' << grid_voltage[1] << ','
                << grid_voltage[2] << ',' << input.PWM1 << ',' << input.PWM2 << ','
                << input.PWM3 << ',' << input.PWM4 << ',' << input.PWM5 << ','
                << input.PWM6 << ',' << input.PWM7 << ',' << input.PWM8 << ','
                << input.PWM9 << ',' << input.PWM10 << ',' << input.PWM11 << ','
                << input.PWM12 << ',' << output.UBUS << ',' << output.n_20 << ','
                << grid_current[0] << ',' << grid_current[1] << ',' << grid_current[2] << ','
                << output_voltage[0] << ',' << output_voltage[1] << ','
                << output_voltage[2] << ',' << output_current[0] << ','
                << output_current[1] << ',' << output_current[2] << ',' << topology << ','
                << circuit.last_calculation_state_index() << '\n';
        }

        if (time_s + dt_s > duration_s - analysis_duration_s) {
            const double sample_time_s = time_s + dt_s;
            const double output_angle = 2.0 * kPi * kOutputFrequencyHz * sample_time_s;
            const double current_angle = 2.0 * kPi * kGridFrequencyHz * sample_time_s;
            bus_stats.add(output.UBUS, 0.0);
            maximum_bus_split_v =
                std::max(maximum_bus_split_v, std::abs(output.UBUS - output.n_20));
            for (std::size_t phase = 0; phase < 3; ++phase) {
                output_voltage_stats[phase].add(output_voltage[phase], output_angle);
                output_current_stats[phase].add(output_current[phase], output_angle);
                grid_current_stats[phase].add(grid_current[phase], current_angle);
                grid_power_sum += grid_voltage[phase] * grid_current[phase];
                output_power_sum += output_voltage[phase] * output_current[phase];
            }
            ++analysis_count;
        }
    }
    const double wall_s = std::chrono::duration<double>(
                              std::chrono::steady_clock::now() - wall_start)
                              .count();

    const double count = static_cast<double>(analysis_count);
    std::array<double, 3> output_voltage_peak{};
    std::array<double, 3> output_current_peak{};
    std::array<double, 3> grid_current_peak{};
    for (std::size_t phase = 0; phase < 3; ++phase) {
        output_voltage_peak[phase] = output_voltage_stats[phase].fundamental_peak(count);
        output_current_peak[phase] = output_current_stats[phase].fundamental_peak(count);
        grid_current_peak[phase] = grid_current_stats[phase].fundamental_peak(count);
    }
    const auto voltage_range =
        std::minmax_element(output_voltage_peak.begin(), output_voltage_peak.end());
    const std::array<double, 3> phase_cosine{
        phasor_cosine(output_voltage_stats[0], output_voltage_stats[1]),
        phasor_cosine(output_voltage_stats[1], output_voltage_stats[2]),
        phasor_cosine(output_voltage_stats[2], output_voltage_stats[0]),
    };

    std::cout << std::setprecision(12)
              << "B2B three-phase stress test: states=23, logical topologies="
              << B2bInvCircuit::topology_count << ", visited=" << visited_count
              << ", archive="
              << static_cast<double>(
                     std::filesystem::file_size(B2bInvCircuit::archive_filename)) /
                     (1024.0 * 1024.0)
              << " MiB, load=" << archive_load_s << " s\n"
              << "grid: line_rms=32 V, frequency=50 Hz, rectifier m=" << kGridModulation
              << "; output: frequency=30 Hz, m=0.8\n"
              << "DC bus mean/min/max=" << bus_stats.mean(count) << '/' << bus_stats.minimum
              << '/' << bus_stats.maximum << " V, maximum bridge-bus split="
              << maximum_bus_split_v << " V\n"
              << "mean grid/output power=" << grid_power_sum / count << '/'
              << output_power_sum / count << " W\n";
    for (std::size_t phase = 0; phase < 3; ++phase) {
        std::cout << "phase " << static_cast<char>('A' + phase)
                  << ": grid current fundamental_peak=" << grid_current_peak[phase]
                  << " A; output voltage/current fundamental_peak="
                  << output_voltage_peak[phase] << '/' << output_current_peak[phase] << '\n';
    }
    std::cout << "output voltage phasor cosines AB/BC/CA=" << phase_cosine[0] << '/'
              << phase_cosine[1] << '/' << phase_cosine[2] << '\n'
              << "runtime=" << wall_s << " s, rate="
              << static_cast<double>(normal_steps) / wall_s / 1.0e6 << " Mstep/s\n";

    if (bus_stats.mean(count) < 54.0 || bus_stats.mean(count) > 60.0 ||
        bus_stats.minimum < 50.0 || bus_stats.maximum > 62.0 ||
        maximum_bus_split_v > 0.5) {
        std::cerr << "DC bus did not settle in the expected open-loop region\n";
        return 5;
    }
    for (std::size_t phase = 0; phase < 3; ++phase) {
        if (grid_current_peak[phase] < 3.0 || grid_current_peak[phase] > 3.6 ||
            output_voltage_peak[phase] < 21.0 || output_voltage_peak[phase] > 23.5 ||
            output_current_peak[phase] < 2.0 || output_current_peak[phase] > 2.4) {
            std::cerr << "output fundamental is outside the expected open-loop region\n";
            return 6;
        }
        if (phase_cosine[phase] < -0.65 || phase_cosine[phase] > -0.35) {
            std::cerr << "output phase separation is not approximately 120 degrees\n";
            return 7;
        }
    }
    if (*voltage_range.second - *voltage_range.first > 0.1 ||
        grid_power_sum / count < 70.0 || grid_power_sum / count > 90.0 ||
        output_power_sum / count < 65.0 || output_power_sum / count > 85.0 ||
        visited_count < 40U) {
        std::cerr << "B2B power flow, balance, or topology coverage is insufficient\n";
        return 8;
    }
    return 0;
}
