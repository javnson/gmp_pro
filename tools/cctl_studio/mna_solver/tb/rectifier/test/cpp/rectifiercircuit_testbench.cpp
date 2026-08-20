#include "rectifiercircuit.hpp"

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
constexpr double kSourceRmsV = 32.0;
constexpr double kSourceFrequencyHz = 50.0;
constexpr double kPrechargeBypassTimeS = 60.0e-3;

double source_voltage(double time_s) {
    return kSourceRmsV * std::sqrt(2.0) *
           std::sin(2.0 * kPi * kSourceFrequencyHz * time_s);
}

std::uint32_t switch_command(double time_s) {
    return time_s >= kPrechargeBypassTimeS ? 1U : 0U;
}

}  // namespace

int main() {
    RectifierCircuit circuit;
    constexpr double duration_s = 200.0e-3;
    constexpr double normal_dt_s = RectifierCircuit::normal_step_s;
    constexpr double short_dt_s = RectifierCircuit::short_step_s;
    constexpr std::size_t startup_short_steps = 100;
    constexpr std::size_t normal_steps =
        static_cast<std::size_t>(duration_s / normal_dt_s + 0.5);

    double startup_time_s = 0.0;
    for (std::size_t index = 0; index < startup_short_steps; ++index) {
        circuit.step_short(
            switch_command(startup_time_s), source_voltage(startup_time_s));
        startup_time_s += short_dt_s;
    }

    std::ofstream csv("rectifier_cpp_32vrms_50hz.csv");
    if (!csv) {
        std::cerr << "cannot create rectifier_cpp_32vrms_50hz.csv\n";
        return 2;
    }
    csv << "time_s,SWGPIO1,VS1,V(1,2),V(VF1),topology_index\n";
    csv << std::setprecision(17);

    double precharge_sum = 0.0;
    double bypass_sum = 0.0;
    double bypass_min = std::numeric_limits<double>::infinity();
    double bypass_max = -std::numeric_limits<double>::infinity();
    double source_error_max = 0.0;
    double output_min = std::numeric_limits<double>::infinity();
    std::size_t precharge_count = 0;
    std::size_t bypass_count = 0;
    std::size_t positive_pair_samples = 0;
    std::size_t negative_pair_samples = 0;

    for (std::size_t index = 0; index < normal_steps; ++index) {
        const double time_s = static_cast<double>(index) * normal_dt_s;
        const double input_v = source_voltage(time_s);
        const std::uint32_t command = switch_command(time_s);
        const auto& output = circuit(command, input_v);
        const double measured_input = output["V(1,2)"];
        const double dc_output = output["V(VF1)"];
        if (!std::isfinite(measured_input) || !std::isfinite(dc_output)) {
            std::cerr << "non-finite rectifier output at t=" << time_s << " s\n";
            return 3;
        }
        const std::size_t topology = circuit.last_topology_index();
        if ((topology & 1U) != static_cast<std::size_t>(command)) {
            std::cerr << "SWGPIO1 did not select the requested switch path at t="
                      << time_s << " s\n";
            return 4;
        }
        if (index % 20U == 0U || index + 1U == normal_steps) {
            csv << time_s + normal_dt_s << ',' << command << ',' << input_v << ','
                << measured_input << ',' << dc_output << ',' << topology << '\n';
        }

        source_error_max = std::max(source_error_max, std::abs(measured_input - input_v));
        output_min = std::min(output_min, dc_output);
        if (time_s >= 40.0e-3 && time_s < 60.0e-3) {
            precharge_sum += dc_output;
            ++precharge_count;
        }
        if (time_s >= 180.0e-3) {
            bypass_sum += dc_output;
            bypass_min = std::min(bypass_min, dc_output);
            bypass_max = std::max(bypass_max, dc_output);
            ++bypass_count;
            // With the diode order D4,D3,D2,D1, the two physical bridge pairs
            // select binary topology indices 13 and 19 when SW2 is closed.
            positive_pair_samples += topology == 13U ? 1U : 0U;
            negative_pair_samples += topology == 19U ? 1U : 0U;
        }
    }

    const double precharge_mean = precharge_sum / static_cast<double>(precharge_count);
    const double bypass_mean = bypass_sum / static_cast<double>(bypass_count);
    std::cout << std::setprecision(12)
              << "rectifier: VS1=32 Vrms, 50 Hz; SW2 bypass at 60 ms\n"
              << "precharge-resistor mean V(VF1)=" << precharge_mean << " V\n"
              << "bypassed mean=" << bypass_mean << " V, min=" << bypass_min
              << " V, max=" << bypass_max << " V\n"
              << "bridge-pair samples: positive=" << positive_pair_samples
              << ", negative=" << negative_pair_samples << '\n'
              << "source probe maximum error=" << source_error_max << " V\n";

    if (source_error_max > 1e-9 || output_min < -1e-6) {
        std::cerr << "source probe or rectified-output polarity is incorrect\n";
        return 5;
    }
    if (precharge_mean < 12.0 || precharge_mean > 17.0 || bypass_mean < 29.0 ||
        bypass_mean > 33.0 || bypass_mean < 1.8 * precharge_mean ||
        bypass_max < 43.0 || bypass_max > 45.0) {
        std::cerr << "precharge/bypass operating regions are outside the expected range\n";
        return 6;
    }
    if (positive_pair_samples == 0U || negative_pair_samples == 0U) {
        std::cerr << "both full-wave bridge conduction pairs were not observed\n";
        return 7;
    }
    return 0;
}
