#include "buckcircuit.hpp"

// Handwritten circuit-specific testbench; cpp_codegen.py regenerates the header and archive.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>

int main() {
    BuckCircuit circuit;
    constexpr double supply = 5.0;
    constexpr double pwm_frequency = 10000.0;
    constexpr double duty = 0.5;
    constexpr double duration = 0.05;
    constexpr double normal_dt = 1.0000000000000001e-07;
    constexpr double short_dt = 1.0000000000000001e-09;
    constexpr std::size_t startup_short_steps = 200;
    constexpr std::size_t normal_steps = static_cast<std::size_t>(duration / normal_dt);
    const double period = 1.0 / pwm_frequency;

    double startup_time = 0.0;
    for (std::size_t index = 0; index < startup_short_steps; ++index) {
        const std::uint32_t pwm = std::fmod(startup_time, period) < duty * period ? 1U : 0U;
        circuit.step_short(pwm, supply);
        startup_time += short_dt;
    }

    std::ofstream csv("buck_cpp_50pct.csv");
    if (!csv) {
        std::cerr << "cannot create buck_cpp_50pct.csv\n";
        return 2;
    }
    csv << "time_s,PWM,I(VAM1),V(VF1),topology_index\n";
    csv << std::setprecision(17);
    double tail_sum = 0.0;
    double tail_min = std::numeric_limits<double>::infinity();
    double tail_max = -std::numeric_limits<double>::infinity();
    std::size_t tail_count = 0;
    for (std::size_t index = 0; index < normal_steps; ++index) {
        const double time = static_cast<double>(index) * normal_dt;
        const std::uint32_t pwm = std::fmod(time, period) < duty * period ? 1U : 0U;
        const auto& output = circuit(pwm, supply);
        if (index % 100U == 0U || index + 1U == normal_steps) {
            csv << time + normal_dt << ',' << pwm << ',' << output.VAM1 << ',' << output.VF1 << ','
                << circuit.last_topology_index() << '\n';
        }
        if (index >= normal_steps * 9U / 10U) {
            tail_sum += output.VF1;
            tail_min = std::min(tail_min, output.VF1);
            tail_max = std::max(tail_max, output.VF1);
            ++tail_count;
        }
    }
    const double mean = tail_sum / static_cast<double>(tail_count);
    std::cout << std::setprecision(12)
              << "50% PWM tail mean=V(VF1): " << mean
              << ", min=" << tail_min << ", max=" << tail_max << '\n';
    std::cout << "string access V(VF1)=" << circuit["V(VF1)"] << '\n';
    if (!std::isfinite(mean) || mean <= 0.0) {
        std::cerr << "Circuit output is not finite and positive\n";
        return 3;
    }
    return 0;
}
