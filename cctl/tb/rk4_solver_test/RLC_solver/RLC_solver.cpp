#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

#include <cctl/cctl.hpp>

class rlc_model
{
  public:
    typedef double scalar_type;
    typedef cctl::fixed_vector<double, 2U> state_type;

    struct input_type
    {
        double voltage_v;
    };

    double capacitance_f;
    double inductance_h;
    double resistance_ohm;

    state_type derivative(double time, const state_type &state, const input_type &input) const
    {
        (void)time;
        state_type result;
        result[0] = state[1] / capacitance_f;
        result[1] = (input.voltage_v - state[0] - resistance_ohm * state[1]) / inductance_h;
        return result;
    }
};

int main()
{
    rlc_model model;
    model.capacitance_f = 1e-6;
    model.inductance_h = 1e-3;
    model.resistance_ohm = 4.0;

    rlc_model::state_type state;
    rlc_model::input_type input = {10.0};
    const double dt = 1e-7;
    std::ofstream file("output.csv");
    if (!file)
        return EXIT_FAILURE;
    file << "time_s,capacitor_voltage_v,inductor_current_a\n";

    for (std::size_t i = 0U; i < 100000U; ++i)
    {
        const double time = double(i) * dt;
        input.voltage_v = (i % 1000U < 500U) ? 0.0 : 10.0;
        cctl::runge_kutta_4_step(model, time, dt, state, input);
        file << time + dt << ',' << state[0] << ',' << state[1] << '\n';
    }

    std::cout << "RLC RK4 trace written to output.csv\n";
    return EXIT_SUCCESS;
}
