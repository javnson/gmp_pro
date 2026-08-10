#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include <cctl/cctl.hpp>

namespace
{

void require_near(const std::string &name, double actual, double expected, double tolerance)
{
    const double error = std::abs(actual - expected);
    if (!(error <= tolerance))
        throw std::runtime_error(name + " error=" + std::to_string(error) +
                                 " tolerance=" + std::to_string(tolerance));
    std::cout << "PASS " << std::setw(34) << std::left << name << " actual=" << std::setprecision(12)
              << actual << " expected=" << expected << " error=" << error << '\n';
}

void require_true(const std::string &name, bool condition)
{
    if (!condition)
        throw std::runtime_error(name);
    std::cout << "PASS " << name << '\n';
}

class scalar_decay_model
{
  public:
    typedef double scalar_type;
    typedef cctl::fixed_vector<double, 1U> state_type;

    struct input_type
    {
        double rate;
    };

    state_type derivative(double time, const state_type &state, const input_type &input) const
    {
        (void)time;
        return state_type{-input.rate * state[0]};
    }
};

double integrate_decay_euler(double dt)
{
    scalar_decay_model model;
    scalar_decay_model::state_type state{1.0};
    scalar_decay_model::input_type input = {1.0};
    const std::size_t steps = static_cast<std::size_t>(std::lround(1.0 / dt));
    for (std::size_t i = 0U; i < steps; ++i)
        cctl::explicit_euler_step(model, double(i) * dt, dt, state, input);
    return state[0];
}

double integrate_decay_rk4(double dt)
{
    scalar_decay_model model;
    scalar_decay_model::state_type state{1.0};
    scalar_decay_model::input_type input = {1.0};
    const std::size_t steps = static_cast<std::size_t>(std::lround(1.0 / dt));
    for (std::size_t i = 0U; i < steps; ++i)
        cctl::runge_kutta_4_step(model, double(i) * dt, dt, state, input);
    return state[0];
}

cctl::pmsm_average_model<double> make_motor(double inertia = 0.02, double friction = 0.004)
{
    cctl::pmsm_average_parameters<double> motor;
    motor.stator_resistance_ohm = 2.0;
    motor.d_axis_inductance_h = 0.01;
    motor.q_axis_inductance_h = 0.01;
    motor.permanent_magnet_flux_wb = 0.03;
    motor.pole_pairs = 4U;
    motor.inertia_kg_m2 = inertia;
    motor.viscous_friction_nm_s = friction;
    motor.open_circuit_current_decay_per_s = 2000.0;

    cctl::three_phase_average_inverter_parameters<double> inverter;
    inverter.switching_frequency_hz = 20e3;
    inverter.dead_time_s = 0.0;
    inverter.switch_resistance_ohm = 0.0;
    inverter.diode_forward_voltage_v = 0.0;
    inverter.current_sign_threshold_a = 1e-9;
    return cctl::pmsm_average_model<double>(motor, inverter);
}

void test_fixed_vector()
{
    cctl::fixed_vector<double, 4U> lhs{1.0, 2.0, 3.0, 4.0};
    cctl::fixed_vector<double, 4U> rhs{4.0, 3.0, 2.0, 1.0};
    const cctl::fixed_vector<double, 4U> result = lhs + rhs * 2.0;
    require_near("fixed vector element 0", result[0], 9.0, 0.0);
    require_near("fixed vector element 3", result[3], 6.0, 0.0);
    require_near("fixed vector dot product", cctl::dot(lhs, rhs), 20.0, 0.0);
}

void test_solver_order()
{
    const double exact = std::exp(-1.0);
    const double euler_coarse_error = std::abs(integrate_decay_euler(0.1) - exact);
    const double euler_fine_error = std::abs(integrate_decay_euler(0.05) - exact);
    const double rk4_coarse_error = std::abs(integrate_decay_rk4(0.2) - exact);
    const double rk4_fine_error = std::abs(integrate_decay_rk4(0.1) - exact);

    require_true("Euler exhibits first-order convergence",
                 euler_coarse_error / euler_fine_error > 1.8 &&
                     euler_coarse_error / euler_fine_error < 2.3);
    require_true("RK4 exhibits fourth-order convergence",
                 rk4_coarse_error / rk4_fine_error > 12.0 &&
                     rk4_coarse_error / rk4_fine_error < 20.0);
    require_near("RK4 scalar decay", integrate_decay_rk4(0.05), exact, 3e-8);
}

void test_average_inverter()
{
    cctl::three_phase_average_inverter_parameters<double> parameters;
    cctl::three_phase_average_inverter<double> inverter(parameters);
    cctl::three_phase_average_inverter_input<double> input;
    input.enabled = true;
    input.dc_bus_voltage_v = 24.0;
    input.duty = cctl::fixed_vector<double, 3U>{0.625, 0.4375, 0.4375};
    const cctl::fixed_vector<double, 3U> currents;
    const cctl::three_phase_average_inverter_output<double> output = inverter.evaluate(input, currents);

    require_near("average inverter phase A", output.phase_voltage_v[0], 3.0, 1e-12);
    require_near("average inverter phase B", output.phase_voltage_v[1], -1.5, 1e-12);
    require_near("average inverter phase C", output.phase_voltage_v[2], -1.5, 1e-12);
    require_near("average inverter zero sum",
                 output.phase_voltage_v[0] + output.phase_voltage_v[1] + output.phase_voltage_v[2], 0.0, 1e-12);

    parameters.dead_time_s = 1e-6;
    parameters.diode_forward_voltage_v = 0.3;
    inverter.set_parameters(parameters);
    input.duty = cctl::fixed_vector<double, 3U>{0.5, 0.5, 0.5};
    const cctl::fixed_vector<double, 3U> signed_currents{1.0, -0.5, -0.5};
    const cctl::three_phase_average_inverter_output<double> dead_time_output =
        inverter.evaluate(input, signed_currents);
    const double expected_line_error = 2.0 * (24.0 + 0.6) * 1e-6 * 20e3;
    require_near("dead-time line-voltage error",
                 dead_time_output.phase_voltage_v[1] - dead_time_output.phase_voltage_v[0],
                 expected_line_error, 1e-12);

    input.enabled = false;
    const cctl::three_phase_average_inverter_output<double> disabled = inverter.evaluate(input, signed_currents);
    require_near("disabled inverter voltage", cctl::squared_norm(disabled.phase_voltage_v), 0.0, 0.0);
}

void test_locked_rotor_rl_response()
{
    cctl::pmsm_average_model<double> motor = make_motor();
    cctl::pmsm_average_model<double>::state_type state;
    cctl::pmsm_average_input<double> input;
    input.inverter.enabled = true;
    input.inverter.dc_bus_voltage_v = 24.0;
    input.inverter.duty = cctl::fixed_vector<double, 3U>{0.625, 0.4375, 0.4375};

    const double dt = 1e-5;
    const double duration = 5e-3;
    const std::size_t steps = static_cast<std::size_t>(std::lround(duration / dt));
    for (std::size_t i = 0U; i < steps; ++i)
        cctl::runge_kutta_4_step(motor, double(i) * dt, dt, state, input);

    const double expected_id = 3.0 / 2.0 * (1.0 - std::exp(-2.0 / 0.01 * duration));
    require_near("PMSM locked-rotor d current", state[motor.state_id], expected_id, 1e-10);
    require_near("PMSM locked-rotor q current", state[motor.state_iq], 0.0, 1e-12);
    require_near("PMSM locked-rotor speed", state[motor.state_mechanical_speed], 0.0, 1e-12);
}

void test_torque_and_power_invariance()
{
    cctl::pmsm_average_model<double> motor = make_motor();
    cctl::pmsm_average_parameters<double> parameters = motor.parameters();
    parameters.d_axis_inductance_h = 0.008;
    parameters.q_axis_inductance_h = 0.012;
    motor.set_parameters(parameters);
    cctl::pmsm_average_model<double>::state_type state{1.0, 2.0, 12.0, 0.31};
    cctl::pmsm_average_input<double> input;
    input.inverter.enabled = true;
    input.inverter.dc_bus_voltage_v = 24.0;
    input.inverter.duty = cctl::fixed_vector<double, 3U>{0.7, 0.4, 0.2};
    input.load_torque_nm = 0.05;
    const cctl::pmsm_average_output<double> output = motor.observe(state, input);
    const cctl::pmsm_average_model<double>::state_type derivative = motor.derivative(0.0, state, input);

    const double expected_torque = 1.5 * 4.0 * (0.03 * 2.0 + (0.008 - 0.012) * 1.0 * 2.0);
    require_near("salient PMSM torque", output.electromagnetic_torque_nm, expected_torque, 1e-12);

    const double abc_power = cctl::dot(output.phase_voltage_v, output.phase_current_a);
    const double dq_power = 1.5 *
                            (output.d_axis_voltage_v * state[motor.state_id] +
                             output.q_axis_voltage_v * state[motor.state_iq]);
    require_near("abc/dq power invariance", abc_power, dq_power, 1e-11);

    const double copper_loss = 1.5 * parameters.stator_resistance_ohm *
                               (state[motor.state_id] * state[motor.state_id] +
                                state[motor.state_iq] * state[motor.state_iq]);
    const double magnetic_energy_rate =
        1.5 * (parameters.d_axis_inductance_h * state[motor.state_id] * derivative[motor.state_id] +
               parameters.q_axis_inductance_h * state[motor.state_iq] * derivative[motor.state_iq]);
    const double converted_power = output.electromagnetic_torque_nm * state[motor.state_mechanical_speed];
    require_near("PMSM electrical power balance", abc_power,
                 copper_loss + magnetic_energy_rate + converted_power, 2e-11);

    const double mechanical_energy_rate = parameters.inertia_kg_m2 *
                                          state[motor.state_mechanical_speed] *
                                          derivative[motor.state_mechanical_speed];
    const double mechanical_power_balance =
        converted_power - parameters.viscous_friction_nm_s *
                              state[motor.state_mechanical_speed] * state[motor.state_mechanical_speed] -
        input.load_torque_nm * state[motor.state_mechanical_speed];
    require_near("PMSM mechanical power balance", mechanical_energy_rate, mechanical_power_balance, 1e-12);
}

void test_free_coast_analytic_response()
{
    const double inertia = 0.02;
    const double friction = 0.004;
    const double load_torque = 0.05;
    cctl::pmsm_average_model<double> motor = make_motor(inertia, friction);
    cctl::pmsm_average_model<double>::state_type state{0.0, 0.0, 100.0, 0.0};
    cctl::pmsm_average_input<double> input;
    input.inverter.enabled = false;
    input.load_torque_nm = load_torque;

    const double dt = 1e-3;
    const double duration = 0.5;
    const std::size_t steps = static_cast<std::size_t>(std::lround(duration / dt));
    for (std::size_t i = 0U; i < steps; ++i)
        cctl::runge_kutta_4_step(motor, double(i) * dt, dt, state, input);

    const double rate = friction / inertia;
    const double load_speed = load_torque / friction;
    const double expected_speed = (100.0 + load_speed) * std::exp(-rate * duration) - load_speed;
    const double expected_angle =
        (100.0 + load_speed) * (1.0 - std::exp(-rate * duration)) / rate - load_speed * duration;
    require_near("PMSM coast speed analytic", state[motor.state_mechanical_speed], expected_speed, 2e-11);
    require_near("PMSM coast angle analytic", state[motor.state_mechanical_angle], expected_angle, 2e-11);
}

} // namespace

int main()
{
    try
    {
        test_fixed_vector();
        test_solver_order();
        test_average_inverter();
        test_locked_rotor_rl_response();
        test_torque_and_power_invariance();
        test_free_coast_analytic_response();
        std::cout << "All CCTL PMSM average-model tests passed.\n";
        return EXIT_SUCCESS;
    }
    catch (const std::exception &error)
    {
        std::cerr << "FAIL " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
