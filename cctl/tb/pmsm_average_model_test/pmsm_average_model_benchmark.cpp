#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>

#include <cctl/cctl.hpp>

int main()
{
    typedef double scalar_type;
    typedef cctl::pmsm_average_model<scalar_type> motor_type;

    cctl::pmsm_average_parameters<scalar_type> motor_parameters;
    motor_parameters.stator_resistance_ohm = 4.7;
    motor_parameters.d_axis_inductance_h = 8.5e-3;
    motor_parameters.q_axis_inductance_h = 8.5e-3;
    motor_parameters.permanent_magnet_flux_wb = 3.8197e-3;
    motor_parameters.pole_pairs = 4U;
    motor_parameters.inertia_kg_m2 = 5e-4;
    motor_parameters.viscous_friction_nm_s = 1e-4;
    motor_parameters.open_circuit_current_decay_per_s = 20e3;

    cctl::three_phase_average_inverter_parameters<scalar_type> inverter_parameters;
    inverter_parameters.switching_frequency_hz = 20e3;
    inverter_parameters.dead_time_s = 1e-6;
    inverter_parameters.switch_resistance_ohm = 0.1;
    inverter_parameters.diode_forward_voltage_v = 0.3;

    motor_type motor(motor_parameters, inverter_parameters);
    motor_type::state_type state;
    motor_type::input_type input;
    input.inverter.enabled = true;
    input.inverter.dc_bus_voltage_v = 24.0;
    input.load_torque_nm = 2e-4;

    const scalar_type dt = 1.0 / 20e3;
    const std::size_t steps = static_cast<std::size_t>(20.0 / dt);
    const std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();

    for (std::size_t i = 0U; i < steps; ++i)
    {
        const scalar_type electrical_angle = 2.0 * 3.14159265358979323846 * 50.0 * scalar_type(i) * dt;
        input.inverter.duty[0] = 0.5 + 0.05 * std::sin(electrical_angle);
        input.inverter.duty[1] = 0.5 + 0.05 * std::sin(electrical_angle - 2.09439510239319549);
        input.inverter.duty[2] = 0.5 + 0.05 * std::sin(electrical_angle + 2.09439510239319549);
        cctl::runge_kutta_4_step(motor, scalar_type(i) * dt, dt, state, input);
    }

    const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - started;
    const volatile scalar_type checksum =
        state[motor_type::state_id] + state[motor_type::state_iq] +
        state[motor_type::state_mechanical_speed] + state[motor_type::state_mechanical_angle];
    std::cout << std::setprecision(12)
              << "integrator=RK4\n"
              << "state_dimension=4\n"
              << "simulated_seconds=20\n"
              << "steps=" << steps << '\n'
              << "wall_seconds=" << elapsed.count() << '\n'
              << "steps_per_wall_second=" << scalar_type(steps) / elapsed.count() << '\n'
              << "realtime_factor=" << 20.0 / elapsed.count() << '\n'
              << "checksum=" << checksum << '\n';
    return 0;
}
