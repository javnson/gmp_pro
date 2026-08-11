#ifndef CCTL_POWER_ELECTRONICS_PMSM_AVERAGE_MODEL_HPP
#define CCTL_POWER_ELECTRONICS_PMSM_AVERAGE_MODEL_HPP

#include <cmath>
#include <cstdint>
#include <stdexcept>

#include <cctl/numerical_solver/fixed_vector.hpp>
#include <cctl/power_electronics_objects/inverter/three_phase_average_inverter.hpp>

namespace cctl
{

template <typename T> struct pmsm_average_parameters
{
    T stator_resistance_ohm;
    T d_axis_inductance_h;
    T q_axis_inductance_h;
    T permanent_magnet_flux_wb;
    std::uint16_t pole_pairs;
    T inertia_kg_m2;
    T viscous_friction_nm_s;
    T open_circuit_current_decay_per_s;

    pmsm_average_parameters()
        : stator_resistance_ohm(T(0)), d_axis_inductance_h(T(0)), q_axis_inductance_h(T(0)),
          permanent_magnet_flux_wb(T(0)), pole_pairs(1U), inertia_kg_m2(T(0)),
          viscous_friction_nm_s(T(0)), open_circuit_current_decay_per_s(T(20e3))
    {
    }
};

template <typename T> struct pmsm_average_input
{
    three_phase_average_inverter_input<T> inverter;
    T load_torque_nm;

    pmsm_average_input() : inverter(), load_torque_nm(T(0))
    {
    }
};

template <typename T> struct pmsm_average_output
{
    fixed_vector<T, 3U> phase_current_a;
    fixed_vector<T, 3U> phase_voltage_v;
    T d_axis_voltage_v;
    T q_axis_voltage_v;
    T electromagnetic_torque_nm;
    T electrical_speed_rad_s;
    T electrical_angle_rad;

    pmsm_average_output()
        : phase_current_a(), phase_voltage_v(), d_axis_voltage_v(T(0)), q_axis_voltage_v(T(0)),
          electromagnetic_torque_nm(T(0)), electrical_speed_rad_s(T(0)), electrical_angle_rad(T(0))
    {
    }
};

/**
 * @brief Fixed-parameter PMSM and averaged inverter model.
 *
 * State order is [id, iq, mechanical speed, unwrapped mechanical angle].
 * The load torque input is a signed torque subtracted from motor torque.
 */
template <typename T> class pmsm_average_model
{
  public:
    typedef T scalar_type;
    typedef fixed_vector<T, 4U> state_type;
    typedef pmsm_average_input<T> input_type;
    typedef pmsm_average_output<T> output_type;
    typedef pmsm_average_parameters<T> parameter_type;
    typedef three_phase_average_inverter<T> inverter_type;

    enum state_index
    {
        state_id = 0,
        state_iq = 1,
        state_mechanical_speed = 2,
        state_mechanical_angle = 3
    };

    pmsm_average_model() = delete;

    pmsm_average_model(const parameter_type &parameters,
                       const typename inverter_type::parameter_type &inverter_parameters)
        : parameters_(parameters), inverter_(inverter_parameters)
    {
        validate_parameters(parameters_);
    }

    void set_parameters(const parameter_type &parameters)
    {
        validate_parameters(parameters);
        parameters_ = parameters;
    }

    void set_inverter_parameters(const typename inverter_type::parameter_type &parameters)
    {
        inverter_.set_parameters(parameters);
    }

    const parameter_type &parameters() const
    {
        return parameters_;
    }

    const inverter_type &inverter() const
    {
        return inverter_;
    }

    state_type derivative(T time, const state_type &state, const input_type &input) const
    {
        (void)time;
        state_type result;
        const output_type output = observe(state, input);
        const T id = state[state_id];
        const T iq = state[state_iq];
        const T omega_m = state[state_mechanical_speed];

        if (input.inverter.enabled)
        {
            result[state_id] = (output.d_axis_voltage_v - parameters_.stator_resistance_ohm * id +
                                output.electrical_speed_rad_s * parameters_.q_axis_inductance_h * iq) /
                               parameters_.d_axis_inductance_h;
            result[state_iq] = (output.q_axis_voltage_v - parameters_.stator_resistance_ohm * iq -
                                output.electrical_speed_rad_s *
                                    (parameters_.d_axis_inductance_h * id + parameters_.permanent_magnet_flux_wb)) /
                               parameters_.q_axis_inductance_h;
        }
        else
        {
            // An ideal disabled bridge is open circuit.  The finite decay rate
            // removes the final commutation current without creating the
            // braking torque of an incorrectly modelled three-phase short.
            result[state_id] = -parameters_.open_circuit_current_decay_per_s * id;
            result[state_iq] = -parameters_.open_circuit_current_decay_per_s * iq;
        }

        result[state_mechanical_speed] =
            (output.electromagnetic_torque_nm - parameters_.viscous_friction_nm_s * omega_m -
             input.load_torque_nm) /
            parameters_.inertia_kg_m2;
        result[state_mechanical_angle] = omega_m;
        return result;
    }

    output_type observe(const state_type &state, const input_type &input) const
    {
        output_type result;
        const T id = state[state_id];
        const T iq = state[state_iq];
        const T omega_m = state[state_mechanical_speed];
        const T theta_e = T(parameters_.pole_pairs) * state[state_mechanical_angle];
        const T sin_theta = std::sin(theta_e);
        const T cos_theta = std::cos(theta_e);
        const T i_alpha = id * cos_theta - iq * sin_theta;
        const T i_beta = id * sin_theta + iq * cos_theta;

        result.phase_current_a[0] = i_alpha;
        result.phase_current_a[1] = -T(0.5) * i_alpha + sqrt_three_over_two() * i_beta;
        result.phase_current_a[2] = -T(0.5) * i_alpha - sqrt_three_over_two() * i_beta;

        const typename inverter_type::output_type inverter_output =
            inverter_.evaluate(input.inverter, result.phase_current_a);
        result.phase_voltage_v = inverter_output.phase_voltage_v;

        const T v_alpha = two_over_three() *
                          (result.phase_voltage_v[0] - T(0.5) *
                           (result.phase_voltage_v[1] + result.phase_voltage_v[2]));
        const T v_beta = one_over_sqrt_three() *
                         (result.phase_voltage_v[1] - result.phase_voltage_v[2]);
        result.d_axis_voltage_v = v_alpha * cos_theta + v_beta * sin_theta;
        result.q_axis_voltage_v = -v_alpha * sin_theta + v_beta * cos_theta;
        result.electrical_speed_rad_s = T(parameters_.pole_pairs) * omega_m;
        result.electrical_angle_rad = theta_e;

        if (input.inverter.enabled)
        {
            result.electromagnetic_torque_nm =
                T(1.5) * T(parameters_.pole_pairs) *
                (parameters_.permanent_magnet_flux_wb * iq +
                 (parameters_.d_axis_inductance_h - parameters_.q_axis_inductance_h) * id * iq);
        }
        return result;
    }

  private:
    static T two_over_three()
    {
        return T(2) / T(3);
    }

    static T one_over_sqrt_three()
    {
        return T(1) / std::sqrt(T(3));
    }

    static T sqrt_three_over_two()
    {
        return std::sqrt(T(3)) / T(2);
    }

    static void validate_parameters(const parameter_type &parameters)
    {
        if (!(parameters.stator_resistance_ohm >= T(0)) || !std::isfinite(parameters.stator_resistance_ohm) ||
            !(parameters.d_axis_inductance_h > T(0)) || !std::isfinite(parameters.d_axis_inductance_h) ||
            !(parameters.q_axis_inductance_h > T(0)) || !std::isfinite(parameters.q_axis_inductance_h) ||
            !(parameters.permanent_magnet_flux_wb >= T(0)) || !std::isfinite(parameters.permanent_magnet_flux_wb) ||
            parameters.pole_pairs == 0U || !(parameters.inertia_kg_m2 > T(0)) ||
            !std::isfinite(parameters.inertia_kg_m2) || !(parameters.viscous_friction_nm_s >= T(0)) ||
            !std::isfinite(parameters.viscous_friction_nm_s) ||
            !(parameters.open_circuit_current_decay_per_s >= T(0)) ||
            !std::isfinite(parameters.open_circuit_current_decay_per_s))
            throw std::invalid_argument("invalid PMSM average-model parameters");
    }

    parameter_type parameters_;
    inverter_type inverter_;
};

} // namespace cctl

#endif // CCTL_POWER_ELECTRONICS_PMSM_AVERAGE_MODEL_HPP
