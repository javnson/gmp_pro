#ifndef CCTL_CIRCUIT_MODEL_PMSM_CS_HPP
#define CCTL_CIRCUIT_MODEL_PMSM_CS_HPP

#include <cmath>
#include <cstdint>
#include <stdexcept>

#include <cctl/numerical_solver/fixed_vector.hpp>

namespace cctl
{

/** Parameters for a three-wire PMSM represented as three controlled current sources. */
template <typename T> struct pmsm_cs_parameters
{
    T sample_period_s;
    T stator_resistance_ohm;
    T d_axis_inductance_h;
    T q_axis_inductance_h;
    T permanent_magnet_flux_wb;
    std::uint16_t pole_pairs;
    T inertia_kg_m2;
    T viscous_friction_nm_s;

    pmsm_cs_parameters()
        : sample_period_s(T(100e-9)), stator_resistance_ohm(T(0.165)),
          d_axis_inductance_h(T(0.45e-3)), q_axis_inductance_h(T(0.45e-3)),
          permanent_magnet_flux_wb(T(0.0066843949493427743)), pole_pairs(4U),
          inertia_kg_m2(T(497e-7)), viscous_friction_nm_s(T(755e-6))
    {
    }
};

template <typename T> struct pmsm_cs_input
{
    fixed_vector<T, 3U> phase_voltage_v;
    T load_torque_nm;

    pmsm_cs_input() : phase_voltage_v(), load_torque_nm(T(0))
    {
    }
};

template <typename T> struct pmsm_cs_output
{
    fixed_vector<T, 3U> phase_current_a;
    T d_axis_current_a;
    T q_axis_current_a;
    T d_axis_voltage_v;
    T q_axis_voltage_v;
    T electromagnetic_torque_nm;
    T mechanical_speed_rad_s;
    T mechanical_speed_rpm;
    T mechanical_angle_rad;
    T electrical_speed_rad_s;
    T electrical_frequency_hz;
    T electrical_angle_rad;

    pmsm_cs_output()
        : phase_current_a(), d_axis_current_a(T(0)), q_axis_current_a(T(0)),
          d_axis_voltage_v(T(0)), q_axis_voltage_v(T(0)), electromagnetic_torque_nm(T(0)),
          mechanical_speed_rad_s(T(0)), mechanical_speed_rpm(T(0)), mechanical_angle_rad(T(0)),
          electrical_speed_rad_s(T(0)), electrical_frequency_hz(T(0)), electrical_angle_rad(T(0))
    {
    }
};

/**
 * @brief Discrete current-source PMSM model for coupling to a switched MNA circuit.
 *
 * Positive phase current flows from each inverter terminal into the motor neutral,
 * matching the SPICE direction of IPMSMx_A/B/C. The input voltages are measured
 * from the corresponding phase terminal to that common neutral. One call advances
 * the motor by the initialized sample period using a fixed-step RK4 update.
 */
template <typename T = double> class pmsm_cs
{
  public:
    typedef T scalar_type;
    typedef pmsm_cs_parameters<T> parameter_type;
    typedef pmsm_cs_input<T> input_type;
    typedef pmsm_cs_output<T> output_type;
    typedef fixed_vector<T, 4U> state_type;

    enum state_index
    {
        state_id = 0,
        state_iq = 1,
        state_mechanical_speed = 2,
        state_mechanical_angle = 3
    };

    output_type output;

    pmsm_cs()
    {
        initialize();
    }

    explicit pmsm_cs(const parameter_type &parameters)
    {
        initialize(parameters);
    }

    /** Validate parameters, precompute real-time coefficients, and clear the state. */
    void initialize(const parameter_type &parameters = parameter_type())
    {
        validate_parameters(parameters);
        parameters_ = parameters;
        inverse_ld_ = T(1) / parameters_.d_axis_inductance_h;
        inverse_lq_ = T(1) / parameters_.q_axis_inductance_h;
        resistance_over_ld_ = parameters_.stator_resistance_ohm * inverse_ld_;
        resistance_over_lq_ = parameters_.stator_resistance_ohm * inverse_lq_;
        lq_over_ld_ = parameters_.q_axis_inductance_h * inverse_ld_;
        ld_over_lq_ = parameters_.d_axis_inductance_h * inverse_lq_;
        flux_over_lq_ = parameters_.permanent_magnet_flux_wb * inverse_lq_;
        inverse_inertia_ = T(1) / parameters_.inertia_kg_m2;
        torque_flux_coefficient_ =
            T(1.5) * T(parameters_.pole_pairs) * parameters_.permanent_magnet_flux_wb;
        torque_saliency_coefficient_ = T(1.5) * T(parameters_.pole_pairs) *
                                       (parameters_.d_axis_inductance_h -
                                        parameters_.q_axis_inductance_h);
        half_step_s_ = parameters_.sample_period_s * T(0.5);
        step_over_six_s_ = parameters_.sample_period_s / T(6);
        reset();
    }

    void reset(const state_type &initial_state = state_type())
    {
        state_ = initial_state;
        output = observe(input_type());
    }

    const output_type &step(const input_type &input)
    {
        const state_type k1 = derivative(state_, input);
        const state_type k2 = derivative(state_ + k1 * half_step_s_, input);
        const state_type k3 = derivative(state_ + k2 * half_step_s_, input);
        const state_type k4 = derivative(state_ + k3 * parameters_.sample_period_s, input);
        state_ += (k1 + k2 * T(2) + k3 * T(2) + k4) * step_over_six_s_;
        output = observe(input);
        return output;
    }

    const output_type &step(T phase_a_voltage_v, T phase_b_voltage_v, T phase_c_voltage_v,
                            T load_torque_nm = T(0))
    {
        input_type input;
        input.phase_voltage_v =
            fixed_vector<T, 3U>{phase_a_voltage_v, phase_b_voltage_v, phase_c_voltage_v};
        input.load_torque_nm = load_torque_nm;
        return step(input);
    }

    const output_type &run(const input_type &input)
    {
        return step(input);
    }

    const output_type &operator()(const input_type &input)
    {
        return step(input);
    }

    const output_type &operator()(T phase_a_voltage_v, T phase_b_voltage_v,
                                  T phase_c_voltage_v, T load_torque_nm = T(0))
    {
        return step(phase_a_voltage_v, phase_b_voltage_v, phase_c_voltage_v, load_torque_nm);
    }

    const state_type &state() const
    {
        return state_;
    }

    const parameter_type &parameters() const
    {
        return parameters_;
    }

    output_type observe(const input_type &input) const
    {
        return observe_state(state_, input);
    }

  private:
    struct electrical_values
    {
        T vd;
        T vq;
        T torque;
        T sin_theta;
        T cos_theta;
        T theta_e;
    };

    state_type derivative(const state_type &state, const input_type &input) const
    {
        const electrical_values values = transform(state, input);
        const T id = state[state_id];
        const T iq = state[state_iq];
        const T omega_m = state[state_mechanical_speed];
        const T omega_e = T(parameters_.pole_pairs) * omega_m;
        state_type result;
        result[state_id] = values.vd * inverse_ld_ - resistance_over_ld_ * id +
                           omega_e * lq_over_ld_ * iq;
        result[state_iq] = values.vq * inverse_lq_ - resistance_over_lq_ * iq -
                           omega_e * (ld_over_lq_ * id + flux_over_lq_);
        result[state_mechanical_speed] =
            (values.torque - parameters_.viscous_friction_nm_s * omega_m -
             input.load_torque_nm) *
            inverse_inertia_;
        result[state_mechanical_angle] = omega_m;
        return result;
    }

    electrical_values transform(const state_type &state, const input_type &input) const
    {
        electrical_values result;
        result.theta_e = T(parameters_.pole_pairs) * state[state_mechanical_angle];
        result.sin_theta = std::sin(result.theta_e);
        result.cos_theta = std::cos(result.theta_e);
        const T v_alpha = two_over_three() *
                          (input.phase_voltage_v[0] -
                           T(0.5) * (input.phase_voltage_v[1] + input.phase_voltage_v[2]));
        const T v_beta = one_over_sqrt_three() *
                         (input.phase_voltage_v[1] - input.phase_voltage_v[2]);
        result.vd = v_alpha * result.cos_theta + v_beta * result.sin_theta;
        result.vq = -v_alpha * result.sin_theta + v_beta * result.cos_theta;
        const T id = state[state_id];
        const T iq = state[state_iq];
        result.torque = torque_flux_coefficient_ * iq +
                        torque_saliency_coefficient_ * id * iq;
        return result;
    }

    output_type observe_state(const state_type &state, const input_type &input) const
    {
        const electrical_values values = transform(state, input);
        output_type result;
        result.d_axis_current_a = state[state_id];
        result.q_axis_current_a = state[state_iq];
        result.d_axis_voltage_v = values.vd;
        result.q_axis_voltage_v = values.vq;
        result.electromagnetic_torque_nm = values.torque;
        result.mechanical_speed_rad_s = state[state_mechanical_speed];
        result.mechanical_speed_rpm = state[state_mechanical_speed] * T(30) / pi();
        result.mechanical_angle_rad = state[state_mechanical_angle];
        result.electrical_speed_rad_s =
            T(parameters_.pole_pairs) * state[state_mechanical_speed];
        result.electrical_frequency_hz = result.electrical_speed_rad_s / (T(2) * pi());
        result.electrical_angle_rad = wrap_angle(values.theta_e);

        const T i_alpha = result.d_axis_current_a * values.cos_theta -
                          result.q_axis_current_a * values.sin_theta;
        const T i_beta = result.d_axis_current_a * values.sin_theta +
                         result.q_axis_current_a * values.cos_theta;
        result.phase_current_a[0] = i_alpha;
        result.phase_current_a[1] = -T(0.5) * i_alpha + sqrt_three_over_two() * i_beta;
        result.phase_current_a[2] = -T(0.5) * i_alpha - sqrt_three_over_two() * i_beta;
        return result;
    }

    static T pi()
    {
        return T(3.141592653589793238462643383279502884L);
    }

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

    static T wrap_angle(T angle)
    {
        const T two_pi = T(2) * pi();
        angle = std::fmod(angle + pi(), two_pi);
        if (angle < T(0))
            angle += two_pi;
        return angle - pi();
    }

    static void validate_parameters(const parameter_type &parameters)
    {
        if (!(parameters.sample_period_s > T(0)) || !std::isfinite(parameters.sample_period_s) ||
            !(parameters.stator_resistance_ohm >= T(0)) ||
            !std::isfinite(parameters.stator_resistance_ohm) ||
            !(parameters.d_axis_inductance_h > T(0)) ||
            !std::isfinite(parameters.d_axis_inductance_h) ||
            !(parameters.q_axis_inductance_h > T(0)) ||
            !std::isfinite(parameters.q_axis_inductance_h) ||
            !(parameters.permanent_magnet_flux_wb >= T(0)) ||
            !std::isfinite(parameters.permanent_magnet_flux_wb) ||
            parameters.pole_pairs == 0U || !(parameters.inertia_kg_m2 > T(0)) ||
            !std::isfinite(parameters.inertia_kg_m2) ||
            !(parameters.viscous_friction_nm_s >= T(0)) ||
            !std::isfinite(parameters.viscous_friction_nm_s))
            throw std::invalid_argument("invalid PMSM current-source model parameters");
    }

    parameter_type parameters_;
    state_type state_;
    T inverse_ld_{};
    T inverse_lq_{};
    T resistance_over_ld_{};
    T resistance_over_lq_{};
    T lq_over_ld_{};
    T ld_over_lq_{};
    T flux_over_lq_{};
    T inverse_inertia_{};
    T torque_flux_coefficient_{};
    T torque_saliency_coefficient_{};
    T half_step_s_{};
    T step_over_six_s_{};
};

} // namespace cctl

#endif // CCTL_CIRCUIT_MODEL_PMSM_CS_HPP
