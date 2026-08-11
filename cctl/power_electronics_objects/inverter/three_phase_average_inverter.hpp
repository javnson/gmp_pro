#ifndef CCTL_POWER_ELECTRONICS_THREE_PHASE_AVERAGE_INVERTER_HPP
#define CCTL_POWER_ELECTRONICS_THREE_PHASE_AVERAGE_INVERTER_HPP

#include <cmath>
#include <stdexcept>

#include <cctl/numerical_solver/fixed_vector.hpp>

namespace cctl
{

template <typename T> struct three_phase_average_inverter_parameters
{
    T switching_frequency_hz;
    T dead_time_s;
    T switch_resistance_ohm;
    T diode_forward_voltage_v;
    T current_sign_threshold_a;

    three_phase_average_inverter_parameters()
        : switching_frequency_hz(T(20e3)), dead_time_s(T(0)), switch_resistance_ohm(T(0)),
          diode_forward_voltage_v(T(0)), current_sign_threshold_a(T(1e-6))
    {
    }
};

template <typename T> struct three_phase_average_inverter_input
{
    T dc_bus_voltage_v;
    fixed_vector<T, 3U> duty;
    bool enabled;

    three_phase_average_inverter_input() : dc_bus_voltage_v(T(0)), duty(), enabled(false)
    {
    }
};

template <typename T> struct three_phase_average_inverter_output
{
    fixed_vector<T, 3U> leg_voltage_v;
    fixed_vector<T, 3U> phase_voltage_v;
    T dc_bus_current_a;

    three_phase_average_inverter_output() : leg_voltage_v(), phase_voltage_v(), dc_bus_current_a(T(0))
    {
    }
};

/**
 * @brief Three-phase two-level inverter averaged over one PWM period.
 */
template <typename T> class three_phase_average_inverter
{
  public:
    typedef T scalar_type;
    typedef three_phase_average_inverter_parameters<T> parameter_type;
    typedef three_phase_average_inverter_input<T> input_type;
    typedef three_phase_average_inverter_output<T> output_type;

    three_phase_average_inverter() : parameters_()
    {
    }

    explicit three_phase_average_inverter(const parameter_type &parameters) : parameters_(parameters)
    {
        validate_parameters(parameters_);
    }

    void set_parameters(const parameter_type &parameters)
    {
        validate_parameters(parameters);
        parameters_ = parameters;
    }

    const parameter_type &parameters() const
    {
        return parameters_;
    }

    output_type evaluate(const input_type &input, const fixed_vector<T, 3U> &phase_current_a) const
    {
        output_type result;
        if (!input.enabled)
            return result;
        if (!(input.dc_bus_voltage_v >= T(0)) || !std::isfinite(input.dc_bus_voltage_v))
            throw std::invalid_argument("DC bus voltage must be finite and non-negative");

        const T dead_time_drop = (input.dc_bus_voltage_v + T(2) * parameters_.diode_forward_voltage_v) *
                                 parameters_.dead_time_s * parameters_.switching_frequency_hz;
        T neutral_voltage = T(0);

        for (std::size_t phase = 0U; phase < 3U; ++phase)
        {
            if (!std::isfinite(input.duty[phase]) || !std::isfinite(phase_current_a[phase]))
                throw std::invalid_argument("inverter duty and current must be finite");
            const T duty = clamp(input.duty[phase], T(0), T(1));
            const T current_sign = sign_with_dead_band(phase_current_a[phase]);
            result.leg_voltage_v[phase] = duty * input.dc_bus_voltage_v -
                                          current_sign * dead_time_drop -
                                          parameters_.switch_resistance_ohm * phase_current_a[phase];
            result.dc_bus_current_a += duty * phase_current_a[phase];
            neutral_voltage += result.leg_voltage_v[phase];
        }

        neutral_voltage /= T(3);
        for (std::size_t phase = 0U; phase < 3U; ++phase)
            result.phase_voltage_v[phase] = result.leg_voltage_v[phase] - neutral_voltage;
        return result;
    }

  private:
    static T clamp(T value, T low, T high)
    {
        return (value < low) ? low : ((value > high) ? high : value);
    }

    T sign_with_dead_band(T current) const
    {
        if (current > parameters_.current_sign_threshold_a)
            return T(1);
        if (current < -parameters_.current_sign_threshold_a)
            return T(-1);
        return T(0);
    }

    static void validate_parameters(const parameter_type &parameters)
    {
        if (!(parameters.switching_frequency_hz > T(0)) || !std::isfinite(parameters.switching_frequency_hz) ||
            !(parameters.dead_time_s >= T(0)) || !std::isfinite(parameters.dead_time_s) ||
            !(parameters.switch_resistance_ohm >= T(0)) || !std::isfinite(parameters.switch_resistance_ohm) ||
            !(parameters.diode_forward_voltage_v >= T(0)) || !std::isfinite(parameters.diode_forward_voltage_v) ||
            !(parameters.current_sign_threshold_a >= T(0)) || !std::isfinite(parameters.current_sign_threshold_a))
            throw std::invalid_argument("invalid three-phase average inverter parameters");
    }

    parameter_type parameters_;
};

} // namespace cctl

#endif // CCTL_POWER_ELECTRONICS_THREE_PHASE_AVERAGE_INVERTER_HPP
