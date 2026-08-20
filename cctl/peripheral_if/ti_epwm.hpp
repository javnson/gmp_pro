#ifndef CCTL_PERIPHERAL_IF_TI_EPWM_HPP
#define CCTL_PERIPHERAL_IF_TI_EPWM_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace cctl
{

/** C2000 ePWM parameters used by the host-side center-aligned model. */
template <typename T> struct ti_epwm_config
{
    T time_base_clock_hz;
    std::uint32_t period_count;
    std::uint32_t rising_edge_delay_count;
    std::uint32_t falling_edge_delay_count;
    bool upper_active_above_compare;

    ti_epwm_config()
        : time_base_clock_hz(T(100e6)), period_count(2499U),
          rising_edge_delay_count(100U), falling_edge_delay_count(100U),
          upper_active_above_compare(true)
    {
    }
};

struct ti_epwm_gate_pair
{
    std::uint32_t upper;
    std::uint32_t lower;
};

/**
 * @brief Host model of one complementary C2000 ePWM channel.
 *
 * The time base counts up and down between zero and TBPRD. CMPA is shadowed
 * into the next sample immediately at the controller boundary. DBRED delays
 * upper-device turn-on and DBFED delays lower-device turn-on; turn-off remains
 * immediate, so overlap is impossible.
 */
template <typename T = double> class ti_epwm
{
  public:
    typedef T scalar_type;
    typedef ti_epwm_config<T> config_type;
    typedef ti_epwm_gate_pair output_type;

    ti_epwm()
    {
        initialize(config_type());
    }

    explicit ti_epwm(const config_type &config)
    {
        initialize(config);
    }

    void initialize(const config_type &config = config_type())
    {
        if (!(config.time_base_clock_hz > T(0)) ||
            !std::isfinite(config.time_base_clock_hz) || config.period_count == 0U)
            throw std::invalid_argument("invalid TI ePWM configuration");
        config_ = config;
        half_cycle_count_ = T(config_.period_count) + T(1);
        full_cycle_count_ = T(2) * half_cycle_count_;
        compare_a_ = config_.period_count / 2U;
        enabled_ = false;
    }

    void set_compare_a(std::int32_t compare_count) noexcept
    {
        const std::int32_t maximum = static_cast<std::int32_t>(config_.period_count);
        compare_a_ = static_cast<std::uint32_t>(std::max(0, std::min(compare_count, maximum)));
    }

    std::uint32_t compare_a() const noexcept
    {
        return compare_a_;
    }

    void set_enabled(bool enabled) noexcept
    {
        enabled_ = enabled;
    }

    bool enabled() const noexcept
    {
        return enabled_;
    }

    T switching_frequency_hz() const noexcept
    {
        return config_.time_base_clock_hz / full_cycle_count_;
    }

    T switching_period_s() const noexcept
    {
        return full_cycle_count_ / config_.time_base_clock_hz;
    }

    output_type sample(T time_s) const
    {
        if (!enabled_)
            return output_type{0U, 0U};
        if (!std::isfinite(time_s))
            throw std::invalid_argument("non-finite TI ePWM sample time");

        const T compare = T(compare_a_);
        if (compare <= T(0))
            return active_pair(true);
        if (compare >= half_cycle_count_ - T(1))
            return active_pair(false);

        T phase_count = std::fmod(time_s * config_.time_base_clock_hz, full_cycle_count_);
        if (phase_count < T(0))
            phase_count += full_cycle_count_;

        const T upper_rise = compare + T(config_.rising_edge_delay_count);
        const T upper_fall = full_cycle_count_ - compare;
        const T lower_second_rise = upper_fall + T(config_.falling_edge_delay_count);
        const bool center_pulse = phase_count >= upper_rise && phase_count < upper_fall;
        const bool edge_pulse = phase_count < compare || phase_count >= lower_second_rise;

        if (config_.upper_active_above_compare)
            return output_type{center_pulse ? 1U : 0U, edge_pulse ? 1U : 0U};
        return output_type{edge_pulse ? 1U : 0U, center_pulse ? 1U : 0U};
    }

    const config_type &config() const noexcept
    {
        return config_;
    }

  private:
    output_type active_pair(bool above_compare_active) const noexcept
    {
        const bool upper = config_.upper_active_above_compare == above_compare_active;
        return output_type{upper ? 1U : 0U, upper ? 0U : 1U};
    }

    config_type config_;
    T half_cycle_count_{};
    T full_cycle_count_{};
    std::uint32_t compare_a_{};
    bool enabled_{};
};

} // namespace cctl

#endif // CCTL_PERIPHERAL_IF_TI_EPWM_HPP
