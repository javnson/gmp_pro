#ifndef CCTL_PERIPHERAL_IF_TI_ADC_HPP
#define CCTL_PERIPHERAL_IF_TI_ADC_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace cctl
{

/** Configuration shared by one C2000-style ADC module. */
template <typename T> struct ti_adc_config
{
    std::uint16_t resolution_bits;
    T reference_voltage_v;

    ti_adc_config() : resolution_bits(12U), reference_voltage_v(T(3.3))
    {
    }
};

/**
 * @brief Host model of the result registers of a TI C2000 ADC module.
 *
 * Conversions are immediate because acquisition/conversion latency is much
 * shorter than the controller period represented by this host model. Results
 * are right justified and saturated to the selected 12- or 16-bit range.
 */
template <typename T = double, std::size_t Channels = 16U> class ti_adc
{
  public:
    typedef T scalar_type;
    typedef std::uint32_t result_type;
    typedef ti_adc_config<T> config_type;

    ti_adc()
    {
        initialize(config_type());
    }

    explicit ti_adc(const config_type &config)
    {
        initialize(config);
    }

    void initialize(const config_type &config = config_type())
    {
        if ((config.resolution_bits != 12U && config.resolution_bits != 16U) ||
            !(config.reference_voltage_v > T(0)) ||
            !std::isfinite(config.reference_voltage_v))
            throw std::invalid_argument("invalid TI ADC configuration");
        config_ = config;
        maximum_code_ = (result_type(1U) << config_.resolution_bits) - result_type(1U);
        result_.fill(result_type(0U));
    }

    result_type sample_adc_voltage(std::size_t channel, T adc_voltage_v)
    {
        validate_channel(channel);
        if (!std::isfinite(adc_voltage_v))
            throw std::invalid_argument("non-finite TI ADC input");
        const T clipped = std::max(T(0), std::min(adc_voltage_v, config_.reference_voltage_v));
        result_[channel] = static_cast<result_type>(
            std::llround(clipped * T(maximum_code_) / config_.reference_voltage_v));
        return result_[channel];
    }

    /** Sample a physical signal after a linear sensor front end. */
    result_type sample_physical(std::size_t channel, T physical_value,
                                T sensitivity_v_per_unit, T bias_voltage_v = T(0))
    {
        if (!std::isfinite(physical_value) || !std::isfinite(sensitivity_v_per_unit) ||
            !std::isfinite(bias_voltage_v))
            throw std::invalid_argument("non-finite TI ADC sensor input");
        return sample_adc_voltage(
            channel, physical_value * sensitivity_v_per_unit + bias_voltage_v);
    }

    result_type result(std::size_t channel) const
    {
        validate_channel(channel);
        return result_[channel];
    }

    const std::array<result_type, Channels> &results() const noexcept
    {
        return result_;
    }

    const config_type &config() const noexcept
    {
        return config_;
    }

    result_type maximum_code() const noexcept
    {
        return maximum_code_;
    }

  private:
    static void validate_channel(std::size_t channel)
    {
        if (channel >= Channels)
            throw std::out_of_range("TI ADC channel index is out of range");
    }

    config_type config_;
    std::array<result_type, Channels> result_{};
    result_type maximum_code_{};
};

} // namespace cctl

#endif // CCTL_PERIPHERAL_IF_TI_ADC_HPP
