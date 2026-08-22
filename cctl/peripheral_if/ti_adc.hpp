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

/** Immutable configuration shared by one C2000-style ADC module. */
template <typename T> class ti_adc_config
{
  public:
    ti_adc_config() : resolution_bits(12U), reference_voltage_v(T(3.3))
    {
    }

    /** @return Configured converter resolution. */
    std::uint16_t resolution() const noexcept
    {
        return resolution_bits;
    }

    /** @return Configured positive reference voltage. */
    T reference_voltage() const noexcept
    {
        return reference_voltage_v;
    }

  private:
    template <typename, std::size_t> friend class ti_adc;

    ti_adc_config(std::uint16_t requested_resolution_bits,
                  T requested_reference_voltage_v)
        : resolution_bits(requested_resolution_bits),
          reference_voltage_v(requested_reference_voltage_v)
    {
    }

    std::uint16_t resolution_bits;
    T reference_voltage_v;
};

/**
 * @brief Host model of the result registers of a TI C2000 ADC module.
 *
 * Analog inputs are staged independently from conversion. trigger() latches
 * every staged input into its result register and raises the interrupt flag;
 * the callback overload models immediate ADC-interrupt dispatch. Results are
 * right justified and saturated to the selected 12- or 16-bit range.
 */
template <typename T = double, std::size_t Channels = 16U> class ti_adc
{
  public:
    typedef T scalar_type;
    typedef std::uint32_t result_type;
    typedef ti_adc_config<T> config_type;
    typedef void (*interrupt_handler_type)(void *context);

    /** Build a validated ADC configuration without exposing field mutation. */
    static config_type make_config(std::uint16_t resolution_bits,
                                   T reference_voltage_v)
    {
        return config_type(resolution_bits, reference_voltage_v);
    }

    /** Construct an ADC directly from user-facing configuration arguments. */
    static ti_adc make(std::uint16_t resolution_bits,
                       T reference_voltage_v)
    {
        return ti_adc(make_config(resolution_bits, reference_voltage_v));
    }

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
        quantization_levels_ = result_type(1U) << config_.resolution_bits;
        maximum_code_ = quantization_levels_ - result_type(1U);
        input_voltage_.fill(T(0));
        result_.fill(result_type(0U));
        interrupt_pending_ = false;
        trigger_count_ = 0U;
    }

    /** Reset conversion state while retaining configuration and ISR binding. */
    void reset()
    {
        initialize(config_);
    }

    /** Bind the conversion-complete interrupt dispatched by trigger(). */
    void set_interrupt_handler(interrupt_handler_type handler,
                               void *context = nullptr) noexcept
    {
        interrupt_handler_ = handler;
        interrupt_context_ = context;
    }

    result_type sample_adc_voltage(std::size_t channel, T adc_voltage_v)
    {
        validate_channel(channel);
        if (!std::isfinite(adc_voltage_v))
            throw std::invalid_argument("non-finite TI ADC input");
        const T clipped = std::max(T(0), std::min(adc_voltage_v, config_.reference_voltage_v));
        const result_type code = static_cast<result_type>(
            clipped * T(quantization_levels_) / config_.reference_voltage_v);
        result_[channel] = std::min(code, maximum_code_);
        return result_[channel];
    }

    /** Update one sample-and-hold input without starting a conversion. */
    void set_input_voltage(std::size_t channel, T adc_voltage_v)
    {
        validate_channel(channel);
        if (!std::isfinite(adc_voltage_v))
            throw std::invalid_argument("non-finite TI ADC input");
        input_voltage_[channel] = adc_voltage_v;
    }

    /** Replace every sample-and-hold input without exposing ADC storage. */
    void set_input_voltages(const std::array<T, Channels> &adc_voltages_v)
    {
        for (std::size_t channel = 0U; channel < Channels; ++channel)
            set_input_voltage(channel, adc_voltages_v[channel]);
    }

    T input_voltage(std::size_t channel) const
    {
        validate_channel(channel);
        return input_voltage_[channel];
    }

    /** Latch all staged analog inputs and raise the ADC interrupt flag. */
    bool trigger()
    {
        latch_conversion();
        dispatch_interrupt();
        return true;
    }

    /** Copy result registers through the ADC boundary with type conversion. */
    template <typename Destination>
    void transfer_results(Destination *destination,
                          std::size_t destination_count) const
    {
        if (destination_count > Channels ||
            (destination == nullptr && destination_count != 0U))
            throw std::invalid_argument("invalid TI ADC result destination");
        for (std::size_t channel = 0U; channel < destination_count; ++channel)
            destination[channel] = static_cast<Destination>(result_[channel]);
    }

    /** Latch, transfer result registers, then dispatch the bound ISR. */
    template <typename Destination>
    bool trigger_and_transfer(Destination *destination,
                              std::size_t destination_count)
    {
        latch_conversion();
        transfer_results(destination, destination_count);
        dispatch_interrupt();
        return true;
    }

    bool interrupt_pending() const noexcept
    {
        return interrupt_pending_;
    }

    void acknowledge_interrupt() noexcept
    {
        interrupt_pending_ = false;
    }

    std::uint64_t trigger_count() const noexcept
    {
        return trigger_count_;
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
    void latch_conversion()
    {
        for (std::size_t channel = 0U; channel < Channels; ++channel)
            sample_adc_voltage(channel, input_voltage_[channel]);
        interrupt_pending_ = true;
        ++trigger_count_;
    }

    void dispatch_interrupt()
    {
        if (interrupt_handler_ != nullptr)
            interrupt_handler_(interrupt_context_);
    }

    static void validate_channel(std::size_t channel)
    {
        if (channel >= Channels)
            throw std::out_of_range("TI ADC channel index is out of range");
    }

    config_type config_;
    std::array<T, Channels> input_voltage_{};
    std::array<result_type, Channels> result_{};
    result_type quantization_levels_{};
    result_type maximum_code_{};
    std::uint64_t trigger_count_{};
    bool interrupt_pending_{};
    interrupt_handler_type interrupt_handler_{};
    void *interrupt_context_{};
};

} // namespace cctl

#endif // CCTL_PERIPHERAL_IF_TI_ADC_HPP
