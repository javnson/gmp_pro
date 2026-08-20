#ifndef CCTL_PERIPHERAL_IF_TI_EQEP_HPP
#define CCTL_PERIPHERAL_IF_TI_EQEP_HPP

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace cctl
{

/** Host model of the QPOSCNT/QPOSMAX portion of a TI C2000 eQEP. */
template <typename T = double> class ti_eqep
{
  public:
    explicit ti_eqep(std::uint32_t counts_per_revolution = 16384U)
    {
        initialize(counts_per_revolution);
    }

    void initialize(std::uint32_t counts_per_revolution = 16384U)
    {
        if (counts_per_revolution < 2U)
            throw std::invalid_argument("invalid TI eQEP counts per revolution");
        counts_per_revolution_ = counts_per_revolution;
        position_count_ = 0U;
        revolution_count_ = 0;
    }

    std::uint32_t sample_mechanical_angle(T mechanical_angle_rad)
    {
        if (!std::isfinite(mechanical_angle_rad))
            throw std::invalid_argument("non-finite TI eQEP angle");
        const T revolutions = mechanical_angle_rad / two_pi();
        const T whole_revolutions = std::floor(revolutions);
        T fractional_revolution = revolutions - whole_revolutions;
        if (fractional_revolution < T(0))
            fractional_revolution += T(1);
        revolution_count_ = static_cast<std::int64_t>(whole_revolutions);
        position_count_ = static_cast<std::uint32_t>(
            std::floor(fractional_revolution * T(counts_per_revolution_)));
        if (position_count_ >= counts_per_revolution_)
            position_count_ = counts_per_revolution_ - 1U;
        return position_count_;
    }

    std::uint32_t position_count() const noexcept
    {
        return position_count_;
    }

    std::int64_t revolution_count() const noexcept
    {
        return revolution_count_;
    }

    std::uint32_t counts_per_revolution() const noexcept
    {
        return counts_per_revolution_;
    }

  private:
    static T two_pi()
    {
        return T(6.283185307179586476925286766559005768L);
    }

    std::uint32_t counts_per_revolution_{};
    std::uint32_t position_count_{};
    std::int64_t revolution_count_{};
};

} // namespace cctl

#endif // CCTL_PERIPHERAL_IF_TI_EQEP_HPP
