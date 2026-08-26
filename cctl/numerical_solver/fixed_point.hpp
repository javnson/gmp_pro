#ifndef CCTL_NUMERICAL_SOLVER_FIXED_POINT_HPP
#define CCTL_NUMERICAL_SOLVER_FIXED_POINT_HPP

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace cctl
{

template <typename T, std::size_t N> class fixed_vector;

/**
 * @brief Saturating signed 32-bit fixed-point scalar for generated plant models.
 *
 * FractionalBits defines the binary point. Arithmetic stays in integer storage;
 * floating-point conversion is intended only for model inputs and observable
 * outputs at the generated circuit boundary.
 */
template <int FractionalBits> class fixed_point32
{
  public:
    static_assert(FractionalBits > 0 && FractionalBits < 31,
                  "fixed_point32 requires 1..30 fractional bits");
    typedef std::int32_t storage_type;
    static constexpr int fractional_bits = FractionalBits;
    static constexpr std::int64_t scale = INT64_C(1) << FractionalBits;

    constexpr fixed_point32() noexcept : raw_(0)
    {
    }

    explicit constexpr fixed_point32(int value) noexcept
        : raw_(saturate(static_cast<std::int64_t>(value) * scale))
    {
    }

    static constexpr fixed_point32 from_raw(storage_type raw) noexcept
    {
        fixed_point32 value;
        value.raw_ = raw;
        return value;
    }

    static constexpr fixed_point32 from_wide_raw(std::int64_t raw) noexcept
    {
        return from_raw(saturate(raw));
    }

    static fixed_point32 from_double(double value) noexcept
    {
        if (!std::isfinite(value))
            return from_raw(value < 0.0 ? minimum_raw() : maximum_raw());
        const long double scaled = static_cast<long double>(value) * scale;
        if (scaled >= static_cast<long double>(maximum_raw()))
            return from_raw(maximum_raw());
        if (scaled <= static_cast<long double>(minimum_raw()))
            return from_raw(minimum_raw());
        return from_raw(static_cast<storage_type>(std::llround(scaled)));
    }

    constexpr storage_type raw() const noexcept
    {
        return raw_;
    }

    double to_double() const noexcept
    {
        return static_cast<double>(raw_) / static_cast<double>(scale);
    }

    fixed_point32 &operator+=(fixed_point32 rhs) noexcept
    {
        raw_ = saturate(static_cast<std::int64_t>(raw_) + rhs.raw_);
        return *this;
    }

    fixed_point32 &operator-=(fixed_point32 rhs) noexcept
    {
        raw_ = saturate(static_cast<std::int64_t>(raw_) - rhs.raw_);
        return *this;
    }

    fixed_point32 &operator*=(fixed_point32 rhs) noexcept
    {
        const std::int64_t product = static_cast<std::int64_t>(raw_) * rhs.raw_;
        const std::int64_t magnitude = product >= 0 ? product : -product;
        const std::int64_t rounded = (magnitude + (scale >> 1)) / scale;
        raw_ = saturate(product >= 0 ? rounded : -rounded);
        return *this;
    }

    fixed_point32 &operator/=(fixed_point32 rhs) noexcept
    {
        if (rhs.raw_ == 0)
        {
            raw_ = raw_ < 0 ? minimum_raw() : maximum_raw();
            return *this;
        }
        const std::int64_t numerator = static_cast<std::int64_t>(raw_) * scale;
        raw_ = saturate(numerator / rhs.raw_);
        return *this;
    }

    friend fixed_point32 operator+(fixed_point32 lhs, fixed_point32 rhs) noexcept
    {
        lhs += rhs;
        return lhs;
    }

    friend fixed_point32 operator-(fixed_point32 lhs, fixed_point32 rhs) noexcept
    {
        lhs -= rhs;
        return lhs;
    }

    friend fixed_point32 operator-(fixed_point32 value) noexcept
    {
        return from_raw(
            value.raw_ == minimum_raw() ? maximum_raw() : -value.raw_);
    }

    friend fixed_point32 operator*(fixed_point32 lhs, fixed_point32 rhs) noexcept
    {
        lhs *= rhs;
        return lhs;
    }

    friend fixed_point32 operator/(fixed_point32 lhs, fixed_point32 rhs) noexcept
    {
        lhs /= rhs;
        return lhs;
    }

    friend constexpr bool operator==(fixed_point32 lhs, fixed_point32 rhs) noexcept
    {
        return lhs.raw_ == rhs.raw_;
    }

    friend constexpr bool operator!=(fixed_point32 lhs, fixed_point32 rhs) noexcept
    {
        return !(lhs == rhs);
    }

    friend constexpr bool operator<(fixed_point32 lhs, fixed_point32 rhs) noexcept
    {
        return lhs.raw_ < rhs.raw_;
    }

    friend constexpr bool operator<=(fixed_point32 lhs, fixed_point32 rhs) noexcept
    {
        return lhs.raw_ <= rhs.raw_;
    }

    friend constexpr bool operator>(fixed_point32 lhs, fixed_point32 rhs) noexcept
    {
        return lhs.raw_ > rhs.raw_;
    }

    friend constexpr bool operator>=(fixed_point32 lhs, fixed_point32 rhs) noexcept
    {
        return lhs.raw_ >= rhs.raw_;
    }

  private:
    static constexpr storage_type minimum_raw() noexcept
    {
        return (std::numeric_limits<storage_type>::min)();
    }

    static constexpr storage_type maximum_raw() noexcept
    {
        return (std::numeric_limits<storage_type>::max)();
    }

    static constexpr storage_type saturate(std::int64_t value) noexcept
    {
        return value > maximum_raw()
                   ? maximum_raw()
                   : value < minimum_raw() ? minimum_raw()
                                           : static_cast<storage_type>(value);
    }

    storage_type raw_;
};

namespace detail
{

inline std::int64_t saturating_add_int64(std::int64_t lhs,
                                         std::int64_t rhs) noexcept
{
    if (rhs > 0 && lhs > (std::numeric_limits<std::int64_t>::max)() - rhs)
        return (std::numeric_limits<std::int64_t>::max)();
    if (rhs < 0 && lhs < (std::numeric_limits<std::int64_t>::min)() - rhs)
        return (std::numeric_limits<std::int64_t>::min)();
    return lhs + rhs;
}

template <int FractionalBits>
inline std::int64_t rounded_fixed_shift(std::int64_t value) noexcept
{
    const std::int64_t divisor = INT64_C(1) << FractionalBits;
    const std::int64_t half = divisor >> 1;
    if (value >= 0)
    {
        if (value > (std::numeric_limits<std::int64_t>::max)() - half)
            return value / divisor;
        return (value + half) / divisor;
    }
    // Avoid negating INT64_MIN while retaining symmetric rounding.
    if (value == (std::numeric_limits<std::int64_t>::min)())
        return value / divisor;
    const std::int64_t magnitude = -value;
    if (magnitude > (std::numeric_limits<std::int64_t>::max)() - half)
        return value / divisor;
    return -((magnitude + half) / divisor);
}

} // namespace detail

/**
 * Accumulate unlike-Q coefficient/value products in signed 64-bit storage.
 * The result retains ValueFractionalBits and saturates only after the complete
 * dot product, preserving cancellation between large MNA state terms.
 */
template <int CoefficientFractionalBits, int ValueFractionalBits, std::size_t N>
inline fixed_point32<ValueFractionalBits>
mixed_dot(
    const fixed_vector<fixed_point32<CoefficientFractionalBits>, N> &coefficients,
    const fixed_vector<fixed_point32<ValueFractionalBits>, N> &values) noexcept
{
    std::int64_t accumulator = 0;
    for (std::size_t index = 0U; index < N; ++index)
    {
        const std::int64_t product =
            static_cast<std::int64_t>(coefficients[index].raw()) *
            static_cast<std::int64_t>(values[index].raw());
        accumulator = detail::saturating_add_int64(accumulator, product);
    }
    return fixed_point32<ValueFractionalBits>::from_wide_raw(
        detail::rounded_fixed_shift<CoefficientFractionalBits>(accumulator));
}

} // namespace cctl

#endif // CCTL_NUMERICAL_SOLVER_FIXED_POINT_HPP
