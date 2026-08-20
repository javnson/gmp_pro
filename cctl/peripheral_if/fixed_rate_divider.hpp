#ifndef CCTL_PERIPHERAL_IF_FIXED_RATE_DIVIDER_HPP
#define CCTL_PERIPHERAL_IF_FIXED_RATE_DIVIDER_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace cctl
{

/** Deterministic integer divider between a fast plant step and a slow control step. */
template <typename T = double> class fixed_rate_divider
{
  public:
    fixed_rate_divider(T fast_step_s, T slow_step_s)
    {
        initialize(fast_step_s, slow_step_s);
    }

    void initialize(T fast_step_s, T slow_step_s)
    {
        if (!(fast_step_s > T(0)) || !(slow_step_s >= fast_step_s) ||
            !std::isfinite(fast_step_s) || !std::isfinite(slow_step_s))
            throw std::invalid_argument("invalid fixed-rate divider periods");
        const T exact_division = slow_step_s / fast_step_s;
        const T rounded_division = std::round(exact_division);
        const T tolerance = T(1e-9) * std::max(T(1), std::abs(exact_division));
        if (std::abs(exact_division - rounded_division) > tolerance)
            throw std::invalid_argument("slow period is not an integer multiple of fast period");
        division_ = static_cast<std::size_t>(rounded_division);
        counter_ = 0U;
    }

    /** Return true at the first fast step and every division steps thereafter. */
    bool step() noexcept
    {
        const bool dispatch = counter_ == 0U;
        ++counter_;
        if (counter_ >= division_)
            counter_ = 0U;
        return dispatch;
    }

    void reset() noexcept
    {
        counter_ = 0U;
    }

    std::size_t division() const noexcept
    {
        return division_;
    }

  private:
    std::size_t division_{};
    std::size_t counter_{};
};

} // namespace cctl

#endif // CCTL_PERIPHERAL_IF_FIXED_RATE_DIVIDER_HPP
