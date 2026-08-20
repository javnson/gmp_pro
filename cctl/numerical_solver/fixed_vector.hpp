#ifndef CCTL_NUMERICAL_SOLVER_FIXED_VECTOR_HPP
#define CCTL_NUMERICAL_SOLVER_FIXED_VECTOR_HPP

#include <array>
#include <cstddef>
#include <initializer_list>
#include <stdexcept>

#if defined(CCTL_FIXED_MATH_USE_AVX) && CCTL_FIXED_MATH_USE_AVX
#include <immintrin.h>
#endif

namespace cctl
{

#if (defined(_MSVC_LANG) && _MSVC_LANG >= 201402L) || \
    (!defined(_MSVC_LANG) && __cplusplus >= 201402L)
#define CCTL_FIXED_VECTOR_CONSTEXPR14 constexpr
#else
#define CCTL_FIXED_VECTOR_CONSTEXPR14
#endif

namespace detail
{

template <typename T, std::size_t N> struct fixed_dot_kernel
{
    static T evaluate(const T *lhs, const T *rhs)
    {
        T result = T(0);
        for (std::size_t index = 0U; index < N; ++index)
            result += lhs[index] * rhs[index];
        return result;
    }
};

#if defined(CCTL_FIXED_MATH_USE_AVX) && CCTL_FIXED_MATH_USE_AVX
template <std::size_t N> struct fixed_dot_kernel<double, N>
{
    static double evaluate(const double *lhs, const double *rhs)
    {
        __m256d accumulator = _mm256_setzero_pd();
        std::size_t index = 0U;
        for (; index + 4U <= N; index += 4U)
        {
            const __m256d lhs_values = _mm256_loadu_pd(lhs + index);
            const __m256d rhs_values = _mm256_loadu_pd(rhs + index);
            accumulator = _mm256_add_pd(accumulator, _mm256_mul_pd(lhs_values, rhs_values));
        }
        alignas(32) double lanes[4];
        _mm256_store_pd(lanes, accumulator);
        double result = lanes[0] + lanes[1] + lanes[2] + lanes[3];
        for (; index < N; ++index)
            result += lhs[index] * rhs[index];
        return result;
    }
};

template <std::size_t N> struct fixed_dot_kernel<float, N>
{
    static float evaluate(const float *lhs, const float *rhs)
    {
        __m256 accumulator = _mm256_setzero_ps();
        std::size_t index = 0U;
        for (; index + 8U <= N; index += 8U)
        {
            const __m256 lhs_values = _mm256_loadu_ps(lhs + index);
            const __m256 rhs_values = _mm256_loadu_ps(rhs + index);
            accumulator = _mm256_add_ps(accumulator, _mm256_mul_ps(lhs_values, rhs_values));
        }
        alignas(32) float lanes[8];
        _mm256_store_ps(lanes, accumulator);
        float result = lanes[0] + lanes[1] + lanes[2] + lanes[3] +
                       lanes[4] + lanes[5] + lanes[6] + lanes[7];
        for (; index < N; ++index)
            result += lhs[index] * rhs[index];
        return result;
    }
};
#endif

} // namespace detail

/**
 * @brief Small fixed-size arithmetic vector for host-side simulation.
 *
 * The storage is inline and has no heap allocation.  Keeping the dimension a
 * compile-time constant lets an optimizing compiler unroll and vectorize the
 * short loops used by fixed-step ODE solvers.
 */
template <typename T, std::size_t N> class fixed_vector
{
  public:
    typedef T value_type;
    static const std::size_t dimension = N;

    CCTL_FIXED_VECTOR_CONSTEXPR14 fixed_vector() : data_{}
    {
    }

    CCTL_FIXED_VECTOR_CONSTEXPR14 fixed_vector(std::initializer_list<T> values) : data_{}
    {
        if (values.size() > N)
            throw std::length_error("too many values for cctl::fixed_vector");

        std::size_t index = 0U;
        for (typename std::initializer_list<T>::const_iterator it = values.begin(); it != values.end(); ++it)
            data_[index++] = *it;
    }

    CCTL_FIXED_VECTOR_CONSTEXPR14 T &operator[](std::size_t index)
    {
        return data_[index];
    }

    CCTL_FIXED_VECTOR_CONSTEXPR14 const T &operator[](std::size_t index) const
    {
        return data_[index];
    }

    T *data()
    {
        return data_.data();
    }

    const T *data() const
    {
        return data_.data();
    }

    typename std::array<T, N>::iterator begin()
    {
        return data_.begin();
    }

    typename std::array<T, N>::iterator end()
    {
        return data_.end();
    }

    typename std::array<T, N>::const_iterator begin() const
    {
        return data_.begin();
    }

    typename std::array<T, N>::const_iterator end() const
    {
        return data_.end();
    }

    fixed_vector &operator+=(const fixed_vector &rhs)
    {
        for (std::size_t i = 0U; i < N; ++i)
            data_[i] += rhs.data_[i];
        return *this;
    }

    fixed_vector &operator-=(const fixed_vector &rhs)
    {
        for (std::size_t i = 0U; i < N; ++i)
            data_[i] -= rhs.data_[i];
        return *this;
    }

    fixed_vector &operator*=(T scale)
    {
        for (std::size_t i = 0U; i < N; ++i)
            data_[i] *= scale;
        return *this;
    }

    fixed_vector &operator/=(T scale)
    {
        for (std::size_t i = 0U; i < N; ++i)
            data_[i] /= scale;
        return *this;
    }

  private:
    std::array<T, N> data_;
};

template <typename T, std::size_t N>
inline fixed_vector<T, N> operator+(fixed_vector<T, N> lhs, const fixed_vector<T, N> &rhs)
{
    lhs += rhs;
    return lhs;
}

template <typename T, std::size_t N>
inline fixed_vector<T, N> operator-(fixed_vector<T, N> lhs, const fixed_vector<T, N> &rhs)
{
    lhs -= rhs;
    return lhs;
}

template <typename T, std::size_t N> inline fixed_vector<T, N> operator*(fixed_vector<T, N> value, T scale)
{
    value *= scale;
    return value;
}

template <typename T, std::size_t N> inline fixed_vector<T, N> operator*(T scale, fixed_vector<T, N> value)
{
    value *= scale;
    return value;
}

template <typename T, std::size_t N> inline fixed_vector<T, N> operator/(fixed_vector<T, N> value, T scale)
{
    value /= scale;
    return value;
}

template <typename T, std::size_t N> inline T dot(const fixed_vector<T, N> &lhs, const fixed_vector<T, N> &rhs)
{
    return detail::fixed_dot_kernel<T, N>::evaluate(lhs.data(), rhs.data());
}

template <typename T, std::size_t N> inline T squared_norm(const fixed_vector<T, N> &value)
{
    return dot(value, value);
}

} // namespace cctl

#undef CCTL_FIXED_VECTOR_CONSTEXPR14

#endif // CCTL_NUMERICAL_SOLVER_FIXED_VECTOR_HPP
