#ifndef CCTL_NUMERICAL_SOLVER_FIXED_VECTOR_HPP
#define CCTL_NUMERICAL_SOLVER_FIXED_VECTOR_HPP

#include <array>
#include <cstddef>
#include <initializer_list>
#include <stdexcept>

namespace cctl
{

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

    fixed_vector()
    {
        data_.fill(T(0));
    }

    fixed_vector(std::initializer_list<T> values)
    {
        data_.fill(T(0));
        if (values.size() > N)
            throw std::length_error("too many values for cctl::fixed_vector");

        std::size_t index = 0U;
        for (typename std::initializer_list<T>::const_iterator it = values.begin(); it != values.end(); ++it)
            data_[index++] = *it;
    }

    T &operator[](std::size_t index)
    {
        return data_[index];
    }

    const T &operator[](std::size_t index) const
    {
        return data_[index];
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
    T result = T(0);
    for (std::size_t i = 0U; i < N; ++i)
        result += lhs[i] * rhs[i];
    return result;
}

template <typename T, std::size_t N> inline T squared_norm(const fixed_vector<T, N> &value)
{
    return dot(value, value);
}

} // namespace cctl

#endif // CCTL_NUMERICAL_SOLVER_FIXED_VECTOR_HPP
