/**
 * @file vector.hpp
 * @brief Heap-free, fixed-size C++ vector for arbitrary arithmetic types.
 */

#ifndef GMP_CTL_VECTOR_LITE_HPP
#define GMP_CTL_VECTOR_LITE_HPP

#include <stddef.h>

namespace ctl
{
namespace math
{

/** @brief Fixed-size vector with inline storage and generic arithmetic. */
template <typename T, size_t N> class vector_lite
{
  public:
    typedef T value_type;
    enum
    {
        dimension = N
    };

    vector_lite()
    {
        clear();
    }

    explicit vector_lite(const T (&values)[N])
    {
        for (size_t i = 0U; i < N; ++i)
            data_[i] = values[i];
    }

    T& operator[](size_t index)
    {
        return data_[index];
    }

    const T& operator[](size_t index) const
    {
        return data_[index];
    }

    T* data()
    {
        return data_;
    }

    const T* data() const
    {
        return data_;
    }

    void clear()
    {
        for (size_t i = 0U; i < N; ++i)
            data_[i] = T();
    }

    void fill(const T& value)
    {
        for (size_t i = 0U; i < N; ++i)
            data_[i] = value;
    }

    vector_lite& operator+=(const vector_lite& rhs)
    {
        for (size_t i = 0U; i < N; ++i)
            data_[i] = data_[i] + rhs.data_[i];
        return *this;
    }

    vector_lite& operator-=(const vector_lite& rhs)
    {
        for (size_t i = 0U; i < N; ++i)
            data_[i] = data_[i] - rhs.data_[i];
        return *this;
    }

    vector_lite& operator*=(const T& scalar)
    {
        for (size_t i = 0U; i < N; ++i)
            data_[i] = data_[i] * scalar;
        return *this;
    }

    vector_lite& operator/=(const T& scalar)
    {
        for (size_t i = 0U; i < N; ++i)
            data_[i] = data_[i] / scalar;
        return *this;
    }

  private:
    T data_[N];
};

template <typename T, size_t N>
inline vector_lite<T, N> operator+(vector_lite<T, N> lhs, const vector_lite<T, N>& rhs)
{
    lhs += rhs;
    return lhs;
}

template <typename T, size_t N>
inline vector_lite<T, N> operator-(vector_lite<T, N> lhs, const vector_lite<T, N>& rhs)
{
    lhs -= rhs;
    return lhs;
}

template <typename T, size_t N> inline vector_lite<T, N> operator*(vector_lite<T, N> value, const T& scalar)
{
    value *= scalar;
    return value;
}

template <typename T, size_t N> inline vector_lite<T, N> operator*(const T& scalar, vector_lite<T, N> value)
{
    value *= scalar;
    return value;
}

template <typename T, size_t N> inline vector_lite<T, N> operator/(vector_lite<T, N> value, const T& scalar)
{
    value /= scalar;
    return value;
}

template <typename T, size_t N> inline T dot(const vector_lite<T, N>& lhs, const vector_lite<T, N>& rhs)
{
    T result = T();
    for (size_t i = 0U; i < N; ++i)
        result = result + lhs[i] * rhs[i];
    return result;
}

template <typename T, size_t N> inline T squared_norm(const vector_lite<T, N>& value)
{
    return dot(value, value);
}

template <typename T, size_t N>
inline vector_lite<T, N> hadamard(const vector_lite<T, N>& lhs, const vector_lite<T, N>& rhs)
{
    vector_lite<T, N> result;
    for (size_t i = 0U; i < N; ++i)
        result[i] = lhs[i] * rhs[i];
    return result;
}

} // namespace math
} // namespace ctl

#endif // GMP_CTL_VECTOR_LITE_HPP
