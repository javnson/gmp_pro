/**
 * @file matrix.hpp
 * @brief Heap-free, fixed-size C++ matrix for arbitrary arithmetic types.
 */

#ifndef GMP_CTL_MATRIX_LITE_HPP
#define GMP_CTL_MATRIX_LITE_HPP

#include <stddef.h>
#include <ctl/math_block/vector_lite/vector.hpp>

namespace ctl
{
namespace math
{

/** @brief Fixed-size matrix with inline storage and generic arithmetic. */
template <typename T, size_t Rows, size_t Cols> class matrix_lite
{
  public:
    typedef T value_type;
    enum
    {
        row_count = Rows,
        column_count = Cols
    };

    matrix_lite()
    {
        clear();
    }

    explicit matrix_lite(const T (&values)[Rows * Cols])
    {
        for (size_t i = 0U; i < Rows * Cols; ++i)
            data_[i] = values[i];
    }

    T& operator()(size_t row, size_t column)
    {
        return data_[row * Cols + column];
    }

    const T& operator()(size_t row, size_t column) const
    {
        return data_[row * Cols + column];
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
        for (size_t i = 0U; i < Rows * Cols; ++i)
            data_[i] = T();
    }

    void fill(const T& value)
    {
        for (size_t i = 0U; i < Rows * Cols; ++i)
            data_[i] = value;
    }

    matrix_lite& operator+=(const matrix_lite& rhs)
    {
        for (size_t i = 0U; i < Rows * Cols; ++i)
            data_[i] = data_[i] + rhs.data_[i];
        return *this;
    }

    matrix_lite& operator-=(const matrix_lite& rhs)
    {
        for (size_t i = 0U; i < Rows * Cols; ++i)
            data_[i] = data_[i] - rhs.data_[i];
        return *this;
    }

    matrix_lite& operator*=(const T& scalar)
    {
        for (size_t i = 0U; i < Rows * Cols; ++i)
            data_[i] = data_[i] * scalar;
        return *this;
    }

    matrix_lite& operator/=(const T& scalar)
    {
        for (size_t i = 0U; i < Rows * Cols; ++i)
            data_[i] = data_[i] / scalar;
        return *this;
    }

    static matrix_lite identity()
    {
        matrix_lite result;
        const size_t diagonal = (Rows < Cols) ? Rows : Cols;
        for (size_t i = 0U; i < diagonal; ++i)
            result(i, i) = T(1);
        return result;
    }

  private:
    T data_[Rows * Cols];
};

template <typename T, size_t Rows, size_t Cols>
inline matrix_lite<T, Rows, Cols> operator+(matrix_lite<T, Rows, Cols> lhs,
                                             const matrix_lite<T, Rows, Cols>& rhs)
{
    lhs += rhs;
    return lhs;
}

template <typename T, size_t Rows, size_t Cols>
inline matrix_lite<T, Rows, Cols> operator-(matrix_lite<T, Rows, Cols> lhs,
                                             const matrix_lite<T, Rows, Cols>& rhs)
{
    lhs -= rhs;
    return lhs;
}

template <typename T, size_t Rows, size_t Cols>
inline matrix_lite<T, Rows, Cols> operator*(matrix_lite<T, Rows, Cols> value, const T& scalar)
{
    value *= scalar;
    return value;
}

template <typename T, size_t Rows, size_t Cols>
inline matrix_lite<T, Rows, Cols> operator*(const T& scalar, matrix_lite<T, Rows, Cols> value)
{
    value *= scalar;
    return value;
}

template <typename T, size_t Rows, size_t Cols>
inline matrix_lite<T, Rows, Cols> operator/(matrix_lite<T, Rows, Cols> value, const T& scalar)
{
    value /= scalar;
    return value;
}

template <typename T, size_t Rows, size_t Inner, size_t Cols>
inline matrix_lite<T, Rows, Cols> operator*(const matrix_lite<T, Rows, Inner>& lhs,
                                             const matrix_lite<T, Inner, Cols>& rhs)
{
    matrix_lite<T, Rows, Cols> result;
    for (size_t row = 0U; row < Rows; ++row)
    {
        for (size_t column = 0U; column < Cols; ++column)
        {
            T value = T();
            for (size_t index = 0U; index < Inner; ++index)
                value = value + lhs(row, index) * rhs(index, column);
            result(row, column) = value;
        }
    }
    return result;
}

template <typename T, size_t Rows, size_t Cols>
inline vector_lite<T, Rows> operator*(const matrix_lite<T, Rows, Cols>& matrix,
                                      const vector_lite<T, Cols>& vector)
{
    vector_lite<T, Rows> result;
    for (size_t row = 0U; row < Rows; ++row)
    {
        T value = T();
        for (size_t column = 0U; column < Cols; ++column)
            value = value + matrix(row, column) * vector[column];
        result[row] = value;
    }
    return result;
}

template <typename T, size_t Rows, size_t Cols>
inline matrix_lite<T, Cols, Rows> transpose(const matrix_lite<T, Rows, Cols>& matrix)
{
    matrix_lite<T, Cols, Rows> result;
    for (size_t row = 0U; row < Rows; ++row)
        for (size_t column = 0U; column < Cols; ++column)
            result(column, row) = matrix(row, column);
    return result;
}

} // namespace math
} // namespace ctl

#endif // GMP_CTL_MATRIX_LITE_HPP
