#ifndef CCTL_NUMERICAL_SOLVER_FIXED_MATRIX_HPP
#define CCTL_NUMERICAL_SOLVER_FIXED_MATRIX_HPP

#include <array>
#include <cstddef>
#include <initializer_list>
#include <stdexcept>

#include <cctl/numerical_solver/fixed_vector.hpp>

namespace cctl
{

#if (defined(_MSVC_LANG) && _MSVC_LANG >= 201402L) || \
    (!defined(_MSVC_LANG) && __cplusplus >= 201402L)
#define CCTL_FIXED_MATRIX_CONSTEXPR14 constexpr
#else
#define CCTL_FIXED_MATRIX_CONSTEXPR14
#endif

namespace detail
{

template <typename T, std::size_t N> struct fixed_row_axpy_kernel
{
    static void apply(T *result, const T *rhs, T scale)
    {
        for (std::size_t column = 0U; column < N; ++column)
            result[column] += scale * rhs[column];
    }
};

#if defined(CCTL_FIXED_MATH_USE_AVX) && CCTL_FIXED_MATH_USE_AVX
template <std::size_t N> struct fixed_row_axpy_kernel<double, N>
{
    static void apply(double *result, const double *rhs, double scale)
    {
        const __m256d scale_values = _mm256_set1_pd(scale);
        std::size_t column = 0U;
        for (; column + 4U <= N; column += 4U)
        {
            const __m256d result_values = _mm256_loadu_pd(result + column);
            const __m256d rhs_values = _mm256_loadu_pd(rhs + column);
            _mm256_storeu_pd(
                result + column,
                _mm256_add_pd(result_values, _mm256_mul_pd(scale_values, rhs_values)));
        }
        for (; column < N; ++column)
            result[column] += scale * rhs[column];
    }
};

template <std::size_t N> struct fixed_row_axpy_kernel<float, N>
{
    static void apply(float *result, const float *rhs, float scale)
    {
        const __m256 scale_values = _mm256_set1_ps(scale);
        std::size_t column = 0U;
        for (; column + 8U <= N; column += 8U)
        {
            const __m256 result_values = _mm256_loadu_ps(result + column);
            const __m256 rhs_values = _mm256_loadu_ps(rhs + column);
            _mm256_storeu_ps(
                result + column,
                _mm256_add_ps(result_values, _mm256_mul_ps(scale_values, rhs_values)));
        }
        for (; column < N; ++column)
            result[column] += scale * rhs[column];
    }
};
#endif

} // namespace detail

/**
 * @brief Row-major, fixed-size matrix with inline storage.
 *
 * Every row is a fixed_vector and all dimensions are compile-time constants.
 * The type performs no allocation and its short loops are visible to the
 * optimizer for unrolling and vectorization.
 */
template <typename T, std::size_t Rows, std::size_t Columns> class fixed_matrix
{
  public:
    typedef T value_type;
    typedef fixed_vector<T, Columns> row_type;
    static const std::size_t row_count = Rows;
    static const std::size_t column_count = Columns;
    static const std::size_t element_count = Rows * Columns;

    CCTL_FIXED_MATRIX_CONSTEXPR14 fixed_matrix() : rows_{}
    {
    }

    /** Construct from row-major values. Missing values remain zero. */
    CCTL_FIXED_MATRIX_CONSTEXPR14 fixed_matrix(std::initializer_list<T> values) : rows_{}
    {
        if (values.size() > element_count)
            throw std::length_error("too many values for cctl::fixed_matrix");

        const std::size_t safe_columns = Columns == 0U ? 1U : Columns;
        std::size_t index = 0U;
        for (typename std::initializer_list<T>::const_iterator it = values.begin();
             it != values.end(); ++it, ++index)
            rows_[index / safe_columns][index % safe_columns] = *it;
    }

    row_type &operator[](std::size_t row)
    {
        return rows_[row];
    }

    const row_type &operator[](std::size_t row) const
    {
        return rows_[row];
    }

    T &operator()(std::size_t row, std::size_t column)
    {
        return rows_[row][column];
    }

    const T &operator()(std::size_t row, std::size_t column) const
    {
        return rows_[row][column];
    }

    typename std::array<row_type, Rows>::iterator begin()
    {
        return rows_.begin();
    }

    typename std::array<row_type, Rows>::iterator end()
    {
        return rows_.end();
    }

    typename std::array<row_type, Rows>::const_iterator begin() const
    {
        return rows_.begin();
    }

    typename std::array<row_type, Rows>::const_iterator end() const
    {
        return rows_.end();
    }

    void set_zero()
    {
        for (std::size_t row = 0U; row < Rows; ++row)
            rows_[row] = row_type();
    }

    fixed_matrix &operator+=(const fixed_matrix &rhs)
    {
        for (std::size_t row = 0U; row < Rows; ++row)
            rows_[row] += rhs.rows_[row];
        return *this;
    }

    fixed_matrix &operator-=(const fixed_matrix &rhs)
    {
        for (std::size_t row = 0U; row < Rows; ++row)
            rows_[row] -= rhs.rows_[row];
        return *this;
    }

    fixed_matrix &operator*=(T scale)
    {
        for (std::size_t row = 0U; row < Rows; ++row)
            rows_[row] *= scale;
        return *this;
    }

    fixed_matrix &operator/=(T scale)
    {
        for (std::size_t row = 0U; row < Rows; ++row)
            rows_[row] /= scale;
        return *this;
    }

  private:
    std::array<row_type, Rows> rows_;
};

template <typename T, std::size_t Rows, std::size_t Columns>
inline fixed_matrix<T, Rows, Columns>
operator+(fixed_matrix<T, Rows, Columns> lhs,
          const fixed_matrix<T, Rows, Columns> &rhs)
{
    lhs += rhs;
    return lhs;
}

template <typename T, std::size_t Rows, std::size_t Columns>
inline fixed_matrix<T, Rows, Columns>
operator-(fixed_matrix<T, Rows, Columns> lhs,
          const fixed_matrix<T, Rows, Columns> &rhs)
{
    lhs -= rhs;
    return lhs;
}

template <typename T, std::size_t Rows, std::size_t Columns>
inline fixed_matrix<T, Rows, Columns>
operator*(fixed_matrix<T, Rows, Columns> value, T scale)
{
    value *= scale;
    return value;
}

template <typename T, std::size_t Rows, std::size_t Columns>
inline fixed_matrix<T, Rows, Columns>
operator*(T scale, fixed_matrix<T, Rows, Columns> value)
{
    value *= scale;
    return value;
}

template <typename T, std::size_t Rows, std::size_t Columns>
inline fixed_matrix<T, Rows, Columns>
operator/(fixed_matrix<T, Rows, Columns> value, T scale)
{
    value /= scale;
    return value;
}

template <typename T, std::size_t Rows, std::size_t Columns>
inline fixed_vector<T, Rows>
operator*(const fixed_matrix<T, Rows, Columns> &matrix,
          const fixed_vector<T, Columns> &vector)
{
    fixed_vector<T, Rows> result;
    for (std::size_t row = 0U; row < Rows; ++row)
        result[row] = dot(matrix[row], vector);
    return result;
}

/** Evaluate A*x + B*u + bias without materializing intermediate vectors. */
template <typename T, std::size_t Rows, std::size_t StateColumns,
          std::size_t InputColumns>
inline fixed_vector<T, Rows>
affine_transform(const fixed_matrix<T, Rows, StateColumns> &state_matrix,
                 const fixed_vector<T, StateColumns> &state,
                 const fixed_matrix<T, Rows, InputColumns> &input_matrix,
                 const fixed_vector<T, InputColumns> &input,
                 const fixed_vector<T, Rows> &bias)
{
    fixed_vector<T, Rows> result;
    for (std::size_t row = 0U; row < Rows; ++row)
        result[row] = dot(state_matrix[row], state) +
                      dot(input_matrix[row], input) + bias[row];
    return result;
}

template <typename T, std::size_t Rows, std::size_t Inner, std::size_t Columns>
inline fixed_matrix<T, Rows, Columns>
operator*(const fixed_matrix<T, Rows, Inner> &lhs,
          const fixed_matrix<T, Inner, Columns> &rhs)
{
    fixed_matrix<T, Rows, Columns> result;
    for (std::size_t row = 0U; row < Rows; ++row)
        for (std::size_t inner = 0U; inner < Inner; ++inner)
        {
            const T lhs_value = lhs[row][inner];
            detail::fixed_row_axpy_kernel<T, Columns>::apply(
                result[row].data(), rhs[inner].data(), lhs_value);
        }
    return result;
}

} // namespace cctl

#undef CCTL_FIXED_MATRIX_CONSTEXPR14

#endif // CCTL_NUMERICAL_SOLVER_FIXED_MATRIX_HPP
