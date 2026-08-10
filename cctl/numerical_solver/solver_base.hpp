
#ifndef CCTL_NUMERICAL_SOLVER_SOLVER_BASE_HPP
#define CCTL_NUMERICAL_SOLVER_SOLVER_BASE_HPP

#include <cctl/numerical_solver/fixed_vector.hpp>

template <typename template_type> class st_null
{
  public:
    typedef template_type _T;

  public:
    // ctor & dtor
    st_null() = default;

  public:
    st_null operator*(const _T &num) const
    {
        return st_null();
    }

    st_null operator+(const st_null &num) const
    {
        return st_null();
    }

    st_null &operator+=(const st_null &num)
    {
        return *this;
    }
};

#endif // CCTL_NUMERICAL_SOLVER_SOLVER_BASE_HPP
