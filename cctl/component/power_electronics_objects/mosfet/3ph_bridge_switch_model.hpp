

// 使用标准的MOSFET半桥模型，将半桥解耦为输入侧和输出侧两个部分。
// 输入侧为一个电流受控源，输出侧为带内阻的电压源。
// 参考《电力电子混杂系统动力学表征与控制》第5章，工业仿真计算机自动化实现。

#include <cctl/numerical_solver/solver_base.hpp>

#ifndef _FILE_3PH_BRIDGE_SWITCH_HPP_
#define _FILE_3PH_BRIDGE_SWITCH_HPP_

template <typename template_type> class mdl_idel_3ph_bridge
{
  public:
    typedef typename template_type _T;
    typedef typename st_null<_T> _st;

  public:
    // parameters
    _T R_on;

    // input variables
    // upper side switch
    uint_fast8_t *sw[3];

    _T *Ubus;
    _T *Ibridge[3];

    // output variables
    _T Ubridge[3]; //  Ubridge = Ubus - (Ibridge[0] + Ibridge[1] + Ibridge[2])*R_on
    _T Ibus;

  public: // ctor
    mdl_idel_3ph_bridge() : R_on(1.0), Ibus(0)
    {
        Ubridge[0] = 0;
        Ubridge[1] = 0;
        Ubridge[2] = 0;
        
    }

  public:
    void operator()()
    {
        this->Ibus = 0;

        for (size_t i = 0; i < 3; ++i)
        {
            if (*sw[i])
            {
                this->Ibus += (*Ibridge[i]);
                this->Ubridge[i] = (*Ubus) - (*Ibridge[i]) * R_on;
            }
            else
            {
                this->Ibus += 0;
                this->Ubridge[i] = -(*Ibridge[i]) * R_on;
            }
        }
    }

    void bind(_T *Ubus, _T *Ibridge[3], uint_fast8_t *sw[3])
    {
        this->Ubus = Ubus;
        for (size_t i = 0; i < 3; ++i)
        {
            this->Ibridge[i] = Ibridge[i];
            this->sw[i] = sw[i];
        }
    }
};

#endif // _FILE_3PH_BRIDGE_SWITCH_HPP_
