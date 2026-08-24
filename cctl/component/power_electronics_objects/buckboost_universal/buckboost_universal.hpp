
#ifndef _FILE_BUCKBOOST_UNIVERSAL_H_
#define _FILE_BUCKBOOST_UNIVERSAL_H_

// universal buck boost model
template <typename template_type> class st_universal_buck_boost
{
  public:
    typedef template_type _T;

  public:
    // state variables
    _T il, uc;

    // ctor & dtor
  public:
    st_universal_buck_boost() : il(0), uc(0)
    {
    }

    st_universal_buck_boost(_T il, _T uc) : il(il), uc(uc)
    {
    }

  public:
    st_universal_buck_boost operator*(const _T &num) const
    {
        return st_universal_buck_boost(il * num, ic * num);
    }

    st_universal_buck_boost operator+(const st_universal_buck_boost &num) const
    {
        return st_universal_buck_boost(il + num.il, uc + num.uc);
    }

    st_universal_buck_boost &operator+=(const st_universal_buck_boost &&num)
    {
        this->il = this->il + num.il;
        this->uc = this->uc + num.uc;

        return *this;
    }
};

template <typename template_type> class diff_universal_buck_boost
{
  public:
    typedef typename template_type _T;
    typedef typename st_universal_buck_boost<_T> _st;

  public:
    _T *u_in;
    uint16_t sw;

    _T v_out;

    // parameters
    // system parameter
    _T L;
    _T L_esr;
    _T C;
    _T C_esr;

    _T R_ld; // load

    // transfer matrix
    // transfer function
    _T A_swon[4];
    _T A_swoff[4];

    // input matrix
    _T B_swon[2];
    _T B_swoff[2];

    // state output matrix
    _T C_swon[2];
    _T C_swoff[2];

    // state output direct matrix
    _T D_swon;
    _T D_swoff;

    // main operator
    _st operator()(const _st &st, uint16_t sw)
    {
        _st diff;

        this->sw = sw; // is this correct?!

        if (sw)
        {
            diff.il = A_swon[0] * st.il + A_swon[1] * st.uc + B_swon[0] * (*u_in);
            diff.uc = A_swon[2] * st.il + A_swon[3] * st.uc + B_swon[1] * (*u_in);
            v_out = C_swon[0] * st.il + C_swon[1] * st.uc + D_swon * (*u_in);
        }
        else
        {
            diff.il = A_swoff[0] * st.il + A_swoff[1] * st.uc + B_swoff[0] * (*u_in);
            diff.uc = A_swoff[2] * st.il + A_swoff[3] * st.uc + B_swoff[1] * (*u_in);
            v_out = C_swoff[0] * st.il + C_swoff[1] * st.uc + D_swoff * (*u_in);
        }

        return diff;
    }

    void init_buck_param(_T L, _T L_esr, _T C, _T C_esr, _T R_ld)
    {
        // pass basic structure
        this->L = L;
        this->L_esr = L_esr;
        this->C = C;
        this->C_esr = C_esr;

        // calculate A matrix
        this->A_swon[0] = -(R_ld * C_esr + R_ld * L_esr + C_esr * L_esr) / L / (R_ld + C_esr);
        this->A_swon[1] = -R_ld / L / (R_ld + C_esr);
        this->A_swon[2] = R_ld / C / (R_ld + C_esr);
        this->A_swon[3] = -1.0 / C / (C_esr + R_ld);

        this->A_swoff[0] = -(R_ld * C_esr + R_ld * L_esr + C_esr * L_esr) / L / (R_ld + C_esr);
        this->A_swoff[1] = -R_ld / L / (R_ld + C_esr);
        this->A_swoff[2] = R_ld / C / (R_ld + C_esr);
        this->A_swoff[3] = -1.0 / C / (C_esr + R_ld);

        // calculate B matrix
        this->B_swon[0] = 1 / L;
        this->B_swon[1] = 0;

        this->B_swoff[0] = 0;
        this->B_swoff[1] = 0;

        // calculate C matrix
        this->C_swon[0] = R_ld * C_esr / (R_ld + C_esr);
        this->C_swon[1] = R_ld / (R_ld + C_esr);

        this->C_swoff[0] = R_ld * C_esr / (R_ld + C_esr);
        this->C_swoff[1] = R_ld / (R_ld + C_esr);

        // calculate D matrix
        this->D_swon = 0;
        this->D_swoff = 0;
    }


};

#endif // _FILE_BUCKBOOST_UNIVERSAL_H_
