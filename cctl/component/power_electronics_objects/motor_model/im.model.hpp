
#ifndef _FILE_IM_MODEL_H_
#define _FILE_IM_MODEL_H_

// tex:
//   $$
//[                                           (Lr*Rs)/(Lm^2 - Lr*Ls),  (Lm^2*omega_dA)/(Lm^2 - Lr*Ls) -
//(Lr*Ls*omega_da)/(Lm^2 - Lr*Ls),                                           -(Lm*Rr)/(Lm^2 - Lr*Ls),
//(Lm*Lr*omega_dA)/(Lm^2 - Lr*Ls) - (Lm*Lr*omega_da)/(Lm^2 - Lr*Ls)] \\
//[ (Lr*Ls*omega_da)/(Lm^2 - Lr*Ls) - (Lm^2*omega_dA)/(Lm^2 - Lr*Ls), (Lr*Rs)/(Lm^2 - Lr*Ls), (Lm*Lr*omega_da)/(Lm^2 -
//  Lr*Ls) - (Lm*Lr*omega_dA)/(Lm^2 - Lr*Ls),                                           -(Lm*Rr)/(Lm^2 - Lr*Ls)] \\
//[                                          -(Lm*Rs)/(Lm^2 - Lr*Ls), (Lm*Ls*omega_da)/(Lm^2 - Lr*Ls) -
//(Lm*Ls*omega_dA)/(Lm^2 - Lr*Ls),                                            (Ls*Rr)/(Lm^2 - Lr*Ls),
//(Lm^2*omega_da)/(Lm^2 - Lr*Ls) - (Lr*Ls*omega_dA)/(Lm^2 - Lr*Ls)] \\
//[(Lm*Ls*omega_dA)/(Lm^2 - Lr*Ls) - (Lm*Ls*omega_da)/(Lm^2 - Lr*Ls), -(Lm*Rs)/(Lm^2 - Lr*Ls),  (Lr*Ls*omega_dA)/(Lm^2 -
//  Lr*Ls) - (Lm^2*omega_da)/(Lm^2 - Lr*Ls),                                            (Ls*Rr)/(Lm^2 - Lr*Ls)]
//   $$
//

template <typename template_type> class st_im_motor
{
  public:
    typedef template_type _T;

  public:
    _T isd, isq, ird, irq;
    _T omega_e, theta;

    // ctor & dtor
  public:
    st_im_motor() : isd(0), isq(0), ird(0), irq(0), omega_e(0), theta(0)
    {
    }

    st_im_motor(_T isd, _T isq, _T ird, _T irq, _T omega_e, _T theta)
        : isd(isd), isq(isq), ird(ird), irq(irq), omega_e(omega_e), theta(theta)
    {
    }

  public:
    st_im_motor operator*(const _T &num) const
    {
        return st_im_motor(isd * num, isq * num, ird * num, irq * num, omega_e * num, theta * num);
    }

    st_im_motor operator+(const st_im_motor &num) const
    {
        return st_im_motor(isd * num.isd, isq * num.isq, ird * num.ird, irq * num.irq, omega_e * num.omega_e,
                           theta * num.theta);
    }

    st_im_motor &operator+=(const st_im_motor &&num)
    {
        this->isd = this->isd + num.isd;
        this->isq = this->isq + num.isq;
        this->ird = this->ird + num.ird;
        this->irq = this->irq + num.irq;

        this->omega_e = this->omega_e + num.omega_e;
        this->theta = this->theta + num.theta;

        return *this;
    }
};

template <typename template_type> class diff_im_motor
{
  public:
    typedef typename template_type _T;
    typedef typename st_im_motor<_T> _st;

  public:
    // input variables
    _T *usa, *usb, *usc;
    _T *ura, *urb, *urc;
    _T *t_load;

  public:
    // system parameters
    _T Rs, Rr;
    _T Ls, Lr, Lm;

    uint16_t pole_pair;
    _T damping;
    _T J;

    // A matrix for EM process
    _T A_em[16];

    // B matrix for input process
    _T B_em[16];

    // inductor motor coefficient;
    _T sigma;

    // intermediate variables
    _T lmlm, lmlr, lmls, lrls;

    // output variables
    _T torque;
    _T isa, isb, isc;
    _T ira, irb, irc;

  private:
    // private const
    const _T k_alpha = _T(2.0 / 3.0);      // 2/3
    const _T k_beta = _T(1.0 / sqrt(3.0)); // 1/sqrt(3)
    const _T k_gamma = _T(1.0 / 3.0);      // 1/3
    const _T k_zeta = _T(sqrt(3) / 2.0);   // sqrt(3)/2

  public:
    // ctor:
    diff_im_motor() : torque(0), isa(0), isb(0), isc(0), ira(0), irb(0), irc(0)
    {
    }

  public:
    void bind(_T *usa, _T *usb, _T *usc, _T *ura, _T *urb, _T *urc, _T *load)
    {
        this->usa = usa;
        this->usb = usb;
        this->usc = usc;
        this->ura = ura;
        this->urb = urb;
        this->urc = urc;
        this->t_load = load;
    }

    void set_param(_T Rs, _T Rr, _T Ls, _T Lr, _T Lm, _T J, _T damping, uint16_t p)
    {
        this->Rs = Rs;
        this->Rr = Rr;
        this->Ls = Ls;
        this->Lr = Lr;
        this->Lm = Lm;

        this->J = J;
        this->damping = damping;
        this->pole_pair = p;

        update_state_matrix();
    }

    void update_state_matrix()
    {
        _T omega_da = 0; // 定子固定不转
                         // _T omega_dA = -omega_e; // 转子的速度
        _T omega_dA = 0; // 转子的速度

        sigma = 1 / (Lm * Lm - Lr * Ls);

        // A_em = -(inv(L) * R + inv(L) * Omega * L)
        // A_em[0] = (Lr * Rs) * sigma;
        // A_em[1] = ((Lm * Lm * omega_dA) - (Lr * Ls * omega_da)) * sigma;
        // A_em[2] = -(Lm * Rr) * sigma;
        // A_em[3] = ((Lm * Lr * omega_dA) - (Lm * Lr * omega_da)) * sigma;
        // A_em[4] = (-(Lm * Lm * omega_dA) + (Lr * Ls * omega_da)) * sigma;
        // A_em[5] = (Lr * Rs) * sigma;
        // A_em[6] = (-(Lm * Lr * omega_dA) + (Lm * Lr * omega_da)) * sigma;
        // A_em[7] = -(Lm * Rr) * sigma;
        // A_em[8] = -(Lm * Rs) * sigma;
        // A_em[9] = (-(Lm * Ls * omega_dA) + (Lm * Ls * omega_da)) * sigma;
        // A_em[10] = (Ls * Rr) * sigma;
        // A_em[11] = (-(Lr * Ls * omega_dA) + (Lm * Lm * omega_da)) * sigma;
        // A_em[12] = ((Lm * Ls * omega_dA) - (Lm * Ls * omega_da)) * sigma;
        // A_em[13] = -(Lm * Rs) * sigma;
        // A_em[14] = ((Lr * Ls * omega_dA) - (Lm * Lm * omega_da)) * sigma;
        // A_em[15] = (Ls * Rr) * sigma;

        A_em[0] = (Lr * Rs) * sigma;
        A_em[2] = -(Lm * Rr) * sigma;
        A_em[5] = (Lr * Rs) * sigma;
        A_em[7] = -(Lm * Rr) * sigma;
        A_em[8] = -(Lm * Rs) * sigma;
        A_em[10] = (Ls * Rr) * sigma;
        A_em[13] = -(Lm * Rs) * sigma;
        A_em[15] = (Ls * Rr) * sigma;

        lmlm = Lm * Lm * sigma;
        lmlr = Lm * Lr * sigma;
        lmls = Lm * Ls * sigma;
        lrls = Lm * Ls * sigma;

        _T lmlm_omega = lmlm * omega_dA;
        _T lmlr_omega = lmlr * omega_dA;
        _T lmls_omega = lmls * omega_dA;
        _T lrls_omega = lrls * omega_dA;

        A_em[1] = lmlm_omega;
        A_em[3] = lmlr_omega;
        A_em[4] = -lmlm_omega;
        A_em[6] = -lmlr_omega;
        A_em[9] = -lmls_omega;
        A_em[11] = -lrls_omega;
        A_em[12] = lmls_omega;
        A_em[14] = lrls_omega;

        // B_em = inv[L]
        B_em[0] = -Lr * sigma;
        B_em[1] = 0;
        B_em[2] = Lm * sigma;
        B_em[3] = 0;
        B_em[4] = 0;
        B_em[5] = -Lr * sigma;
        B_em[6] = 0;
        B_em[7] = Lm * sigma;
        B_em[8] = Lm * sigma;
        B_em[9] = 0;
        B_em[10] = -Ls * sigma;
        B_em[11] = 0;
        B_em[12] = 0;
        B_em[13] = Lm * sigma;
        B_em[14] = 0;
        B_em[15] = -Ls * sigma;
    }

    _st operator()(const _st &st)
    {
        _st diff;

        // park transform
        _T vs_alpha, vs_beta;
        _T vsd, vsq;
        _T vr_alpha, vr_beta;
        _T vrd, vrq;

        // stator & rotor clarke and park
        // make transformation for rotor and stator.
        _T sin_theta = sin(st.theta);
        _T cos_theta = cos(st.theta);

        // stator clark and park
        // tex:
        //   $$i_\alpha = 2/3\times i_a - 1/3 \times i_b - 1/3 \times i_c $$
        vs_alpha = k_alpha * ((*usa) - ((*usb) + (*usc)) / 2);
        // tex:
        //   $$i_\beta = \sqrt{3}/3\times i_b - \sqrt{3}/3\times i_c $$
        vs_beta = k_beta * ((*usb) - (*usc));

        // tex:
        //  $$i_d = i_\alpha \times cos\;(\theta) + i_\beta \times sin\;(\theta) $$
        vsd = vs_alpha * cos_theta + vs_beta * sin_theta;

        // tex:
        //$$i_q = - i_\alpha \times sin\;(\theta) + i_\beta \times cos\;(\theta)$$
        vsq = -vs_alpha * sin_theta + vs_beta * cos_theta;

        // tex:
        //   $$i_\alpha = 2/3\times i_a - 1/3 \times i_b - 1/3 \times i_c $$
        vr_alpha = k_alpha * ((*ura) - ((*urb) + (*urc)) / 2);
        // tex:
        //   $$i_\beta = \sqrt{3}/3\times i_b - \sqrt{3}/3\times i_c $$
        vr_beta = k_beta * ((*urb) - (*urc));

        // tex:
        //  $$i_d = i_\alpha \times cos\;(\theta) + i_\beta \times sin\;(\theta) $$
        vrd = vr_alpha * cos_theta + vr_beta * sin_theta;
        // tex:
        //$$i_q = - i_\alpha \times sin\;(\theta) + i_\beta \times cos\;(\theta)$$
        vrq = -vr_alpha * sin_theta + vr_beta * cos_theta;

        // update A matrix
        _T omega_dA = -st.omega_e;

        _T lmlm_omega = lmlm * omega_dA;
        _T lmlr_omega = lmlr * omega_dA;
        _T lmls_omega = lmls * omega_dA;
        _T lrls_omega = lrls * omega_dA;

        // update A matrix items
        A_em[1] = lmlm_omega;
        A_em[3] = lmlr_omega;
        A_em[4] = -lmlm_omega;
        A_em[6] = -lmlr_omega;
        A_em[9] = -lmls_omega;
        A_em[11] = -lrls_omega;
        A_em[12] = lmls_omega;
        A_em[14] = lrls_omega;

        // perform p Idq = A*Idq + B*Udq;
        diff.isd = 0;
        diff.isq = 0;
        diff.ird = 0;
        diff.irq = 0;

        diff.isd += A_em[0 + 0] * st.isd + A_em[0 + 1] * st.isq + A_em[0 + 2] * st.ird + A_em[0 + 3] * st.irq;
        diff.isq += A_em[4 + 0] * st.isd + A_em[4 + 1] * st.isq + A_em[4 + 2] * st.ird + A_em[4 + 3] * st.irq;
        diff.ird += A_em[8 + 0] * st.isd + A_em[8 + 1] * st.isq + A_em[8 + 2] * st.ird + A_em[8 + 3] * st.irq;
        diff.irq += A_em[12 + 0] * st.isd + A_em[12 + 1] * st.isq + A_em[12 + 2] * st.ird + A_em[12 + 3] * st.irq;

        diff.isd += B_em[0 + 0] * vsd + B_em[0 + 1] * vsq + B_em[0 + 2] * vrd + B_em[0 + 3] * vrq;
        diff.isq += B_em[4 + 0] * vsd + B_em[4 + 1] * vsq + B_em[4 + 2] * vrd + B_em[4 + 3] * vrq;
        diff.ird += B_em[8 + 0] * vsd + B_em[8 + 1] * vsq + B_em[8 + 2] * vrd + B_em[8 + 3] * vrq;
        diff.irq += B_em[12 + 0] * vsd + B_em[12 + 1] * vsq + B_em[12 + 2] * vrd + B_em[12 + 3] * vrq;

        // mechanical equations
        // CHECK POINT: constant amplitude transformation has an effort here.
        torque = 3.0 / 2 * pole_pair * Lm * (st.isq * st.ird - st.isd * st.irq);
        diff.omega_e = (torque - (*t_load) - damping * st.omega_e / pole_pair) / J * pole_pair;
        diff.theta = st.omega_e / pole_pair;

        // iClarke iPark to get iabc

        // stator current
        isa = st.isd;
        isb = -st.isd / 2 + st.isq * k_zeta;
        isc = -st.isd / 2 - st.isq * k_zeta;

        // rotor current
        _T ira, irb, irc;

        _T ir_alpha = st.ird * cos_theta - st.irq * sin_theta;
        _T ir_beta = st.ird * sin_theta + st.irq * cos_theta;

        // check point remove i0 is correct, because common mode resistance is infinity.
        ira = ir_alpha;
        irb = -ir_alpha / 2 + ir_beta * k_zeta;
        irc = -ir_alpha / 2 - ir_beta * k_zeta;

        return diff;
    }
};

#endif // _FILE_IM_MODEL_H_
