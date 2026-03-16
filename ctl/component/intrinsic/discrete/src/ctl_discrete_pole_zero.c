#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// Pole Zero controller
#include <ctl/component/intrinsic/discrete/pole_zero.h>

// Helper function to multiply two first-order polynomials: (b0 + b1*z^-1) * (c0 + c1*z^-1) -> out[0] + out[1]*z^-1 + out[2]*z^-2
//static void _multiply_poly1_poly1(const parameter_gt b[2], const parameter_gt c[2], parameter_gt out[3])
//{
//    out[0] = b[0] * c[0];
//    out[1] = b[0] * c[1] + b[1] * c[0];
//    out[2] = b[1] * c[1];
//}

// Helper function to multiply a second-order and a first-order polynomial
//static void _multiply_poly2_poly1(const parameter_gt b[3], const parameter_gt c[2], parameter_gt out[4])
//{
//    out[0] = b[0] * c[0];
//    out[1] = b[0] * c[1] + b[1] * c[0];
//    out[2] = b[1] * c[1] + b[2] * c[0];
//    out[3] = b[2] * c[1];
//}

// Helper function to multiply a second-order numerator and a first-order numerator polynomial
static void _multiply_num_poly2_poly1(const parameter_gt b[3], const parameter_gt c[2], parameter_gt out[4])
{
    out[0] = b[0] * c[0];
    out[1] = b[0] * c[1] + b[1] * c[0];
    out[2] = b[1] * c[1] + b[2] * c[0];
    out[3] = b[2] * c[1];
}

// Helper function to multiply a 1st-order denominator (1 + a1*z^-1) and a 2nd-order denominator (1 + b1*z^-1 + b2*z^-2)
// The output corresponds to the coefficients of the resulting 3rd-order denominator: 1 + c1*z^-1 + c2*z^-2 + c3*z^-3
static void _multiply_den_poly2_poly1(const parameter_gt p2[2], const parameter_gt p1[1], parameter_gt out_a[3])
{
    // (1 + p2[0]z^-1 + p2[1]z^-2) * (1 + p1[0]z^-1)
    // = 1 + p1[0]z^-1 + p2[0]z^-1 + p1[0]p2[0]z^-2 + p2[1]z^-2 + p1[0]p2[1]z^-3
    // = 1 + (p1[0] + p2[0])z^-1 + (p2[1] + p1[0]p2[0])z^-2 + (p1[0]p2[1])z^-3
    out_a[0] = p1[0] + p2[0];
    out_a[1] = p2[1] + p1[0] * p2[0];
    out_a[2] = p1[0] * p2[1];
}

// Helper to calculate the z-domain polynomial coefficients from s-plane roots.
// Can handle two real roots (r1_hz, r2_hz) or a complex conjugate pair (real_hz, imag_hz).
// The s-plane polynomial is assumed to be s^2 + c1*s + c0.
// The output is the unnormalized z-domain polynomial: coeffs[0] + coeffs[1]*z^-1 + coeffs[2]*z^-2
static void _calc_poly2_coeffs(parameter_gt r1_hz, parameter_gt r2_hz, int is_complex, parameter_gt fs,
                               parameter_gt coeffs[3])
{
    gmp_base_assert(fs > 0.0);

    parameter_gt c0, c1;
    if (is_complex)
    {
        parameter_gt sigma = 2.0 * CTL_PARAM_CONST_PI * r1_hz;
        parameter_gt wd = 2.0 * CTL_PARAM_CONST_PI * r2_hz;
        c1 = 2.0 * sigma;
        c0 = sigma * sigma + wd * wd;
    }
    else
    {
        parameter_gt w1 = 2.0 * CTL_PARAM_CONST_PI * r1_hz;
        parameter_gt w2 = 2.0 * CTL_PARAM_CONST_PI * r2_hz;
        c1 = w1 + w2;
        c0 = w1 * w2;
    }

    parameter_gt k = 2.0 * fs;
    parameter_gt k2 = k * k;

    // 直接返回未归一化的 Z 域系数
    coeffs[0] = k2 + c1 * k + c0;
    coeffs[1] = 2.0 * c0 - 2.0 * k2;
    coeffs[2] = k2 - c1 * k + c0;
}

/*---------------------------------------------------------------------------*/
/* 1P1Z Implementation                                                       */
/*---------------------------------------------------------------------------*/
void ctl_init_1p1z(ctrl_1p1z_t* c, parameter_gt gain, parameter_gt f_z, parameter_gt f_p, parameter_gt fs)
{
    gmp_base_assert(fs > 0.0);

    parameter_gt Kz = tanf(CTL_PARAM_CONST_PI * f_z / fs);
    parameter_gt Kp = tanf(CTL_PARAM_CONST_PI * f_p / fs);

    parameter_gt den_norm = Kp + 1.0;
    if (den_norm < 1e-9)
        den_norm = 1e-9;

    parameter_gt b0 = (Kz + 1.0) / den_norm;
    parameter_gt b1 = (Kz - 1.0) / den_norm;
    parameter_gt a1 = (Kp - 1.0) / den_norm;

    parameter_gt dc_gain_comp = (f_p > 1e-9 && f_z > 1e-9) ? (f_p / f_z) : 1.0;

    // 修复 2：为了补偿零极点带来的固有衰减/放大，必须用乘法！
    parameter_gt final_gain = gain * dc_gain_comp;

    c->coef_b[0] = float2ctrl(b0 * final_gain);
    c->coef_b[1] = float2ctrl(b1 * final_gain);
    c->coef_a[0] = float2ctrl(-a1);

    ctl_clear_1p1z(c);
}

/*---------------------------------------------------------------------------*/
/* 2P2Z Implementation                                                       */
/*---------------------------------------------------------------------------*/
void ctl_init_2p2z_real(ctrl_2p2z_t* c, parameter_gt gain, parameter_gt f_z1, parameter_gt f_z2, parameter_gt f_p1,
                        parameter_gt f_p2, parameter_gt fs)
{
    gmp_base_assert(fs > 0.0);

    parameter_gt num_poly_z[3], den_poly_z[3];
    _calc_poly2_coeffs(f_z1, f_z2, 0, fs, num_poly_z);
    _calc_poly2_coeffs(f_p1, f_p2, 0, fs, den_poly_z);

    parameter_gt norm = 1.0f / den_poly_z[0];
    parameter_gt a1 = den_poly_z[1] * norm;
    parameter_gt a2 = den_poly_z[2] * norm;

    parameter_gt dc_gain_comp = (f_p1 * f_p2) / (f_z1 * f_z2);
    if (f_z1 < 1e-9f || f_z2 < 1e-9f)
        dc_gain_comp = 1.0f;
    parameter_gt final_gain = gain * dc_gain_comp;

    c->coef_b[0] = float2ctrl(num_poly_z[0] * norm * final_gain);
    c->coef_b[1] = float2ctrl(num_poly_z[1] * norm * final_gain);
    c->coef_b[2] = float2ctrl(num_poly_z[2] * norm * final_gain);
    c->coef_a[0] = float2ctrl(a1);
    c->coef_a[1] = float2ctrl(a2);

    ctl_clear_2p2z(c);
}

void ctl_init_2p2z_complex_zeros(ctrl_2p2z_t* c, parameter_gt gain, parameter_gt f_czr, parameter_gt f_czi,
                                 parameter_gt f_p1, parameter_gt f_p2, parameter_gt fs)
{
    gmp_base_assert(fs > 0.0);

    parameter_gt num_poly_z[3], den_poly_z[3];
    _calc_poly2_coeffs(f_czr, f_czi, 1, fs, num_poly_z);
    _calc_poly2_coeffs(f_p1, f_p2, 0, fs, den_poly_z);

    parameter_gt norm = 1.0f / den_poly_z[0];
    parameter_gt a1 = den_poly_z[1] * norm;
    parameter_gt a2 = den_poly_z[2] * norm;

    parameter_gt dc_gain_comp = (f_p1 * f_p2) / (f_czr * f_czr + f_czi * f_czi);
    if (f_czr < 1e-9f && f_czi < 1e-9f)
        dc_gain_comp = 1.0f;
    parameter_gt final_gain = gain * dc_gain_comp;

    c->coef_b[0] = float2ctrl(num_poly_z[0] * norm * final_gain);
    c->coef_b[1] = float2ctrl(num_poly_z[1] * norm * final_gain);
    c->coef_b[2] = float2ctrl(num_poly_z[2] * norm * final_gain);
    c->coef_a[0] = float2ctrl(a1);
    c->coef_a[1] = float2ctrl(a2);

    ctl_clear_2p2z(c);
}

/*---------------------------------------------------------------------------*/
/* 3P3Z Implementation                                                       */
/*---------------------------------------------------------------------------*/
void ctl_init_3p3z_real(ctrl_3p3z_t* c, parameter_gt gain, parameter_gt f_z1, parameter_gt f_z2, parameter_gt f_z3,
                        parameter_gt f_p1, parameter_gt f_p2, parameter_gt f_p3, parameter_gt fs)
{
    int i;

    gmp_base_assert(fs > 0.0);

    ctrl_2p2z_t sec1;
    ctrl_1p1z_t sec2;

    ctl_init_2p2z_real(&sec1, 1.0, f_z1, f_z2, f_p1, f_p2, fs);
    ctl_init_1p1z(&sec2, 1.0, f_z3, f_p3, fs);

    parameter_gt b2[3] = {sec1.coef_b[0], sec1.coef_b[1], sec1.coef_b[2]};
    parameter_gt c2[2] = {sec2.coef_b[0], sec2.coef_b[1]};
    parameter_gt b3[4];
    _multiply_num_poly2_poly1(b2, c2, b3);

    parameter_gt a2[2] = {sec1.coef_a[0], sec1.coef_a[1]};
    parameter_gt a1[1] = {sec2.coef_a[0]};
    parameter_gt a3[3];
    _multiply_den_poly2_poly1(a2, a1, a3);

    parameter_gt dc_gain_comp = (f_p1 * f_p2 * f_p3) / (f_z1 * f_z2 * f_z3);
    if (f_z1 < 1e-9f || f_z2 < 1e-9f || f_z3 < 1e-9f)
        dc_gain_comp = 1.0f;
    parameter_gt final_gain = gain * dc_gain_comp;

    for (i = 0; i < 4; ++i)
        c->coef_b[i] = float2ctrl(b3[i] * final_gain);
    for (i = 0; i < 3; ++i)
        c->coef_a[i] = float2ctrl(a3[i]);
    ctl_clear_3p3z(c);
}

void ctl_init_3p3z_complex_zeros(ctrl_3p3z_t* c, parameter_gt gain, parameter_gt f_czr, parameter_gt f_czi,
                                 parameter_gt f_z3, parameter_gt f_p1, parameter_gt f_p2, parameter_gt f_p3,
                                 parameter_gt fs)
{
    int i;

    gmp_base_assert(fs > 0.0);

    ctrl_2p2z_t sec1;
    ctrl_1p1z_t sec2;
    ctl_init_2p2z_complex_zeros(&sec1, 1.0, f_czr, f_czi, f_p1, f_p2, fs);
    ctl_init_1p1z(&sec2, 1.0, f_z3, f_p3, fs);

    parameter_gt b2[3] = {sec1.coef_b[0], sec1.coef_b[1], sec1.coef_b[2]};
    parameter_gt c2[2] = {sec2.coef_b[0], sec2.coef_b[1]};
    parameter_gt b3[4];
    _multiply_num_poly2_poly1(b2, c2, b3);

    parameter_gt a2[2] = {sec1.coef_a[0], sec1.coef_a[1]};
    parameter_gt a1[1] = {sec2.coef_a[0]};
    parameter_gt a3[3];
    _multiply_den_poly2_poly1(a2, a1, a3);

    parameter_gt dc_gain_comp = (f_p1 * f_p2 * f_p3) / ((f_czr * f_czr + f_czi * f_czi) * f_z3);
    if (f_z3 < 1e-9f)
        dc_gain_comp = 1.0f;
    parameter_gt final_gain = gain * dc_gain_comp;

    for (i = 0; i < 4; ++i)
        c->coef_b[i] = float2ctrl(b3[i] * final_gain);
    for (i = 0; i < 3; ++i)
        c->coef_a[i] = float2ctrl(a3[i]);
    ctl_clear_3p3z(c);
}

void ctl_init_3p3z_complex_poles(ctrl_3p3z_t* c, parameter_gt gain, parameter_gt f_z1, parameter_gt f_z2,
                                 parameter_gt f_z3, parameter_gt f_cpr, parameter_gt f_cpi, parameter_gt f_p3,
                                 parameter_gt fs)
{
    int i;

    gmp_base_assert(fs > 0.0);

    ctrl_2p2z_t sec1;
    ctrl_1p1z_t sec2;

    // Build the 2P2Z section with two real zeros and one complex pole pair
    parameter_gt num_poly[3], den_poly[3];
    _calc_poly2_coeffs(f_z1, f_z2, 0, fs, num_poly);
    _calc_poly2_coeffs(f_cpr, f_cpi, 1, fs, den_poly);

    parameter_gt norm = 1.0f / den_poly[0];
    sec1.coef_a[0] = den_poly[1] * norm;
    sec1.coef_a[1] = den_poly[2] * norm;
    sec1.coef_b[0] = num_poly[0] * norm;
    sec1.coef_b[1] = num_poly[1] * norm;
    sec1.coef_b[2] = num_poly[2] * norm;

    // Build the 1P1Z section with the remaining real pole and zero
    ctl_init_1p1z(&sec2, 1.0, f_z3, f_p3, fs);

    // Multiply the polynomials
    parameter_gt b2[3] = {sec1.coef_b[0], sec1.coef_b[1], sec1.coef_b[2]};
    parameter_gt c2[2] = {sec2.coef_b[0], sec2.coef_b[1]};
    parameter_gt b3[4];
    _multiply_num_poly2_poly1(b2, c2, b3);

    parameter_gt a2[2] = {sec1.coef_a[0], sec1.coef_a[1]};
    parameter_gt a1[1] = {sec2.coef_a[0]};
    parameter_gt a3[3];
    _multiply_den_poly2_poly1(a2, a1, a3);

    parameter_gt dc_gain_comp = ((f_cpr * f_cpr + f_cpi * f_cpi) * f_p3) / (f_z1 * f_z2 * f_z3);
    if (f_z1 < 1e-9f || f_z2 < 1e-9f || f_z3 < 1e-9f)
        dc_gain_comp = 1.0f;
    parameter_gt final_gain = gain * dc_gain_comp;

    for (i = 0; i < 4; ++i)
        c->coef_b[i] = float2ctrl(b3[i] * final_gain);
    for (i = 0; i < 3; ++i)
        c->coef_a[i] = float2ctrl(a3[i]);
    ctl_clear_3p3z(c);
}

void ctl_init_3p3z_complex_pair(ctrl_3p3z_t* c, parameter_gt gain, parameter_gt f_czr, parameter_gt f_czi,
                                parameter_gt f_z3, parameter_gt f_cpr, parameter_gt f_cpi, parameter_gt f_p3,
                                parameter_gt fs)
{
    int i;

    gmp_base_assert(fs > 0.0);

    ctrl_2p2z_t complex_sec;
    ctrl_1p1z_t real_sec;

    // Build the 2P2Z section from the complex pairs
    parameter_gt num_poly[3], den_poly[3];
    _calc_poly2_coeffs(f_czr, f_czi, 1, fs, num_poly);
    _calc_poly2_coeffs(f_cpr, f_cpi, 1, fs, den_poly);

    parameter_gt norm = 1.0f / den_poly[0];
    complex_sec.coef_a[0] = den_poly[1] * norm;
    complex_sec.coef_a[1] = den_poly[2] * norm;
    complex_sec.coef_b[0] = num_poly[0] * norm;
    complex_sec.coef_b[1] = num_poly[1] * norm;
    complex_sec.coef_b[2] = num_poly[2] * norm;

    // Build the 1P1Z section from the real pair
    ctl_init_1p1z(&real_sec, 1.0, f_z3, f_p3, fs);

    // Multiply the polynomials
    parameter_gt b2[3] = {complex_sec.coef_b[0], complex_sec.coef_b[1], complex_sec.coef_b[2]};
    parameter_gt c2[2] = {real_sec.coef_b[0], real_sec.coef_b[1]};
    parameter_gt b3[4];
    _multiply_num_poly2_poly1(b2, c2, b3);

    parameter_gt a2[2] = {complex_sec.coef_a[0], complex_sec.coef_a[1]};
    parameter_gt a1[1] = {real_sec.coef_a[0]};
    parameter_gt a3[3];
    _multiply_den_poly2_poly1(a2, a1, a3);

    parameter_gt dc_gain_comp_c = (f_cpr * f_cpr + f_cpi * f_cpi) / (f_czr * f_czr + f_czi * f_czi);
    parameter_gt dc_gain_comp_r = (f_p3 / f_z3);
    if (f_z3 < 1e-9f)
        dc_gain_comp_r = 1.0f;
    parameter_gt final_gain = gain * (dc_gain_comp_c * dc_gain_comp_r);

    for (i = 0; i < 4; ++i)
        c->coef_b[i] = float2ctrl(b3[i] * final_gain);
    for (i = 0; i < 3; ++i)
        c->coef_a[i] = float2ctrl(a3[i]);
    ctl_clear_3p3z(c);
}

/*---------------------------------------------------------------------------*/
/* Analysis Functions                                                        */
/*---------------------------------------------------------------------------*/
parameter_gt ctl_get_2p2z_gain(ctrl_2p2z_t* c, parameter_gt fs, parameter_gt f)
{
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w), sin_w = sinf(w);
    parameter_gt cos_2w = cosf(2 * w), sin_2w = sinf(2 * w);

    parameter_gt num_real = c->coef_b[0] + c->coef_b[1] * cos_w + c->coef_b[2] * cos_2w;
    parameter_gt num_imag = -c->coef_b[1] * sin_w - c->coef_b[2] * sin_2w;
    parameter_gt den_real = 1.0f + c->coef_a[0] * cos_w + c->coef_a[1] * cos_2w;
    parameter_gt den_imag = -c->coef_a[0] * sin_w - c->coef_a[1] * sin_2w;

    parameter_gt mag_num = sqrtf(num_real * num_real + num_imag * num_imag);
    parameter_gt mag_den = sqrtf(den_real * den_real + den_imag * den_imag);

    return (mag_den < 1e-9f) ? 0.0f : (mag_num / mag_den);
}

parameter_gt ctl_get_2p2z_phase_lag(ctrl_2p2z_t* c, parameter_gt fs, parameter_gt f)
{
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w), sin_w = sinf(w);
    parameter_gt cos_2w = cosf(2 * w), sin_2w = sinf(2 * w);

    parameter_gt num_real = c->coef_b[0] + c->coef_b[1] * cos_w + c->coef_b[2] * cos_2w;
    parameter_gt num_imag = -c->coef_b[1] * sin_w - c->coef_b[2] * sin_2w;
    parameter_gt den_real = 1.0f + c->coef_a[0] * cos_w + c->coef_a[1] * cos_2w;
    parameter_gt den_imag = -c->coef_a[0] * sin_w - c->coef_a[1] * sin_2w;

    parameter_gt phase_num = atan2f(num_imag, num_real);
    parameter_gt phase_den = atan2f(den_imag, den_real);

    return -(phase_num - phase_den);
}

parameter_gt ctl_get_3p3z_gain(ctrl_3p3z_t* c, parameter_gt fs, parameter_gt f)
{
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w), sin_w = sinf(w);
    parameter_gt cos_2w = cosf(2 * w), sin_2w = sinf(2 * w);
    parameter_gt cos_3w = cosf(3 * w), sin_3w = sinf(3 * w);

    parameter_gt num_real = c->coef_b[0] + c->coef_b[1] * cos_w + c->coef_b[2] * cos_2w + c->coef_b[3] * cos_3w;
    parameter_gt num_imag = -c->coef_b[1] * sin_w - c->coef_b[2] * sin_2w - c->coef_b[3] * sin_3w;
    parameter_gt den_real = 1.0f + c->coef_a[0] * cos_w + c->coef_a[1] * cos_2w + c->coef_a[2] * cos_3w;
    parameter_gt den_imag = -c->coef_a[0] * sin_w - c->coef_a[1] * sin_2w - c->coef_a[2] * sin_3w;

    parameter_gt mag_num = sqrtf(num_real * num_real + num_imag * num_imag);
    parameter_gt mag_den = sqrtf(den_real * den_real + den_imag * den_imag);

    return (mag_den < 1e-9f) ? 0.0f : (mag_num / mag_den);
}

parameter_gt ctl_get_3p3z_phase_lag(ctrl_3p3z_t* c, parameter_gt fs, parameter_gt f)
{
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w), sin_w = sinf(w);
    parameter_gt cos_2w = cosf(2 * w), sin_2w = sinf(2 * w);
    parameter_gt cos_3w = cosf(3 * w), sin_3w = sinf(3 * w);

    parameter_gt num_real = c->coef_b[0] + c->coef_b[1] * cos_w + c->coef_b[2] * cos_2w + c->coef_b[3] * cos_3w;
    parameter_gt num_imag = -c->coef_b[1] * sin_w - c->coef_b[2] * sin_2w - c->coef_b[3] * sin_3w;
    parameter_gt den_real = 1.0f + c->coef_a[0] * cos_w + c->coef_a[1] * cos_2w + c->coef_a[2] * cos_3w;
    parameter_gt den_imag = -c->coef_a[0] * sin_w - c->coef_a[1] * sin_2w - c->coef_a[2] * sin_3w;

    parameter_gt phase_num = atan2f(num_imag, num_real);
    parameter_gt phase_den = atan2f(den_imag, den_real);

    return -(phase_num - phase_den);
}
