/**
 * @file param_esti_RLS.h
 * @author Lin (lammanyee@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-12-31 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <ctl/component/motor_control/basic/motor_current_ctrl.h>
#include <vector>
// #include <gmp_math.h>



// typedef struct _tag_RLS
// {
//     // model: y(t) = w^T * x(t)

//     // RLS param
//     RLS_data_t data;
//     int input_order;  // order of system
//     int output_order;
//     ctrl_gt lambda;  // forgetting factor, regularly 0.998~1
//     std::vector<std::vector<ec_gt>> weights;  // weight coefficient
//     std::vector<std::vector<ctrl_gt>> P;  // covariance matrix

//     // RLS constructor
//     _tag_RLS(int input_order, int output_order, ec_gt forgetting_factor)
//         : input_order(input_order), output_order(output_order), lambda(forgetting_factor)
//     {
//         data_initialize();
//         param_initialize();
//     }

//     // RLS initialize
//     ec_gt param_initialize()
//     {
//         lambda = 1;
//         weights.resize(output_order, std::vector<ec_gt>(input_order, 0.0));
//         P = std::vector<std::vector<ec_gt>>(input_order, std::vector<ec_gt>(input_order, 0.0));
//         for (int i = 0; i < input_order; ++i) {
//             P[i][i] = 1;        // initialize P as const*I, const regularly as 1~10E5
//         }
//         return GMP_EC_OK;
//     }
//     ec_gt data_initialize()
//     {
//         data.input_state.resize(input_order, 0.0);
//         data.output_state.resize(output_order, 0.0);
//         return GMP_EC_OK;
//     }

//     // RLS update
//     ec_gt step(const RLS_data_t& rls_data)
//     {
//         data = rls_data;
//         // // dimension match
//         // if (input_state.size() != order || output_state.size() != order) {
//         //     return GMP_EC_GENERAL_ERROR;
//         // }

//         // **** Kn cal
//         // P(n-1)X(n)
//         std::vector<ctrl_gt> Px(input_order, 0.0);
//         for (int i = 0; i < input_order; ++i)
//         {
//             for (int j = 0; j < input_order; ++j)
//             {
//                 Px[i] += P[i][j] * data.input_state[j];
//             }
//         }

//         // **** for each equation
//         for (int m = 0; m < output_order; ++m)
//         {
//             // K(n) = P(n-1)X(n) / ( lamdba + X(n)'P(n-1)X(n) )
//             ctrl_gt gamma = lambda;
//             ctrl_gt alpha = 0.0;
//             for (int i = 0; i < input_order; ++i)
//             {
//                 alpha += data.input_state[i] * Px[i];
//             }
//             alpha = 1.0 / (gamma + alpha);

//             std::vector<ec_gt> K(input_order, 0.0);
//             for (int i = 0; i < input_order; ++i)
//             {
//                 K[i] = Px[i] * alpha;
//             }

//             // error(n) = y(n) - X(n)'w(n-1)
//             ec_gt error = data.output_state[m];
//             for (int i = 0; i < input_order; ++i)
//             {
//                 error -= weights[m][i] * data.input_state[i];
//             }
//             // w(n) = w(n-1) + k(n)*error(n)
//             for (int i = 0; i < input_order; ++i)
//             {
//                 weights[m][i] += K[i] * error;
//             }

//             // **** Pn update
//             // Pn = (P(n-1)-K(n)X(n)'P(n-1))/lambda
//             std::vector<std::vector<ec_gt>> P_new(input_order, std::vector<ec_gt>(input_order, 0.0));
//             for (int i = 0; i < input_order; ++i)
//             {
//                 for (int j = 0; j < output_order; ++j)
//                 {
//                     P_new[i][j] = (P[i][j] - K[i] * Px[j]) / lambda;
//                 }
//             }
//             // P = P_new;
//         }
//         P = P_new;
//         return GMP_EC_OK;
//     }

//     // get weights result
//     const std::vector<std::vector<ec_gt>>& get_weights() const {
//         return weights;
//     }

// } RLS_t;

typedef struct _tag_RLS_data
{
    std::vector<ctrl_gt> y_vec;
    std::vector<std::vector<ctrl_gt>> x_mat;
} RLS_data_t;

// matrix �?vector 支持2�?�?�?
// 加法，点乘，数乘
// matrix-vector：左乘、右乘，
// matrix: 逆。乘法，加法，转置，行列式，数乘
// 清零，赋�?
// 定义和常数的加法加常数乘以单位矩�?

typedef struct _tag_RLS
{
    // 电机场景其实是Ldq�?
    // model: Y(i) = X(i)^T * W(i)
    // U = L * I

    // Parameter
    ctl_matrix2_t x;
    ctl_vector2_t y;

    ctl_vector2_t weight;

    ctrl_gt forgetting_factor; // default = 1, regularly 0.998~1

    ctl_matrix2_t covariance; // covariance matrix

    // weight

    // RLS param
    // RLS_data_t data;
    // int input_order;  // order of system
    // int output_order;
    // ctrl_gt lambda;  // forgetting factor, regularly 0.998~1
    // std::vector<ctrl_gt>  weights;  // weight coefficient
    // std::vector<std::vector<ctrl_gt>> P;  // covariance matrix

    // // RLS constructor
    // _tag_RLS(int input_order, int output_order, ec_gt forgetting_factor)
    //     : input_order(input_order), output_order(output_order), lambda(forgetting_factor)
    // {
    //     data_initialize();
    //     param_initialize();
    // }
} RLS_t;

void ctl_init_rls(rls_t *rls)
{

    rls->forgetting_factor = float2ctrl(1);

    // rls->covariance.dat[0] = float2ctrl(10);
    // rls->covariance.dat[1] = float2ctrl(0);
    // rls->covariance.dat[2] = float2ctrl(0);
    // rls->covariance.dat[3] = float2ctrl(10);
    ctl_matrix2_set(&rls->covariance, float2ctrl(10), 0, 0, float2ctrl(10));

    // rls->x.dat[0] = 0;
    // rls->x.dat[1] = 0;
    // rls->x.dat[2] = 0;
    // rls->x.dat[3] = 0;

    ctl_matrix2_clear(&rls->x);

    ctl_vector2_clear(&rls->y);

    ctl_vector2_clear(&rls->weight);
}

void ctl_setup_rls(rls_t *rls, ctrl_gt forget_f, ctl_matrix2_t *covariance_0, ctl_vector2_t *weight_0)
{
    rls->forgetting_factor = forget_f;

    ctl_matrix2_copy(&rls->covariance, covariance_0); //  equs memcpy, 用宏计算长度

    ctl_vector2_copy(&rls->weight, weight_0);
}

// RLS initialize
// ec_gt RLS_reset()
// {
//     lambda = 1;
//     weights.resize(output_order, std::vector<ec_gt>(input_order, 0.0));
//     P = std::vector<std::vector<ec_gt>>(input_order, std::vector<ec_gt>(input_order, 0.0));
//     for (int i = 0; i < input_order; ++i) {
//         P[i][i] = 1;        // initialize P as const*I, const regularly as 1~10E5
//     }
//     return GMP_EC_OK;
// }

void RLS_clear(rls_t *rls)
{
    ctl_vector2_clear(&rls->weight);
    ctl_matrix2_clear(&rls->covariance);
}

void ctl_rls_input(rls_t *rls, ctl_vector2_t *x, ctl_matrix2_t *y)
{
    ctl_vector2_copy(&rls->x, x);
    ctl_matrix2_copy(&rls->y, y);
}

// RLS update
void ctl_rls_step(rls_t *rls) //,const RLS_data_t& rls_data)
{

    ctl_matrix2_t px;

    // **** Kn cal
    // P(n-1)X(n)
    ctl_matrix2_mul(&px, &rls->covariance, &rls->x);

    // **** for each equation

    // K(n) = P(n-1)X(n) / ( lamdba + X(n)'P(n-1)X(n) )
    ctl_vector2_t nominator;
    ctl_matrix2_mul_vector(&nominator, &rls->covariance, &rls->x);

    ctl_matrix2_t x_t;
    ctl_matrix2_trans(&x_t, &rls->x);

    ctl_vector2_t denominator;
    ctl_matrix2_mul(&x_t, &x_t, px);

    ctl_matrix2_add_eye(&denominator, &denominator, rls->lambda);

    ctl_vector2_t denominator_inv;
    ctl_matrix2_inv(&denominator_inv, &denominator);

    ctl_matrix2_t kn;
    ctl_matrix2_mul(&denominator_inv, nominator);

    // error(n) = y(n) - X(n)'w(n-1)
    ctl_matrix2_t xw;
    ctl_matrix2_mul(&xw, &x_t, &rls->weight);

    ctl_matrix2_t error;
    ctl_matrix2_sub(&error, &rls->y, &xw);

    // **** Pn update
    // Pn = (P(n-1)-K(n)X(n)'P(n-1))/lambda

    ctl_matrix2_t kxt;
    ctl_matrix2_mul(&kxt, &kn, &x_t);

    ctl_matrix2_t kxtp;
    ctl_matrix2_mul(&kxtp, &kxt, &rls->covariance);

    ctl_matrix2_sub(&rls->covariance, &rls->covariance, &kxtp);

    ctl_matrix2_gain(&rls->covariance, &rls->covariance, 1 / rls->forgetting_factor);

    // data = rls_data;
    // // // dimension match
    // // if (input_state.size() != order || output_state.size() != order) {
    // //     return GMP_EC_GENERAL_ERROR;
    // // }

    // // **** Kn cal
    // // P(n-1)X(n)
    // std::vector<ctrl_gt> Px(input_order, 0.0);
    // for (int i = 0; i < input_order; ++i)
    // {
    //     for (int j = 0; j < input_order; ++j)
    //     {
    //         Px[i] += P[i][j] * data.input_state[j];
    //     }
    // }

    // **** for each equation

    // for (int m = 0; m < output_order; ++m)
    // {
    //      // K(n) = P(n-1)X(n) / ( lamdba + X(n)'P(n-1)X(n) )
    //     ctrl_gt gamma = lambda;
    //     ctrl_gt alpha = 0.0;
    //     for (int i = 0; i < input_order; ++i)
    //     {
    //         alpha += data.input_state[i] * Px[i];
    //     }
    //     alpha = 1.0 / (gamma + alpha);

    //     std::vector<ctrl_gt> K(input_order, 0.0);
    //     for (int i = 0; i < input_order; ++i)
    //     {
    //         K[i] = Px[i] * alpha;
    //     }

    //     // error(n) = y(n) - X(n)'w(n-1)
    //     ctrl_gt error = data.output_state[m];
    //     for (int i = 0; i < input_order; ++i)
    //     {
    //         error -= weights[m][i] * data.input_state[i];
    //     }
    //     // w(n) = w(n-1) + k(n)*error(n)
    //     for (int i = 0; i < input_order; ++i)
    //     {
    //         weights[m][i] += K[i] * error;
    //     }

    //     // **** Pn update
    //     // Pn = (P(n-1)-K(n)X(n)'P(n-1))/lambda
    //     std::vector<std::vector<ctrl_gt>> P_new(input_order, std::vector<ctrl_gt>(input_order, 0.0));
    //     for (int i = 0; i < input_order; ++i)
    //     {
    //         for (int j = 0; j < output_order; ++j)
    //         {
    //             P_new[i][j] = (P[i][j] - K[i] * Px[j]) / lambda;
    //         }
    //     }
    //     // P = P_new;
    // }
    // P = P_new;
}

// get weights result
const std::vector<ctrl_gt> &RLS_get_weights(rls_t) const
{
    return weights;
}

void ctl_get_rls_weight(rls_t *rls, ctl_vector2_t &weight)
{
    ctl_vector2_copy(rls->weight, weight);
}

typedef struct _tag_motor_param
{
    ctl_motor_current_ctrl_t current_value;

    // param
    ctrl_gt Ld;
    ctrl_gt Lq;
    ctrl_gt Rs;
    ctrl_gt Psif;
    ctrl_gt Psif;

    // motion
    ctrl_gt we;

    RLS_t rls;

} ctl_motor_param_t;

ec_gt ctl_param_esti_RLS(ctl_motor_param_t *obj)
{
    obj->rls.y.dat[0] = obj->current_value.vdq0[0] - obj->Rs * obj->current_value.idq0[0];
    obj->rls.y.dat[1] = obj.current_value.vdq0[1] - obj->Rs * obj.current_value.idq0[1] - obj->we * psi_f;
    obj->rls.x.dat[0] = (idq[0] - idq_old[0]) / Ts;
    obj->rls.x.dat[1] = omega_e * idq[1];
    obj->rls.x.dat[2] = omega_e * idq[0];
    obj->rls.x.dat[3] = (idq[1] - idq_old[1]) / Ts;
    ;

    ctl_vector2_copy(&idq_old, &idq);

    return GMP_EC_OK;
}
