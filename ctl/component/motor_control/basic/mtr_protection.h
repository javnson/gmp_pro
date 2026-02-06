/**
 * @file basic_pos_loop_p.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implements a Motor protection module.
 * + Over Voltage (ISR Protect)
 * + Under Voltage (Main Loop)
 * + Over Current (ISR Protect)
 * + Control Deviation (ISR Protect)
 * + Stall (Main Loop)
 * @version 0.2
 * @date 2026-02-06
 *
 * @copyright Copyright GMP(c) 2024
 */

#ifndef _FILE_MTR_PROTECTION_H_
#define _FILE_MTR_PROTECTION_H_

#include <ctl/component/intrinsic/basic/divider.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// ==========================================
// Protection Error Codes (Bitmask)
// ==========================================
#define MTR_PROT_NONE       (0x0000)
#define MTR_PROT_OVER_VOLT  (0x0001) // 母线过压
#define MTR_PROT_UNDER_VOLT (0x0002) // 母线欠压
#define MTR_PROT_OVER_CURR  (0x0004) // 软件过流
#define MTR_PROT_DEVIATION  (0x0008) // 控制偏差(失控)

/**
 * @brief Motor Protection Module Structure
 */
typedef struct _tag_mtr_protect
{
    // --- 1. Input Interfaces (Bind by Pointer) ---
    // Use const to ensure safety
    const ctrl_gt* ptr_udc;       //!< DC bus voltage (PU)
    const ctl_vector2_t* ptr_idq; //!< Real Idq current (PU)
    const ctl_vector2_t* ptr_ref; //!< Ref Idq current (PU)

    // --- 2. Output State ---
    uint32_t error_code; //!< Cumulative error flags

    // --- 3. Threshold Parameters (Config) ---
    ctrl_gt limit_ov_pu;     //!< Over Voltage Limit
    ctrl_gt limit_uv_pu;     //!< Under Voltage Limit
    ctrl_gt limit_oc_sq_pu;  //!< Over Current Limit (Squared)
    ctrl_gt limit_dev_sq_pu; //!< Deviation Limit (Squared)

    // --- 4. Counter Limits (Config) ---
    // Small value for fast response, Large value for slow filter
    uint16_t limit_cnt_ov;  //!< Limit for OV counter (e.g., 5)
    uint16_t limit_cnt_uv;  //!< Limit for UV counter (e.g., 100)
    uint16_t limit_cnt_oc;  //!< Limit for OC counter (e.g., 5)
    uint16_t limit_cnt_dev; //!< Limit for Deviation counter (e.g., 1000)

    // --- 5. Internal Counters (State) ---
    uint16_t cnt_ov;  //!< Counter for Over Voltage
    uint16_t cnt_uv;  //!< Counter for Under Voltage
    uint16_t cnt_oc;  //!< Counter for Over Current
    uint16_t cnt_dev; //!< Counter for Deviation

} ctl_mtr_protect_t;

void ctl_init_mtr_protect(ctl_mtr_protect_t* prot, const ctrl_gt* u_dc, const ctl_vector2_t* i_meas,
                          const ctl_vector2_t* i_ref)
{
    // 1. 绑定指针
    prot->ptr_udc = u_dc;
    prot->ptr_idq = i_meas;
    prot->ptr_ref = i_ref;

    // 2. 清除状态
    prot->error_code = PROT_ERR_NONE;
    prot->integrator_val = 0;
    prot->fast_runaway_cnt = 0;

    // 3. 配置默认阈值 (示例值，需根据实际调整)
    // 假设在外部设置，或者这里给默认值
    prot->integ_dec_step = float2ctrl(0.001f);  // 衰减慢
    prot->integ_inc_step = float2ctrl(0.01f);   // 积累快
    prot->limit_error_integ = float2ctrl(1.0f); // 积分上限
}



// return 1 if over sup
GMP_STATIC_INLINE fast_gt ctl_mtr_protect_ov(ctrl_gt dc_bus_voltage, ctrl_gt limit_ov_pu)
{
    return (limit_ov_pu < dc_bus_voltage);
}

// return 1 if over sup
GMP_STATIC_INLINE fast_gt ctl_mtr_protect_oc(ctrl_gt is_sq, ctrl_gt limit_oc_sq_pu)
{
    //ctrl_gt is_sq = ctl_mul(idq->dat[phase_d], idq->dat[phase_d]) + ctl_mul(idq->dat[phase_q], idq->dat[phase_q]);
    return limit_oc_sq_pu < is_sq;
}

// return 1 if under inf
GMP_STATIC_INLINE fast_gt ctl_mtr_protect_uv(ctrl_gt dc_bus_voltage, ctrl_gt limit_uv_pu)
{
    return limit_uv_pu > dc_bus_voltage;
}

// return 1 if over
GMP_STATIC_INLINE fast_gt ctl_mtr_protect_deviation(ctl_vector2_t* idq_ref, ctl_vector2_t* idq, ctrl_gt limit_deviation)
{
    ctrl_gt id_err = idq_ref->dat[phase_d] - idq->dat[phase_d];
    ctrl_gt iq_err = idq_ref->dat[phase_q] - idq->dat[phase_q];

    ctrl_gt is_err_sq = ctl_mul(id_err, id_err) + ctl_mul(iq_err, iq_err);

    return limit_deviation < is_err_sq;
}

GMP_STATIC_INLINE fast_gt ctl_mtr_protect_inc(fast_gt condition, uint16_t* counter, uint16_t cnt_limit)
{
    if (*condition)
        *counter += 1;
    else
    {
        if (*counter >= 1)
            *counter -= 1;
        else
            *counter = 0;
    }

    if (*counter > cnt_limit)
        return 1;
    else
        return 0;
}

/**
 * @brief ISR Level Protection Step
 * Focus: Hardware Safety (OC, OV) and Fast Runaway
 */
GMP_STATIC_INLINE int ctl_step_mtr_protect(ctl_mtr_protect_t* prot)
{
    // 如果已有故障，保持 (Latch)
    if (prot->error_code != PROT_ERR_NONE)
        return 1;

    // 1. 获取值 (通过指针解引用)
    ctrl_gt u_dc = *(prot->ptr_udc);
    ctrl_gt id = prot->ptr_idq->dat[0];
    ctrl_gt iq = prot->ptr_idq->dat[1];

    // 2. 过压保护 (Over Voltage) - 最优先
    if (u_dc > prot->limit_ov_pu)
    {
        prot->error_code |= PROT_ERR_OVER_VOLT;
        return 1;
    }

    // 3. 软件过流 (Software Over Current) - 使用平方比较
    ctrl_gt i_sq = ctl_mul(id, id) + ctl_mul(iq, iq);
    if (i_sq > prot->limit_oc_sq_pu)
    {
        // 这里可以加一个极短的滤波(比如连续3次)防止ADC噪声
        // 为简化代码直接判
        prot->error_code |= PROT_ERR_SW_OC;
        return 1;
    }

    // 4. 快速失控检测 (Fast Runaway) - 针对 FOC 算法发散
    // 逻辑：如果 Id 偏差极大 (例如 > 1.0pu)，说明完全失控
    ctrl_gt id_ref = prot->ptr_ref->dat[0];
    ctrl_gt id_err = ctl_abs(id_ref - id);

    if (id_err > float2ctrl(0.8f))
    { // 0.8pu 巨大偏差
        prot->fast_runaway_cnt++;
        if (prot->fast_runaway_cnt > 20)
        { // 2ms (10kHz)
            prot->error_code |= PROT_ERR_RUNAWAY_FAST;
            return 1;
        }
    }
    else
    {
        if (prot->fast_runaway_cnt > 0)
            prot->fast_runaway_cnt--;
    }

    return 0; // Safe
}

/**
 * @brief Main Loop Protection Dispatch
 * Focus: Slow dynamics, Integration logic (Stall/Overload), UV
 */
void ctl_dispatch_mtr_protect(ctl_mtr_protect_t* prot)
{
    if (prot->error_code != PROT_ERR_NONE)
        return;

    // 1. 欠压保护 (Under Voltage)
    // 放在主循环是因为电池电压变化慢，且瞬间跌落可能不需要立即停机
    if (*(prot->ptr_udc) < prot->limit_uv_pu)
    {
        prot->error_code |= PROT_ERR_UNDER_VOLT;
        return;
    }

    // 2. 跟随误差积分保护 (Integrator Method) - 你的核心需求
    // 计算总电流误差: |I_ref - I_meas|
    // 为节省算力，可用曼哈顿距离 (|dx|+|dy|) 近似欧几里得距离，或只看 Q 轴
    ctrl_gt id_err = prot->ptr_ref->dat[0] - prot->ptr_idq->dat[0];
    ctrl_gt iq_err = prot->ptr_ref->dat[1] - prot->ptr_idq->dat[1];

    // 误差模值平方 (更能反映能量差异)
    ctrl_gt err_sq = ctl_mul(id_err, id_err) + ctl_mul(iq_err, iq_err);

    // 设定一个“正常偏差阈值”，例如 0.2pu 的电流波动是允许的
    ctrl_gt allowed_err_sq = float2ctrl(0.04f); // 0.2 * 0.2

    if (err_sq > allowed_err_sq)
    {
        // 误差过大 -> 积分器增加 (充水)
        prot->integrator_val += prot->integ_inc_step;
    }
    else
    {
        // 误差正常 -> 积分器衰减 (漏水)
        if (prot->integrator_val > prot->integ_dec_step)
            prot->integrator_val -= prot->integ_dec_step;
        else
            prot->integrator_val = 0;
    }

    // 3. 判断积分器是否溢出
    if (prot->integrator_val > prot->limit_error_integ)
    {
        prot->error_code |= PROT_ERR_STALL_INT;
    }

    // 限制积分器最大值防止溢出 (Anti-windup for protection integrator)
    if (prot->integrator_val > ctl_mul(prot->limit_error_integ, float2ctrl(1.5f)))
    {
        prot->integrator_val = ctl_mul(prot->limit_error_integ, float2ctrl(1.5f));
    }
}

/** @} */ // end of POSITION_CONTROLLER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_MTR_PROTECTION_H_
