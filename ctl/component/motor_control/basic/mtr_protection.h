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
// Using generic uint32_t for error codes is standard and efficient.
#define MTR_PROT_NONE          (0x00000000)
#define MTR_PROT_OVER_VOLT     (0x00000001) // Bus Over Voltage
#define MTR_PROT_UNDER_VOLT    (0x00000002) // Bus Under Voltage
#define MTR_PROT_OVER_CURR     (0x00000004) // Software Over Current
#define MTR_PROT_DEVIATION     (0x00000008) // Control Deviation (Runaway)
#define MTR_PROT_MTR_OVER_TEMP (0x00000010) // Motor Over Temp
#define MTR_PROT_INV_OVER_TEMP (0x00000020) // Inverter Over Temp

/**
 * @brief Error Code Union
 * Allows accessing individual flags or the whole word efficiently.
 */
typedef union _tag_mtr_protect_error {
    uint32_t all;
    struct
    {
        uint32_t over_voltage : 1;
        uint32_t under_voltage : 1;
        uint32_t over_current : 1;
        uint32_t deviation : 1;
        uint32_t mtr_over_temp : 1;
        uint32_t inv_over_temp : 1;
        uint32_t reserved : 26;
    } bit;
} mtr_protect_error_t;

/**
 * @brief Motor Protection Module Structure
 */
typedef struct _tag_mtr_protect
{
    // --- 1. Input Interfaces (Bind by Pointer) ---
    // Inputs are const to prevent accidental modification
    ctrl_gt* ptr_udc;       //!< DC bus voltage (PU)
    ctl_vector2_t* ptr_idq; //!< Real Idq current (PU)
    ctl_vector2_t* ptr_ref; //!< Ref Idq current (PU)

    adc_ift* ptr_mtr_temp; //!< Motor Temp Input
    adc_ift* ptr_inv_temp; //!< Inverter Temp Input

    // --- 2. Output State ---
    mtr_protect_error_t error_code; //!< Active Faults
    mtr_protect_error_t error_mask; //!< Mask bits (1 = Disable protection)

    // --- 3. Threshold Parameters (Config) ---
    ctrl_gt limit_ov_pu;     //!< OV Limit
    ctrl_gt limit_uv_pu;     //!< UV Limit
    ctrl_gt limit_oc_sq_pu;  //!< OC Limit (Squared)
    ctrl_gt limit_dev_sq_pu; //!< Deviation Limit (Squared error vector)
    adc_gt limit_mtr_ot;     //!< Motor OT Limit
    adc_gt limit_inv_ot;     //!< Inverter OT Limit

    // --- 4. Counter Limits (Config) ---
    uint16_t limit_cnt_ov;
    uint16_t limit_cnt_uv;
    uint16_t limit_cnt_oc;
    uint16_t limit_cnt_dev;
    uint16_t limit_cnt_mtr_ot;
    uint16_t limit_cnt_inv_ot;

    // --- 5. Internal Counters (State) ---
    uint16_t cnt_ov;
    uint16_t cnt_uv;
    uint16_t cnt_oc;
    uint16_t cnt_dev;
    uint16_t cnt_mtr_ot;
    uint16_t cnt_inv_ot;

} ctl_mtr_protect_t;

// ==========================================
// Core Functions
// ==========================================

GMP_STATIC_INLINE void ctl_clear_mtr_protect(ctl_mtr_protect_t* prot)
{
    prot->error_code.all = MTR_PROT_NONE;

    prot->cnt_ov = 0;
    prot->cnt_uv = 0;
    prot->cnt_oc = 0;
    prot->cnt_dev = 0;
    prot->cnt_mtr_ot = 0;
    prot->cnt_inv_ot = 0;
}

void ctl_init_mtr_protect(ctl_mtr_protect_t* prot, parameter_gt fs);

void ctl_attach_mtr_protect_port(ctl_mtr_protect_t* prot, ctrl_gt* u_dc, ctl_vector2_t* i_meas, ctl_vector2_t* i_ref,
                                 adc_ift* mtr_temp, adc_ift* inv_temp);

// ==========================================
// Helper Functions (Static Inline)
// ==========================================

/**
 * @brief Counter-based Debounce Logic
 * @return 1 if confirmed fault, 0 otherwise
 */
GMP_STATIC_INLINE fast_gt ctl_mtr_protect_debounce(fast_gt condition, uint16_t* counter, uint16_t limit)
{
    if (condition)
    {
        // Increment (Saturate at limit + 1 to prevent overflow)
        if (*counter <= limit)
            (*counter)++;
    }
    else
    {
        // Decrement
        if (*counter > 0)
            (*counter)--;
    }

    return (*counter > limit);
}

// Comparison Helpers
GMP_STATIC_INLINE fast_gt ctl_is_greater(ctrl_gt val, ctrl_gt limit)
{
    return val > limit;
}
GMP_STATIC_INLINE fast_gt ctl_is_less(ctrl_gt val, ctrl_gt limit)
{
    return val < limit;
}

GMP_STATIC_INLINE fast_gt ctl_is_greater_ot(adc_gt temp, adc_gt limit_temp)
{
    return limit_temp > temp;
}

GMP_STATIC_INLINE fast_gt ctl_is_less_ot(adc_gt temp, adc_gt limit_temp)
{
    return limit_temp < temp;
}

GMP_STATIC_INLINE void ctl_set_mtr_protect_mask(ctl_mtr_protect_t* prot, uint32_t mask)
{
    prot->error_mask.all = mask;
}

GMP_STATIC_INLINE uint32_t ctl_get_mtr_protect_mask(ctl_mtr_protect_t *prot)
{
    return prot->error_mask.all;
}

/**
 * @brief ISR Level Protection Step
 * @return 1 if fault active, 0 if safe
 */
GMP_STATIC_INLINE fast_gt ctl_step_mtr_protect_fast(ctl_mtr_protect_t* prot)
{
    // 1. Latch Check
    if ((prot->error_code.all & (~prot->error_mask.all)) != MTR_PROT_NONE)
        return 1;

    // 2. Load Inputs
    ctrl_gt u_dc = *(prot->ptr_udc);
    ctrl_gt id = prot->ptr_idq->dat[0];
    ctrl_gt iq = prot->ptr_idq->dat[1];

    // 3. Check Over Voltage (Critical)
    // Condition: u_dc > limit
    if (ctl_mtr_protect_debounce(ctl_is_greater(u_dc, prot->limit_ov_pu), &prot->cnt_ov, prot->limit_cnt_ov))
    {
        prot->error_code.bit.over_voltage = 1;

        if (!prot->error_mask.bit.over_voltage)
        {
            return 1; // Trip if not masked
        }
    }

    // 4. Check Software Over Current
    // Condition: (id^2 + iq^2) > limit^2
    ctrl_gt i_sq = ctl_mul(id, id) + ctl_mul(iq, iq);
    if (ctl_mtr_protect_debounce(ctl_is_greater(i_sq, prot->limit_oc_sq_pu), &prot->cnt_oc, prot->limit_cnt_oc))
    {
        prot->error_code.bit.over_current = 1;

        if (!prot->error_mask.bit.over_current)
        {
            return 1;
        }
    }

    // 5. Check Control Deviation (Runaway)
    // Condition: |I_ref - I_meas|^2 > limit
    ctrl_gt id_ref = prot->ptr_ref->dat[0];
    ctrl_gt iq_ref = prot->ptr_ref->dat[1];

    ctrl_gt err_d = id_ref - id;
    ctrl_gt err_q = iq_ref - iq;
    ctrl_gt err_sq = ctl_mul(err_d, err_d) + ctl_mul(err_q, err_q);

    if (ctl_mtr_protect_debounce(ctl_is_greater(err_sq, prot->limit_dev_sq_pu), &prot->cnt_dev, prot->limit_cnt_dev))
    {
        prot->error_code.bit.deviation = 1;

        if (!prot->error_mask.bit.deviation)
        {
            return 1;
        }
    }

    return 0; // Safe
}

/**
 * @brief Main Loop Protection Dispatch. 
 * This calling frequency of this function should be less than 1kHz.
 * @return 1 if fault active, 0 if safe
 */
fast_gt ctl_dispatch_mtr_protect_slow(ctl_mtr_protect_t* prot);

/** @} */ // end of POSITION_CONTROLLER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_MTR_PROTECTION_H_
