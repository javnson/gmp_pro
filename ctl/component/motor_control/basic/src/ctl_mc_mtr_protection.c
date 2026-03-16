/**
 * @file ctl_mc_mtr_protection.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <gmp_core.h>


//////////////////////////////////////////////////////////////////////////
// Motor protection module

#include <ctl/component/motor_control/basic/mtr_protection.h>

void ctl_init_mtr_protect(ctl_mtr_protect_t* prot, parameter_gt fs)
{
    // Default safe values (User should overwrite these)
    prot->limit_ov_pu = float2ctrl(1.2f);
    prot->limit_uv_pu = float2ctrl(0.5f);
    prot->limit_oc_sq_pu = float2ctrl(1.5f * 1.5f);  // 1.5x Overload
    prot->limit_dev_sq_pu = float2ctrl(0.5f * 0.5f); // 0.5pu Deviation
    prot->limit_mtr_ot = 1000;                       // Raw ADC value
    prot->limit_inv_ot = 1000;

    // Default Filtering
    prot->limit_cnt_ov = (uint16_t)(fs / 2000); // Very Fast
    prot->limit_cnt_oc = (uint16_t)(fs / 2000); // Very Fast
    prot->limit_cnt_dev = (uint16_t)(fs / 5);   // Very Slow (e.g., 200ms @ 10kHz)

    prot->limit_cnt_uv = (uint16_t)(fs / 10); // Slow
    prot->limit_cnt_mtr_ot = (uint16_t)fs;
    prot->limit_cnt_inv_ot = (uint16_t)fs;

    // Ensure minimum count is 5 to prevent false triggering on noise
    if (prot->limit_cnt_ov <= 5)
        prot->limit_cnt_ov = 5;
    if (prot->limit_cnt_oc <= 5)
        prot->limit_cnt_oc = 5;

    // Enable all protections by default (Mask = 0)
    prot->error_mask.all = 0;

    ctl_clear_mtr_protect(prot);
}

void ctl_attach_mtr_protect_port(ctl_mtr_protect_t* prot, ctrl_gt* u_dc, ctl_vector2_t* i_meas, ctl_vector2_t* i_ref,
                                 adc_ift* mtr_temp, adc_ift* inv_temp)
{
    prot->ptr_udc = u_dc;
    prot->ptr_idq = i_meas;
    prot->ptr_ref = i_ref;
    prot->ptr_mtr_temp = mtr_temp;
    prot->ptr_inv_temp = inv_temp;
}

/**
 * @brief Main Loop Protection Dispatch. 
 * This calling frequency of this function should be less than 1kHz.
 * @return 1 if fault active, 0 if safe
 */
fast_gt ctl_dispatch_mtr_protect_slow(ctl_mtr_protect_t* prot)
{
    // 1. Latch Check
    if ((prot->error_code.all & (~prot->error_mask.all)) != MTR_PROT_NONE)
        return 1;

    // 2. Check Under Voltage
    ctrl_gt u_dc = *(prot->ptr_udc);
    if (ctl_mtr_protect_debounce(ctl_is_less(u_dc, prot->limit_uv_pu), &prot->cnt_uv, prot->limit_cnt_uv))
    {
        prot->error_code.bit.under_voltage = 1;

        if (!prot->error_mask.bit.under_voltage)
        {
            return 1;
        }
    }

    // 3. Check Motor Over Temp
    if (prot->ptr_mtr_temp)
    {
        // Assuming ptr_mtr_temp->value holds the data
        ctrl_gt temp = prot->ptr_mtr_temp->value;
        if (ctl_mtr_protect_debounce(ctl_is_greater_ot(temp, prot->limit_mtr_ot), &prot->cnt_mtr_ot,
                                     prot->limit_cnt_mtr_ot))
        {
            prot->error_code.bit.mtr_over_temp = 1;

            if (!prot->error_mask.bit.mtr_over_temp)
            {
                return 1;
            }
        }
    }

    // 4. Check Inverter Over Temp
    if (prot->ptr_inv_temp)
    {
        ctrl_gt temp = prot->ptr_inv_temp->value; // Fixed: was reading mtr_temp
        if (ctl_mtr_protect_debounce(ctl_is_greater_ot(temp, prot->limit_inv_ot), &prot->cnt_inv_ot,
                                     prot->limit_cnt_inv_ot))
        {
            prot->error_code.bit.inv_over_temp = 1;

            if (!prot->error_mask.bit.inv_over_temp)
            {
                return 1;
            }
        }
    }

    return 0;
}
