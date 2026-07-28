/**
 * @file ctl_main.cpp
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <xplt.peripheral.h>

//=================================================================================================
// include Necessary control modules

#include <ctl/component/interface/adc_channel.h>
#include <ctl/component/interface/pwm_channel.h>

#include <ctl/component/digital_power/inv/gfl_core.h>
#include <ctl/component/digital_power/inv/gfl_pq_droop_ctrl.h>
#include <ctl/component/digital_power/inv/gfl_pq_ctrl.h>
#include <ctl/component/digital_power/inv/inv_hcm.h>
#include <ctl/component/digital_power/inv/inv_neg_ctrl.h>
#include <ctl/component/digital_power/inv/inv_voltage_ctrl.h>
#include <ctl/component/digital_power/inv/inv_zero_ctrl.h>

#include <ctl/component/interface/spwm_modulator.h>
#include <ctl/math_block/coordinate/svpwm_3d.h>

#include <ctl/framework/cia402_state_machine.h>

#ifndef _FILE_CTL_MAIN_H_
#define _FILE_CTL_MAIN_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//=================================================================================================
// extern controller modules

// System framework
extern cia402_sm_t cia402_sm;

// Control Law Core
extern gfl_inv_ctrl_init_t gfl_init;
extern gfl_inv_ctrl_t inv_ctrl;
extern gfl_pq_ctrl_t pq_ctrl;
extern gfl_pq_droop_init_t pq_droop_init;
extern gfl_pq_droop_ctrl_t pq_droop_ctrl;
extern ctrl_gt gfl_pll_frequency_hz;
extern inv_neg_ctrl_init_t gfl_neg_init;
extern inv_neg_ctrl_t neg_current_ctrl;
extern inv_voltage_ctrl_init_t gfl_voltage_init;
extern inv_voltage_ctrl_t gfl_voltage_ctrl;
extern inv_zero_ctrl_init_t gfl_zero_init;
extern inv_zero_ctrl_t gfl_zero_ctrl;

// Input channel

// Output channel
#if defined USING_NPC_MODULATOR
extern npc_modulator_t spwm;
#else
extern spwm_modulator_t spwm;
#endif // USING_NPC_MODULATOR
#if defined USING_3D_SVPWM
extern ctl_vector4_t pwm_3d_duty;
extern pwm_gt pwm_3d_out[4];
#endif

// Protection module

// ADC Calibrator
extern adc_bias_calibrator_t adc_calibrator;
extern volatile fast_gt flag_enable_adc_calibrator;
extern volatile fast_gt index_adc_calibrator;
extern uint32_t pq_loop_tick;

// User commands

//=================================================================================================
// controller process

// periodic callback function things.
GMP_STATIC_INLINE void ctl_dispatch(void)
{
    // ADC calibrator routine
    if (flag_enable_adc_calibrator)
    {
        if (index_adc_calibrator == 13)
            ctl_step_adc_calibrator(&adc_calibrator, idc.control_port.value);
        else if (index_adc_calibrator == 12)
            ctl_step_adc_calibrator(&adc_calibrator, udc.control_port.value);
        else if (index_adc_calibrator <= 11 && index_adc_calibrator >= 9)
            ctl_step_adc_calibrator(&adc_calibrator, uuvw.control_port.value.dat[index_adc_calibrator - 9]);
        else if (index_adc_calibrator <= 8 && index_adc_calibrator >= 6)
            ctl_step_adc_calibrator(&adc_calibrator, vabc.control_port.value.dat[index_adc_calibrator - 6]);
        else if (index_adc_calibrator <= 5 && index_adc_calibrator >= 3)
            ctl_step_adc_calibrator(&adc_calibrator, iabc.control_port.value.dat[index_adc_calibrator - 3]);
        else if (index_adc_calibrator <= 2)
            ctl_step_adc_calibrator(&adc_calibrator, iuvw.control_port.value.dat[index_adc_calibrator]);
    }

    // normal controller routine
    else
    {
        /*
         * Commissioning levels use fixed references. Reassert them after any
         * CiA402 clear/reset so the ISR cannot run with a stale zero command.
         */
#if BUILD_LEVEL == 1
        ctl_set_gfl_inv_voltage_openloop(&inv_ctrl, float2ctrl(GFL_OPEN_LOOP_VD_PU),
                                         float2ctrl(GFL_OPEN_LOOP_VQ_PU));
#elif BUILD_LEVEL == 2
        ctl_set_gfl_inv_current(&inv_ctrl, float2ctrl(GFL_CURRENT_LEVEL2_ID_PU),
                                float2ctrl(GFL_CURRENT_LEVEL2_IQ_PU));
#elif BUILD_LEVEL == 3
        ctl_set_gfl_inv_current(&inv_ctrl, float2ctrl(GFL_CURRENT_LEVEL3_ID_PU),
                                float2ctrl(GFL_CURRENT_LEVEL3_IQ_PU));
#elif BUILD_LEVEL == 4
        ctl_set_gfl_inv_current(&inv_ctrl, float2ctrl(GFL_CURRENT_LEVEL4_ID_PU),
                                float2ctrl(GFL_CURRENT_LEVEL4_IQ_PU));
#elif BUILD_LEVEL == 5
        ctl_enable_gfl_pq_ctrl(&pq_ctrl);
#if defined GFL_ENABLE_PQ_DROOP
        ctl_set_gfl_pq_ref(&pq_ctrl, pq_droop_ctrl.pq_ref.dat[0],
                           pq_droop_ctrl.pq_ref.dat[1]);
#else
        ctl_set_gfl_pq_ref(&pq_ctrl, float2ctrl(GFL_ACTIVE_POWER_REF_PU),
                           float2ctrl(GFL_REACTIVE_POWER_REF_PU));
#endif
        ctl_set_gfl_inv_current(&inv_ctrl, pq_ctrl.idq_set_out.dat[phase_d],
                                pq_ctrl.idq_set_out.dat[phase_q]);
#endif

        // run controller body
        ctl_step_gfl_inv_ctrl(&inv_ctrl);
#ifdef USING_DSOGI_PLL
        gfl_pll_frequency_hz =
            ctl_mul(inv_ctrl.pll.srf_pll.freq_pu,
                    float2ctrl(GFL_GRID_FREQUENCY_HZ));
#else
        gfl_pll_frequency_hz =
            ctl_mul(inv_ctrl.pll.freq_pu, float2ctrl(GFL_GRID_FREQUENCY_HZ));
#endif
        ctl_step_neg_inv_ctrl(&neg_current_ctrl);
        ctl_step_voltage_inv_ctrl(&gfl_voltage_ctrl);
        ctl_step_zero_inv_ctrl(&gfl_zero_ctrl);

        // Run the P/Q outer loop at its own lower rate. The current loop keeps
        // executing every ISR and consumes the most recent current reference.
        ++pq_loop_tick;
        if (pq_loop_tick >= GFL_PQ_LOOP_DIVIDER)
        {
            pq_loop_tick = 0;
#if defined GFL_ENABLE_PQ_DROOP
            ctl_step_gfl_pq_droop(&pq_droop_ctrl);
            ctl_set_gfl_pq_ref(&pq_ctrl, pq_droop_ctrl.pq_ref.dat[0],
                               pq_droop_ctrl.pq_ref.dat[1]);
#endif
            ctl_step_gfl_pq(&pq_ctrl);

            if (pq_ctrl.flag_enable)
            {
                ctl_set_gfl_inv_current(&inv_ctrl, pq_ctrl.idq_set_out.dat[phase_d],
                                        pq_ctrl.idq_set_out.dat[phase_q]);
            }
        }

        // mix all output
        spwm.vab0_out.dat[phase_A] = inv_ctrl.vab0_out.dat[phase_A] + neg_current_ctrl.vab_out.dat[phase_A];
        spwm.vab0_out.dat[phase_B] = inv_ctrl.vab0_out.dat[phase_B] + neg_current_ctrl.vab_out.dat[phase_B];
        spwm.vab0_out.dat[phase_0] = inv_ctrl.vab0_out.dat[phase_0] + gfl_zero_ctrl.v0_out;

        // modulation
#if defined USING_3D_SVPWM
        {
            ctl_vector3_t vab0_vdc;
            int phase;

            /* Existing controller voltage is normalized to Vdc/2; 3D-SVPWM
             * accepts phase-to-neutral voltage normalized to Vdc. */
            for (phase = 0; phase < 3; ++phase)
                vab0_vdc.dat[phase] = ctl_div2(spwm.vab0_out.dat[phase]);

            ctl_ct_svpwm_3d_calc(&vab0_vdc, &pwm_3d_duty);
            for (phase = 0; phase < 4; ++phase)
                pwm_3d_out[phase] =
                    pwm_sat(pwm_mul(pwm_3d_duty.dat[phase], CTRL_PWM_CMP_MAX), CTRL_PWM_CMP_MAX, 0);
        }
#elif defined USING_NPC_MODULATOR
        ctl_step_npc_modulator(&spwm);
#else
        ctl_step_svpwm_modulator(&spwm);
#endif
    }
}

#ifdef __cplusplus
}
#endif // _cplusplus

#endif // _FILE_CTL_MAIN_H_
