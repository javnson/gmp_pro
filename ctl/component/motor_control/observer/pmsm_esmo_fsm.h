/**
 * @file ctl_pmsm_esmo_fsm.h
 * @author GMP Library Contributors
 * @brief 5-State Sensorless FSM Orchestrator (Bumpless Transfer).
 * 
 * @details
 * Implements a fine-grained 5-state machine for permanent magnet synchronous motors (PMSM).
 * It manages the seamless transition between Open-Loop I-F forced dragging and Closed-Loop 
 * Extended Sliding Mode Observer (ESMO) execution. Features dynamic braking current capture 
 * and external slope generator binding for perfect acceleration/deceleration synchronization.
 * 
 * @note 
 * =====================================================================================
 * 1. 状态机切换逻辑 (State Machine Logic)
 * =====================================================================================
 * - [0] ESMO_FSM_STATE_IF_STARTUP:
 *   纯开环 I-F 启动阶段。完全依赖外部传入的 if_slope_ptr 产生虚拟角度。
 *   系统将 if_is_startup 电流完全注入 Q 轴。持续监控 SMO 观测器，一旦观测器锁定且
 *   转速双双越过 speed_up_th_pu 阈值，进入过渡状态。
 * 
 * - [1] ESMO_FSM_STATE_TRANSITION_UP:
 *   加速切入闭环。维持设定时间 (t_trans_sec)。
 *   角度：瞬间硬切至 SMO 物理角度。
 *   电流：根据物理投影计算出的无扰动初值，平滑加权过渡至外部控制器的 PI 输出和 MTPA 给定。
 * 
 * - [2] ESMO_FSM_STATE_CLOSED_LOOP:
 *   全闭环无感运行。角度完全由 SMO 提供，电流完全由外部的速度环和电流环接管。
 *   实时监控转速跌落或 SMO 失锁情况，一旦触发向下阈值，瞬间抓取当前的 Id/Iq 作为刹车电流，
 *   并进入降速过渡状态。
 * 
 * - [3] ESMO_FSM_STATE_TRANSITION_DOWN:
 *   降速切出闭环。
 *   角度：无缝交接给 if_slope_ptr，由开环虚拟角度接管。
 *   电流：外部 PI 输出的权重逐渐降为 0，电流平滑交叉融合至上一步抓取的恒定刹车电流。
 * 
 * - [4] ESMO_FSM_STATE_IF_STOP:
 *   开环停机阶段。维持固定的刹车电流，完全依赖 if_slope_ptr 的减速度将频率降至 0。
 *   当频率接近 0 (<= 0.001pu) 时，状态机自动重置回 IF_STARTUP，等待下一次起步。
 * 
 * =====================================================================================
 * 2. 用户事件钩子响应 (Event Hook Actions)
 * =====================================================================================
 * 当 ctl_step_esmo_fsm() 返回特定的事件标志时，用户（应用层 ISR）必须执行以下操作以保证无扰动：
 * 
 * - 收到 ESMO_FSM_EVENT_JUST_ENTERED_CLOSED_LOOP:
 *   1. 必须立即将 fsm.out_idq_ref.dat[phase_d] 预装载到 Id 电流环的积分器中。
 *   2. 必须立即将 fsm.out_idq_ref.dat[phase_q] 预装载到 Iq 速度环(或电流环)的积分器中。
 *   3. 将外部转速轨迹发生器 (Speed Trajectory) 的当前值，硬对齐至 SMO 的实时转速。
 * 
 * - 收到 ESMO_FSM_EVENT_JUST_EXITED_CLOSED_LOOP:
 *   1. 将开环斜坡发生器 (if_slope_ptr) 的当前频率/转速，硬对齐至 SMO 的实时转速，
 *      确保开环接管的第一拍不会发生频率阶跃。
 * 
 * =====================================================================================
 * 3. 标准初始化与调用流 (Standard Execution Pipeline)
 * =====================================================================================
 * [Init]: 
 *   1. 初始化 esmo 和 if_slope 实例。
 *   2. 调用 ctl_init_esmo_fsm() 绑定指针并配置阈值。
 * [ISR Step]:
 *   1. 执行 ctl_step_esmo_fsm()，获取事件枚举。
 *   2. 使用 switch-case 处理返回的事件（参考上方钩子响应）。
 *   3. 调用 ctl_esmo_fsm_get_active_pos_ift() 获取当前被路由的正确角度，送入 Park 变换。
 *   4. 将 FSM 输出的 fsm.out_idq_ref 直接作为 Id/Iq 电流环控制器的目标给定值运行。
 * 
 * @version 3.0
 * @copyright Copyright GMP(c) 2024-2026
 */

#ifndef _CTL_PMSM_ESMO_FSM_H_
#define _CTL_PMSM_ESMO_FSM_H_

#include <ctl/component/intrinsic/basic/slope_limiter.h>
#include <ctl/component/motor_control/basic/vf_generator.h>
#include <ctl/component/motor_control/observer/pmsm_esmo.h>
#include <ctl/math_block/coordinate/Park.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* --- Enums for State and Events --- */

/**
 * @brief Internal operational states of the 5-State Sensorless FSM.
 */
typedef enum _tag_esmo_fsm_state_e
{
    ESMO_FSM_STATE_IF_STARTUP = 0,      //!< IF startup phase (pure Iq dragging).
    ESMO_FSM_STATE_TRANSITION_UP = 1,   //!< Cross-fade UP: Coordinate projection and power compensation.
    ESMO_FSM_STATE_CLOSED_LOOP = 2,     //!< Full closed-loop sensorless operation.
    ESMO_FSM_STATE_TRANSITION_DOWN = 3, //!< Cross-fade DOWN: Live closed-loop fades to captured brake current.
    ESMO_FSM_STATE_IF_STOP = 4          //!< IF stop phase: Decelerates to 0 using captured brake current.
} ctl_esmo_fsm_state_e;

/**
 * @brief Events emitted by the FSM to trigger Bumpless Transfer actions in the application layer.
 */
typedef enum _tag_esmo_fsm_event_e
{
    ESMO_FSM_EVENT_NONE = 0,                     //!< No critical state change occurred in this cycle.
    ESMO_FSM_EVENT_JUST_ENTERED_CLOSED_LOOP = 1, //!< Triggered on the exact 1st tick of UP transition.
    ESMO_FSM_EVENT_JUST_EXITED_CLOSED_LOOP = 2   //!< Triggered on the exact 1st tick of DOWN transition.
} ctl_esmo_fsm_event_e;

/* --- Main FSM Structure --- */

/**
 * @brief Core structure for the Sensorless State Machine Orchestrator.
 */
typedef struct _tag_pmsm_esmo_fsm_t
{
    // --- Output ---
    ctl_vector2_t out_idq_ref; //!< Highly unified IDQ target output for Current PI loops.

    // --- Current State ---
    ctl_esmo_fsm_state_e state;             //!< Internal state machine tracker.
    ctl_esmo_fsm_event_e current_fsm_event; //!< Output event flag for the current control cycle.

    // --- Core Sub-modules (Pointer-bound) ---
    ctl_slope_f_pu_controller* if_slope_ptr; //!< Pointer to external open-loop slope generator.
    ctl_slope_limiter_t alpha_ramp;          //!< Generates the 0.0 to 1.0 transition weight.
    ctl_pmsm_esmo_t* esmo_ptr;               //!< Pointer to the associated ESMO instance.

    // --- Thresholds & Control Parameters ---
    ctrl_gt speed_up_th_pu;   //!< Speed threshold (PU) to trigger transition to Closed-Loop.
    ctrl_gt speed_down_th_pu; //!< Speed threshold (PU) to fall back to Open-Loop.
    ctrl_gt if_is_startup;    //!< The heavy Is vector magnitude used during forced startup.

    // --- Internal Radar (Hidden from User) ---
    ctrl_gt delta_theta_pu;     //!< Internal load angle (theta_IF - theta_SMO).
    ctl_vector2_t delta_phasor; //!< Internal load angle phasor [sin(delta), cos(delta)] for projection.

    // --- Transition Held Variables ---
    ctrl_gt alpha;               //!< Real-time transition interpolation weight (0.0 to 1.0).
    ctrl_gt transition_id_start; //!< Calculated projected Id used in UP transition.
    ctrl_gt transition_iq_boost; //!< Compensating Iq boost used in UP transition.
    ctrl_gt brake_id_held;       //!< Snapshot of closed-loop Id for DOWN/STOP braking.
    ctrl_gt brake_iq_held;       //!< Snapshot of closed-loop Iq for DOWN/STOP braking.

} ctl_pmsm_esmo_fsm_t;

/*---------------------------------------------------------------------------*/
/* Initialization Functions                                                  */
/*---------------------------------------------------------------------------*/

/**
 * @brief Safely clears and resets the FSM to its default startup state.
 * @param[in,out] fsm Pointer to the FSM instance.
 */
GMP_STATIC_INLINE void ctl_clear_esmo_fsm(ctl_pmsm_esmo_fsm_t* fsm)
{
    fsm->state = ESMO_FSM_STATE_IF_STARTUP;
    fsm->current_fsm_event = ESMO_FSM_EVENT_NONE;

    ctl_vector2_clear(&fsm->out_idq_ref);

    fsm->delta_theta_pu = float2ctrl(0.0f);
    fsm->delta_phasor.dat[phasor_sin] = float2ctrl(0.0f);
    fsm->delta_phasor.dat[phasor_cos] = float2ctrl(1.0f);
}

/**
 * @brief Initializes the ESMO FSM module, binds external pointers, and configures thresholds.
 * @param[out] fsm Pointer to the FSM instance.
 * @param[in] esmo_ptr Pointer to the existing ESMO observer instance.
 * @param[in] if_slope_ptr Pointer to the external IF slope generator instance.
 * @param[in] fs Controller execution frequency in Hz.
 * @param[in] t_trans_sec Desired cross-fade transition duration in seconds.
 * @param[in] up_th_pu Speed threshold (PU) to enter closed-loop.
 * @param[in] down_th_pu Speed threshold (PU) to drop out of closed-loop.
 * @param[in] if_is_start The fixed magnitude of current injected during open-loop start.
 */
void ctl_init_esmo_fsm(ctl_pmsm_esmo_fsm_t* fsm, ctl_pmsm_esmo_t* esmo_ptr, ctl_slope_f_pu_controller* if_slope_ptr,
                       parameter_gt fs, parameter_gt t_trans_sec, parameter_gt up_th_pu, parameter_gt down_th_pu,
                       parameter_gt if_is_start);

/*---------------------------------------------------------------------------*/
/* Core Execution Step Function                                              */
/*---------------------------------------------------------------------------*/

/**
 * @brief Executes one high-frequency step of the 5-State Sensorless FSM Orchestrator.
 * 
 * @param[in,out] fsm Pointer to the FSM instance.
 * @param[in] target_spd_pu The final target speed from user/system (determines startup polarity).
 * @param[in] speed_pi_iq_out The live Iq calculation from the external Speed Controller.
 * @param[in] mtpa_id_ref The live Id calculation from the external MTPA/Field Weakening module.
 * 
 * @return ctl_esmo_fsm_event_e Returns the state change event to trigger user-level bumpless alignment.
 */
GMP_STATIC_INLINE ctl_esmo_fsm_event_e ctl_step_esmo_fsm(ctl_pmsm_esmo_fsm_t* fsm, ctrl_gt target_spd_pu,
                                                         ctrl_gt speed_pi_iq_out, ctrl_gt mtpa_id_ref)
{
    fsm->current_fsm_event = ESMO_FSM_EVENT_NONE;

    ctrl_gt smo_spd = fsm->esmo_ptr->spd_out.speed;
    ctrl_gt smo_theta = fsm->esmo_ptr->pos_out.elec_position;
    fast_gt is_smo_lock = fsm->esmo_ptr->flag_observer_locked;

    // IF startup acts purely on Iq. Polarity follows the target speed direction.
    ctrl_gt sign_dir = (target_spd_pu >= float2ctrl(0.0f)) ? float2ctrl(1.0f) : float2ctrl(-1.0f);
    ctrl_gt iq_startup_signed = ctl_mul(fsm->if_is_startup, sign_dir);

    switch (fsm->state)
    {
    case ESMO_FSM_STATE_IF_STARTUP: {
        ctrl_gt theta_if = ctl_step_slope_f_pu(fsm->if_slope_ptr);

        // Directly inject Is fully into Iq, Id = 0.
        fsm->out_idq_ref.dat[phase_d] = float2ctrl(0.0f);
        fsm->out_idq_ref.dat[phase_q] = iq_startup_signed;

        if (is_smo_lock)
        {
            fsm->delta_theta_pu = ctrl_mod_1(theta_if - smo_theta + float2ctrl(1.0f));
            ctl_set_phasor_via_angle(fsm->delta_theta_pu, &fsm->delta_phasor);

            ctrl_gt abs_smo_spd = ctl_abs(smo_spd);
            ctrl_gt abs_if_spd = ctl_abs(fsm->if_slope_ptr->current_freq_pu);

            if ((abs_smo_spd > fsm->speed_up_th_pu) && (abs_if_spd > fsm->speed_up_th_pu))
            {
                fsm->state = ESMO_FSM_STATE_TRANSITION_UP;
                ctl_set_slope_limiter_current(&fsm->alpha_ramp, float2ctrl(0.0f));

                // iPark2 physical projection of the (0, Is) vector into real rotor frame
                ctl_vector2_t dq_if, dq_real;
                dq_if.dat[phase_d] = float2ctrl(0.0f);
                dq_if.dat[phase_q] = iq_startup_signed;

                ctl_ct_ipark2(&dq_if, &fsm->delta_phasor, &dq_real);

                fsm->transition_id_start = dq_real.dat[phase_alpha];
                fsm->transition_iq_boost = iq_startup_signed - dq_real.dat[phase_beta];

                fsm->out_idq_ref = dq_real;
                fsm->current_fsm_event = ESMO_FSM_EVENT_JUST_ENTERED_CLOSED_LOOP;
            }
        }
        break;
    }

    case ESMO_FSM_STATE_TRANSITION_UP: {
        fsm->alpha = ctl_step_slope_limiter(&fsm->alpha_ramp, float2ctrl(1.0f));
        ctrl_gt one_minus_alpha = float2ctrl(1.0f) - fsm->alpha;

        // Id fades to MTPA, Iq tracks Speed PI + compensating boost.
        fsm->out_idq_ref.dat[phase_d] =
            ctl_mul(fsm->transition_id_start, one_minus_alpha) + ctl_mul(mtpa_id_ref, fsm->alpha);
        fsm->out_idq_ref.dat[phase_q] = speed_pi_iq_out + ctl_mul(fsm->transition_iq_boost, one_minus_alpha);

        fsm->delta_theta_pu = float2ctrl(0.0f);
        fsm->delta_phasor.dat[phasor_sin] = float2ctrl(0.0f);
        fsm->delta_phasor.dat[phasor_cos] = float2ctrl(1.0f);

        if (fsm->alpha >= float2ctrl(1.0f))
        {
            fsm->state = ESMO_FSM_STATE_CLOSED_LOOP;
        }
        break;
    }

    case ESMO_FSM_STATE_CLOSED_LOOP: {
        fsm->out_idq_ref.dat[phase_d] = mtpa_id_ref;
        fsm->out_idq_ref.dat[phase_q] = speed_pi_iq_out;

        fsm->delta_theta_pu = float2ctrl(0.0f);
        fsm->delta_phasor.dat[phasor_sin] = float2ctrl(0.0f);
        fsm->delta_phasor.dat[phasor_cos] = float2ctrl(1.0f);

        if ((ctl_abs(smo_spd) < fsm->speed_down_th_pu) || (is_smo_lock == 0))
        {
            fsm->state = ESMO_FSM_STATE_TRANSITION_DOWN;
            ctl_set_slope_limiter_current(&fsm->alpha_ramp, float2ctrl(0.0f));

            // [Dynamic Braking Capture]: Snapshot live closed-loop current!
            fsm->brake_iq_held = speed_pi_iq_out;
            fsm->brake_id_held = mtpa_id_ref;

            fsm->current_fsm_event = ESMO_FSM_EVENT_JUST_EXITED_CLOSED_LOOP;
        }
        break;
    }

    case ESMO_FSM_STATE_TRANSITION_DOWN: {
        fsm->alpha = ctl_step_slope_limiter(&fsm->alpha_ramp, float2ctrl(1.0f));
        ctrl_gt one_minus_alpha = float2ctrl(1.0f) - fsm->alpha;

        // Ensure the slope generator continues advancing for seamless angle output
        ctl_step_slope_f_pu(fsm->if_slope_ptr);

        // Cross-fade from LIVE speed loop to FROZEN captured braking current
        fsm->out_idq_ref.dat[phase_q] =
            ctl_mul(speed_pi_iq_out, one_minus_alpha) + ctl_mul(fsm->brake_iq_held, fsm->alpha);
        fsm->out_idq_ref.dat[phase_d] = ctl_mul(mtpa_id_ref, one_minus_alpha) + ctl_mul(fsm->brake_id_held, fsm->alpha);

        fsm->delta_theta_pu = float2ctrl(0.0f);
        fsm->delta_phasor.dat[phasor_sin] = float2ctrl(0.0f);
        fsm->delta_phasor.dat[phasor_cos] = float2ctrl(1.0f);

        if (fsm->alpha >= float2ctrl(1.0f))
        {
            fsm->state = ESMO_FSM_STATE_IF_STOP;
        }
        break;
    }

    case ESMO_FSM_STATE_IF_STOP: {
        // Decelerating purely based on the externally configured if_slope
        ctl_step_slope_f_pu(fsm->if_slope_ptr);

        // Hold the dynamic braking current until rotation ceases
        fsm->out_idq_ref.dat[phase_d] = fsm->brake_id_held;
        fsm->out_idq_ref.dat[phase_q] = fsm->brake_iq_held;

        fsm->delta_theta_pu = float2ctrl(0.0f);
        fsm->delta_phasor.dat[phasor_sin] = float2ctrl(0.0f);
        fsm->delta_phasor.dat[phasor_cos] = float2ctrl(1.0f);

        // Reset back to STARTUP if the motor essentially stops rotating
        if (ctl_abs(fsm->if_slope_ptr->current_freq_pu) <= float2ctrl(0.001f))
        {
            fsm->state = ESMO_FSM_STATE_IF_STARTUP;
        }
        break;
    }
    }

    return fsm->current_fsm_event;
}

/*---------------------------------------------------------------------------*/
/* Helper Functions (External API)                                           */
/*---------------------------------------------------------------------------*/

/**
 * @brief Router: Dynamically selects and returns the active Rotation Interface.
 * 
 * @details Instead of exposing raw angles, the FSM acts as a router. The application 
 * layer calls this function to get a pointer to the correct `rotation_ift` 
 * (either from the IF slope generator or the SMO) based on the current state.
 * 
 * @param[in] fsm Pointer to the FSM instance.
 * @return const rotation_ift* Pointer to the active position sensor interface.
 */
GMP_STATIC_INLINE const rotation_ift* ctl_esmo_fsm_get_active_pos_ift(const ctl_pmsm_esmo_fsm_t* fsm)
{
    // During UP transition or full CLOSED LOOP, the angle is hard-cut to SMO.
    if ((fsm->state == ESMO_FSM_STATE_CLOSED_LOOP) || (fsm->state == ESMO_FSM_STATE_TRANSITION_UP))
    {
        return &fsm->esmo_ptr->pos_out;
    }

    // During STARTUP, DOWN transition, or STOP, the virtual slope generator drives the angle.
    return &fsm->if_slope_ptr->enc;
}

#ifdef __cplusplus
}
#endif

#endif // _CTL_PMSM_ESMO_FSM_H_
