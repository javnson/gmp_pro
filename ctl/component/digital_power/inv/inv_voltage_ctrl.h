/**
 * @file inv_voltage_ctrl.h
 * @brief LC-filter capacitor-voltage outer loop for stand-alone three-phase inverters.
 */

#ifndef _FILE_DP_INV_VOLTAGE_CTRL_H_
#define _FILE_DP_INV_VOLTAGE_CTRL_H_

#include <ctl/math_block/coordinate/coord_trans.h>

#include <ctl/component/interface/interface_base.h>

#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/component/intrinsic/discrete/biquad_filter.h>

#include <ctl/component/digital_power/inv/gfl_core.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @brief Stand-alone inverter capacitor-voltage controller.
 * @details This is an outer loop. It transforms the attached capacitor voltage
 * with the inner current loop's phasor, regulates d/q voltage with parallel PI
 * controllers, and writes the resulting current reference to the inner loop.
 */
typedef struct _tag_inv_voltage_ctrl
{
    /* Attached ports. */
    ctl_vector2_t* vab;       //!< Capacitor voltage feedback in alpha-beta.
    ctl_vector2_t* phasor;    //!< Inner current-loop phasor {sin, cos}.
    ctl_vector2_t* idq_sink;  //!< Current-reference input of the attached inner loop.

    /* References, feedback and outputs. */
    ctl_vector2_t vdq_set;         //!< Desired capacitor voltage in d-q.
    ctl_vector2_t vdq;             //!< Measured capacitor voltage in d-q.
    ctl_vector2_t idq_pi;          //!< PI contribution to the current reference.
    ctl_vector2_t idq_ff_decouple; //!< Capacitor cross-coupling feed-forward.
    ctl_vector2_t idq_unlimited;   //!< Complete PI plus feed-forward command before vector limiting.
    ctl_vector2_t idq_out;         //!< Limited output current reference.

    ctl_pid_t pid_vdq[2]; //!< Ordinary capacitor-voltage PI controllers.

    ctrl_gt coef_ff_decouple; //!< omega*C*Vbase/Ibase in per-unit.
    ctrl_gt current_circle_limit; //!< D-q current-vector magnitude limit (p.u.).
    ctrl_gt current_square_limit; //!< Independent absolute d/q-axis limit (p.u.).

    fast_gt flag_enable;
    fast_gt flag_enable_decouple;
    fast_gt flag_enable_circle_limit;
    fast_gt flag_enable_square_limit;
} inv_voltage_ctrl_t;

/**
 * @brief Initialization parameters for the stand-alone voltage outer loop.
 */
typedef struct _tag_inv_voltage_ctrl_init
{
    parameter_gt fs;        //!< Control execution frequency (Hz).
    parameter_gt freq_base; //!< Fundamental output frequency (Hz).
    parameter_gt v_base;    //!< Voltage base (V).
    parameter_gt i_base;    //!< Current base (A).
    parameter_gt filter_C;  //!< LC-filter capacitance per phase (F).

    parameter_gt voltage_loop_bw;   //!< Desired voltage-loop bandwidth (Hz).
    parameter_gt voltage_loop_zero; //!< PI zero frequency (Hz).

    parameter_gt current_circle_limit; //!< D-q current-vector magnitude limit (p.u.).
    parameter_gt current_square_limit; //!< Independent absolute d/q-axis limit (p.u.).
    fast_gt flag_enable_circle_limit;  //!< Enables circular current limiting.
    fast_gt flag_enable_square_limit;  //!< Enables square current limiting.
} inv_voltage_ctrl_init_t;

void ctl_auto_tuning_voltage_inv(inv_voltage_ctrl_init_t* voltage_init, const gfl_inv_ctrl_init_t* gfl_init);
void ctl_update_voltage_inv_coeff(inv_voltage_ctrl_t* voltage, const inv_voltage_ctrl_init_t* init);
void ctl_init_voltage_inv(inv_voltage_ctrl_t* voltage, const inv_voltage_ctrl_init_t* init);

GMP_STATIC_INLINE void ctl_clear_voltage_inv(inv_voltage_ctrl_t* voltage)
{
    ctl_clear_pid(&voltage->pid_vdq[phase_d]);
    ctl_clear_pid(&voltage->pid_vdq[phase_q]);
    ctl_vector2_clear(&voltage->vdq);
    ctl_vector2_clear(&voltage->idq_pi);
    ctl_vector2_clear(&voltage->idq_ff_decouple);
    ctl_vector2_clear(&voltage->idq_unlimited);
    ctl_vector2_clear(&voltage->idq_out);
}

/**
 * @brief Executes one capacitor-voltage outer-loop step.
 * @note Run this after the inner controller has updated its phasor and voltage
 * samples. The produced current reference is consumed by the inner loop.
 */
GMP_STATIC_INLINE void ctl_step_voltage_inv_ctrl(inv_voltage_ctrl_t* voltage)
{
    gmp_base_assert(voltage);

    if (!voltage->flag_enable)
    {
        ctl_vector2_clear(&voltage->idq_pi);
        ctl_vector2_clear(&voltage->idq_ff_decouple);
        ctl_vector2_clear(&voltage->idq_unlimited);
        ctl_vector2_clear(&voltage->idq_out);
        if (voltage->idq_sink != NULL)
            ctl_vector2_clear(voltage->idq_sink);
        return;
    }

    gmp_base_assert(voltage->vab);
    gmp_base_assert(voltage->phasor);
    gmp_base_assert(voltage->idq_sink);

    ctl_ct_park2(voltage->vab, voltage->phasor, &voltage->vdq);

    if (voltage->flag_enable_decouple)
    {
        /*
         * iCd = C(dvd/dt - omega*vq)
         * iCq = C(dvq/dt + omega*vd)
         */
        voltage->idq_ff_decouple.dat[phase_d] =
            -ctl_mul(voltage->coef_ff_decouple, voltage->vdq.dat[phase_q]);
        voltage->idq_ff_decouple.dat[phase_q] =
            ctl_mul(voltage->coef_ff_decouple, voltage->vdq.dat[phase_d]);
    }
    else
    {
        ctl_vector2_clear(&voltage->idq_ff_decouple);
    }

    voltage->idq_pi.dat[phase_d] =
        ctl_step_pid_par_raw(&voltage->pid_vdq[phase_d],
                             voltage->vdq_set.dat[phase_d] - voltage->vdq.dat[phase_d]);
    voltage->idq_pi.dat[phase_q] =
        ctl_step_pid_par_raw(&voltage->pid_vdq[phase_q],
                             voltage->vdq_set.dat[phase_q] - voltage->vdq.dat[phase_q]);

    ctl_vector2_add(&voltage->idq_unlimited, &voltage->idq_pi, &voltage->idq_ff_decouple);
    ctl_vector2_copy(&voltage->idq_out, &voltage->idq_unlimited);

    /* Limit the complete command so circular limiting retains the d-q direction. */
    if (voltage->flag_enable_circle_limit)
        ctl_vector2_sat_circle(&voltage->idq_out, &voltage->idq_out, voltage->current_circle_limit);

    if (voltage->flag_enable_square_limit)
        ctl_vector2_sat_square(&voltage->idq_out, &voltage->idq_out, voltage->current_square_limit);

    /*
     * Return the final actuator result (with feed-forward removed) to each
     * ordinary PID integrator. This supports either limiter and their
     * intersection without imposing an early per-axis PID saturation.
     */
    ctl_pid_clamping_correction_using_real_output(
        &voltage->pid_vdq[phase_d],
        voltage->idq_out.dat[phase_d] - voltage->idq_ff_decouple.dat[phase_d]);
    ctl_pid_clamping_correction_using_real_output(
        &voltage->pid_vdq[phase_q],
        voltage->idq_out.dat[phase_q] - voltage->idq_ff_decouple.dat[phase_q]);
    voltage->pid_vdq[phase_d].out =
        voltage->idq_out.dat[phase_d] - voltage->idq_ff_decouple.dat[phase_d];
    voltage->pid_vdq[phase_q].out =
        voltage->idq_out.dat[phase_q] - voltage->idq_ff_decouple.dat[phase_q];

    ctl_vector2_copy(voltage->idq_sink, &voltage->idq_out);
}

/**
 * @brief Attaches generic voltage, angle, and current-reference ports.
 */
GMP_STATIC_INLINE void ctl_attach_voltage_inv(inv_voltage_ctrl_t* voltage, ctl_vector2_t* vab,
                                              ctl_vector2_t* phasor, ctl_vector2_t* idq_sink)
{
    gmp_base_assert(voltage);
    voltage->vab = vab;
    voltage->phasor = phasor;
    voltage->idq_sink = idq_sink;
}

/**
 * @brief Attaches the voltage loop to a GFL current-loop core.
 * @details The current core's internal RG/PLL owns the phasor. Its measured
 * alpha-beta voltage is used as capacitor-voltage feedback and its idq_set is
 * driven by this outer loop.
 */
GMP_STATIC_INLINE void ctl_attach_voltage_inv_to_gfl(inv_voltage_ctrl_t* voltage, gfl_inv_ctrl_t* current)
{
    gmp_base_assert(current);
    ctl_attach_voltage_inv(voltage, (ctl_vector2_t*)&current->vab0, &current->phasor, &current->idq_set);
}

GMP_STATIC_INLINE void ctl_set_voltage_inv_reference(inv_voltage_ctrl_t* voltage, ctrl_gt vd, ctrl_gt vq)
{
    voltage->vdq_set.dat[phase_d] = vd;
    voltage->vdq_set.dat[phase_q] = vq;
}

GMP_STATIC_INLINE void ctl_enable_voltage_inv(inv_voltage_ctrl_t* voltage)
{
    voltage->flag_enable = 1;
}

GMP_STATIC_INLINE void ctl_disable_voltage_inv(inv_voltage_ctrl_t* voltage)
{
    voltage->flag_enable = 0;
}

GMP_STATIC_INLINE void ctl_enable_voltage_inv_decouple(inv_voltage_ctrl_t* voltage)
{
    voltage->flag_enable_decouple = 1;
}

GMP_STATIC_INLINE void ctl_disable_voltage_inv_decouple(inv_voltage_ctrl_t* voltage)
{
    voltage->flag_enable_decouple = 0;
}

GMP_STATIC_INLINE void ctl_set_voltage_inv_circle_limit(inv_voltage_ctrl_t* voltage, ctrl_gt limit)
{
    gmp_base_assert(limit >= float2ctrl(0.0f));
    voltage->current_circle_limit = limit;
}

GMP_STATIC_INLINE void ctl_set_voltage_inv_square_limit(inv_voltage_ctrl_t* voltage, ctrl_gt limit)
{
    gmp_base_assert(limit >= float2ctrl(0.0f));
    voltage->current_square_limit = limit;
}

GMP_STATIC_INLINE void ctl_enable_voltage_inv_circle_limit(inv_voltage_ctrl_t* voltage)
{
    voltage->flag_enable_circle_limit = 1;
}

GMP_STATIC_INLINE void ctl_disable_voltage_inv_circle_limit(inv_voltage_ctrl_t* voltage)
{
    voltage->flag_enable_circle_limit = 0;
}

GMP_STATIC_INLINE void ctl_enable_voltage_inv_square_limit(inv_voltage_ctrl_t* voltage)
{
    voltage->flag_enable_square_limit = 1;
}

GMP_STATIC_INLINE void ctl_disable_voltage_inv_square_limit(inv_voltage_ctrl_t* voltage)
{
    voltage->flag_enable_square_limit = 0;
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_DP_INV_VOLTAGE_CTRL_H_
