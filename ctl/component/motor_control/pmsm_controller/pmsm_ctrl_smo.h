
// Necessary support
#include <ctl/component/interface/interface_base.h>
#include <ctl/component/motor_control/basic/motor_universal_interface.h>

#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
#include <ctl/component/intrinsic/discrete/track_discrete_pid.h>
#else
#include <ctl/component/intrinsic/continuous/track_pid.h>
#endif // PMSM_CTRL_USING_DISCRETE_CTRL

#include <ctl/component/motor_control/basic/decouple.h>

#include <ctl/math_block/coordinate/coord_trans.h>

#include <ctl/component/motor_control/basic/vf_generator.h>
#include <ctl/component/motor_control/observer/pmsm.smo.h>

#ifndef _FILE_PMSM_CTRL_BARE_H_
#define _FILE_PMSM_CTRL_BARE_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// .....................................................................//
// Some config options here.

// Select 2phase current measurement or 3 phase current measurement
// Default is 3, user may choose 2 in user config file
#ifndef MTR_CTRL_CURRENT_MEASUREMENT_PHASES
#define MTR_CTRL_CURRENT_MEASUREMENT_PHASES ((3))
#endif // MTR_CTRL_CURRENT_MEASUREMENT_PHASES

// Select 2 phases voltage measurement or 3 phase voltage measurement or disable voltage measurement
// Default is 3 channels, user may choose:
// + 0 no voltage sensors
// + 2 2 phases voltage sensors
// + 3 3 phases voltage sensors
#ifndef MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES
#define MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES ((3))
#endif // MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES

// Select feedforward strategy.
// 0 user mode, feed forward is specified by user manually.
// 1 decouple mode, feed forward item is counted.
//   only the time velocity controller is working, this config is valid.
//   or, the decouple module will bypass.
//
#ifndef MTR_CTRL_FEEDFORWARD_STRATEGY
#define MTR_CTRL_FEEDFORWARD_STRATEGY (1)
#endif // MTR_CTRL_FEEDFORWARD_STRATEGY

//
// PMSM Bare controller is a x-platform PMSM controller.
//
// PMSM Bare controller usage:
// + Attach physical interface with pmsm_bare_controller
//  This task should complete in a xPlatform file.
//  @ctl_attach_pmsm_bare_output function is used to attach pwm interface.
//  @ctl_attach_mtr_adc_channels function is used to attach adc interface.
//  @ctl_attach_mtr_position function is used to attach position encoder.
//  @ctl_attach_mtr_velocity function is used to attach velocity encoder.
//  @ctl_attach_mtr_torque function is used to attach torque sensor.
//
// + init the controller structure entity
//  fill struct @pmsm_bare_controller_init_t to specify motor controller parameters.
//  call function @ctl_init_pmsm_bare_controller to initialize the controller entity, based on the parameters.
//
// + select a operating mode and provide a control target
// PMSM bare controller support the PMSM controller running in the following mode:
//   - $v_\alpha, v_\beta$ mode
//   user may call function @ctl_pmsm_ctrl_valphabeta_mode to enter this mode.
//   user may call function @ctl_set_pmsm_ctrl_valphabeta to provide control target
//
//   - Voltage mode (vdq mode)
//   user may call function @ctl_pmsm_ctrl_voltage_mode to enter this mode.
//   user may call function @ctl_set_pmsm_ctrl_vdq_ff to provide control target.
//
//   - Current mode (idq mode)
//   user may call function @ctl_pmsm_ctrl_current_mode to enter current mode.
//   current controller will active.
//   user may call function @ctl_set_pmsm_ctrl_idq_ff to provide control target.
//
//   - Velocity mode (spd mode)
//   user may call function @ctl_pmsm_ctrl_velocity_mode to enter velocity mode.
//   current controller and speed controller will active.
//   user may call function @ctl_set_pmsm_ctrl_speed to provide control target.
//
//   Note: These function is just a controller switch, when you need to change controller mode
//   at time the controller is running you should implement a extra algorithm to ensure switch smoothly.
//
// + invoke this controller in Main ISR.
//  User should call mtr_interface' step function firstly to get correct input.
//  Then, call @ctl_step_pmsm_ctrl in MainISR function.
//  Last, call PWM interface to output modulation result.
//
// + enable / disable the controller
//  function @ctl_enable_pmsm_ctrl is to enable the controller or the whole controller will bypass.
//  function @ctl_disable_pmsm_ctrl is to disable the controller.
//  Note: you should call function @ctl_clear_pmsm_ctrl before controller is switch on.
//

typedef struct _tag_pmsm_smo_bare_controller
{

    // .....................................................................//
    // interfaces
    //

    // input interfaces
    mtr_ift mtr_interface;

    // output interfaces
    tri_pwm_ift *pwm_out;

    // .....................................................................//
    // controller entity
    //
#ifdef PMSM_CTRL_USING_DISCRETE_CTRL

    // current controller
    discrete_pid_t current_ctrl[2];

    // speed controller
    track_discrete_pid_t spd_ctrl;
#else // use continuous controller

    // current controller
    pid_regular_t current_ctrl[2];

    // speed controller
    track_pid_t spd_ctrl;
#endif

    // SMO Observer
    pmsm_smo_t smo;

    // ramp start up
    ctl_slope_f_controller ramp_gen;

    // ramp_frequency to speed set scale factor
    ctrl_gt ramp_freq_spd_set_sf;

    // .....................................................................//
    // controller intermediate variable
    //

    // i_alpha, i_beta, i0
    vector3_gt iab0;

    // i_d, i_q, i0
    vector3_gt idq0;

    // v_alpha, v_beta, v0
    vector3_gt uab0;

    // v_d, v_q, v0
    vector3_gt udq0;

    // .....................................................................//
    // controller feed forward parameters
    //

    // i_d_ff, i_q_ff
    vector2_gt idq_ff;

    // v_d_ff, v_q_ff
    vector2_gt vdq_ff;

    // .....................................................................//
    // controller set parameters
    //

    // speed target, p.u.
    ctrl_gt speed_set;

    // id, iq set
    vector2_gt idq_set;

    // vd, vq, v0 set
    vector3_gt vdq_set;

    // valpha, vbeta, v0 set
    vector3_gt vab0_set;

    // .....................................................................//
    // flag stack
    //

    // enable whole controller
    fast_gt flag_enable_controller;

    // enable PWM output
    fast_gt flag_enable_output;

    // enable Modulation
    fast_gt flag_enable_modulation;

    // enable current controller
    fast_gt flag_enable_current_ctrl;

    // enable velocity controller
    fast_gt flag_enable_velocity_ctrl;

    // enable SMO observer
    fast_gt flag_enable_smo;

    // switch smo completed
    fast_gt flag_switch_cplt;

} pmsm_smo_bare_controller_t;

// This function should be called in MainISR.
// This function implement a universal PMSM controller
GMP_STATIC_INLINE
void ctl_step_pmsm_smo_ctrl(pmsm_smo_bare_controller_t *ctrl)
{
    ctl_vector2_t phasor;
    ctrl_gt etheta;

    if (ctrl->flag_enable_controller)
    {
        //
        // const slope frequency controller
        //
        ctl_step_slope_f(&ctrl->ramp_gen);

        //
        // Clark Transformation
        //

        // iab = clark(iabc)
#if MTR_CTRL_CURRENT_MEASUREMENT_PHASES == 3
        ctl_ct_clark(&ctrl->mtr_interface.iabc->value, &ctrl->iab0);
#elif MTR_CTRL_CURRENT_MEASUREMENT_PHASES == 2
        ctl_ct_clark_2ph(&ctrl->mtr_interface.iabc->value, &ctrl->iab0);
#else
#error("Wrong parameter for macro MTR_CTRL_CURRENT_MEASUREMENT_PHASES, this parameter means how many current sensors have for each motor.")
#endif // MTR_CTRL_CURRENT_MEASUREMENT_PHASES

        // uab = clark(uabc)
#if MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES == 3
        ctl_ct_clark(&ctrl->mtr_interface.uabc->value, &ctrl->uab0);
#elif MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES == 2
        ctl_ct_clark_2ph(&ctrl->mtr_interface.uabc->value, &ctrl->uab0);
#elif MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES == 0
        ctl_vector3_clear(&ctrl->uab0);
#else
#error("Wrong parameter for macro MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES, this parameter means how many voltage sensors have for each motor.")
#endif // MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES

        //
        // Park Transformation
        //

        // current & voltage transformation
        etheta = ctl_get_mtr_elec_theta(&ctrl->mtr_interface);

        // phasor = \angle(theta)
        ctl_set_phasor_via_angle(etheta, &phasor);

        // idq = park(iab)
        ctl_ct_park(&ctrl->iab0, &phasor, &ctrl->idq0);

        // vdq = park(vab)
#if MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES != 0
        ctl_ct_park(&ctrl->uab0, &phasor, &ctrl->udq0);
#else
        ctl_vector3_clear(&ctrl->uab0);
#endif // MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES

        //
        // Velocity Controller
        //

        if (ctrl->flag_enable_velocity_ctrl)
        {
            ctrl->idq_set.dat[phase_d] = ctrl->idq_ff.dat[phase_d];
#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
            ctrl->idq_set.dat[phase_q] = ctl_step_discrete_track_pid(&ctrl->spd_ctrl, ctrl->speed_set,
                                                                     ctl_get_mtr_velocity(&ctrl->mtr_interface)) +
                                         ctrl->idq_ff.dat[phase_q];
#else  // using continuous controller
            ctrl->idq_set.dat[phase_q] =
                ctl_step_track_pid(&ctrl->spd_ctrl, ctrl->speed_set, ctl_get_mtr_velocity(&ctrl->mtr_interface)) +
                ctrl->idq_ff.dat[phase_q];
#endif // PMSM_CTRL_USING_DISCRETE_CTRL

#if MTR_CTRL_FEEDFORWARD_STRATEGY == 1
            // ctl_mtr_pmsm_decouple(&ctrl->vdq_set, &ctrl->idq_set, ctrl->Ld, ctrl->Lq,
            //                       ctl_get_mtr_velocity(ctrl->mtr_interface), ctrl->psi_e);
#endif // MTR_CTRL_FEEDFORWARD_STRATEGY
        }
        else
        {
            ctrl->idq_set.dat[phase_d] = ctrl->idq_ff.dat[phase_d];
            ctrl->idq_set.dat[phase_q] = ctrl->idq_ff.dat[phase_q];
        }

        //
        // Current Controller
        //

        if (ctrl->flag_enable_current_ctrl)
        {
#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
            ctrl->vdq_set.dat[phase_d] = ctl_step_discrete_pid(&ctrl->current_ctrl[phase_d],
                                                               ctrl->idq_set.dat[phase_d] - ctrl->idq0.dat[phase_d]) +
                                         ctrl->vdq_ff.dat[phase_d];

            ctrl->vdq_set.dat[phase_q] = ctl_step_discrete_pid(&ctrl->current_ctrl[phase_q],
                                                               ctrl->idq_set.dat[phase_q] - ctrl->idq0.dat[phase_q]) +
                                         ctrl->vdq_ff.dat[phase_q];
#else  //  using continuous controller
            ctrl->vdq_set.dat[phase_d] =
                ctl_step_pid_par(&ctrl->current_ctrl[phase_d], ctrl->idq_set.dat[phase_d] - ctrl->idq0.dat[phase_d]) +
                ctrl->vdq_ff.dat[phase_d];

            ctrl->vdq_set.dat[phase_q] =
                ctl_step_pid_par(&ctrl->current_ctrl[phase_q], ctrl->idq_set.dat[phase_q] - ctrl->idq0.dat[phase_q]) +
                ctrl->vdq_ff.dat[phase_q];
#endif // PMSM_CTRL_USING_DISCRETE_CTRL

            ctrl->vdq_set.dat[phase_0] = 0;
        }
        else
        {
            ctrl->vdq_set.dat[phase_d] = ctrl->vdq_ff.dat[phase_d];
            ctrl->vdq_set.dat[phase_q] = ctrl->vdq_ff.dat[phase_q];
            ctrl->vdq_set.dat[phase_0] = 0;
        }

        //
        // ipark transformation
        //

        if (ctrl->flag_enable_modulation)
        {
            ctl_ct_ipark(&ctrl->vdq_set, &phasor, &ctrl->vab0_set);
        }
        // else // vab0_set will be output.

        //
        // SMO
        //
        ctl_input_pmsm_smo(&ctrl->smo, ctrl->vab0_set.dat[phase_alpha], ctrl->vab0_set.dat[phase_beta],
                           ctrl->iab0.dat[phase_alpha], ctrl->iab0.dat[phase_beta]);
        ctl_step_pmsm_smo(&ctrl->smo);

        //
        // SVPWM modulation
        //

        if (ctrl->flag_enable_output)
        {
            ctl_ct_svpwm_calc(&ctrl->vab0_set, &ctrl->pwm_out->value);
        }
        else
        {
            ctrl->pwm_out->value.dat[phase_A] = 0;
            ctrl->pwm_out->value.dat[phase_B] = 0;
            ctrl->pwm_out->value.dat[phase_C] = 0;
        }
    }
}

GMP_STATIC_INLINE
void ctl_clear_pmsm_smo_ctrl(pmsm_smo_bare_controller_t *ctrl)
{
#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
    // clear controller intermediate variables
    ctl_clear_discrete_pid(&ctrl->current_ctrl[phase_d]);
    ctl_clear_discrete_pid(&ctrl->current_ctrl[phase_q]);

    ctl_clear_discrete_track_pid(&ctrl->spd_ctrl);
#else  // continuous controller
       // clear controller intermediate variables
    ctl_clear_pid(&ctrl->current_ctrl[phase_d]);
    ctl_clear_pid(&ctrl->current_ctrl[phase_q]);

    ctl_clear_track_pid(&ctrl->spd_ctrl);
#endif // PMSM_CTRL_USING_DISCRETE_CTRL

    ctl_attach_mtr_position(&ctrl->mtr_interface, &ctrl->ramp_gen.enc);
    ctl_clear_slope_f(&ctrl->ramp_gen);

    ctrl->flag_switch_cplt = 0;
}

// .....................................................................//
// enable and disable
//

// enable PMSM controller
GMP_STATIC_INLINE
void ctl_enable_pmsm_smo_ctrl(pmsm_smo_bare_controller_t *ctrl)
{
    ctrl->flag_enable_controller = 1;
}

// disable PMSM controller
GMP_STATIC_INLINE
void ctl_disable_pmsm_smo_ctrl(pmsm_smo_bare_controller_t *ctrl)
{
    ctrl->flag_enable_controller = 0;
}

// .....................................................................//
// v alpha & v beta mode
//

// PMSM controller run in valpha vbeta mode,
// user should specify valpha and vbeta by function ctl_set_pmsm_ctrl_valphabeta
GMP_STATIC_INLINE
void ctl_pmsm_smo_ctrl_valphabeta_mode(pmsm_smo_bare_controller_t *ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 0;
    ctrl->flag_enable_current_ctrl = 0;
    ctrl->flag_enable_velocity_ctrl = 0;
}

// Set motor target v alpha and v beta.
// only in valphabeta mode this function counts.
GMP_STATIC_INLINE
void ctl_set_pmsm_smo_ctrl_valphabeta(pmsm_smo_bare_controller_t *ctrl, ctrl_gt valpha, ctrl_gt vbeta)
{
    ctrl->vab0_set.dat[phase_A] = valpha;
    ctrl->vab0_set.dat[phase_B] = vbeta;
    ctrl->vab0_set.dat[phase_0] = 0;
}

// .....................................................................//
// vdq mode
//

// PMSM controller run in valpha vbeta mode
// user should specify udq0 by function ctl_set_pmsm_ctrl_vdq
GMP_STATIC_INLINE
void ctl_pmsm_smo_ctrl_voltage_mode(pmsm_smo_bare_controller_t *ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 1;
    ctrl->flag_enable_current_ctrl = 0;
    ctrl->flag_enable_velocity_ctrl = 0;
}

// this function set vdq reference for vdq mode.
// PMSM controller run in vdq mode this function counts.
GMP_STATIC_INLINE
void ctl_set_pmsm_smo_ctrl_vdq_ff(pmsm_smo_bare_controller_t *ctrl, ctrl_gt vd, ctrl_gt vq)
{
    ctrl->vdq_ff.dat[phase_d] = vd;
    ctrl->vdq_ff.dat[phase_q] = vq;
    //        ctrl->vdq_ff.dat[phase_0] = 0;
}

// .....................................................................//
// idq mode
//

// this function set pmsm controller run in current mode.
GMP_STATIC_INLINE
void ctl_pmsm_smo_ctrl_current_mode(pmsm_smo_bare_controller_t *ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 1;
    ctrl->flag_enable_current_ctrl = 1;
    ctrl->flag_enable_velocity_ctrl = 0;
}

// this function set pmsm idq feed forward.
// in current mode this value means idq reference.
GMP_STATIC_INLINE
void ctl_set_pmsm_smo_ctrl_idq_ff(pmsm_smo_bare_controller_t *ctrl, ctrl_gt id, ctrl_gt iq)
{
    ctrl->idq_ff.dat[phase_d] = id;
    ctrl->idq_ff.dat[phase_q] = iq;
    //        ctrl->idq_ff.dat[phase_0] = 0;
}

// .....................................................................//
// velocity mode
//

// this function set pmsm controller run in speed mode.
GMP_STATIC_INLINE
void ctl_pmsm_smo_ctrl_velocity_mode(pmsm_smo_bare_controller_t *ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 1;
    ctrl->flag_enable_current_ctrl = 1;
    ctrl->flag_enable_velocity_ctrl = 1;
}

// this fucntion set pmsm target speed.
GMP_STATIC_INLINE
void ctl_set_pmsm_smo_ctrl_speed(pmsm_smo_bare_controller_t *ctrl, ctrl_gt spd)
{
    ctrl->speed_set = spd;
}

// .....................................................................//
// SMO relatied function
//

GMP_STATIC_INLINE
void ctl_enable_pmsm_smo(pmsm_smo_bare_controller_t *ctrl)
{
    ctrl->flag_enable_smo = 1;
}

GMP_STATIC_INLINE
void ctl_switch_pmsm_smo_ctrl_using_smo(pmsm_smo_bare_controller_t *ctrl)
{
    ctl_vector2_t phasor;
    vector3_gt udq0;
    vector3_gt idq0;

    if ((ctrl->flag_enable_smo) && (!ctrl->flag_switch_cplt))
    {
        ctl_attach_mtr_position(&ctrl->mtr_interface, &ctrl->smo.encif);
        ctl_pmsm_smo_ctrl_velocity_mode(ctrl);
        ctl_set_pmsm_smo_ctrl_speed(ctrl, ctl_mul(ctrl->ramp_gen.target_frequency, ctrl->ramp_freq_spd_set_sf));

        ctl_set_phasor_via_angle(ctrl->smo.encif.elec_position, &phasor);
        ctl_ct_park(&ctrl->vab0_set, &phasor, &udq0);
        ctl_ct_park(&ctrl->iab0, &phasor, &idq0);

        ctrl->idq_ff.dat[phase_q] = 0;
        ctrl->idq_ff.dat[phase_d] = 0;

#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
        ctrl->spd_ctrl.pid.output_1 = idq0.dat[phase_q];

        ctrl->current_ctrl[phase_d].output_1 = udq0.dat[phase_d];
        ctrl->current_ctrl[phase_q].output_1 = udq0.dat[phase_q];
#else  // using continuous controller
        ctrl->idq_set.dat[phase_q] =
            ctl_step_track_pid(&ctrl->spd_ctrl, ctrl->speed_set, ctl_get_mtr_velocity(&ctrl->mtr_interface)) +
            ctrl->idq_ff.dat[phase_q];
#endif // PMSM_CTRL_USING_DISCRETE_CTRL

        ctrl->flag_switch_cplt = 1;
    }
}

// .....................................................................//
// initial function
//

typedef struct _tag_pmsm_bare_controller_init
{
    // controller frequency
    parameter_gt fs;

    // .....................................................................//
    // Current controller
    //

    // Current Controller P param
    parameter_gt current_pid_gain;

    // current controller integral time constant
    parameter_gt current_Ti;

    // current controller differential time constant
    parameter_gt current_Td;

    // current output saturation
    parameter_gt voltage_limit_max;
    parameter_gt voltage_limit_min;

    // .....................................................................//
    // speed controller
    //

    // speed controller P item
    parameter_gt spd_pid_gain;

    // speed controller integral time constant
    parameter_gt spd_Ti;

    // speed controller differential time constant
    parameter_gt spd_Td;

    // speed controller output saturation
    parameter_gt current_limit_max;
    parameter_gt current_limit_min;

    // acceleration saturation
    // unit p.s./s
    parameter_gt acc_limit_max;
    parameter_gt acc_limit_min;

    // speed controller divider
    uint32_t spd_ctrl_div;

    // .....................................................................//
    // SMO controller
    //

    // Motor parameters, unit SI
    parameter_gt Rs;
    parameter_gt Ld;
    parameter_gt Lq;

    uint16_t pole_pairs;

    // per unit base value
    parameter_gt u_base;
    parameter_gt i_base;

    // Base speed RPM
    parameter_gt speed_base_rpm;

    // EMF filter cut frequency, Hz
    parameter_gt smo_fc_e;
    // Speed filter cut frequency, Hz
    parameter_gt smo_fc_omega;
    // PLL PI controller parameters
    ctrl_gt smo_kp;
    ctrl_gt smo_Ti;
    ctrl_gt smo_Td;
    ctrl_gt smo_k_slide;

    // .....................................................................//
    // Ramp Generator
    //

    // target frequency, Hz
    parameter_gt ramp_target_freq;

    // target frequency slope, Hz/s
    parameter_gt ramp_target_freq_slope;

} pmsm_smo_bare_controller_init_t;

// init pmsm_bare_controller struct
void ctl_init_pmsm_smo_bare_controller(pmsm_smo_bare_controller_t *ctrl, pmsm_smo_bare_controller_init_t *init);

// attach to output port
void ctl_attach_pmsm_smo_bare_output(pmsm_smo_bare_controller_t *ctrl, tri_pwm_ift *pwm_out);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_CTRL_BARE_H_
