
#include <gmp_core.h>

#include <ctl/component/motor_control/basic/vf_generator.h>
#include <ctl/component/motor_control/current_loop/foc_core.h>
#include <ctl/component/motor_control/interface/encoder.h>
#include <ctl/component/motor_control/interface/encoder_switcher.h>
#include <ctl/component/motor_control/observer/pmsm_esmo.h>

#include <ctl/component/motor_control/pmsm_offline_id/pmsm_offline_id_sm.h>

mtr_current_ctrl_t foc_core;      //!< The FOC current controller core.
ctl_slope_f_pu_controller vf_gen; //!< The V/F slope frequency generator.
ctl_pmsm_esmo_t esmo;
ctl_pos_autoturn_encoder_t enc; //!< physical encoder

pmsm_offline_id_sm_t state_machine;
