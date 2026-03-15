/**
 * @file ctl_motor_init.c
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
// DTC

#include <ctl/component/motor_control/current_loop/dtc.h>

// The optimal voltage vector switching table for a 2-level DTC scheme.
uint8_t DTC_SWITCH_TABLE[6][4] = {
    // S=1      S=2      S=3      S=4      S=5      S=6
    {5, 6, 2, 3}, // Flux Sector 1
    {6, 1, 3, 4}, // Flux Sector 2
    {1, 2, 4, 5}, // Flux Sector 3
    {2, 3, 5, 6}, // Flux Sector 4
    {3, 4, 6, 1}, // Flux Sector 5
    {4, 5, 1, 2}  // Flux Sector 6
};

// Table to convert voltage vector index (0-7) to alpha-beta voltages.
ctrl_gt V_ALPHA_BETA_TABLE[8][2] = {
    {float2ctrl(0.0f), float2ctrl(0.0f)},             // V0
    {float2ctrl(0.666667f), float2ctrl(0.0f)},        // V1
    {float2ctrl(0.333333f), float2ctrl(0.577350f)},   // V2
    {float2ctrl(-0.333333f), float2ctrl(0.577350f)},  // V3
    {float2ctrl(-0.666667f), float2ctrl(0.0f)},       // V4
    {float2ctrl(-0.333333f), float2ctrl(-0.577350f)}, // V5
    {float2ctrl(0.333333f), float2ctrl(-0.577350f)},  // V6
    {float2ctrl(0.0f), float2ctrl(0.0f)}              // V7
};

void ctl_init_dtc(ctl_dtc_controller_t* dtc, const ctl_dtc_init_t* init)
{
    dtc->ts = 1.0f / (ctrl_gt)init->f_ctrl;
    dtc->rs = (ctrl_gt)init->Rs;
    dtc->pole_pairs = (ctrl_gt)init->pole_pairs;

    // Initialize hysteresis controllers
    // Flux: Output 1 means INCREASE flux
    ctl_init_hysteresis_controller(&dtc->flux_hcc, 1, init->flux_hyst_width);
    // Torque: Output 1 means INCREASE torque
    ctl_init_hysteresis_controller(&dtc->torque_hcc, 1, init->torque_hyst_width);

    // Clear state variables
    ctl_vector2_clear(&dtc->stator_flux);
    dtc->flux_mag_est = 0.0f;
    dtc->torque_est = 0.0f;
    dtc->flux_sector = 1;
    dtc->voltage_vector_index = 0; // Start with zero vector
}
