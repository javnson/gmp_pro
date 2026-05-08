// ============================================================================
// SDPE Auto-Generated Component: Shunt-based Current Sensor with Op-Amp
// ============================================================================

/* --- Shunt-based Current Sensor with Op-Amp (iuvw) Parameters --- */

#define CTRL_INVERTER_CURRENT_R_SHUNT_OHM          (0.002f)

#define CTRL_INVERTER_CURRENT_AMP_GAIN             (20.0f)

#define CTRL_INVERTER_CURRENT_BIAS_V               (1.65f)

#define CTRL_INVERTER_CURRENT_MAX_CONT_ARMS        (15.0)

#define CTRL_INVERTER_CURRENT_MAX_PEAK_AP          (30.0)

#define CTRL_INVERTER_CURRENT_ACCURACY_CLASS_PCT   (0.5)

#define CTRL_INVERTER_CURRENT_CHIP_PART_NUMBER     (INA240A1)


#define CTRL_INVERTER_CURRENT_SENSITIVITY         GMP_SDPE_CALC_SHUNT_SENSITIVITY(CTRL_INVERTER_CURRENT_R_SHUNT_OHM, CTRL_INVERTER_CURRENT_AMP_GAIN)
#define CTRL_INVERTER_CURRENT_BIAS                GMP_SDPE_CALC_BIAS(CTRL_INVERTER_CURRENT_BIAS_V)

/* --- Declarations --- */
// Shunt-based Current Sensor with Op-Amp object declaration
tri_ptr_adc_channel_t iuvw;

adc_gt iuvw_src[3];

/* --- Initialization --- */
// Initialize Shunt-based Current Sensor with Op-Amp (Ref: 3.3V, 12-bit)
    ctl_init_tri_ptr_adc_channel(
        &iuvw, iuvw_src,
        ctl_gain_calc_generic(3.3f, CTRL_INVERTER_CURRENT_SENSITIVITY, 10.0f),
        ctl_bias_calc_via_Vref_Vbias(3.3f, CTRL_INVERTER_CURRENT_BIAS),
        12, 24);

/* --- IO Step --- */
ctl_step_tri_ptr_adc_channel(&iuvw);

