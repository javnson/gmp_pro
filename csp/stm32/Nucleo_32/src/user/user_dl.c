/** @file user_dl.c GMP Data Link validation services for Nucleo-32. */

#include <gmp_core.h>

#include <math.h>

#include <core/dev/datalink/datalink.h>
#include <core/dev/datalink/mem_presp.h>
#include <core/dev/datalink/pil_core.h>
#include <core/dev/datalink/tunable.h>
#include <ctl/component/dsa/dsa_dl_scope.h>
#include <xplt.peripheral.h>

#include "user_dl.h"

#if GMP_PORT_DATA_SIZE_PER_BYTES != 1
#error "STM32 Nucleo-32 Data Link validation requires the u8 backend"
#endif

#define USER_DL_PIL_COMMAND     0x10U
#define USER_DL_TUNABLE_COMMAND 0x30U
#define USER_DL_MEMORY_COMMAND  0x50U
#define USER_DL_SCOPE_COMMAND   0x60U
#define USER_DSA_DEPTH          400UL
#define USER_DSA_CHANNELS       2U
#define USER_CONTROL_RATE       20000UL
#define USER_DSA_SAMPLE_RATE    USER_CONTROL_RATE
#define USER_SIGNAL_TWO_PI      6.2831853071795864769F

static gmp_datalink_t datalink;
static gmp_pil_sim_t pil;
static gmp_param_tunable_t tunable;
static gmp_mem_persp_t memory_perspective;
static ctl_dsa_dl_scope_t dl_scope;

volatile uint32_t gmp_nucleo_dl_facility_init_errors;
float gmp_nucleo_signal_frequency_hz = 50.0F;
float gmp_nucleo_signal_gain = 1.0F;
float gmp_nucleo_signal_dc_offset = 0.0F;
static float applied_frequency_hz = -1.0F;
static float applied_signal_gain = -1.0F;
static float applied_signal_dc_offset = -100.0F;
static byte_gt memory_window[128];
static ctrl_gt dl_scope_storage[
    CTL_DSA_DL_SCOPE_STORAGE_ELEMENTS(USER_DSA_CHANNELS, USER_DSA_DEPTH)];
static volatile ctrl_gt oscillator_sine;
static volatile ctrl_gt oscillator_cosine;
static volatile ctrl_gt oscillator_step_sine;
static volatile ctrl_gt oscillator_step_cosine;
static volatile ctrl_gt active_signal_gain;
static volatile ctrl_gt active_signal_dc_offset;
static uint16_t oscillator_index;

static const gmp_param_item_t tunable_dictionary[] = {
    {&gmp_nucleo_signal_frequency_hz, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW,
     "Signal Frequency (Hz)"},
    {&gmp_nucleo_signal_gain, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW,
     "Signal Gain (x)"},
    {&gmp_nucleo_signal_dc_offset, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW,
     "Signal DC Offset (V)"},
};

static const gmp_mem_region_t memory_regions[] = {
    {memory_window, sizeof(memory_window), GMP_MEM_PERM_RW, "Scratch Memory"},
    {(void*)gmp_nucleo_adc_raw, sizeof(gmp_nucleo_adc_raw), GMP_MEM_PERM_RO,
     "ADC Feedback Raw"},
    {(void*)&gmp_nucleo_qep_count, sizeof(gmp_nucleo_qep_count),
     GMP_MEM_PERM_RO, "QEP Count"},
    {(void*)gmp_nucleo_pwm_compare, sizeof(gmp_nucleo_pwm_compare),
     GMP_MEM_PERM_RW, "PWM Compare (outputs remain disabled)"},
    {(void*)gmp_nucleo_platform_diag, sizeof(gmp_nucleo_platform_diag),
     GMP_MEM_PERM_RO, "Platform Diagnostics"},
};

static void user_dl_append(fast_gt result)
{
    if (!result)
        gmp_nucleo_dl_facility_init_errors++;
}

static void user_dl_apply_signal_parameters(void)
{
    float frequency_hz = gmp_nucleo_signal_frequency_hz;
    float gain = gmp_nucleo_signal_gain;
    float dc_offset = gmp_nucleo_signal_dc_offset;
    float angle;

    if (frequency_hz < 1.0F)
        frequency_hz = 1.0F;
    else if (frequency_hz > 200.0F)
        frequency_hz = 200.0F;
    if (gain < 0.0F)
        gain = 0.0F;
    else if (gain > 10.0F)
        gain = 10.0F;
    if (dc_offset < -10.0F)
        dc_offset = -10.0F;
    else if (dc_offset > 10.0F)
        dc_offset = 10.0F;

    gmp_nucleo_signal_frequency_hz = frequency_hz;
    gmp_nucleo_signal_gain = gain;
    gmp_nucleo_signal_dc_offset = dc_offset;
    if (frequency_hz == applied_frequency_hz && gain == applied_signal_gain &&
        dc_offset == applied_signal_dc_offset)
        return;

    angle = USER_SIGNAL_TWO_PI * frequency_hz / (float)USER_DSA_SAMPLE_RATE;
    gmp_base_enter_critical();
    oscillator_step_sine = real2ctrl(sinf(angle));
    oscillator_step_cosine = real2ctrl(cosf(angle));
    active_signal_gain = real2ctrl(gain);
    active_signal_dc_offset = real2ctrl(dc_offset);
    gmp_base_leave_critical();
    applied_frequency_hz = frequency_hz;
    applied_signal_gain = gain;
    applied_signal_dc_offset = dc_offset;
}

void user_dl_init(void)
{
    size_gt index;

    for (index = 0U; index < sizeof(memory_window); ++index)
        memory_window[index] = (byte_gt)index;

    gmp_nucleo_dl_facility_init_errors = 0U;
    gmp_dev_dl_init(&datalink);
    gmp_pil_sim_init(&pil, &datalink, USER_DL_PIL_COMMAND);
    user_dl_append(gmp_dev_dl_append_facility(&datalink, &pil.facility));
    gmp_param_tunable_init(&tunable, &datalink, USER_DL_TUNABLE_COMMAND,
                           tunable_dictionary,
                           (fast16_gt)(sizeof(tunable_dictionary) /
                                       sizeof(tunable_dictionary[0])));
    user_dl_append(gmp_dev_dl_append_facility(&datalink, &tunable.facility));
    gmp_mem_persp_init(&memory_perspective, &datalink,
                       USER_DL_MEMORY_COMMAND, memory_regions,
                       (fast16_gt)(sizeof(memory_regions) /
                                   sizeof(memory_regions[0])));
    user_dl_append(gmp_dev_dl_append_facility(
        &datalink, &memory_perspective.facility));
    if (!ctl_init_dsa_dl_scope_workspace(
            &dl_scope, &datalink, USER_DL_SCOPE_COMMAND,
            "Sine and Cosine Scope", dl_scope_storage,
            (uint32_t)(sizeof(dl_scope_storage) /
                       sizeof(dl_scope_storage[0])),
            USER_DSA_CHANNELS, USER_DSA_SAMPLE_RATE))
        gmp_nucleo_dl_facility_init_errors++;
    else
        user_dl_append(gmp_dev_dl_append_facility(
            &datalink, ctl_dsa_dl_scope_facility(&dl_scope)));

    oscillator_sine = real2ctrl(0.0F);
    oscillator_cosine = real2ctrl(1.0F);
    oscillator_index = 0U;
    user_dl_apply_signal_parameters();
    xplt_dl_bind(&datalink);
}

gmp_task_status_t user_dl_task(gmp_task_t* task)
{
    gmp_dl_event_t event;
    GMP_UNUSED_VAR(task);

    event = gmp_dev_dl_loop_cb(&datalink);
    if (event == GMP_DL_EVENT_TX_RDY)
        xplt_dl_start_tx(&datalink);
    else if (event == GMP_DL_EVENT_RX_OK)
        (void)gmp_dev_dl_dispatch_rx(&datalink);
    user_dl_apply_signal_parameters();
    return GMP_TASK_DONE;
}

void user_dl_control_step(void)
{
    ctrl_gt unit_sine;
    ctrl_gt unit_cosine;
    ctrl_gt sine_sample;
    ctrl_gt cosine_sample;

    unit_sine = oscillator_sine;
    unit_cosine = oscillator_cosine;
    sine_sample = unit_sine * active_signal_gain + active_signal_dc_offset;
    cosine_sample = unit_cosine * active_signal_gain + active_signal_dc_offset;
    ctl_step_dsa_dl_scope_2ch(&dl_scope, sine_sample, cosine_sample);
    oscillator_sine =
        unit_sine * oscillator_step_cosine + unit_cosine * oscillator_step_sine;
    oscillator_cosine =
        unit_cosine * oscillator_step_cosine - unit_sine * oscillator_step_sine;
    oscillator_index++;
    if (oscillator_index >= 256U)
    {
        ctrl_gt magnitude_squared = oscillator_sine * oscillator_sine +
                                    oscillator_cosine * oscillator_cosine;
        ctrl_gt correction =
            real2ctrl(1.5F) - real2ctrl(0.5F) * magnitude_squared;
        oscillator_index = 0U;
        oscillator_sine *= correction;
        oscillator_cosine *= correction;
    }
}

void gmp_pil_sim_step(const gmp_sim_rx_buf_t* rx, gmp_sim_tx_buf_t* tx)
{
    tx->digital_out = rx->digital_input;
    tx->pwm_cmp[0] = rx->adc_result[0];
    tx->monitor[0] = rx->panel[0];
}
