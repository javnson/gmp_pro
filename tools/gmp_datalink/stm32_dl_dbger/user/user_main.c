/**
 * @file user_main.c
 * @brief STM32 Data Link validation application.
 */

#include <gmp_core.h>
#include <math.h>

#include "user_main.h"
#include <core/dev/datalink/datalink.h>
#include <core/dev/datalink/mem_presp.h>
#include <core/dev/datalink/pil_core.h>
#include <core/dev/datalink/tunable.h>
#include <ctl/component/dsa/dsa_dl_scope.h>
#include <xplt.peripheral.h>

#if GMP_PORT_DATA_SIZE_PER_BYTES != 1
#error "The NUCLEO-C092RC validation firmware requires the u8 Data Link backend"
#endif

#define USER_DL_PIL_CMD           0x10U
#define USER_DL_TUNABLE_CMD       0x30U
#define USER_DL_MEMORY_CMD        0x50U
#define USER_DL_SCOPE_CMD         0x60U
#define USER_DSA_SAMPLE_RATE      1000UL
#define USER_DSA_DEPTH            400UL
#define USER_DSA_CHANNELS         2U
#define USER_SIGNAL_MIN_RATE_HZ   1.0F
#define USER_SIGNAL_MAX_RATE_HZ   200.0F
#define USER_SIGNAL_MAX_GAIN      10.0F
#define USER_SIGNAL_MAX_OFFSET    10.0F
#define USER_SIGNAL_TWO_PI        6.2831853071795864769F
#define USER_OSC_NORMALIZE_PERIOD 256U

static gmp_scheduler_t scheduler;
gmp_datalink_t datalink;
gmp_pil_sim_t pil;
gmp_param_tunable_t tunable;
gmp_mem_persp_t memory_perspective;
ctl_dsa_dl_scope_t dl_scope;
volatile uint32_t dl_facility_init_errors;

float signal_frequency_hz = 50.0F;
float signal_gain = 1.0F;
float signal_dc_offset = 0.0F;
float applied_frequency_hz = -1.0F;
float applied_signal_gain = -1.0F;
float applied_signal_dc_offset = -100.0F;
byte_gt memory_window[128];
ctrl_gt dl_scope_storage[
    CTL_DSA_DL_SCOPE_STORAGE_ELEMENTS(USER_DSA_CHANNELS, USER_DSA_DEPTH)];
volatile ctrl_gt oscillator_sine;
volatile ctrl_gt oscillator_cosine;
volatile ctrl_gt oscillator_step_sine;
volatile ctrl_gt oscillator_step_cosine;
volatile ctrl_gt active_signal_gain;
volatile ctrl_gt active_signal_dc_offset;
uint16_t oscillator_index;

static const gmp_param_item_t tunable_dictionary[] = {
    {&signal_frequency_hz, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW,
     "Signal Frequency (Hz)"},
    {&signal_gain, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW,
     "Signal Gain (x)"},
    {&signal_dc_offset, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW,
     "Signal DC Offset (V)"},
};

static const gmp_mem_region_t memory_regions[] = {
    {memory_window, sizeof(memory_window), GMP_MEM_PERM_RW, "Scratch Memory"},
};

static void user_append_facility(fast_gt result)
{
    if (!result)
        dl_facility_init_errors++;
}

static void user_apply_signal_parameters(void)
{
    float frequency_hz = signal_frequency_hz;
    float gain = signal_gain;
    float dc_offset = signal_dc_offset;
    float angle;
    ctrl_gt step_sine;
    ctrl_gt step_cosine;

    if (frequency_hz < USER_SIGNAL_MIN_RATE_HZ)
        frequency_hz = USER_SIGNAL_MIN_RATE_HZ;
    else if (frequency_hz > USER_SIGNAL_MAX_RATE_HZ)
        frequency_hz = USER_SIGNAL_MAX_RATE_HZ;
    if (gain < 0.0F)
        gain = 0.0F;
    else if (gain > USER_SIGNAL_MAX_GAIN)
        gain = USER_SIGNAL_MAX_GAIN;
    if (dc_offset < -USER_SIGNAL_MAX_OFFSET)
        dc_offset = -USER_SIGNAL_MAX_OFFSET;
    else if (dc_offset > USER_SIGNAL_MAX_OFFSET)
        dc_offset = USER_SIGNAL_MAX_OFFSET;

    signal_frequency_hz = frequency_hz;
    signal_gain = gain;
    signal_dc_offset = dc_offset;
    if (frequency_hz == applied_frequency_hz &&
        gain == applied_signal_gain &&
        dc_offset == applied_signal_dc_offset)
        return;

    angle = USER_SIGNAL_TWO_PI * frequency_hz / (float)USER_DSA_SAMPLE_RATE;
    step_sine = real2ctrl(sinf(angle));
    step_cosine = real2ctrl(cosf(angle));
    gmp_base_enter_critical();
    oscillator_step_sine = step_sine;
    oscillator_step_cosine = step_cosine;
    active_signal_gain = real2ctrl(gain);
    active_signal_dc_offset = real2ctrl(dc_offset);
    gmp_base_leave_critical();
    applied_frequency_hz = frequency_hz;
    applied_signal_gain = gain;
    applied_signal_dc_offset = dc_offset;
}

static gmp_task_status_t user_task_datalink(gmp_task_t* task)
{
    gmp_dl_event_t event;
    GMP_UNUSED_VAR(task);
    event = gmp_dev_dl_loop_cb(&datalink);
    if (event == GMP_DL_EVENT_TX_RDY)
        xplt_dl_start_tx(&datalink);
    else if (event == GMP_DL_EVENT_RX_OK)
        (void)gmp_dev_dl_dispatch_rx(&datalink);
    user_apply_signal_parameters();
    return GMP_TASK_DONE;
}

static gmp_task_status_t user_task_heartbeat(gmp_task_t* task)
{
    GMP_UNUSED_VAR(task);
    xplt_toggle_user_led();
    return GMP_TASK_DONE;
}

static gmp_task_t tasks[] = {
    {"datalink", user_task_datalink, 1U, 0U, 1, NULL},
    {"heartbeat", user_task_heartbeat, 500U, 0U, 1, NULL},
};

void init(void)
{
    size_gt index;
    size_gt task_index;

    for (index = 0; index < sizeof(memory_window); ++index)
        memory_window[index] = (byte_gt)index;

    gmp_scheduler_init(&scheduler);
    for (task_index = 0; task_index < sizeof(tasks) / sizeof(tasks[0]); ++task_index)
        (void)gmp_scheduler_add_task(&scheduler, &tasks[task_index]);

    dl_facility_init_errors = 0U;
    gmp_dev_dl_init(&datalink);
    gmp_pil_sim_init(&pil, &datalink, USER_DL_PIL_CMD);
    user_append_facility(gmp_dev_dl_append_facility(&datalink, &pil.facility));
    gmp_param_tunable_init(&tunable, &datalink, USER_DL_TUNABLE_CMD,
                           tunable_dictionary,
                           (fast16_gt)(sizeof(tunable_dictionary) /
                                       sizeof(tunable_dictionary[0])));
    user_append_facility(gmp_dev_dl_append_facility(&datalink, &tunable.facility));
    gmp_mem_persp_init(&memory_perspective, &datalink, USER_DL_MEMORY_CMD,
                       memory_regions,
                       (fast16_gt)(sizeof(memory_regions) / sizeof(memory_regions[0])));
    user_append_facility(gmp_dev_dl_append_facility(
        &datalink, &memory_perspective.facility));
    if (!ctl_init_dsa_dl_scope_workspace(
            &dl_scope, &datalink, USER_DL_SCOPE_CMD, "Sine and Cosine Scope",
            dl_scope_storage,
            (uint32_t)(sizeof(dl_scope_storage) / sizeof(dl_scope_storage[0])),
            USER_DSA_CHANNELS, USER_DSA_SAMPLE_RATE))
    {
        dl_facility_init_errors++;
    }
    else
    {
        user_append_facility(gmp_dev_dl_append_facility(
            &datalink, ctl_dsa_dl_scope_facility(&dl_scope)));
    }

    oscillator_sine = real2ctrl(0.0F);
    oscillator_cosine = real2ctrl(1.0F);
    oscillator_index = 0U;
    user_apply_signal_parameters();
    xplt_dl_bind(&datalink);
    xplt_start_sample_timer();
}

void mainloop(void)
{
    gmp_scheduler_dispatch(&scheduler);
}

void user_dsa_timer_step(void)
{
    ctrl_gt unit_sine = oscillator_sine;
    ctrl_gt unit_cosine = oscillator_cosine;
    ctrl_gt step_sine = oscillator_step_sine;
    ctrl_gt step_cosine = oscillator_step_cosine;
    ctrl_gt gain = active_signal_gain;
    ctrl_gt dc_offset = active_signal_dc_offset;
    ctrl_gt sine_sample = unit_sine * gain + dc_offset;
    ctrl_gt cosine_sample = unit_cosine * gain + dc_offset;

    ctl_step_dsa_dl_scope_2ch(&dl_scope, sine_sample, cosine_sample);
    oscillator_sine = unit_sine * step_cosine + unit_cosine * step_sine;
    oscillator_cosine = unit_cosine * step_cosine - unit_sine * step_sine;
    oscillator_index++;
    if (oscillator_index >= USER_OSC_NORMALIZE_PERIOD)
    {
        ctrl_gt magnitude_squared;
        ctrl_gt correction;
        oscillator_index = 0U;
        magnitude_squared = oscillator_sine * oscillator_sine +
                            oscillator_cosine * oscillator_cosine;
        correction = real2ctrl(1.5F) - real2ctrl(0.5F) * magnitude_squared;
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
