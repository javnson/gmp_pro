/**
 * @file user_main.c
 * @brief GMP user application providing scheduled Data Link and LED tasks.
 */

#include <gmp_core.h>

#include <math.h>

#include "user_main.h"
#include <core/dev/datalink/datalink.h>
#include <core/dev/datalink/mem_presp.h>
#include <core/dev/datalink/scope.h>
#include <core/dev/datalink/tunable.h>
#include <ctl/component/dsa/dsa_scope.h>
#include <ctl/component/dsa/dsa_trigger.h>
#include <xplt.peripheral.h>

#if GMP_PORT_DATA_SIZE_PER_BYTES != 2
#error "The LAUNCHXL-F280049C validation firmware requires the u16 Data Link backend"
#endif

#define USER_DL_CMD_INFO          0x02U
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

/** @brief DSA acquisition states exported to the desktop debugger. */
typedef enum
{
    USER_DSA_WAITING = 0,
    USER_DSA_CAPTURING,
    USER_DSA_READY
} user_dsa_state_t;

static gmp_scheduler_t scheduler;
static gmp_datalink_t datalink;
static gmp_param_tunable_t tunable;
static gmp_mem_persp_t memory_perspective;
static gmp_scope_service_t scope_service;
static ctl_dsa_trigger_t dsa_trigger;
static ctl_dsa_scope_t dsa_scope;
static volatile user_dsa_state_t dsa_state;
static volatile uint32_t dsa_generation;
static volatile uint16_t dsa_history_write;
static volatile uint16_t dsa_history_count;
static volatile uint16_t dsa_trigger_position_permille = 500U;
static volatile fast16_gt dsa_trigger_channel;
static volatile ctl_dsa_trigger_option_t dsa_trigger_mode = DSA_TRIGGER_OPTION_RISING_EDGE;
static volatile uint32_t dsa_auto_timeout_ms = 1000UL;
static volatile parameter_gt dsa_trigger_level;

static float signal_frequency_hz = 50.0F;
static float signal_gain = 1.0F;
static float signal_dc_offset = 0.0F;
static float applied_frequency_hz = -1.0F;
static float applied_signal_gain = -1.0F;
static float applied_signal_dc_offset = -100.0F;
static data_gt memory_window[64];
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(dsa_buffer, "ramgs0")
#endif
static ctrl_gt dsa_buffer[USER_DSA_DEPTH * USER_DSA_CHANNELS];
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(dsa_history, "ramgs1")
#endif
static ctrl_gt dsa_history[USER_DSA_DEPTH * USER_DSA_CHANNELS];
static volatile ctrl_gt oscillator_sine;
static volatile ctrl_gt oscillator_cosine;
static volatile ctrl_gt oscillator_step_sine;
static volatile ctrl_gt oscillator_step_cosine;
static volatile ctrl_gt active_signal_gain;
static volatile ctrl_gt active_signal_dc_offset;
static uint16_t oscillator_index;

static fast_gt user_scope_configure(void* user_context,
                                    const gmp_scope_config_t* config);
static fast_gt user_scope_arm(void* user_context);
static gmp_scope_capture_state_t user_scope_status(void* user_context,
                                                   uint32_t* generation);

static const gmp_param_item_t tunable_dictionary[] = {
    {&signal_frequency_hz, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW,
     "Signal Frequency (Hz)"},
    {&signal_gain, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW,
     "Signal Gain (x)"},
    {&signal_dc_offset, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW,
     "Signal DC Offset (V)"},
};

static const gmp_mem_region_t memory_regions[] = {
    {memory_window, sizeof(memory_window) * GMP_PORT_DATA_SIZE_PER_BYTES,
     GMP_MEM_PERM_RW, "Scratch Memory"},
};

static const gmp_scope_resource_t scope_resources[] = {
    {"Sine and Cosine Scope", dsa_buffer,
     sizeof(dsa_buffer) * GMP_PORT_DATA_SIZE_PER_BYTES,
     GMP_SCOPE_SAMPLE_F32, GMP_SCOPE_LAYOUT_SOA, USER_DSA_CHANNELS,
     USER_DSA_DEPTH, USER_DSA_SAMPLE_RATE, user_scope_configure,
     user_scope_arm, user_scope_status, NULL},
};

/**
 * @brief Clamp and apply the user-visible waveform parameters.
 *
 * @details The trigonometric step is recalculated only when the frequency
 * changes. The 1 kHz timer ISR then advances the oscillator with a two-term
 * rotation, avoiding a sine-library call on every sample.
 */
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
    step_sine = float2ctrl(sinf(angle));
    step_cosine = float2ctrl(cosf(angle));
    gmp_base_enter_critical();
    oscillator_step_sine = step_sine;
    oscillator_step_cosine = step_cosine;
    active_signal_gain = float2ctrl(gain);
    active_signal_dc_offset = float2ctrl(dc_offset);
    gmp_base_leave_critical();

    applied_frequency_hz = frequency_hz;
    applied_signal_gain = gain;
    applied_signal_dc_offset = dc_offset;
}

/** @brief Return the requested number of pre-trigger samples. */
static uint16_t user_dsa_pretrigger_samples(void)
{
    uint32_t samples = (USER_DSA_DEPTH * dsa_trigger_position_permille) / 1000U;
    if (samples >= USER_DSA_DEPTH)
        samples = USER_DSA_DEPTH - 1U;
    return (uint16_t)samples;
}

/** @brief Store one two-channel sample in the circular pre-trigger history. */
static void user_dsa_store_history(ctrl_gt channel_0, ctrl_gt channel_1)
{
    dsa_history[dsa_history_write] = channel_0;
    dsa_history[USER_DSA_DEPTH + dsa_history_write] = channel_1;
    dsa_history_write++;
    if (dsa_history_write >= USER_DSA_DEPTH)
        dsa_history_write = 0U;
    if (dsa_history_count < USER_DSA_DEPTH)
        dsa_history_count++;
}

/** @brief Copy the chronological pre-trigger history into the scope buffer. */
static void user_dsa_copy_history(uint16_t sample_count)
{
    uint16_t index;
    uint16_t source = (uint16_t)((dsa_history_write + USER_DSA_DEPTH - sample_count) %
                                 USER_DSA_DEPTH);
    for (index = 0U; index < sample_count; ++index)
    {
        dsa_buffer[index] = dsa_history[source];
        dsa_buffer[USER_DSA_DEPTH + index] = dsa_history[USER_DSA_DEPTH + source];
        source++;
        if (source >= USER_DSA_DEPTH)
            source = 0U;
    }
}

/** @brief Reset the trigger and arm one coherent DSA snapshot. */
static void user_arm_dsa(void)
{
    uint32_t timeout_ticks;
    gmp_base_enter_critical();
    ctl_clear_dsa_trigger(&dsa_trigger);
    dsa_trigger.option = dsa_trigger_mode;
    dsa_trigger.trigger_level = float2ctrl(dsa_trigger_level);
    timeout_ticks = (dsa_auto_timeout_ms * USER_DSA_SAMPLE_RATE) / 1000U;
    dsa_trigger.auto_timeout_ticks = (timeout_ticks == 0U) ? 1U : timeout_ticks;
    dsa_trigger.flag_is_force_trigger = 0;
    ctl_reset_dsa_scope_tracker(&dsa_scope);
    dsa_history_write = 0U;
    dsa_history_count = 0U;
    dsa_state = USER_DSA_WAITING;
    gmp_base_leave_critical();
}

/** @brief Apply one validated Data Link Scope configuration. */
static fast_gt user_scope_configure(void* user_context,
                                    const gmp_scope_config_t* config)
{
    GMP_UNUSED_VAR(user_context);
    if (config == NULL ||
        config->mode > DSA_TRIGGER_OPTION_FALLING_EDGE_AUTO ||
        config->channel >= USER_DSA_CHANNELS ||
        config->position_permille > 1000U)
        return 0;
    dsa_trigger_mode = (ctl_dsa_trigger_option_t)config->mode;
    dsa_trigger_channel = config->channel;
    dsa_trigger_position_permille = config->position_permille;
    dsa_trigger_level = config->level;
    dsa_auto_timeout_ms = config->auto_timeout_ms;
    return 1;
}

/** @brief Arm the demo capture through the generic Scope service callback. */
static fast_gt user_scope_arm(void* user_context)
{
    GMP_UNUSED_VAR(user_context);
    user_arm_dsa();
    return 1;
}

/** @brief Export the current demo capture state and generation. */
static gmp_scope_capture_state_t user_scope_status(void* user_context,
                                                   uint32_t* generation)
{
    GMP_UNUSED_VAR(user_context);
    if (generation != NULL)
        *generation = dsa_generation;
    return (gmp_scope_capture_state_t)dsa_state;
}

/** @brief Queue the target and backend discovery response. */
static void user_reply_info(void)
{
    gmp_dev_dl_tx_request_cmd(&datalink, datalink.rx_head.seq_id, USER_DL_CMD_INFO);
    gmp_dev_dl_tx_append_u8(&datalink, 2U);
    gmp_dev_dl_tx_append_u8(&datalink, GMP_PORT_DATA_SIZE_PER_BYTES);
    gmp_dev_dl_tx_append_u8(&datalink, sizeof(data_gt));
    gmp_dev_dl_tx_append_u8(&datalink, 16U);
    gmp_dev_dl_tx_append_u8(&datalink, USER_DL_TUNABLE_CMD);
    gmp_dev_dl_tx_append_u8(&datalink, USER_DL_MEMORY_CMD);
    gmp_dev_dl_tx_append_u8(&datalink, USER_DL_SCOPE_CMD);
    gmp_dev_dl_tx_append_u32(
        &datalink,
        (uint32_t)(uintptr_t)memory_window * GMP_PORT_DATA_SIZE_PER_BYTES);
    gmp_dev_dl_tx_append_u16(
        &datalink,
        (uint16_t)(sizeof(memory_window) * GMP_PORT_DATA_SIZE_PER_BYTES));
    gmp_dev_dl_tx_ready(&datalink);
    gmp_dev_dl_msg_handled(&datalink);
}

/** @brief Service the Data Link state machine without blocking. */
static gmp_task_status_t user_task_datalink(gmp_task_t* task)
{
    gmp_dl_event_t event;
    GMP_UNUSED_VAR(task);

    xplt_dl_poll_rx();
    event = gmp_dev_dl_loop_cb(&datalink);
    if (event == GMP_DL_EVENT_TX_RDY)
    {
        xplt_dl_start_tx(&datalink);
    }
    else if (event == GMP_DL_EVENT_RX_OK)
    {
        if (datalink.rx_head.cmd == USER_DL_CMD_INFO)
            user_reply_info();
        else if (!gmp_param_tunable_rx_cb(&tunable) &&
                 !gmp_mem_persp_rx_cb(&memory_perspective) &&
                 !gmp_scope_rx_cb(&scope_service))
            gmp_dev_dl_default_rx_handler(&datalink);
    }
    user_apply_signal_parameters();
    return GMP_TASK_DONE;
}

/** @brief Toggle the board LED as a scheduler heartbeat. */
static gmp_task_status_t user_task_heartbeat(gmp_task_t* task)
{
    GMP_UNUSED_VAR(task);
    xplt_toggle_user_led();
    return GMP_TASK_DONE;
}

/**
 * @brief The two non-blocking tasks required by this validation target.
 * @details The heartbeat is checked first when its period expires. The Data
 * Link service uses a zero period so residual characters below the SCI FIFO
 * interrupt threshold are drained on every otherwise-idle scheduler pass.
 */
static gmp_task_t tasks[] = {
    {"heartbeat", user_task_heartbeat, 500U, 0U, 1, NULL},
    {"datalink", user_task_datalink, 0U, 0U, 1, NULL},
};

void init(void)
{
    size_gt index;
    size_gt task_index;

    for (index = 0; index < sizeof(memory_window); ++index)
    {
        data_gt low_byte = (data_gt)((index * 2U) & 0xFFU);
        data_gt high_byte = (data_gt)(((index * 2U + 1U) & 0xFFU) << 8U);
        memory_window[index] = low_byte | high_byte;
    }

    gmp_scheduler_init(&scheduler);
    for (task_index = 0; task_index < sizeof(tasks) / sizeof(tasks[0]); ++task_index)
        (void)gmp_scheduler_add_task(&scheduler, &tasks[task_index]);

    gmp_dev_dl_init(&datalink);
    gmp_param_tunable_init(&tunable, &datalink, USER_DL_TUNABLE_CMD,
                           tunable_dictionary,
                           (fast16_gt)(sizeof(tunable_dictionary) /
                                       sizeof(tunable_dictionary[0])));
    gmp_mem_persp_init(&memory_perspective, &datalink, USER_DL_MEMORY_CMD,
                       memory_regions,
                       (fast16_gt)(sizeof(memory_regions) / sizeof(memory_regions[0])));
    gmp_scope_init(&scope_service, &datalink, USER_DL_SCOPE_CMD,
                   scope_resources,
                   (fast16_gt)(sizeof(scope_resources) / sizeof(scope_resources[0])));

    ctl_init_dsa_trigger(&dsa_trigger, DSA_TRIGGER_OPTION_RISING_EDGE,
                         0.0F, 1.0F, (parameter_gt)USER_DSA_SAMPLE_RATE);
    ctl_init_dsa_scope(&dsa_scope, dsa_buffer,
                       (uint32_t)(sizeof(dsa_buffer) / sizeof(dsa_buffer[0])),
                       (parameter_gt)USER_DSA_SAMPLE_RATE);
    ctl_config_dsa_scope(&dsa_scope, USER_DSA_CHANNELS, 1U);
    oscillator_sine = float2ctrl(0.0F);
    oscillator_cosine = float2ctrl(1.0F);
    oscillator_index = 0U;
    user_apply_signal_parameters();
    dsa_generation = 0U;
    dsa_trigger_level = 0.0F;
    user_arm_dsa();

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
    ctrl_gt trigger_sample = (dsa_trigger_channel == 0U) ? sine_sample : cosine_sample;
    uint16_t pretrigger_samples = user_dsa_pretrigger_samples();

    if (dsa_state == USER_DSA_WAITING)
    {
        fast_gt is_triggered = ctl_step_dsa_trigger(&dsa_trigger, trigger_sample);
        if (is_triggered && dsa_history_count >= pretrigger_samples)
        {
            user_dsa_copy_history(pretrigger_samples);
            dsa_buffer[pretrigger_samples] = sine_sample;
            dsa_buffer[USER_DSA_DEPTH + pretrigger_samples] = cosine_sample;
            dsa_scope.current_idx = (uint32_t)pretrigger_samples + 1U;
            ctl_clear_divider(&dsa_scope.divider);
            if (dsa_scope.current_idx >= dsa_scope.depth)
            {
                dsa_generation++;
                dsa_state = USER_DSA_READY;
            }
            else
            {
                dsa_state = USER_DSA_CAPTURING;
            }
        }
        else
        {
            user_dsa_store_history(sine_sample, cosine_sample);
        }
    }

    else if (dsa_state == USER_DSA_CAPTURING)
    {
        (void)ctl_step_dsa_scope_2ch(&dsa_scope, sine_sample, cosine_sample);
        if (dsa_scope.current_idx >= dsa_scope.depth)
        {
            dsa_generation++;
            dsa_state = USER_DSA_READY;
        }
    }

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
        correction = float2ctrl(1.5F) - float2ctrl(0.5F) * magnitude_squared;
        oscillator_sine *= correction;
        oscillator_cosine *= correction;
    }
}
