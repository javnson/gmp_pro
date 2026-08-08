/**
 * @file user_main.c
 * @brief GMP user application providing scheduled Data Link and LED tasks.
 */

#include <gmp_core.h>

#include "user_main.h"
#include <core/dev/datalink.h>
#include <core/dev/mem_presp.h>
#include <core/dev/tunable.h>
#include <ctl/component/dsa/dsa_scope.h>
#include <ctl/component/dsa/dsa_trigger.h>
#include <xplt.peripheral.h>

#if GMP_PORT_DATA_SIZE_PER_BYTES != 1
#error "The NUCLEO-C092RC validation firmware requires the u8 Data Link backend"
#endif

#define USER_DL_CMD_INFO          0x02U
#define USER_DL_DSA_CMD_INFO      0x03U
#define USER_DL_DSA_CMD_ARM       0x04U
#define USER_DL_TUNABLE_CMD       0x30U
#define USER_DL_MEMORY_CMD        0x50U
#define USER_DSA_SAMPLE_RATE      1000UL
#define USER_DSA_SIGNAL_RATE      50UL
#define USER_DSA_DEPTH            400UL
#define USER_DSA_CHANNELS         2U
#define USER_DSA_FORMAT_F32       1U

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
static ctl_dsa_trigger_t dsa_trigger;
static ctl_dsa_scope_t dsa_scope;
static volatile user_dsa_state_t dsa_state;
static volatile uint32_t dsa_generation;

static uint16_t tunable_u16 = 0x1234U;
static int16_t tunable_i16 = -1234;
static uint32_t tunable_u32 = 0x89ABCDEFUL;
static float tunable_f32 = 3.1415926F;
static gmp_dl_octet_t memory_window[128];
static ctrl_gt dsa_buffer[USER_DSA_DEPTH * USER_DSA_CHANNELS];
static ctrl_gt oscillator_sine;
static ctrl_gt oscillator_cosine;
static uint16_t oscillator_index;

static const gmp_param_item_t tunable_dictionary[] = {
    {&tunable_u16, GMP_PARAM_TYPE_U16, GMP_PARAM_PERM_RW},
    {&tunable_i16, GMP_PARAM_TYPE_I16, GMP_PARAM_PERM_RW},
    {&tunable_u32, GMP_PARAM_TYPE_U32, GMP_PARAM_PERM_RW},
    {&tunable_f32, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW},
};

static const gmp_mem_region_t memory_regions[] = {
    {memory_window, sizeof(memory_window), GMP_MEM_PERM_RW},
    {dsa_buffer, sizeof(dsa_buffer), GMP_MEM_PERM_RO},
};

/** @brief Reset the trigger and arm one coherent DSA snapshot. */
static void user_arm_dsa(void)
{
    gmp_base_enter_critical();
    ctl_clear_dsa_trigger(&dsa_trigger);
    dsa_trigger.flag_is_force_trigger = 0;
    ctl_reset_dsa_scope_tracker(&dsa_scope);
    dsa_state = USER_DSA_WAITING;
    gmp_base_leave_critical();
}

/** @brief Queue the target and backend discovery response. */
static void user_reply_info(void)
{
    gmp_dev_dl_tx_request_cmd(&datalink, datalink.rx_head.seq_id, USER_DL_CMD_INFO);
    gmp_dev_dl_tx_append_u8(&datalink, 1U);
    gmp_dev_dl_tx_append_u8(&datalink, GMP_PORT_DATA_SIZE_PER_BYTES);
    gmp_dev_dl_tx_append_u8(&datalink, sizeof(data_gt));
    gmp_dev_dl_tx_append_u8(&datalink, 8U);
    gmp_dev_dl_tx_append_u8(&datalink, USER_DL_TUNABLE_CMD);
    gmp_dev_dl_tx_append_u8(&datalink, USER_DL_MEMORY_CMD);
    gmp_dev_dl_tx_append_u32(&datalink, (uint32_t)(uintptr_t)memory_window);
    gmp_dev_dl_tx_append_u16(&datalink, (uint16_t)sizeof(memory_window));
    gmp_dev_dl_tx_ready(&datalink);
    gmp_dev_dl_msg_handled(&datalink);
}

/** @brief Return DSA snapshot metadata required for Memory Perspective reads. */
static void user_reply_dsa_info(void)
{
    gmp_dev_dl_tx_request_cmd(&datalink, datalink.rx_head.seq_id, USER_DL_DSA_CMD_INFO);
    gmp_dev_dl_tx_append_u8(&datalink, 1U);
    gmp_dev_dl_tx_append_u8(&datalink, (uint8_t)dsa_state);
    gmp_dev_dl_tx_append_u8(&datalink, USER_DSA_FORMAT_F32);
    gmp_dev_dl_tx_append_u8(&datalink, USER_DSA_CHANNELS);
    gmp_dev_dl_tx_append_u16(&datalink, (uint16_t)USER_DSA_DEPTH);
    gmp_dev_dl_tx_append_u32(&datalink, USER_DSA_SAMPLE_RATE);
    gmp_dev_dl_tx_append_u32(&datalink, (uint32_t)(uintptr_t)dsa_buffer);
    gmp_dev_dl_tx_append_u16(&datalink, (uint16_t)sizeof(dsa_buffer));
    gmp_dev_dl_tx_append_u32(&datalink, dsa_generation);
    gmp_dev_dl_tx_ready(&datalink);
    gmp_dev_dl_msg_handled(&datalink);
}

/** @brief Arm a new DSA snapshot and acknowledge the command. */
static void user_reply_dsa_arm(void)
{
    user_arm_dsa();
    gmp_dev_dl_tx_request_cmd(&datalink, datalink.rx_head.seq_id, USER_DL_DSA_CMD_ARM);
    gmp_dev_dl_tx_append_u8(&datalink, 0U);
    gmp_dev_dl_tx_ready(&datalink);
    gmp_dev_dl_msg_handled(&datalink);
}

/** @brief Service the Data Link state machine without blocking. */
static gmp_task_status_t user_task_datalink(gmp_task_t* task)
{
    gmp_dl_event_t event;
    GMP_UNUSED_VAR(task);

    event = gmp_dev_dl_loop_cb(&datalink);
    if (event == GMP_DL_EVENT_TX_RDY)
    {
        xplt_dl_start_tx(&datalink);
    }
    else if (event == GMP_DL_EVENT_RX_OK)
    {
        if (datalink.rx_head.cmd == USER_DL_CMD_INFO)
            user_reply_info();
        else if (datalink.rx_head.cmd == USER_DL_DSA_CMD_INFO)
            user_reply_dsa_info();
        else if (datalink.rx_head.cmd == USER_DL_DSA_CMD_ARM)
            user_reply_dsa_arm();
        else if (!gmp_param_tunable_rx_cb(&tunable) &&
                 !gmp_mem_persp_rx_cb(&memory_perspective))
            gmp_dev_dl_default_rx_handler(&datalink);
    }
    return GMP_TASK_DONE;
}

/** @brief Toggle the board LED as a scheduler heartbeat. */
static gmp_task_status_t user_task_heartbeat(gmp_task_t* task)
{
    GMP_UNUSED_VAR(task);
    xplt_toggle_user_led();
    return GMP_TASK_DONE;
}

/** @brief The two non-blocking tasks required by this validation target. */
static gmp_task_t tasks[] = {
    {"datalink", user_task_datalink, 1U, 0U, 1, NULL},
    {"heartbeat", user_task_heartbeat, 500U, 0U, 1, NULL},
};

void init(void)
{
    size_gt index;
    size_gt task_index;

    for (index = 0; index < sizeof(memory_window); ++index)
        memory_window[index] = (gmp_dl_octet_t)index;

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

    ctl_init_dsa_trigger(&dsa_trigger, DSA_TRIGGER_OPTION_RISING_EDGE,
                         0.0F, 1.0F, (parameter_gt)USER_DSA_SAMPLE_RATE);
    ctl_init_dsa_scope(&dsa_scope, dsa_buffer,
                       (uint32_t)(sizeof(dsa_buffer) / sizeof(dsa_buffer[0])),
                       (parameter_gt)USER_DSA_SAMPLE_RATE);
    ctl_config_dsa_scope(&dsa_scope, USER_DSA_CHANNELS, 1U);
    oscillator_sine = float2ctrl(0.0F);
    oscillator_cosine = float2ctrl(1.0F);
    oscillator_index = 0U;
    dsa_generation = 0U;
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
    ctrl_gt sine_sample = oscillator_sine;
    ctrl_gt cosine_sample = oscillator_cosine;

    if (dsa_state == USER_DSA_WAITING &&
        ctl_step_dsa_trigger(&dsa_trigger, sine_sample))
        dsa_state = USER_DSA_CAPTURING;

    if (dsa_state == USER_DSA_CAPTURING)
    {
        (void)ctl_step_dsa_scope_2ch(&dsa_scope, sine_sample, cosine_sample);
        if (dsa_scope.current_idx >= dsa_scope.depth)
        {
            dsa_generation++;
            dsa_state = USER_DSA_READY;
        }
    }

    oscillator_index++;
    if (oscillator_index >= (USER_DSA_SAMPLE_RATE / USER_DSA_SIGNAL_RATE))
    {
        oscillator_index = 0U;
        oscillator_sine = float2ctrl(0.0F);
        oscillator_cosine = float2ctrl(1.0F);
    }
    else
    {
        oscillator_sine = sine_sample * float2ctrl(0.9510565163F) +
                          cosine_sample * float2ctrl(0.3090169944F);
        oscillator_cosine = cosine_sample * float2ctrl(0.9510565163F) -
                            sine_sample * float2ctrl(0.3090169944F);
    }
}
