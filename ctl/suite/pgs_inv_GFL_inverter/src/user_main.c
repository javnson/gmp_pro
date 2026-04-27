// This is the example of user main.

// GMP basic core header
#include <gmp_core.h>

// user main header
#include "user_main.h"

#include <core/dev/pil_core.h>
#include <core/dev/tunable.h>
#include <core/dev/mem_presp.h>

//=================================================================================================
// global variables

gmp_scheduler_t sched;

gmp_datalink_t dl;
gmp_pil_sim_t pil;

// 1. 静态定义 M1 的字典
const gmp_param_item_t dict_m1[] = {
    { &inv_ctrl.filter_udc.out, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO },
    { &inv_ctrl.idq_set.dat[phase_d], GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW },
    { &inv_ctrl.idq_set.dat[phase_q], GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW },
    { &inv_ctrl.idq.dat[phase_d], GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO },
    { &inv_ctrl.idq.dat[phase_q], GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO },
};
const uint16_t var_tunable_count = sizeof(dict_m1) / sizeof(dict_m1[0]);

// 3. 实例化两个 Server 对象
gmp_param_tunable_t srv_m1;

const gmp_mem_region_t mem_regions[] = {
    // 区域 1: 波形缓存区 (RO 只读)
    {
        .base_addr   = &inv_ctrl, // 直接给原生指针，编译器非常开心
        .byte_length = sizeof(inv_ctrl) * GMP_PORT_DATA_SIZE_PER_BYTES, // 常数运算，完美通过
        .perm        = GMP_MEM_PERM_RW
    }
};
const uint16_t mem_regions_count = sizeof(mem_regions) / sizeof(mem_regions[0]);

gmp_mem_persp_t mem_persp_server;


//=================================================================================================
// task manager

gmp_task_status_t tsk_blink(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    gmp_base_print(TEXT_STRING("Hello World!\r\n"));

    return GMP_TASK_DONE;
}

gmp_task_status_t tsk_at_device(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    flush_dl_rx_buffer();

    gmp_dl_event_t e = gmp_dev_dl_loop_cb(&dl);

    switch (e)
    {
    // if TX data is ready, do transmit
    case GMP_DL_EVENT_TX_RDY:
        // 物理发送逻辑（使用你平台的 UART/DMA 发送函数）
        gmp_hal_uart_write(IRIS_UART_USB_BASE, gmp_dev_dl_get_tx_hw_hdr_ptr(&dl), gmp_dev_dl_get_tx_hw_hdr_size(&dl),
                           10); // 发送 Header (含 SOF 和 转义)
        if (gmp_dev_dl_get_tx_hw_pld_size(&dl) > 0)
        {
            gmp_hal_uart_write(IRIS_UART_USB_BASE, gmp_dev_dl_get_tx_hw_pld_ptr(&dl),
                               gmp_dev_dl_get_tx_hw_pld_size(&dl), 10); // 发送 Payload 和 P_CRC
        }

        gmp_dev_dl_tx_state_done(&dl); // 释放 TX 状态机
        break;

    case GMP_DL_EVENT_RX_OK:

        //
        // Ack PIL simulation message
        //
        if(gmp_pil_sim_rx_cb(&pil)) break;

        //
        // Ack parameter tunable message
        //
        if (gmp_param_tunable_rx_cb(&srv_m1)) break;

        //
        // Ack memory perspective message
        //
        if (gmp_mem_persp_rx_cb(&mem_persp_server)) break;

        if (dl.rx_head.cmd == 0x99)
        {
            // 使用刚刚写好的 Builder 连写 API，原封不动地将 payload_buf 中的数据打包回传
            gmp_dev_dl_tx_request(&dl, dl.rx_head.seq_id, GMP_DL_CMD_ECHO, dl.expected_payload_len, dl.payload_buf);

            // 使用规范的 API 声明该指令已处理
            gmp_dev_dl_msg_handled(&dl);
        }
        else
        {
            gmp_dev_dl_default_rx_handler(&dl);
        }

        break;

    }

    return GMP_TASK_DONE;
}

void send_monitor_data(void);
gmp_task_status_t tsk_monitor(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    send_monitor_data();

    return GMP_TASK_DONE;
}

// All tasks must be non blocking tasks
gmp_task_t tasks[] = {
    // name,     task,      period(ms),  init_phase, is_enabled, pParam
    {"blink_led", tsk_blink, 1000, 0, 1, NULL},
    {"at_device", tsk_at_device, 5, 1, 1, NULL},
    {"monitor_data", tsk_monitor, 2, 0, 1, NULL}};

//=================================================================================================
// initialize routine

GMP_NO_OPT_PREFIX
void init(void) GMP_NO_OPT_SUFFIX
{
    int i;

    gmp_scheduler_init(&sched);

    for (i = 0; i < sizeof(tasks) / sizeof(gmp_task_t); ++i)
        gmp_scheduler_add_task(&sched, &tasks[i]);

    // 1. 初始化数据链路层
    gmp_dev_dl_init(&dl);

    gmp_pil_sim_init(&pil, &dl, 0x10);

    // 绑定 Datalink, M1 占用 0x30
    gmp_param_tunable_init(&srv_m1, &dl, 0x30, dict_m1, var_tunable_count);
    gmp_mem_persp_init(&mem_persp_server, &dl, 0x50, mem_regions, mem_regions_count);
}

//=================================================================================================
// endless loop routine

GMP_NO_OPT_PREFIX
void mainloop(void) GMP_NO_OPT_SUFFIX
{
    // run task scheduler
    gmp_scheduler_dispatch(&sched);
}
