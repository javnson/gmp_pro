/** @file user_dl.c Board-independent GMP Data Link background service. */

#include <gmp_core.h>

#include <core/dev/datalink/datalink.h>
#include <core/dev/datalink/mem_presp.h>
#include <xplt.peripheral.h>

#include "user_dl.h"

static gmp_datalink_t datalink;
static gmp_mem_persp_t memory_perspective;

#define USER_DL_MEMORY_COMMAND 0x50U

static const gmp_mem_region_t memory_regions[] = {
    {(void*)gmp_nucleo_adc_raw, sizeof(gmp_nucleo_adc_raw), GMP_MEM_PERM_RO,
     "ADC Feedback Raw"},
    {(void*)&gmp_nucleo_qep_count, sizeof(gmp_nucleo_qep_count), GMP_MEM_PERM_RO,
     "QEP Count"},
    {(void*)gmp_nucleo_pwm_compare, sizeof(gmp_nucleo_pwm_compare),
     GMP_MEM_PERM_RW, "PWM Compare (outputs remain disabled)"},
};

void user_dl_init(void)
{
    gmp_dev_dl_init(&datalink);
    gmp_mem_persp_init(
        &memory_perspective, &datalink, USER_DL_MEMORY_COMMAND, memory_regions,
        (fast16_gt)(sizeof(memory_regions) / sizeof(memory_regions[0])));
    (void)gmp_dev_dl_append_facility(&datalink, &memory_perspective.facility);
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
    return GMP_TASK_DONE;
}
