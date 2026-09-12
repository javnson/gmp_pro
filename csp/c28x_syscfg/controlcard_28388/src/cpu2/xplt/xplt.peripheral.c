/** @file xplt.peripheral.c CPU2 timer and message-RAM binding. */

#include <gmp_core.h>
#include "device.h"
#include "driverlib.h"
#include "user_main.h"
#include <xplt.peripheral.h>

#pragma DATA_SECTION(cpu1_to_cpu2_command, "GMP_MSGRAM_CPU1_TO_CPU2")
volatile gmp_wave_command_t cpu1_to_cpu2_command;
#pragma DATA_SECTION(cm_to_cpu2_command, "GMP_MSGRAM_CM_TO_CPU")
volatile gmp_wave_command_t cm_to_cpu2_command;
#pragma DATA_SECTION(cpu2_to_cpu1_snapshot, "GMP_MSGRAM_CPU2_TO_CPU1")
volatile gmp_wave_snapshot_t cpu2_to_cpu1_snapshot;
#pragma DATA_SECTION(cpu2_to_cm_snapshot, "GMP_MSGRAM_CPU_TO_CM")
volatile gmp_wave_snapshot_t cpu2_to_cm_snapshot;

__interrupt static void xplt_cpu2_timer_isr(void)
{
    user_cpu2_control_step();
    gmp_step_system_tick();
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

void setup_peripheral(void)
{
    CPUTimer_stopTimer(CPUTIMER0_BASE);
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0U);
    CPUTimer_setPeriod(CPUTIMER0_BASE,
                       (GMP_F28388D_CPU_CLOCK_HZ / 1000UL) - 1UL);
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    CPUTimer_setEmulationMode(CPUTIMER0_BASE, CPUTIMER_EMULATIONMODE_RUNFREE);
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
    Interrupt_register(INT_TIMER0, &xplt_cpu2_timer_isr);
    Interrupt_enable(INT_TIMER0);
    CPUTimer_startTimer(CPUTIMER0_BASE);
}
