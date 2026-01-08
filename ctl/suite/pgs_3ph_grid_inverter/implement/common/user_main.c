// This is the example of user main.

//////////////////////////////////////////////////////////////////////////
// headers here

// GMP basic core header
#include <gmp_core.h>

// user main header
#include "user_main.h"

//////////////////////////////////////////////////////////////////////////
// global variables here

//////////////////////////////////////////////////////////////////////////
// initialize routine here
GMP_NO_OPT_PREFIX
void init(void) GMP_NO_OPT_SUFFIX
{
}

void send_monitor_data(void);


time_gt uart_last_tick;

//////////////////////////////////////////////////////////////////////////
// endless loop function here
void mainloop(void)
{
    send_monitor_data();

    time_gt current_sec = gmp_base_get_system_tick() / 1000;

    if(current_sec != uart_last_tick)
    {
        gmp_base_print(TEXT_STRING("Hello World!\r\n"));

        uart_last_tick = current_sec;
    }

}
