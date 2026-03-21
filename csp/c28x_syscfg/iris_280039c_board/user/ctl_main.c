
//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should define your own controller objects,
// and initilize them.
//
// User should implement a ctl loop function, this
// function would be called every main loop.
//
// User should implement a state machine if you are using
// Controller Nanon framework.
//

#include <gmp_core.h>

#include <ctrl_settings.h>

#include "ctl_main.h"

#include <xplt.peripheral.h>

#include <core/pm/function_scheduler.h>

//=================================================================================================
// global controller variables



//=================================================================================================
// CTL initialize routine

void ctl_init()
{

}

//=================================================================================================
// CTL endless loop routine

void ctl_mainloop(void)
{
    return;
}

//=================================================================================================
// CiA402 default callback routine

