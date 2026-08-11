/**
 * Copy this file into the application only when a selected CTL component uses
 * time helpers.  Replace the body with a free-running millisecond tick source.
 */

#include <gmp_core.h>

time_gt gmp_ctl_portable_get_tick(void)
{
    /* TODO: return a millisecond tick derived from a CPUTIMER or application
     * scheduler.  Returning zero keeps non-time-based CTL modules linkable. */
    return (time_gt)0;
}
