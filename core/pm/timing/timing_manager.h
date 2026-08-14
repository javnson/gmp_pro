/**
 * @file timing_manager.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_TIMING_MANAGER_H_
#define _FILE_TIMING_MANAGER_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

typedef time_gt (*timer_get_tick_fn)(void);

// This struct record one time
//
typedef struct _tag_time_tick
{
    // record the tick of the clock
    time_gt tick;

    // cycle: record the cycle of the clock
    time_gt cycle;
} time_tick_t;

// This struct is bind to a timer entity, to provide a timing service
//
typedef struct _tag_timer_mgr
{
    // to register the period of the timer
    time_gt period;

    // function get time tick
    timer_get_tick_fn get_tick;

    // last timer entity is called
    time_tick_t last_update_time;

    // a flag to decided if cycle_cnt is updated by user.
    // 1: cycle_cnt will be updated by timing_manager
    //    User must ensure that the `update_timer_mgr` function is called frequently enough.
    //    All the timer related function may call the `update_timer_mgr` function.
    //    In another word, the timer period should be long enough.
    // 0: cycle_cnt should be updated by user,
    //    User should call `step_cycle_cnt` in timer overflow interrupt function.
    fast_gt is_auto_update_cycle_cnt;

} timer_mgr_t;

static inline void step_cycle_cnt(timer_mgr_t timer_mgr)
{
    timer_mgr->cycle_cnt += 1;
}

void update_timer_mgr(timer_mgr_t *timer_mgr)
{
    time_gt current_tick = 0;

    if (!timer_mgr)
    {
        return;
    }

    // get current tick
    if (timer_mgr->get_tick)
        current_tick = timer_mgr->get_tick;

    // judge if timer period overflow is happened
    if (timer_mgr->is_auto_update_cycle_cnt)
    {
        // overflow is happened
        if (timer_mgr->last_update_time.tick > current_tick)
            timer_mgr->last_update_time.cycle += 1;
    }

    // set current tick
    timer_mgr->last_update_time.tick = current_tick;
}

void reg_timer_mgr(timer_mgr_t *timer_mgr, time_gt timer_period, timer_get_tick_fn get_tick_fn, fast_gt is_auto_update)
{
    if (!timer_mgr)
        return;

    timer_mgr->period = timer_period;
    timer_mgr->get_tick = get_tick_fn;
    timer_mgr->is_auto_update_cycle_cnt = is_auto_update;

    timer_mgr->last_update_time.cycle = 0;
    timer_mgr->last_update_time.tick = 0;

    update_timer_mgr(timer_mgr);
}

void get_current_time_tick(timer_mgr_t *mgr, time_tick_t *tick)
{
    update_timer_mgr(mgr);

    if (tick && mgr)
    {
        tick->cycle = mgr->last_update_time.cycle;
        tick->tick = mgr->last_update_time.tick;
    }
}

time_gt get_delta_time(timer_mgr_t *mgr, time_tick_t *moment)
{
    time_gt delta = 0;

    update_timer_mgr(mgr);

    if (moment)
    {
        delta = (mgr->last_update_time.cycle - moment->cycle) * mgr->period;
        delta += mgr->last_update_time.tick;
        delta -= moment->tick;
    }

    return delta;
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_TIMING_MANAGER_H_
