#ifndef _FILE_STATE_TRANSFER_H_
#define _FILE_STATE_TRANSFER_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @brief State transfer enumeration for FSM life-cycle management.
 */
typedef enum _tag_state_transfer
{
    CTL_ST_FIRST_ENTRY = 0, /*!< Triggered exactly once when entering a state. */
    CTL_ST_KEEP = 1,        /*!< Triggered continuously while waiting for the target count. */
    CTL_ST_LEAVE = 2        /*!< Triggered exactly once when the target count is reached. */
} ctl_state_transfer_e;

/**
 * @brief Data structure for the state timer/sequencer.
 */
typedef struct _tag_ctl_state_timer
{
    time_gt counter; /*!< Internal tick counter. 0 implies the state has not started. */
} ctl_state_timer_t;


/**
 * @brief Resets the state timer. MUST be called when transitioning to a NEW state.
 * @param[in,out] obj Pointer to the state timer.
 */
GMP_STATIC_INLINE void ctl_reset_state_timer(ctl_state_timer_t* obj)
{
    obj->counter = 0;
}

/**
 * @brief Steps the state timer and returns the current life-cycle phase.
 * @param[in,out] obj          Pointer to the state timer.
 * @param[in]     target_ticks The desired dwell time in ticks.
 * @return ctl_state_transfer_e The action the FSM should take.
 */
GMP_STATIC_INLINE ctl_state_transfer_e ctl_step_state_timer(ctl_state_timer_t* obj, uint32_t target_ticks)
{
    // Phase 1: First Entry
    if (obj->counter == 0)
    {
        obj->counter = 1;
        // If target is 0, we still want to execute FIRST_ENTRY at least once before leaving
        return CTL_ST_FIRST_ENTRY;
    }

    // Phase 3: Leave
    if (obj->counter >= target_ticks)
    {
        // Notice we do NOT auto-reset here. This ensures it keeps returning LEAVE
        // if the outer FSM is blocked, preventing accidental re-entries.
        return CTL_ST_LEAVE;
    }

    // Phase 2: Keep
    obj->counter++;
    return CTL_ST_KEEP;
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_STATE_TRANSFER_H_
