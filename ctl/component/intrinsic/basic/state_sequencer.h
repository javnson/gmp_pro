/**
 * @file state_sequencer.h
 * @defgroup state_sequencer Dual-Context State Sequencer
 * @brief Thread-safe state life-cycle manager for split ISR/Loop architectures.
 * @details Utilizes independent tracking mechanisms for high-frequency interrupts (ISR) 
 * and low-priority background tasks (Loop) to guarantee both contexts receive their 
 * respective FIRST_ENTRY triggers without race conditions.
 * @{
 */

#ifndef _FILE_CTL_STATE_SEQUENCER_H_
#define _FILE_CTL_STATE_SEQUENCER_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief State sequence enumeration for FSM life-cycle management.
 */
typedef enum _tag_state_seq
{
    CTL_ST_FIRST_ENTRY = 0, /*!< Triggered exactly once when entering a state. */
    CTL_ST_KEEP = 1,        /*!< Triggered continuously while waiting for the target count. */
    CTL_ST_LEAVE = 2        /*!< Triggered exactly once when the target count is reached. */
} ctl_state_seq_e;

/**
 * @brief Data structure for the dual-context state sequencer.
 */
typedef struct _tag_ctl_state_seq
{
    time_gt counter;            /*!< Internal tick counter, stepped by the ISR. */
    time_gt target_ticks;       /*!< The target dwell time in ticks for LEAVE evaluation. */
    fast_gt flag_is_first_loop; /*!< Independent flag for the background loop's first entry. */
} ctl_state_seq_t;

/**
 * @brief Clears and arms the state sequencer for a new state.
 * @details MUST be called exactly once before transitioning to the new state. 
 * It resets the ISR counter and arms the independent loop flag.
 * @param[in,out] obj          Pointer to the state sequencer instance.
 * @param[in]     target_ticks The total duration the state should dwell (in ISR ticks).
 */
GMP_STATIC_INLINE void ctl_clear_state_seq(ctl_state_seq_t* obj, time_gt target_ticks)
{
    obj->counter = 0;
    obj->target_ticks = target_ticks;
    obj->flag_is_first_loop = 1; // Arm the loop for its first entry
}

// ============================================================================
// ISR Context Execution
// ============================================================================

/**
 * @brief Steps the state sequencer (Intended for HIGH-FREQUENCY ISR).
 * @details Increments the internal counter and evaluates the lifecycle phase. 
 * Safely consumes the ISR-specific first entry condition without affecting the main loop.
 * @param[in,out] obj Pointer to the state sequencer instance.
 * @return ctl_state_seq_e Lifecycle phase for the ISR context.
 */
GMP_STATIC_INLINE ctl_state_seq_e ctl_step_state_seq(ctl_state_seq_t* obj)
{
    // Phase 1: First Entry (Consumed by counter == 0)
    if (obj->counter == 0)
    {
        obj->counter = 1;
        return CTL_ST_FIRST_ENTRY;
    }

    // Phase 3: Leave
    if (obj->counter >= obj->target_ticks)
    {
        return CTL_ST_LEAVE;
    }

    // Phase 2: Keep (Step the timer)
    obj->counter++;
    return CTL_ST_KEEP;
}

// ============================================================================
// Main Loop Context Execution
// ============================================================================

/**
 * @brief Evaluates the state sequencer (Intended for LOW-PRIORITY MAIN LOOP).
 * @details Does NOT increment the counter. Consumes the independent loop flag 
 * to safely guarantee a FIRST_ENTRY trigger in the background task.
 * @param[in,out] obj Pointer to the state sequencer instance.
 * @return ctl_state_seq_e Lifecycle phase for the Main Loop context.
 */
GMP_STATIC_INLINE ctl_state_seq_e ctl_loop_state_seq(ctl_state_seq_t* obj)
{
    // Phase 1: First Entry (Consumed by the independent flag)
    if (obj->flag_is_first_loop)
    {
        obj->flag_is_first_loop = 0;
        return CTL_ST_FIRST_ENTRY;
    }

    // Phase 3: Leave (Evaluated against the ISR-driven counter)
    if (obj->counter >= obj->target_ticks)
    {
        return CTL_ST_LEAVE;
    }

    // Phase 2: Keep
    return CTL_ST_KEEP;
}

/**
 * @brief Gets the progress of the current state.
 * @param[in] obj Pointer to the state sequencer instance.
 * @return parameter_gt Progress percentage ranging from 0.0f to 1.0f.
 */
GMP_STATIC_INLINE parameter_gt ctl_get_state_seq_progress(ctl_state_seq_t* obj)
{
    if (obj->target_ticks == 0)
        return 1.0f;
    if (obj->counter == 0)
        return 0.0f;
    if (obj->counter >= obj->target_ticks)
        return 1.0f;
    return (parameter_gt)obj->counter / (parameter_gt)obj->target_ticks;
}

/**
 * @brief Gets the current tick counter value of the state sequencer.
 * @param[in] obj Pointer to the state sequencer instance.
 * @return time_gt Current tick count since the state was entered.
 */
GMP_STATIC_INLINE time_gt ctl_get_state_seq_tick(ctl_state_seq_t* obj)
{
    return obj->counter;
}

#ifdef __cplusplus
}
#endif

#endif // _FILE_CTL_STATE_SEQUENCER_H_

/** @} */
