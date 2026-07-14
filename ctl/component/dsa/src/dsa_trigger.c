

#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// trigger memory module
#include <ctl/component/dsa/dsa_trigger.h>

void ctl_init_dsa_logger(ctl_dsa_logger_t* logger, ctrl_gt* mem_pool, uint32_t capacity, uint16_t channels,
                         uint32_t divider_step, ctl_dsa_trigger_option_t trigger_option, parameter_gt trigger_level,
                         parameter_gt trigger_suppression, parameter_gt isr_freq)
{
    logger->sm = DSA_LOGGER_READY;
    ctl_init_divider(&logger->divider, divider_step);
    ctl_init_mem_view(&logger->mem, mem_pool, capacity);
    ctl_init_dsa_trigger(&logger->trigger, trigger_option, trigger_level, trigger_suppression, isr_freq);

    logger->channels = (channels > 4) ? 4 : ((channels == 0) ? 1 : channels);

    // Calculate separate continuous segment size for each channel up front
    logger->channel_capacity = capacity / logger->channels;
    logger->current_position = 0;
}

void ctl_arm_dsa_logger(ctl_dsa_logger_t* logger)
{
    logger->current_position = 0;
    ctl_clear_divider(&logger->divider);
    ctl_clear_dsa_trigger(&logger->trigger);
    logger->sm = DSA_LOGGER_READY;
}

//void dsa_init_basic_trigger(basic_trigger_t *trigger, addr32_gt cell_size)
//{
//    trigger->target_index = 0;
//    trigger->cell_size = cell_size;
//    trigger->last_data = 0;
//    trigger->flag_triggered = 0;
//}

//
// void dsa_init_trigger_memory(
//    // log object
//    trigger_memory_log_t *log,
//    // monitor target 2 channel for all
//    ctrl_gt *target1, ctrl_gt *target2,
//    // memory block
//    ctrl_gt *memory_block1, ctrl_gt *memory_block2,
//    // type size
//    uint32_t type_size,
//    // memory size
//    uint32_t memory_size)
//{
//    log->memory_ch1 = memory_block1;
//    log->memory_ch2 = memory_block2;
//
//    log->monitor_target_ch1 = target1;
//    log->monitor_target_ch2 = target2;
//
//    log->log_length = memory_size;
//    log->cell_size = type_size;
//
//    log->flag_trigger = 0;
//    log->last_target1 = 0;
//    log->current_index = 0;
//}

//////////////////////////////////////////////////////////////////////////
// Sine analyzer module
// 

#include <ctl/component/dsa/sine_analyzer.h>

void ctl_init_sine_analyzer(
    // handle of sine analyzer
    sine_analyzer_t *sine,
    // zero cross detect threshold
    parameter_gt zcd_threshold,
    // grid maximum frequency, grid minimum frequency
    parameter_gt min_freq, parameter_gt max_freq,
    // rated frequency
    parameter_gt rated_freq,
    // controller frequency
    parameter_gt f_ctrl)
{
    sine->threshold = float2ctrl(zcd_threshold);
    sine->sample_min = (fast32_gt)(f_ctrl / max_freq) - 1;
    sine->sample_max = (fast32_gt)(f_ctrl / min_freq) + 1;
    sine->freq_sf = float2ctrl(f_ctrl / rated_freq);

    sine->wave_norm = 0;
    sine->curr_sign = 0;
    sine->sumup = 0;
    sine->sumup_sqr = 0;
    sine->inv_index = 1;
    sine->inv_index_sqrt = 1;
}




