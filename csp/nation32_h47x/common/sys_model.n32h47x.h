#ifndef _FILE_NATION32_H47X_SYS_MODEL_H_
#define _FILE_NATION32_H47X_SYS_MODEL_H_

#ifdef __cplusplus
extern "C"
{
#endif

/** Return the CSP millisecond counter maintained by SysTick_Handler(). */
time_gt gmp_base_get_system_tick(void);

/** Advance the CSP millisecond counter once; call this from SysTick_Handler(). */
void gmp_step_system_tick(void);

#ifdef __cplusplus
}
#endif

#endif // _FILE_NATION32_H47X_SYS_MODEL_H_
