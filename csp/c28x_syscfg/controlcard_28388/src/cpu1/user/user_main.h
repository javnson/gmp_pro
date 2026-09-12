#ifndef GMP_F28388D_CPU1_USER_MAIN_H
#define GMP_F28388D_CPU1_USER_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/** Execute the platform-independent 1 kHz sampling work from the timer ISR. */
void user_cpu1_control_step(void);

#ifdef __cplusplus
}
#endif

#endif
