/**
 * @file ctl_main.h
 * @brief Empty controller declarations for the Pico blink example.
 */

#ifndef GMP_PICO_BLINK_CTL_MAIN_H
#define GMP_PICO_BLINK_CTL_MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

void ctl_init(void);
void ctl_mainloop(void);
void ctl_dispatch(void);

#ifdef __cplusplus
}
#endif

#endif /* GMP_PICO_BLINK_CTL_MAIN_H */
