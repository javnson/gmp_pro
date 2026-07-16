/**
 * @file user_main.h
 * @brief Application lifecycle declarations for the Pico blink example.
 */

#ifndef GMP_PICO_BLINK_USER_MAIN_H
#define GMP_PICO_BLINK_USER_MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

void init(void);
void mainloop(void);
void setup_peripheral(void);

#ifdef __cplusplus
}
#endif

#endif /* GMP_PICO_BLINK_USER_MAIN_H */
