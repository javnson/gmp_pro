/**
 * @file xplt.config.h
 * @brief GMP application configuration for the Pico peripheral example.
 */

#ifndef GMP_PICO_DEMO_XPLT_CONFIG_H
#define GMP_PICO_DEMO_XPLT_CONFIG_H

#define SPECIFY_DISABLE_GMP_CTL
#define SPECIFY_DISABLE_GMP_LOGO

void setup_peripheral(void);
void init(void);
void mainloop(void);

#endif /* GMP_PICO_DEMO_XPLT_CONFIG_H */
