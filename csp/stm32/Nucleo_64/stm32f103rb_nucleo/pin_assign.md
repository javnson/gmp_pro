# NUCLEO-F103RB GMP pin assignment

This target uses the STM32F103RBT6 on the NUCLEO-F103RB. Reset and early
peripheral setup keep the control outputs disabled. After `ctl_init()` installs
neutral half-period compares, the common GMP startup enables all three main and
complementary PWM pairs.

| Resource | MCU pin(s) | Configuration |
| --- | --- | --- |
| TIM1 three-phase PWM | PA8/PA9/PA10 | CH1/CH2/CH3 |
| TIM1 complementary PWM | PB13/PB14/PB15 | CH1N/CH2N/CH3N |
| TIM1 ADC trigger | internal TIM1 TRGO -> TIM3 ITR0 -> TIM3 TRGO | CH4 OC reference; one 20 kHz trigger per center-aligned PWM cycle |
| TIM4 QEP A/B | PB6/PB7 | encoder interface TI12 |
| QEP Z | PC10 | rising-edge EXTI; shared xplt resets TIM4 in software |
| ADC feedback 0..5 | PC0, PC1, PC2, PC3, PA0, PA1 | ADC1 IN10/11/12/13/0/1; timer-triggered regular scan DMA |
| GMP DL / ST-Link VCP | PA2/PA3 | USART2 TX/RX, 921600 baud, circular RX DMA and TX DMA |
| Status LED | PA5 | LD2, active high |
| I2C | PB8/PB9 | remapped I2C1 SCL/SDA; external pull-ups required |
| CAN | PA11/PA12 | classic CAN RX/TX at 500 kbit/s; external transceiver required |
| SWD | PA13/PA14 | board ST-Link debug connection; JTAG disabled so PB3 remains free |

STM32F103RB has no on-chip DAC. The six feedback channels and PWM/QEP signals
are exposed on the ST Morpho headers; the standard Arduino VCP and LED routes
remain intact.
