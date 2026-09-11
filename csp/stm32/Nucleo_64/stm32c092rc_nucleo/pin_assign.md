# NUCLEO-C092RC GMP pin assignment

This target uses the STM32C092RCT6 on the NUCLEO-C092RC (MB2046). PWM outputs are configured by CubeMX but remain disabled after reset until `xplt_pwm_enable()` is called explicitly.

| Resource | MCU pin(s) | Configuration |
| --- | --- | --- |
| TIM1 three-phase PWM | PA8/PA9/PA10 | CH1/CH2/CH3 |
| TIM1 complementary PWM | PB13/PB14/PB15 | CH1N/CH2N/CH3N |
| TIM1 ADC trigger | internal TRGO2 | CH4 OC reference, 20 kHz center-aligned baseline |
| TIM3 QEP A/B | PA6/PA7 | encoder interface TI12 |
| QEP Z | PC10 | rising-edge EXTI; shared xplt resets TIM3 in software |
| ADC feedback 0..5 | PA0, PA1, PA4, PC4, PC5, PB0 | ADC1 IN0, IN1, IN4, IN11, IN12, IN17; fixed-sequence regular scan DMA |
| GMP DL / ST-Link VCP | PA2/PA3 | USART2 TX/RX, 921600 baud, DMA |
| Status LED | PA5 | LED1, active high |
| I2C | PB8/PB9 | I2C1 SCL/SDA |
| On-board CAN FD | PD0/PD1 | FDCAN1 RX/TX through the board transceiver |
| CAN transceiver standby | PD2 | active high; driven low by shared xplt |
| SWD | PA13/PA14 | board ST-Link debug connection |

DAC is not available on STM32C092. PC9/LED2 and the user/boot buttons remain outside the GMP base contract.
