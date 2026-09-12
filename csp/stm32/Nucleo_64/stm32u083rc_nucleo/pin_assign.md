# NUCLEO-U083RC GMP pin assignment

This target uses the STM32U083RCT6 on the NUCLEO-U083RC (MB1932). PWM outputs are configured by CubeMX but remain disabled after reset until `xplt_pwm_enable()` is called explicitly.

| Resource | MCU pin(s) | Configuration |
| --- | --- | --- |
| TIM1 three-phase PWM | PA8/PA9/PA10 | CH1/CH2/CH3 |
| TIM1 complementary PWM | PB13/PB14/PB15 | CH1N/CH2N/CH3N |
| TIM1 ADC trigger | internal TRGO2 | CH4 OC reference, 20 kHz center-aligned baseline |
| TIM3 QEP A/B | PA6/PA7 | encoder interface TI12 |
| QEP Z | PC10 | rising-edge EXTI; shared xplt resets TIM3 in software |
| ADC feedback 0..5 | PC0, PC1, PC2, PC3, PA0, PA1 | ADC1 IN0 through IN5; timer-triggered regular scan DMA |
| GMP DL / ST-Link VCP | PA2/PA3 | USART2 TX/RX, 921600 baud, circular RX DMA and TX DMA |
| Status LED | PA5 | LD4, active high |
| I2C | PB8/PB9 | I2C1 SCL/SDA on Arduino D15/D14; external pull-ups required |
| DAC debug output | PA4 | DAC1 OUT1 on Arduino A2 |
| SWD | PA13/PA14 | board ST-Link debug connection |

The feedback set keeps Arduino A0/A1 and A4/A5 while adding PC2/PC3 on the ST Morpho connector. Arduino A2/PA4 is intentionally reserved for DAC1. STM32U083 has no CAN/FDCAN peripheral; `GMP_NUCLEO_HAS_CAN` is therefore zero.
