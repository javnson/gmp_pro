# NUCLEO-F411RE GMP pin assignment

This target uses the STM32F411RET6 on the NUCLEO-F411RE (MB1136). PWM outputs are configured by CubeMX but remain disabled after reset until `xplt_pwm_enable()` is called explicitly.

| Resource | MCU pin(s) | Configuration |
| --- | --- | --- |
| TIM1 three-phase PWM | PA8/PA9/PA10 | CH1/CH2/CH3 |
| TIM1 complementary PWM | PB13/PB14/PB15 | CH1N/CH2N/CH3N |
| TIM1 ADC trigger | internal TIM1 OC4REF -> TIM2 ITR0 | TIM2 reset/update bridge drives ADC1 through `TIM2_TRGO`; 20 kHz center-aligned baseline |
| TIM3 QEP A/B | PA6/PA7 | encoder interface TI12 |
| QEP Z | PC10 | rising-edge EXTI; shared xplt resets TIM3 in software |
| ADC feedback 0..5 | PA0, PA1, PA4, PC0, PC1, PC4 | ADC1 IN0, IN1, IN4, IN10, IN11, IN14; regular scan DMA |
| GMP DL / ST-Link VCP | PA2/PA3 | USART2 TX/RX, 921600 baud, DMA |
| Status LED | PA5 | LD2, active high |
| I2C | PB8/PB9 | I2C1 SCL/SDA |
| SWD | PA13/PA14 | board ST-Link debug connection |

STM32F411RE provides neither an on-chip DAC nor CAN/FDCAN. PB0 is intentionally reserved for TIM1_CH2N, so Arduino A3 is replaced by the Morpho-exposed PC4/ADC1_IN14 feedback input.

STM32F411 ADC1 cannot select TIM1 TRGO/CC4 as a regular-group trigger. TIM1 CH4 is therefore configured as an internal no-output sampling compare. Its OC4REF becomes TIM1 TRGO, TIM2 receives it through ITR0 in reset slave mode, and each reset emits the TIM2 update TRGO accepted by ADC1. TIM2 consumes no package pin. External I2C devices require suitable SDA/SCL pull-up resistors.
