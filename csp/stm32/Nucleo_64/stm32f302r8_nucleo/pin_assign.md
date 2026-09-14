# NUCLEO-F302R8 GMP pin assignment

This target uses the STM32F302R8T6 on the NUCLEO-F302R8 (MB1136). Reset and early peripheral setup keep the power-stage outputs disabled. After the controller has installed safe half-period compare values, the common GMP startup enables TIM1 CH1/2/3, their complementary outputs, and the main-output gate (MOE).

| Resource | MCU pin(s) | Configuration |
| --- | --- | --- |
| TIM1 three-phase PWM | PA8/PA9/PA10 | CH1/CH2/CH3 |
| TIM1 complementary PWM | PA7/PB0/PB1 | CH1N/CH2N/CH3N |
| TIM1 ADC trigger | internal TRGO2 | CH4 OC reference, 20 kHz center-aligned baseline |
| TIM2 QEP A/B | PA15/PB3 | encoder interface TI12; TIM2 is used because STM32F302R8 has no TIM3/TIM4 |
| QEP Z | PC10 | rising-edge EXTI; shared xplt resets TIM2 in software |
| ADC feedback 0..5 | PC0, PC1, PC2, PC3, PA0, PA1 | ADC1 IN6/7/8/9/1/2; timer-triggered regular scan DMA |
| GMP DL / ST-Link VCP | PA2/PA3 | USART2 TX/RX, 921600 baud, circular RX DMA and TX DMA |
| Status LED | PB13 | LD2, active high |
| I2C | PB8/PB9 | I2C1 SCL/SDA; external pull-ups required |
| DAC debug output | PA4 | DAC OUT1 |
| CAN | PA11/PA12 | classic CAN RX/TX; external transceiver required |
| SWD | PA13/PA14 | board ST-Link debug connection |

The feedback inputs use Arduino A0/A1 plus four ST Morpho pins. PA4 is reserved for the optional DAC output. The PWM pins are exposed on the ST Morpho headers; complementary outputs deliberately avoid PB13 because PB13 drives the board user LED.

board=NUCLEO-F302R8
boardIOC=true
