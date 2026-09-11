# NUCLEO-H533RE GMP control-resource pin assignment

Status: generated, compiled, programmed, and GMP DL/control-runtime validated on
hardware. This software validation must not be interpreted as an electrical
PWM, calibrated ADC, external QEP, I2C or FDCAN test.

The table records logical order. `FB0` through `FB5` retain this order in the
IOC, SDPE board entity, and `ctl_input_callback()`.

| Function | Peripheral signal | MCU pin | Connector note |
| --- | --- | --- | --- |
| PWM U high / low | TIM1_CH1 / TIM1_CH1N | PA8 / PB13 | Arduino/Morpho-accessible |
| PWM V high / low | TIM1_CH2 / TIM1_CH2N | PA9 / PB14 | Arduino/Morpho-accessible |
| PWM W high / low | TIM1_CH3 / TIM1_CH3N | PA10 / PB15 | Arduino/Morpho-accessible |
| QEP A / B / Z | TIM3_CH1 / TIM3_CH2 / TIM3_ETR | PB4 / PB5 / PD2 | ST Morpho; native timer index |
| ADC FB0 | ADC2_INP0, injected rank 1 | PA0 | Arduino A0 / ST Morpho |
| ADC FB1 | ADC2_INP1, injected rank 2 | PA1 | Arduino A1 / ST Morpho |
| ADC FB2 | ADC2_INP3, injected rank 3 | PA6 | Arduino digital / ST Morpho |
| ADC FB3 | ADC2_INP7, injected rank 4 | PA7 | Arduino digital / ST Morpho |
| ADC FB4 | ADC1_INP10, injected rank 1 | PC0 | Arduino A5 option / ST Morpho |
| ADC FB5 | ADC1_INP11, injected rank 2 | PC1 | Arduino A4 option / ST Morpho |
| GMP Data Link TX / RX | USART2_TX / USART2_RX | PA2 / PA3 | Board STLINK-V3EC VCP |
| Status LED | GPIO output | PA5 | Board LD2, active high |
| I2C SCL / SDA | I2C1_SCL / I2C1_SDA | PB6 / PB7 | Arduino A5/A4 options; external pull-ups required |
| FDCAN RX / TX | FDCAN1_RX / FDCAN1_TX | PA11 / PA12 | ST Morpho; external transceiver required |

The H533RE LQFP64 package cannot keep two complete, non-conflicting TIM1/TIM8
three-pair maps in this IOC, so the canonical SDPE option set exposes TIM1 only.
TIM3 is likewise the canonical QEP source because it provides a native ETR index
on PD2.

The internal DAC exists, but DAC1_OUT1 on PA4 is not routed to an expansion
connector on the MB1814 H533RE board, while PA5 is occupied by LD2. The board
therefore declares `GMP_NUCLEO_HAS_DAC=0`.

Connector routing follows the ST UM3121 Rev 6 manual and MB1814-H533RE-C02
schematic. Some Arduino alternatives depend on board solder-bridge population;
the fixed MCU pins above, rather than Arduino aliases, are the control contract.
