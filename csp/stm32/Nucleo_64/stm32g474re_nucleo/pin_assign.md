# NUCLEO-G474RE GMP control-resource pin assignment

Status: configured, not yet hardware validated.

The table records logical order. `FB0` through `FB5` must retain this order in the
IOC, SDPE board entity, and `ctl_input_callback()`.

| Function | Peripheral signal | MCU pin | Notes |
| --- | --- | --- | --- |
| PWM U high / low | TIM1_CH1 / TIM1_CH1N | PA8 / PB13 | Default PWM timer |
| PWM V high / low | TIM1_CH2 / TIM1_CH2N | PA9 / PB14 | Default PWM timer |
| PWM W high / low | TIM1_CH3 / TIM1_CH3N | PA10 / PB15 | Default PWM timer |
| Alternate PWM U high / low | TIM8_CH1 / TIM8_CH1N | PC6 / PC10 | SDPE-selectable |
| Alternate PWM V high / low | TIM8_CH2 / TIM8_CH2N | PC7 / PC11 | SDPE-selectable |
| Alternate PWM W high / low | TIM8_CH3 / TIM8_CH3N | PC8 / PC12 | SDPE-selectable |
| QEP3 A / B / Z | TIM3_CH1 / TIM3_CH2 / TIM3_ETR | PB4 / PB5 / PD2 | Default QEP timer |
| QEP4 A / B / Z | TIM4_CH1 / TIM4_CH2 / TIM4_ETR | PB6 / PB7 / PB3 | SDPE-selectable |
| ADC FB0 | ADC2_IN1, injected rank 1 | PA0 | Arduino A0 |
| ADC FB1 | ADC2_IN2, injected rank 2 | PA1 | Arduino A1 |
| ADC FB2 | ADC2_IN3, injected rank 3 | PA6 | Fixed external feedback |
| ADC FB3 | ADC2_IN4, injected rank 4 | PA7 | Fixed external feedback |
| ADC FB4 | ADC1_IN6, injected rank 1 | PC0 | Fixed external feedback |
| ADC FB5 | ADC1_IN7, injected rank 2 | PC1 | Fixed external feedback |
| ADC AUX0 / AUX1 | ADC1_IN8 / ADC1_IN9 | PC2 / PC3 | Additional injected inputs |
| GMP Data Link TX / RX | USART2_TX / USART2_RX | PA2 / PA3 | Board ST-Link VCP |
| Status LED | GPIO output | PA5 | Board LD2, active high |
| I2C SCL / SDA | I2C1_SCL / I2C1_SDA | PB8 / PB9 | External pull-ups required |
| DAC debug output | DAC1_OUT1 | PA4 | Optional capability |
| FDCAN RX / TX | FDCAN1_RX / FDCAN1_TX | PA11 / PA12 | External CAN transceiver required |

The current mapping does not claim a routed PWM break input. Connector names and
the ST-Link solder-bridge configuration must be checked against the physical board
revision during hardware validation.

