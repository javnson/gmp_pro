# NUCLEO-G431KB GMP control-resource pin assignment

Status: hardware baseline; see `validation.md` for the exact validation scope.

The logical order of `FB0` through `FB5` is an ABI shared by the IOC, SDPE board
entity, generated target settings, and `ctl_input_callback()`.

| Function | Peripheral signal | MCU pin | Nucleo-32 connection |
| --- | --- | --- | --- |
| PWM U high / low | TIM1_CH1 / TIM1_CH1N | PA8 / PA11 | D7 / D9 |
| PWM V high / low | TIM1_CH2 / TIM1_CH2N | PA9 / PA12 | D8 / D10 |
| PWM W high / low | TIM1_CH3 / TIM1_CH3N | PA10 / PF0 | D2 / A6 |
| QEP A / B / Z | TIM3_CH1 / TIM3_CH2 / TIM3_ETR | PB4 / PB5 / PB3 | D12 / D11 / D13 |
| ADC FB0 | ADC1_IN1, injected rank 1 | PA0 | A0 |
| ADC FB1 | ADC1_IN2, injected rank 2 | PA1 | A1 |
| ADC FB2 | ADC2_IN17, injected rank 1 | PA4 | A2 |
| ADC FB3 | ADC2_IN13, injected rank 2 | PA5 | A3 |
| ADC FB4 | ADC2_IN3, injected rank 3 | PA6 | A4 |
| ADC FB5 | ADC2_IN4, injected rank 4 | PA7 | A5 |
| GMP Data Link TX / RX | USART2_TX / USART2_RX | PA2 / PA3 | D1 / D0 and ST-Link VCP |
| Status LED | GPIO output | PB8 | LD2, active high |
| I2C SCL / SDA | I2C1_SCL / I2C1_SDA | PA15 / PB7 | D6 / D4; external pull-ups required |
| SWD debug | SWDIO / SWCLK | PA13 / PA14 | On-board ST-Link V3E |

The control IOC uses HSI16 and its PLL instead of HSE. This releases PF0 for
`TIM1_CH3N`; the MB1430 A-02 schematic marks the external-clock/crystal solder
bridges on PF0/PF1 as not fitted. A board whose bridges have been modified must
disconnect the HSE source before using the W-low output.

This compact topology deliberately does not claim DAC, FDCAN, or PWM Break:
the externally useful pins for those functions overlap the six ADC inputs or
the complete three-phase PWM set. Separate application-specific IOC variants
may trade capabilities, but must not change this baseline silently.
