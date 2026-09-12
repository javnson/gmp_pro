# NUCLEO-H753ZI GMP pin assignment

| Function | Peripheral | Pins |
| --- | --- | --- |
| PWM U/U_N | TIM1 CH1/CH1N | PE9 / PE8 |
| PWM V/V_N | TIM1 CH2/CH2N | PE11 / PE10 |
| PWM W/W_N | TIM1 CH3/CH3N | PE13 / PE12 |
| Encoder A/B/Z | TIM3 CH1/CH2 + EXTI2 | PA6 / PB5 / PG2 |
| ADC FB0..FB5 | ADC1 ranks 1..6 | PA3 / PC0 / PB1 / PA4 / PA5 / PA0 |
| GMP DL VCP TX/RX | USART3 | PD8 / PD9 |
| Status LEDs LD1 / LD2 / LD3 | GPIO | PB0 / PB7 / PB14 |
| SWDIO / SWCLK | DEBUG | PA13 / PA14 |
| Arduino I2C SCL/SDA | I2C1 | PB8 / PB9 |
| RMII REF_CLK | ETH | PA1 |
| RMII MDIO/MDC | ETH | PA2 / PC1 |
| RMII CRS_DV | ETH | PA7 |
| RMII RXD0/RXD1 | ETH | PC4 / PC5 |
| RMII TX_EN | ETH | PG11 |
| RMII TXD0/TXD1 | ETH | PG13 / PB13 |

DAC is disabled because PA4/PA5 are part of the fixed feedback bank. FDCAN is
disabled in the initial target; PD0/PD1 remain the preferred future RX/TX pair.
PWM pin routing is initialized, but CC1..CC3, complementary enables, and MOE
remain clear during the acceptance firmware.
