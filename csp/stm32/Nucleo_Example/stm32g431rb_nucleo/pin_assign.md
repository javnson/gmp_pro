Pin plan for STM32G431RB Nucleo Board and P-NUCLEO-IHM001 Board



Analog resource

| info      | Connector | STM32 Pin | ADC resource |
| --------- | --------- | --------- | ------------ |
| MOTOR_IU  | C7-28     | PA0       | ADC1         |
| MOTOR_IV  | C7-36     | PC1       | ADC1         |
| MOTOR_IW  | C7-38     | PC0       | ADC1         |
| MOTOR_VU  | C7-37     | PC3       | ADC2         |
| MOTOR_VV  | C7-34     | PB0       | ADC1         |
| MOTOR_VW  | C10-15    | PA7       | ADC2         |
| MOTOR_VDC | C7-30     | PA1       | ADC1         |
| MOTOR_TC  | C7-35     | PC2       | ADC2         |



PWM resource

| info     | Connector | STM32 Pin | TIM1 Resource |
| -------- | --------- | --------- | ------------- |
| EPWMU    | C10-23    | PA8       | TIM1 CH1      |
| EPWMU(L) | C7-1      | PC10      | GPIO          |
| EPWMV    | C10-21    | PA9       | TIM1 CH2      |
| EPWMV(L) | C7-2      | PC11      | GPIO          |
| EPWMW    | C10-33    | PA10      | TIM1 CH3      |
| EPWMW(L) | C7-3      | PC12      | GPIO          |



QEP resource

| info  | Connector | STM32 PIN | STM32 Resource            |
| ----- | --------- | --------- | ------------------------- |
| QEP-A | C7-17     | PA15      | TIM2 Encoder Mode         |
| QEP-B | C10-31    | PB3       | TIM2 Encoder Mode         |
| QEP-C | C10-25    | PB10      | GPIO input with interrupt |
| QEP-Z | C10-12    | PA12      | TIM1 (Conflict with PWM)  |





Other resource

| info         | Connector | STM32 Pin | STM32 Resource          |
| ------------ | --------- | --------- | ----------------------- |
| DAC          | C7-32     | PA4       |                         |
| PWMDAC       | C10-29    | PB5       | TIM17-PWM analog output |
| MTR-DRV LEDR | C10-22    | PB2       | GPIO output             |















