# NUCLEO-H753ZI FreeRTOS validation record

Validation date: 2026-09-12

## Configuration

- Board: NUCLEO-H753ZI, STM32H753ZITx revision V
- Probe: onboard ST-LINK V3J5M2
- Toolchain: GNU Arm Embedded 14.3.1, CMake + Ninja
- STM32Cube package: STM32Cube FW_H7 V1.12.1
- FreeRTOS tick: 1 kHz; heap: `heap_4`, 32 KiB
- GMP service task: priority 5 of 0..6, 1024 stack words, 1 ms period
- User task: priority 1, 256 stack words, 1 s period
- Control path: TIM1-triggered ADC1 circular DMA, nominal 20 kHz

## Build evidence

Release cross-build and link passed. Result:

| Region | Used | Capacity |
|---|---:|---:|
| FLASH | 53,256 B | 2 MiB |
| RAM_D1 | 43,496 B | 512 KiB |
| RAM_D2 | 262,336 B | 288 KiB |
| DTCMRAM | 6 KiB | 128 KiB |

The high displayed RAM_D2 percentage includes the linker gap to fixed Ethernet
DMA section addresses inherited from the Nucleo-144 memory contract; LwIP is
not linked in this RTOS configuration.

The final ELF exports FreeRTOS-owned `SVC_Handler`, `PendSV_Handler`, and
`SysTick_Handler`, plus `gmp_rtos_runtime_task`, `gmp_rtos_app_start`, the
strong board `gmp_csp_post_process`, and the ADC completion callback.

## Hardware evidence

The image was downloaded, verified, and reset through STM32CubeProgrammer
2.19.0. Two hot-plug SWD samples separated by approximately two seconds gave:

| Observable | Sample A | Sample B | Interpretation |
|---|---:|---:|---|
| `gmp_rtos_runtime_iterations` | `0x64DA` | `0x6DDA` | GMP service task continues to run |
| `gmp_rtos_user_task_heartbeats` | `0x1A` | `0x1D` | independent RTOS user task runs |
| `gmp_nucleo_platform_diag[0]` | `0x7EBB9` | `0x89FA6` | control ADC DMA ISR continues to run |
| `gmp_nucleo_platform_diag[6]` | `0x34` | `0x38` | `core/pm` heartbeat runs inside GMP task |

This validates concurrent operation of the RTOS scheduler, GMP cooperative
loop, and fast control interrupt on the connected board. It does not yet claim
protocol-level UART Data Link or RTOS LwIP validation.
