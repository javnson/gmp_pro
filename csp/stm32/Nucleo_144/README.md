# GMP STM32 Nucleo-144 platform

This directory defines the reusable GMP contract for STM32 Nucleo-144
boards. It retains the Nucleo-64 control baseline and makes the onboard
Ethernet MAC, RMII PHY connection, and a deterministic network acceptance
endpoint mandatory.

## Required board contract

- TIM1 or TIM8: three complementary PWM pairs, center-aligned, with an OC4
  trigger for ADC sampling. Physical PWM outputs remain disabled until the
  application explicitly enables them.
- TIM3 or TIM4: quadrature encoder A/B inputs and either a native or GPIO
  index input.
- At least six fixed ADC feedback inputs, sampled from the PWM timebase.
- ST-Link VCP UART with circular RX DMA and TX DMA for GMP Data Link.
- One user status LED, one I2C bus, and a 1 kHz system tick.
- Ethernet MAC plus board PHY and media interface. A locally administered MAC
  and a non-DHCP acceptance address must be specified by the board entity.
- DAC and CAN/FDCAN remain optional capabilities.

Each board has one IOC directory and one local SDPE project. All boards share
`src/user`, `src/xplt`, and `src/gmp_src_mgr`. Board differences live in
`ctl/hardware_preset/sdpe_src/mcu_board` and are emitted as stable aliases in
`ctrl_settings.h`.

## H753ZI reference target

`stm32h753zi_nucleo` is the first reference implementation:

- 400 MHz Cortex-M7; 20 kHz TIM1/ADC control cadence
- USART3 ST-Link VCP at 921600 baud
- LAN8742 RMII, MAC `02:47:4D:50:14:01`
- IPv4 `192.168.137.2/24`, gateway `192.168.137.1`
- UDP echo acceptance service on port `50000`

After the first checkout or an IOC change, run STM32CubeMX in command-line mode
with `generate_cubemx.txt`. Then build with `stm32h753zi_nucleo/build.ps1`,
flash with `flash.ps1`, and run `smoke_test.py` for the combined GMP Data Link
and Ethernet acceptance test. The build helper restores the ADC HAL files that
CubeMX omits for this board-template IOC, using the firmware package named by
the IOC. It searches the user's STM32Cube repository by default; set
`STM32_CUBE_REPOSITORY` to override the repository root. Before Ethernet
testing, verify the Nucleo board PHY straps required by the board manual (JP6
and SB72 fitted).

Run `start_sdpe.bat` to open the Nucleo-144 SDPE workspace.
