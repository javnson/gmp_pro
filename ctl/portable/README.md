# CTL Portable (No CSP)

Define `GMP_CTL_PORTABLE` when an application only needs CTL algorithms and
does not need the GMP runtime or CSP peripheral abstraction. Include
`ctl/math_block/gmp_math.h` as the numeric entry; it loads the portable type
contract and skips CSP, peripheral management, and `core/rt/gmp_runtime.h`.

Copy either `ti_dsp` or `stm32` into the application, add that directory and
the GMP root to the include path, and use its `sdpe_requirement.json` plus
`sdpe_generate.bat` to manage the compile-time macros. Define
`GMP_CTL_PORTABLE` globally (or force-include the generated SDPE header), and
edit `gmp_ctl_portable_config.h` when raw platform types need adjustment.

Ordinary CTL modules only require the control/parameter numeric selections,
raw ADC/DAC/PWM integer types, and an assertion mapping. Time-based modules
also require `GMP_CTL_PORTABLE_GET_TICK()`. The STM32 template maps it to
`HAL_GetTick()`; the TI template includes an editable CPUTIMER hook.

This mode does not provide the GMP device layer, debug tool, CTL Nano runtime,
dynamic memory management, or CSP peripheral implementations. Use a full
GMP+CSP project when those services are required.
