# Architecture type defaults

`core/std/arch` provides reusable processor data-model defaults. It describes
the properties shared by a processor architecture; peripheral handles and
device-specific exceptions remain in the CSP.

The assembly order is:

```text
xplt.config.h overrides
        -> csp.config.h / csp.typedef.h overrides
        -> selected core/std/arch defaults
        -> core/std/cfg/types.cfg.h fallbacks
```

`core/std/cfg/arch.cfg.h` detects Cortex-M, C28x, C29x, x86, x86-64, and 32-bit
RISC-V from compiler predefined macros. A user or CSP may explicitly define
`GMP_ARCH_TYPE` to one of `GMP_ARCH_ARM_M`, `GMP_ARCH_C28X`, `GMP_ARCH_C29X`,
`GMP_ARCH_X86`, `GMP_ARCH_X86_64`, or `GMP_ARCH_RISCV_32`.

Architecture headers define `GMP_PORT_*` macros only when they are still
undefined. Therefore a CSP only needs to publish genuine exceptions instead
of copying the complete architecture type table.

Do not use `int8_t` or `uint8_t` as a portable state type: C28x has a 16-bit
addressable C byte and cannot provide exact-width 8-bit integer types. Use:

- `data_gt` for a value stored in the processor's smallest addressable unit;
- `fast_gt` for flags, small signed states, and fast arithmetic;
- `time_gt` for system/control tick values and elapsed-tick counters;
- `size_gt` for collection and buffer sizes.

Exact-width standard types remain appropriate only when an external protocol
or hardware register contract genuinely requires that exact width and the CSP
provides the necessary packing/serialization boundary.

`core/std/tests/arch_compile_smoke.c` checks the selected target and the public
type-width contract with the real target compiler. Its neighboring
`xplt.config.h` intentionally disables the CSP so the test covers architecture
defaults rather than platform overrides.
