# GMP Debugger u16

**English** | [简体中文](readme_cn.md)

GMP Debugger u16 is the legacy Python/PyQt host application for Datalink and
processor-in-the-loop workflows on DSP targets where
`GMP_PORT_DATA_SIZE_PER_BYTES == 2`, primarily TI C28x devices. It supports
target communication, variable inspection, parameter updates, memory views,
and project-specific debug tools.

The maintained UI is English-only. Memory Perspective and Tunable pages can
import named descriptors reported by the target. The Data Link Scope page uses
the independent single-command Scope service rather than Memory Perspective.

Run the guarded launcher in this directory after installing GMP. The launcher
automatically selects the repository-private Python environment when its
completion marker exists; Python packages are maintained centrally in
`tools/gmp_installer/requirements-gmp.txt`.

Target firmware must configure a compatible transport and schedule communication
outside high-frequency control ISRs. Use the owning suite documentation for
packet, UART, and Datalink task configuration.

Memory Perspective uses byte addresses on the wire. Convert a C28x native word
address from a linker map to a byte address by multiplying it by two before
entering it in the debugger.
