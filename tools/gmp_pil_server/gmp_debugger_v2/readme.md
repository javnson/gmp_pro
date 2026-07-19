# GMP Debugger v2

**English** | [简体中文](readme_cn.md)

GMP Debugger v2 is the Python/PyQt host application for Datalink and
processor-in-the-loop workflows. It supports target communication, variable
inspection, parameter updates, memory views, and project-specific debug tools.

Run the guarded launcher in this directory after installing GMP. The launcher
automatically selects the repository-private Python environment when its
completion marker exists; Python packages are maintained centrally in
`tools/gmp_installer/requirements-gmp.txt`.

Target firmware must configure a compatible transport and schedule communication
outside high-frequency control ISRs. Use the owning suite documentation for
packet, UART, and Datalink task configuration.
