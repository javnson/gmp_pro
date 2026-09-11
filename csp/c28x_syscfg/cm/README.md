# C2000 Cortex-M SysConfig CSP

This optional CSP supports the Cortex-M (CM) execution domain integrated in
multicore C2000 devices such as TMS320F28388D. It intentionally selects the
ARM execution/address model while inheriting the family data-unit definition
directly from `csp/c28x_syscfg/csp.typedef.h`. The local `csp.typedef.h` is a
forwarding shim only and contains no CM-specific settings. GMP Data Link uses
the same u16 backend on CPU1, CPU2, and CM. Ethernet adapters explicitly pack
and unpack network octets.

Select `csp|c28x_syscfg_cm` in the GMP source-manager dictionary for a CM-only
project. The target supplies three board hooks declared by `csp.general.h`;
the generic adapter owns the GMP tick and runtime lifecycle.
