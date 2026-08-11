# MCS PMSM NT F29H85x + BOOSTXL-3PHGANINV target

See [README_CN.md](README_CN.md). This CCS 21 target uses the `GMP-Core-C29x`
Product and TI's official LAUNCHXL-F29H85X/BOOSTXL-3PHGANINV pin mapping. It
implements synchronized ADC sampling, the shared GMP PMSM control step, three
ePWM compare writes, QEP feedback, safe one-shot trips, and byte-addressed UART
Data Link. All project macros are managed by the two-layer SDPE configuration:
run the shared `sdpe_general` generator, then `sdpe_mgr/sdpe_generate.bat` and
`sdpe_mgr/sdpe_validate.bat` before building.
