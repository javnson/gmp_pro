# F29H850TU9 LaunchPad GMP baseline

This CCS 21 project validates the byte-addressed C29x GMP CSP with two basic
operations:

- LED0 and LED1 alternate every 500 ms;
- UARTA on the LaunchPad XDS110 back channel runs at 115200-8-N-1 and serves
  the GMP u8 Data Link echo command.

## Import and run

1. Install F29H85x SDK 1.02.01.00 and CCS 21 or later.
2. Run `tools/facilities_generator/ccs_product_installer/install_ccs_products.bat`, then ask
   CCS to rediscover Products or restart CCS.
3. Import `f29h850tu9_gmp_blinky.projectspec` and build `LAUNCHXL_RAM` first.
4. Select the XDS110 application UART in
   `tools/gmp_pil_server/gmp_debugger/run_u8.bat`, set 115200 baud, and use
   the Raw tab to send an Echo request.

The UART is a binary Data Link channel while this example is running. Do not
open a text terminal on the same COM port at the same time.

Hardware execution and serial wiring must still be validated on a real
LAUNCHXL-F29H850TU board; a successful CCS build alone does not prove them.
