# Optional Device Print Service

`src/gmp_dev_print.c` 是依赖 CSP UART 的可选默认打印适配，Facility 模块 ID
为 `core|dev|print`。它不属于 `dev|_internal`，不需要打印的控制代码不会被
迫使引入该源文件。
