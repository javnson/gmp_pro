# Windows CSP Integration

Windows has no native CAN controller, so `windows_can_driver.cpp` implements the
same direct HAL symbols as hardware CSPs and treats every CAN handle as non-native.
The forwarding path is disabled by default.

Enable the asynchronous service and select exactly one hook prefix in the project
configuration:

```c
#define SPECIFY_ENABLE_GMP_CAN_SERVICE
#define GMP_CAN_ENABLE_HOOK
#define GMP_CAN_HOOK_PREFIX my_router
```

The application then provides `my_router_hal_can_get_capabilities()`,
`my_router_hal_can_submit_tx()` and the remaining functions declared by
`core/dev/can/can_hook.h`. This deliberate link-time contract avoids weak symbols,
runtime vtables and silent fallback. The hook can use the Python host service in
`host/gmp_router/service.py`; it owns multiple boards and addresses each request by
device name, endpoint and channel.

The default Windows GPIO/UART/I2C/SPI stubs are unchanged. A later native adapter
can be enabled around those functions independently without changing their legacy
behavior.
