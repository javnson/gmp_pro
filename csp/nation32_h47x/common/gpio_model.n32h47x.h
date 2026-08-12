#ifndef _FILE_NATION32_H47X_GPIO_MODEL_H_
#define _FILE_NATION32_H47X_GPIO_MODEL_H_

#ifdef __cplusplus
extern "C"
{
#endif

/** Initialize a GMP GPIO handle from an N32 port and pin mask. */
#define GMP_N32_GPIO_HANDLE(name, gpio_port, gpio_pin) \
    gmp_gpio_n32h47x_t name = {(gpio_port), (gpio_pin)}

#ifdef __cplusplus
}
#endif

#endif // _FILE_NATION32_H47X_GPIO_MODEL_H_
