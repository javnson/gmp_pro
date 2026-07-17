# Chip Support Package

**English** | [简体中文](readme_cn.md)

describe chip features

Unified device function name

All the C source file should contain the `<core/gmp_core.h>` folder.
This file contains all the C interface of GMP library.
That will not destroy your C

``` C++
#include <core/gmp_core.h>
```

If your project is a C++ project, and has main.cpp, you may directly include the `<gmp_core.h>`

``` C++
#include <core/gmp_core.hpp>
```



You may support your own Chip Support Package, via example.

Each Chip support package should contains at least 4 header files. They are

| file              | summary                                                      |
| ----------------- | ------------------------------------------------------------ |
| `csp.config.h`    | This header file may config the GMP library to enable or disable some parts. And validate user configurations. |
| `csp.general.h`   | This header file may include the chip support related files or headers. |
| `csp.general.hpp` | This header file may include the chip support related files or headers for C++. |
| `csp.typedef.h`   | This header file provide chip related typedef.               |

另外，CSP中必须实现`gmp_hal_`的系列函数，用于支持core模块。这些函数的原型可以在`core/std/gmp_csp_cport`中找到。

另外，CSP中必须要实现每一个外设的的基本类型，这些类型以`_halt`结尾，意为硬件抽象层类型。并且提供硬件抽象层类型的初始化函数，这些初始化函数将会在user的平台相关代码中被执行。
