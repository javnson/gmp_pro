# Core of GMP

**English** | [简体中文](readme_cn.md)

This folder contain all core functions of GMP.



## Headers of GMP

GMP core provide two parts of headers. 
One is the c general header. If you need to complete a C programing file, please use the `<core/gmp_core.h>`. 
Another is the C++ style header. If you need to complete a C++ programing file, please use the `<core/gmp_core.hpp>`.

If your program framework is a C framework, you may include `<core/gmp_core.h>`, and call the `gmp_entry();`. 
This function is C style function and the C header will not disturb your project framework.

On the contrary, if your program framework is a C++ framework, you may include `<core/gmp_core.hpp>`, and call the `gmp_entry()` function. This function is `extern "C"`.

GMP library will take over the whole program. You may start your program in `<user/user_main.cpp>`.

## Components of GMP core

GMP core has the following 5 key parts, which are shown in the follow.

| Part Name | Folder  | Comments |
| --------- | ------  | -------  |
| Standardize | `std` | This folder provides all the cross platform support. Such as standard error code, standard compiler macros, and standard GMP typedef. |
| memory management | mm | Internal Memory Management module. |
| Process management | pm | This folder provide a set of tool kits for user to manage their processes. |
| device peripheral unify abstract | dev | This folder provides the standard device peripheral abstract. |
| FGPA support | fpga | This folder provides the GMP related FPGA support Verilog code and HLS code. |
| utilities tools | util | This folder contains a set of utilities. |



## device interface

All the device interface function is defined in `core/dev` folder. 

`devif.h` is the main device interface for the GMP library. All the communication method is defined based on the header. This file provide all the structure definition and init function.

the init function is aim at init all the members of the structure.

这个文件中定义了全部的通信接口，所有的通信过程将会基于这个文件中提供的结构体展开。这些结构体将通信分为半双工、全双工、特殊的通信协议（IIC或者CAN）等。

这个文件同时提供了初始化函数，这些函数都以`gmp_dev_init_`开头，同时这个文件提供了其他的辅助函数，以`gmp_dev_util_`开始。

| 功能前缀         | Note                                 |
| ---------------- | ------------------------------------ |
| `gmp_dev_init_`  | 外设对象初始化（分配内存、赋予初值） |
| `gmp_dev_setup_` | 外设对象赋予初始值                   |
| `gmp_dev_util_`  | 外设对象的辅助函数                   |



## Memory Management

All the memory management of GMP library is stored in `core/mm` folder.

其中提供了内存管理模块的初始化函数，并且可以利用初始化后的内存模块进行内存的申请和释放。

即提供了`gmp_mm_<ModuleName>_alloc`和`gmp_mm_<ModuleName>_free`用于进行内存管理。初始化函数以`gmp_mm_init_<ModuleName>`，初始值设定函数以`gmp_mm_setup_<ModuleName>`作为前缀，其他的辅助函数以`gmp_mm_util_`作为前缀。

| 函数前缀                        | Note                 |
| ------------------------------- | -------------------- |
| `gmp_mm_init_<ModuleName>`      | 初始化内存模块       |
| `gmp_mm_setup_<ModuleName>`     | 内存管理对象启动     |
| `gmp_mm_<ModuleName>_alloc`     | 内存分配函数         |
| `gmp_mm_<ModuleName>_free`      | 内存释放函数         |
| `gmp_mm_util_<ModuleName>_<op>` | 内存管理其他辅助函数 |



## Process Management

Process management is stored in `core/pm` folder.

进程管理模块提供了Workflow模块、状态机模块、消息管理模块、调度器模块。这一部分内容目前没有正常投入使用。

### FP (Function Player)



### WF (Workflow)







## Standardization

This module translate different platform, different chip into the same GMP macros and types.

整个GMP的Standardization模块可以使用`#include<gmp.std.h>`引入。

GMP对于标准化的设计分为如下几个部分：

+ Compiler Standardization

这些定义在`core/std/cfg/compiler.cfg.h` 实现， and `core/std/cc` provide the implementation of  compiler standard.

下面这些功能是编译器标准化实现的。

基本的C++语义宏的定义，用来支持C++03允许部分C++11的关键字：`constexpr`, `nullptr`, `override`, `final`.

| 补充定义的关键字 | 定义  | 说明           |
| ---------------- | ----- | -------------- |
| constexpr        | const | 常量定义       |
| nullptr          | NULL  | 用于声明空指针 |
| override         |       | 执行类相关操作 |
| final            |       | 执行类相关操作 |



基本的连接器选项：



| 需要编译器和链接器提供的功能               | Note                                                 |
| ------------------------------------------ | ---------------------------------------------------- |
| GMP_STATIC_INLINE                          | 指定函数必须实现为静态内联函数                       |
| GMP_WEAK_FUNC_PERFIX和GMP_WEAK_FUNC_SUFFIX | 作为弱函数的实现标志，需要加在弱函数的函数定义头和尾 |
| GMP_NO_OPT_PREFIX                          | 指定函数不能够被优化，通常应用在调试过程中           |
| GMP_MEM_ALIGN                              | 指定内存对齐（颗粒度）                               |
| GMP_INSTRUCTION_NOP                        | 执行一个空指令                                       |





+ Type Standardization

STD模块提供了标准的GMP类型定义。这些定义在`core/std/cfg/compiler.cfg.h`

每一个类型至少需要提供三项基本信息类型的名字、类型的字节长度、类型的比特长度。

``` C++
#define GMP_PORT_DATA_T				    char
#define GMP_PORT_DATA_SIZE_PER_BITS		(8)
#define GMP_PORT_DATA_SIZE_PER_BYTES    (1)
```

此外，对于一些特殊的类型可以指定类型的最大数（溢出上限）等信息（目前这些信息没有用到）。这些类型的名字通过`_gt`作为后缀。

下面列出了在GMP库中可以使用的标准类型

| 数据类型     | 最小的长度<br />(byte(s)) | 注释                                                         |
| ------------ | ------------------------- | ------------------------------------------------------------ |
| data_gt      | 1                         | STM32或者PC的data_gt的数据长度为8位，对于DSP等特殊设备其长度为32位。作为整个系统的最小单元。 |
| fast_gt      | 1                         | 使用这种类型应当提供尽可能快的响应，不同平台具有不同的响应，建议作为bool类型使用。 |
| fast16_gt    | 2                         | 响应尽可能快的类型，同时保证至少长度为2bytes                 |
| fast32_gt    | 4                         | 响应尽可能快的类型，同时保证长度至少为4bytes                 |
| time_gt      | 4                         | 应当保证至少为4bytes，用来计算时间长度，在进程管理和控制器计算控制间隔中用到。 |
| size_gt      | 2                         | 用来计数和衡量内存的偏移量，应当至少保证和内存的最大宽度相同。同时大于2bytes |
| addr_gt      | 2                         | 用来作为指针的数值类型，最小可能为2bytes                     |
| addr32_gt    | 4                         | 用来作为地址的数值类型，最小可能为4bytes，通常使用在外设和接口中 |
| addr16_gt    | 2                         | 用来作为地址的数值类型，固定为2byte，通常用在外设和接口类型中 |
| diff_gt      | 4                         | 用来作为正负都可能出现的偏移量类型                           |
| param_gt     | 4                         | 作为整数类型的参数存储，至少和addr_gt一样长，即能够保存一个地址指针 |
| adc_gt       | 4                         | 作为ADC的转换结果保存类型，通常为4bytes                      |
| dac_gt       | 4                         | 作为DAC的转换输入类型，通常为4bytes                          |
| pwm_gt       | 4                         | 作为PWM的比较器和计时器的整周期溢出值，通常情况下应当至少大于芯片支持的Timer外设的溢出长度 |
| ctrl_gt      | 4                         | 作为控制器的计算基础类型。有可能为定点数或者浮点数，所以应当保证使用`gmp_math`系列函数进行乘除运算，并且使用`CTRL_T(X)`（对于定点数场景，这一宏被定义为`_IQ`，对于浮点数类型，定义为到float类型的强制类型转换）来定义这一类型，以保证跨平台性能。 |
| parameter_gt | 4                         | 作为用于存储控制器参数的中间计算结果的类型，应当至少为浮点类型。这些参数不应当在整周期控制中断中被使用。否则可能由于芯片对于浮点数支持较差导致系统性能下降。 |

另外有一组以`_halt`为后缀的类型作为外设类型的接口。接口的实际定义在CSP中给出，`default_peripheral_config.h`中给出了这些类型的形式定义，这些定义在代码中实际是无效的。接口相关的函数原型在`gmp_csp_cport.h`中给出，这些函数的实现要求在CSP支持包中实现。

+ Peripheral interface Standardization

对于外设的基本类型符合`<peripheral>_halt`的命名规则。这些定义在`csp`中给出具体的实现，在`core/std/cfg/peripheral.cfg.h`中给出形式定义。

这一组函数在命名上满足以`gmp_hal_<peripheral_type>_<do>`作为典型的命名规则。

其中，下面这些函数的命名方式标志着这些函数是阻塞函数

| 函数名                                                       | Note                                   |
| ------------------------------------------------------------ | -------------------------------------- |
| `gmp_hal_<Peripheral>_recv`, `gmp_hal_<Peripheral>_send`     | 阻塞的外设收发函数                     |
| `gmp_hal_gpio_set`, `gmp_hal_gpio_clear`                     | 作为GPIO的置位和清零标志，只能是阻塞的 |
| `gmp_hal_gpio_read`, `gmp_hal_gpio_write`                    | GPIO的读写函数，只能是阻塞的           |
| `gmp_hal_gpio_set_mode`                                      | 改变GPIO的方向，只能是阻塞的           |
| `gmp_hal_wd_feed`, `gmp_hal_wd_disable`, `gmp_hal_wd_enable` | 看门狗操作函数，只能是阻塞的           |

下面这些函数是异步的

| 函数名                                                       | Note                                                         |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| `gmp_hal_<Peripheral>_asyc_recv`, `gmp_hal_<Peripheral>_async_send` | 异步的收发函数，通常这些函数用于发收一些半双工、双工的接口   |
| `gmp_hal_<Peripheral>_async_post`, `gmp_hal_<Peripheral>_async_request` | 异步收发函数，通常用来发收一些具备特定帧结构的消息。比如应用在IIC外设或者CAN外设等，相应的有一个消息队列和这些函数一起工作 |
| `gmp_hal_read`_<Peripheral>, `gmp_hal_write_<Peripheral>`    | 对于外设的读写函数是异步的                                   |

异步收发机制的配置函数是阻塞的（同步的）



+ Error code Standardization

对于错误代码的内容定义在头文件`core/std/cfg/errorcode.cfg.h`

在GMP项目中错误代码使用一个32位整数表示，使用类型`ec_gt`，error code general type来表示。

在GMP项目中错误的类型分为如下四种类型：



| Error Type | Prefix for Error Code | Note                                                         |
| ---------- | --------------------- | ------------------------------------------------------------ |
| OK         | `GMP_EC_OK`           | 返回值正确                                                   |
| INFO       | `GMP_EC_INFO_`        | 运行结果正常，只是需要作为信息提醒用户                       |
| WARN       | `GMP_EC_WARN_`        | 警告信息，需要程序员和用户关注是否正常运行                   |
| ERROR      | `GMP_EC_ERRO`         | 错误，一定需要用户或者程序员干预，以确保程序正确运行         |
| FATAL      | `GMP_EC_FATAL_`       | 严重错误，程序需要立刻终止的错误发生，控制器的结束程序将会被调用，同时主循环将会卡住，通过调用堆栈可以快速找到发生错误的位置。 |

对于四种类型的错误，可以借助工具配置各个错误的具体代码。

借助以下四个函数可以判定运行状态是否正常。

| function name           | Note                         |
| ----------------------- | ---------------------------- |
| `gmp_is_error(ec_gt)`   | 判定给是否出现错误           |
| `gmp_is_warning(ec_gt)` | 判定是否出现警告             |
| `gmp_is_fine(ec_gt)`    | 判定是否返回值为info或者成功 |



+ GMP Basic Functions

这些函数的原型在`core/std/gmp_cport.h`文件中给出。

GMP提供了一组基础函数供用户可以方便的实现一些通用的系统功能，如下：

| function name               | Note                                                         |
| --------------------------- | ------------------------------------------------------------ |
| `gmp_base_get_syste_tick()` | 获得系统当前的时钟计数，返回值类型为time_gt，每一个tick之间的长度通过宏给出（todo，将i这一组宏定义出来） |
| `gmp_base_system_stuck()`   | 当系统发生严重错误，需要停止时，这个函数应当被调用，这一函数应当组织各个功能有序停止、尽可能保留现场。 |
| `gmp_base_print()`          | 打印调试信息                                                 |
| `gmp_base_malloc()`         | 全局默认的分配内存函数                                       |
| `gmp_base_free()`           | 全局默认的释放内存函数                                       |
| `gmp_base_not_impl()`       | 当一个函数没有实现的时候，可以调用这个函数，这个函数将会决定是否程序仍然能够继续执行。 |
| `gmp_base_show_label()`     | GMP显示label函数                                             |
|                             |                                                              |

Special function

以下两个函数要求用户主动调用。

| function name         | Note                                                         |
| --------------------- | ------------------------------------------------------------ |
| `gmp_base_entry()`    | This function would be called when need to entry GMP.        |
| `gmp_base_ctl_step()` | GMP CTL periodic interrupt function. User should call this function in Main ISR. |





+ Big-endian & Little-endian

大小端转换函数在`gmp_cport.h`文件中给出定义。

LE(x),将x指定为小端数，BE(x)将x指定为大端数，l2b(x)将x进行大小端转换。大小端转换基于

| function / macro     | Note                    |
| -------------------- | ----------------------- |
| `LE_<data size>(x)`  | 将x强制转换为小端数据   |
| `BE_<data size>(x)`  | 将x强制转换为大端数据   |
| `L2B_<data size>(x)` | 将x进行一次大小端转换。 |

其中，`<data size>` 可以是12，16，32，64（如果系统支持64位数据的话）。



