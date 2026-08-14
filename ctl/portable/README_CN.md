# CTL Portable（No CSP）

`GMP_CTL_PORTABLE` 用于只采用 CTL 算法、无需 GMP 跨平台运行框架和 CSP
外设抽象的工程。启用后，`gmp_core.h` 不再加载 `xplt.config.h`、
`csp.config.h`、`csp.general.h`、外设管理及 `core/rt/gmp_runtime.h`。

## 接入步骤

1. 将 `ti_dsp` 或 `stm32` 文件夹内的文件复制到应用工程。
2. 将复制后的目录、GMP 根目录加入头文件搜索路径。
3. 使用文件夹中的 `sdpe_requirement.json` 管理 `GMP_CTL_PORTABLE` 与数值后端，
   运行 `sdpe_generate.bat` 生成配置头；同时在编译器全局预定义
   `GMP_CTL_PORTABLE`（或将生成头设为编译器 forced include）。
4. 按需修改 `gmp_ctl_portable_config.h` 中的平台原始类型，然后包含 `gmp_core.h` 和所需 CTL
   模块头文件。
5. 只有使用延时、校准等时间相关模块时才需要实现 tick：STM32 模板默认
   调用 `HAL_GetTick()`；TI DSP 可把 `gmp_ctl_portable_hooks.c` 加入工程并将
   函数体接到 CPUTIMER 或应用调度节拍。

## 最小契约

普通 CTL 数学、控制器、调制器模块只依赖：

- 由 SDPE 管理的 `GMP_CTL_PORTABLE`、`SPECIFY_CTRL_GT_TYPE` 和
  `SPECIFY_PARAMETER_GT_TYPE`；
- ADC、DAC、PWM 的原始整数类型（需要相应接口模块时）；
- 断言宏 `GMP_CTL_PORTABLE_ASSERT(expr)`（可选，默认使用 C `assert`）；
- `GMP_CTL_PORTABLE_GET_TICK()`（仅时间相关模块需要）。

portable 模式仍使用 GMP 仓库中的 CTL 源码和头文件，但不提供 GMP 设备层、
debug tool、CTL Nano 调度框架、动态内存管理或 CSP 外设实现。需要这些能力时
应使用完整 GMP+CSP 工程。

## 最小示例

```c
#include <gmp_core.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>

static ctl_pid_t current_pid;

void controller_init(void)
{
    ctl_init_pid(&current_pid, float2ctrl(1.0f), float2ctrl(0.1f),
                 float2ctrl(0.0f), float2ctrl(1.0f));
}
```

具体初始化函数请以所选择模块的头文件为准。portable 只改变基础依赖，不改变
CTL API。
