# General Motor Platform (GMP) Controller Template Library (CTL) module

 Welcome to GMP CTL! 

## Generally

GMP CTL provide a set of tools for creating and accessing controller quickly. Because of GMP is a cross-platform library. So the CTL module is easy for user to create a controller for PC simulation, Process In Loop (PIL), Hardware In Loop (HIL), Real-Time Controller (RTC) design.

The basic structure of the CTL is shown as follow.

| Submodule            | Folder Path | Brief Comment                                                |
| -------------------- | ----------- | ------------------------------------------------------------ |
| CTL component        | `component` | This folder (submodule) provide a variety of controller components. All of them are stored in several sub-folders. Each sub-folder has the same source file (for initialization function or some basic support functions), which stored in `component/src`, and named by the sub-modules' name.<br />If user will invoke some of controller components, just include these headers, and then add corresponding source file to your project compiler source list. |
| CTL framework        | `framework` | This folder contains the predefined Controller Framework. User may use these framework to create his standard real-time controller prototype with high efficiency and easily.<br />Each framework has provide a set of function witch should be called on time by user, and another set of function prototype witch should be implemented by user. By doing this, user may insert the framework to his projects. |
| CTL controller suite | `suite`     | This folder contains some predefined Controller Suite, and some examples for these suite.<br />User may create his real-time controller application quickly by controller suite. |
| FPGA support         | `fpga`      | This folder provide some controller utilities which was implemented by HLS or Verilog code. User may use these code to implement a FPGA real-time controller. |
| controlled plants    | objects     | This folder contains a list of control objects. User may implement a simulation via this submodule. |



##  Type of Controller

In GMP CTL submodule, there're two types which would be use all around, that is, `ctrl_gt`, `parameter_gt`.

`ctrl_gt` means control law calculating type, for micro processor this type is generally a fixed point number. For some high performance micro processer this type is generally a float point number. For some simulation environments this type is generally a double float point number. The type which is link to `ctrl_gt` should provide enough calculating rates, and enough accuracy. Generally for DSP or other micro-controller we suggest assign int32_t@Q24 as `ctrl_gt` type.

If you need to create a `ctrl_gt` type variable, you should use `float2ctrl(x)` macro to initialize the number. And x should be an float number. Here's an instance

``` C
ctrl_gt a_fixed_point_number = float2ctrl(1.0f);
```

In addition, `ctrl_gt` has plus (`+`) and minus (`-`) operation with other types, but user may do not use the product (`*`) and divisions (`/`) directly. User should use `gmp_mpy` function and `gmp_div` function instead. These definitions would be found in `ctl/component/common/gmp_math.h` file.

`parameter_gt` means parameter containing type. for micro processor this type is at least a float number. For some high performance micro processor this type may be double. This type may use to save the source parameters of the controller, and the controller initialization function may use these variables to initialize the controller parameters. All the variables with `paramete_gt` type may support all the math operation which float number owned.



## Naming Standard for CTL function

all the functions in CTL model is begin with `ctl_`，all the components has at least three function:

| basic function           | note                   |
| ------------------------ | ---------------------- |
| `ctl_init_<ModuleName>`  | init and assign memory |
| `ctl_setup_<ModuleName>` | setup the component    |
| `ctl_step_<ModuleName>`  | Step the Component     |

And some complex components has the following functions

| extension function         | note                                                         |
| -------------------------- | ------------------------------------------------------------ |
| `ctl_input_<ModuleName>`   | input or unit conversion, copy these input data to components. |
| `ctl_output_<ModuleName>`  | get output of the components.                                |
| `ctl_prestep_<ModuleName>` | 一些模块具备提前运行的内容，这个函数需要在step之前调用       |
| `ctl_bind_<ModuleName>`    | 在初始化时将器件和其他的器件绑定                             |
| `ctl_set_<ModuleName>`     | 设置模块参数                                                 |
| `ctl_get_<ModuleName>`     | 获取模块的参数                                               |

对于控制器框架函数应当用`ctl_fm_`作为开头，其中需要用户定义的函数以`ctl_fmif_`作为开头，意为框架接口。

CTL中使用的数学常数，以`CTL_CONST_`打头，分数的分数线用`OVER_`表示，有特殊含义的符号用特殊符号标记。

## Suite

在CTL模块中提供了数字电源控制器和电机控制器两个基本控制器的类型，并且支持新能源光伏控制器。


这里的难点，也是下一步需要测试的内容：让GMP库能够兼容定点数、浮点数、FPGA-HLS的特殊类型，并且为将来生成代码做好准备。


