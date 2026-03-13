# Motor Control Simulink 项目

这是一个用于电机控制的Simulink模型项目，可以在Windows和Linux平台上编译和运行。本文档提供了使用CMake构建和调试该项目的指南。

## 项目概述

该项目包含了电机控制系统的核心代码，支持基于Simulink的仿真环境，并且可以在不同平台上进行编译和调试。

## 依赖项

在构建项目之前，请确保已安装以下依赖项：

### 通用依赖
- CMake 3.10 或更高版本
- C/C++ 编译器
  - Windows: Visual Studio 2019或更高版本
  - Linux: GCC和G++

### 第三方库
项目依赖以下第三方库，这些库应该已经包含在项目的`third_party`目录中：
- ASIO
- Eigen
- FMT

## Windows平台构建指南

### 使用CMake生成Visual Studio项目

1. 打开命令提示符或PowerShell
2. 导航到项目目录
3. 创建并进入构建目录：
   ```
   mkdir build
   cd build
   ```
4. 运行CMake命令生成Visual Studio项目：
   ```
   cmake .. -G "Visual Studio 17 2022" -A x64
   ```
   注意：根据你的Visual Studio版本调整生成器选项。

5. 打开生成的解决方案文件（.sln）并在Visual Studio中构建项目

## Linux平台构建指南

### 编译步骤

1. 打开终端
2. 导航到项目目录
3. 创建并进入构建目录：
   ```bash
   mkdir build
   cd build
   ```
4. 运行CMake命令：
   ```bash
   cmake .. -DCMAKE_BUILD_TYPE=Release
   # 或者使用Debug模式用于调试
   # cmake .. -DCMAKE_BUILD_TYPE=Debug
   ```
5. 编译项目：
   ```bash
   make -j$(nproc)
   ```

### 在Linux上安装

如果需要安装到系统目录，可以运行：

```bash
# 以root权限安装
sudo make install
```

默认情况下，可执行文件将安装到`/usr/local/bin`目录。

## 使用GDB在Linux上调试

### 构建调试版本

1. 确保使用Debug模式构建项目：
   ```bash
   cd build
   cmake .. -DCMAKE_BUILD_TYPE=Debug
   make -j$(nproc)
   ```

### 运行GDB调试器

1. 在终端中运行：
   ```bash
   cd build
   gdb ./motor_control_simulink
   ```

2. 在GDB提示符下，可以使用以下命令：
   - 设置断点：`break 函数名`或`break 文件名:行号`
   - 运行程序：`run`
   - 继续执行：`continue`
   - 单步执行：`step`或`next`
   - 查看变量：`print 变量名`
   - 退出GDB：`quit`

## 跨平台注意事项

### 文件路径

- 项目中使用了自动路径转换，确保在Windows和Linux上都能正确处理文件路径
- 在修改CMakeLists.txt时，注意使用CMake的路径操作函数而非硬编码路径分隔符

### 平台特定代码

- 项目包含了针对Windows和Linux平台的条件编译
- 在Linux上，使用了`-DGMP_LINUX_PORT`预处理器宏来标识平台
- Windows特定的功能可能需要在Linux上有替代实现

### 库依赖

- Linux上需要链接额外的库如pthread、stdc++fs和dl
- Windows上使用Visual Studio的链接器设置

## 常见问题解决

### CMake错误

- 如果遇到路径相关的错误，请检查CMakeLists.txt中的`GMP_PRO_LOCATION`变量是否正确
- 确保所有第三方库都在正确的位置

### 编译错误

- 在Linux上编译Windows特定代码时可能会遇到错误，请确保代码有适当的条件编译保护
- 如果出现未定义引用错误，可能需要检查缺少的库依赖

### 运行时问题

- 确保网络配置文件（network.json）在正确的位置
- 在Linux上，检查防火墙设置是否允许所需的网络通信

## 支持的平台

- Windows (x86_64)
- Linux (x86_64)

## 许可证

[在此添加项目许可证信息]
