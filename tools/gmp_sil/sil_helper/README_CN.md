# GMP SIL Helper

[English](README.md) | **简体中文**

本目录是 GMP SIL/PIL 通信的唯一维护位置，统一提供 TCP、UDP、应用层帧协议、控制器侧
`gmp_sil_helper` 对象、Simulink S-function、MATLAB MEX 构建入口和回归测试。`slib`
只保存安装时编译出的 `GMP_SIL_Core` MEX，不保存 C++ 源码副本。

## 目录内容

| 文件/目录 | 用途 |
| --- | --- |
| `gmp_sil_protocol.hpp` | 帧头、状态、连接 ID、序号和 ABI 校验 |
| `asio_tcp_helper.hpp` | TCP 字节流上的定长帧传输 |
| `framed_udp_helper.hpp` | UDP 数据报上的同构帧传输 |
| `gmp_sil_helper.hpp` | 控制器侧统一 TCP/UDP 接口和 JSON 配置解析 |
| `mdl_gmp_sil_core.cpp` | Simulink Level-2 C++ S-function 实现 |
| `GMP_SIL_Core.cpp` | MATLAB MEX/生成目标的唯一编译入口 |
| `build_gmp_sil_mex.m` | 使用 MATLAB 当前配置的 `mex` 编译器构建 S-function |
| `tests` | TCP、UDP、协议、并发连接和 Simulink 测试对端 |

TCP 保证连接内字节有序和重传，但不保留应用报文边界；UDP 保留数据报边界，但可能丢包。
两种传输因此共用同一个 24 字节网络序帧头、连接 ID、事务序号、负载长度和仿真状态。
本机地址优先使用 UDP；跨主机或网络质量不稳定时优先使用 TCP。

## 启动、超时和结束

- 常用顺序是控制器先启动并阻塞等待，Simulink 启动后先发送 `hello`/首帧。
- 控制器服务端默认不启用启动超时，可以先启动并一直等待 Simulink；等待前会输出监听协议和端口。
- Simulink MEX 客户端始终启用 5 秒连接/启动握手超时，超时会释放连接并终止模型初始化。
- Simulink 收到的前 10 个完整有效帧使用短超时；第 10 帧通过后才切换到长调试超时。
  半帧、错误连接 ID、错误 ABI 或错误序号不会推进计数。
- 正常结束由显式仿真状态帧通知，不依赖超时。`abort()`可主动打断阻塞的网络操作。
- 每个 `gmp_sil_helper` 对象独占连接和状态；不同连接必须使用不同端口和连接 ID。

JSON 只使用两个传输端口，并同时声明双向负载长度：

```json
{
  "schema_version": 1,
  "protocol_version": 1,
  "transport": "tcp",
  "role": "server",
  "connection_id": "00112233445566778899aabbccddeeff",
  "target_address": "192.168.0.9",
  "bind_address": "0.0.0.0",
  "transmit_port": 12501,
  "receive_port": 12500,
  "simulink_to_controller_bytes": 264,
  "controller_to_simulink_bytes": 200,
  "connect_timeout_ms": 5000,
  "startup_io_timeout_ms": 5000,
  "startup_timeout_enabled": false,
  "established_io_timeout_ms": 2000000,
  "established_after_frames": 10,
  "max_payload": 65536
}
```

## MATLAB MEX

依赖由 GMP 私有环境预先恢复，MATLAB 不调用 vcpkg，也不进行网络下载。在 MATLAB 中：

```matlab
addpath(fullfile(getenv('GMP_PRO_LOCATION'), 'tools', 'gmp_sil', 'sil_helper'));
artifact = build_gmp_sil_mex;
```

正常安装应运行 `slib/install_gmp_simulink_lib.m`。安装器调用上述函数，在临时目录编译，
随后仅把 MEX 复制到 `slib/simulink_lib_src/src` 和对应 MATLAB Release 的安装目录。

## C++ 测试

Windows 可直接使用统一入口，它会通过 `gmp_env.bat` 创建 `build`、编译并运行测试：

```bat
tools\gmp_sil\sil_helper\build.bat Release
```

等价的手工命令为：

```powershell
cmake -S tools/gmp_sil/sil_helper `
      -B tools/gmp_sil/sil_helper/build `
      -DCMAKE_TOOLCHAIN_FILE=bin/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build tools/gmp_sil/sil_helper/build --config Release
ctest --test-dir tools/gmp_sil/sil_helper/build -C Release --output-on-failure
```

测试覆盖 TCP/UDP 请求响应、分片 TCP 帧、连接 ID/ABI/序号拒绝、显式结束、主动中止、
启动超时策略，以及两个并行独立会话。

Simulink 快速加速构建阶段可能探测 S-function。`GMP_SIL_Core` 会在 Rapid target
构建阶段跳过网络连接，在真正的定步长仿真进程中再建立会话，避免构建探测提前消耗
控制器连接。Windows 和 Linux 都应使用各自 MATLAB Release 的 MEX。
