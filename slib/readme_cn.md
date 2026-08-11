# GMP MATLAB/Simulink 库（`slib`）

[English](readme.md) | **简体中文**

`slib` 提供 GMP 的 Simulink 库、标准模型、SDPE 初始化工具，以及统一 TCP/UDP
软件在环（SIL）接口。SIL 的 C++ 源码只在
`tools/gmp_sil/sil_helper` 维护，`slib` 只发布 MATLAB 脚本、模型和安装时编译的
`GMP_SIL_Core` MEX。

## 支持范围

- 支持 MATLAB/Simulink R2022b 至 R2025b。
- 功率级模型需要 Simscape Electrical Specialized Power Systems 及 `powerlib`。
- R2026a 起不再提供上述产品，因此当前安装器会拒绝 R2026a 及更高版本。
- MEX 必须由目标 MATLAB Release 和操作系统自己的 `mex` 编译器构建，不能跨平台复制。
- Windows 和 Linux 均支持统一 TCP/UDP SIL；快速加速验证必须使用定步长模型。

## 源码与生成物边界

| 路径 | 作用 | 维护规则 |
| --- | --- | --- |
| `install_path/R2024b/gmp_sil_core_pack.slx` | SIL 库的 R2024b 可编辑、调试版本 | 先在这里完成 Mask 和模块调试。该目录通常是本机生成物，不作为跨版本发布源。 |
| `simulink_lib_src/gmp_sil_core_pack_src.slx` | 仓库发布的 R2022b 兼容源模型 | 只能通过 `export_gmp_simulink_lib_src` 从 R2024b 导出，不应独立手改。 |
| `simulink_lib_src/src/` | MATLAB 辅助函数和已编译 MEX | MATLAB 脚本在此维护；MEX 由安装器从唯一 C++ 源码重新编译。 |
| `simulink_lib_src/tests/` | MATLAB/SIL 回归测试 | 修改通信、Mask 或安装链后运行。 |
| `tools/gmp_sil/sil_helper/` | TCP/UDP 协议、控制器对象、S-function 和测试的唯一 C++ 源码 | 不在 `slib` 复制源码。 |
| `install_path/<Release>/` | 当前 MATLAB Release 的安装结果 | 除上述 R2024b 编辑流程外，不直接维护；由安装器重新生成。 |

其他 `_src.slx` 仍是各自库的跨版本源。SIL Core 特别采用 R2024b 编辑、R2022b
发布的流程，是为了能够先用当前工具调试，再向较老 MATLAB 发布兼容模型。

## 导出 R2022b 发布源

在 MATLAB R2024b 或更高版本中完成 R2024b SIL 库调试后执行：

```matlab
run(fullfile(getenv('GMP_PRO_LOCATION'), ...
    'slib', 'export_gmp_simulink_lib_src.m'));
```

该脚本会复制 R2024b 模型、升级统一 TCP/UDP Mask，并用
`Simulink.exportToVersion(...,'R2022B')` 原子更新
`simulink_lib_src/gmp_sil_core_pack_src.slx`。导出后应检查：

```matlab
info = Simulink.MDLInfo(fullfile(getenv('GMP_PRO_LOCATION'), ...
    'slib', 'simulink_lib_src', 'gmp_sil_core_pack_src.slx'));
assert(strcmp(info.ReleaseName, 'R2022b'));
```

## 安装

先通过 GMP 环境安装器恢复 ASIO 和 nlohmann-json 头文件，再在 MATLAB 中执行：

```matlab
run(fullfile(getenv('GMP_PRO_LOCATION'), ...
    'slib', 'install_gmp_simulink_lib.m'));
```

安装器按以下顺序工作：

1. 验证 MATLAB Release、`powerlib` 和 GMP 环境中已恢复的依赖；
2. 调用 `tools/gmp_sil/sil_helper/build_gmp_sil_mex.m`，使用 MATLAB 自带 `mex` 编译器；
3. 只把生成的 `GMP_SIL_Core.<mexext>` 放入 `simulink_lib_src/src`；
4. 从 R2022b 源模型生成 `install_path/<Release>` 的兼容库；
5. 复制脚本和 MEX、注册 MATLAB 路径并刷新 Library Browser。

MATLAB 安装阶段不会运行 vcpkg，也不会访问网络。缺少依赖时，应先从
`gmp_env.bat` 修复 GMP 私有环境。

## SIL Core Mask

SIL Core 的网络配置只包含：

- `Transport`：TCP 或 UDP；
- `Target address`：`localhost`、`127.0.0.1` 或数值 IPv4；
- `Target receive port`；
- `Local receive port (UDP)`：仅 UDP 显示，TCP 自动隐藏。

本机连接选择 TCP 时会建议使用 UDP；非本机连接选择 UDP 时会建议使用 TCP。
旧的 command TX/RX 端口、手工连接 ID、ABI 字节数和协议状态均不再作为网络 Mask
输入。Mask 中仍保留打包/解包数据类型、尺寸和对齐参数，因为它们定义 Simulink
信号 ABI，不属于网络连接配置。

连接初始化会按模型和 block 生成独立 connection ID，并在普通单控制器工程的
`network.json` 中记录两个端口、双向 payload 长度及状态协议。一个模型内多个控制器或
多个模型并行时，每个控制器必须使用各自的 JSON、不同端口对和对应 connection ID；
connection ID 会进一步阻止错误会话串线。

控制器通常先启动并无限等待 Simulink，等待前会打印监听协议和端口。Simulink MEX
客户端在连接及启动握手阶段固定使用 5 秒超时；控制器缺失、地址错误或握手不匹配时，
它会释放网络资源并终止模型初始化。正常结束仍使用显式仿真状态帧；前 10 个完整有效
响应通过后，Simulink 才切换到适合调试器暂停的长超时。

## 开发与验证

```matlab
addpath(fullfile(getenv('GMP_PRO_LOCATION'), 'slib', ...
    'simulink_lib_src', 'src'));
results = runtests(fullfile(getenv('GMP_PRO_LOCATION'), 'slib', ...
    'simulink_lib_src', 'tests'));
assert(all([results.Passed]));
```

修改 SIL 库的推荐顺序是：

1. 在 R2024b 编辑和调试 `install_path/R2024b/gmp_sil_core_pack.slx`；
2. 运行 `export_gmp_simulink_lib_src`，检查输出确为 R2022b；
3. 运行 `install_gmp_simulink_lib`，验证当前 Release 安装结果和 MEX；
4. 运行 MATLAB 回归测试、定步长 SIL/快速加速测试，以及至少一个实际 suite 模型。

## 故障排查

| 现象 | 检查内容 |
| --- | --- |
| 找不到 `powerlib` | 使用 R2025b 或更早版本，并安装/授权 Specialized Power Systems。 |
| 缺少 ASIO/JSON | 从 `gmp_env.bat` 修复私有环境；不要让 MATLAB 自行调用 vcpkg。 |
| 找不到 `GMP_SIL_Core` | 重新运行安装器，并确认当前平台的 MEX 位于安装目录 `src`。 |
| TCP/UDP 无法建立 | 核对地址、两个端口、启动顺序、防火墙和生成 JSON 中的 ABI 长度。 |
| 两个 SIL 会话串线 | 为每个会话分配不同端口对，并核对 connection ID。 |
| 快速加速不通信 | 确认使用定步长、当前 Release 编译的 MEX，并查看 Rapid target 的构建日志。 |
| 源库版本错误 | 重新从 R2024b 导出，并用 `Simulink.MDLInfo` 验证 `R2022b`。 |

卸载当前 Release 可运行 `slib/uninstall_gmp_simulink_lib.m`。
