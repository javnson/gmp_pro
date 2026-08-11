# GMP Data Link Python API

[English](README.md) | **简体中文**

本目录提供 GMP Data Link Tunable、Memory Perspective、Scope 和处理器在环服务的无界面
Python 接口，适用于测试脚本、自动化验证、实验室工具以及 AI 辅助硬件调试。API 与
图形化调试器共用协议和资源描述符编解码逻辑，因此下位机不需要分别维护 GUI 与 API
两套固件。

公共入口为 `GmpDatalinkClient`。一个 Client 独占一条串行请求/响应链路，并提供三组
服务：

| 服务 | 属性 | 主要操作 |
| --- | --- | --- |
| Tunable | `client.tunables` | 发现、读取和修改注册参数 |
| Memory Perspective | `client.memory` | 发现、读取和修改白名单内存段 |
| Data Link Scope | `client.scope` | 发现、配置、布防、等待并下载波形 |
| 处理器在环 | `PilApi` / `PilBridge` | 严格执行控制器单步，并桥接标准 Simulink UDP 向量 |

## 安装与导入

建议从调试器目录运行脚本，使 `apis` 位于 Python 导入路径：

```powershell
cd tools/gmp_pil_server/gmp_debugger
python -m pip install pyserial
python -m apis.examples.ai_debug_session --port COM5 --baudrate 256000
```

也可以把 `tools/gmp_pil_server/gmp_debugger` 加入 `PYTHONPATH`。无界面 API
要求 Python 3.10 或更高版本以及 `pyserial`，不依赖 PyQt 和 NumPy。

## 快速开始

```python
from apis import GmpDatalinkClient

with GmpDatalinkClient("COM5", 256000) as dl:
    parameters = dl.tunables.discover()
    regions = dl.memory.discover()
    scopes = dl.scope.discover()

    print(dl.tunables.read_all())
    if regions:
        print(dl.memory.read_region(regions[0], byte_length=16).hex(" "))
```

上下文管理器会自动打开和关闭串口。`timeout` 是单次请求的超时时间，`retries` 是
首次请求失败后的重试次数：

```python
dl = GmpDatalinkClient("COM5", 921600, timeout=0.8, retries=3)
```

不要让 GUI 和无界面 API 同时打开同一串口。传输层内部会串行化多线程请求，但一条
物理串口仍然只能有一个请求/响应所有者。

## Tunable 参数

修改控制器之前应首先发现下位机参数表。每个 `TunableParameter` 包含 `item_id`、
`name`、`data_type` 和 `permission`。下位机名字允许为空；此时主机会生成
`Parameter N` 作为显示名称，ID 或描述符对象仍可作为稳定选择器。

```python
from apis import AccessPermission

with GmpDatalinkClient("COM5", 256000) as dl:
    table = dl.tunables.discover()
    for item in table:
        print(item.item_id, item.name, item.data_type.name, item.permission.name)

    frequency = dl.tunables.read("sine_frequency_hz")
    dl.tunables.write("sine_frequency_hz", 50.0)
    assert dl.tunables.read("sine_frequency_hz") == 50.0

    writable = [item for item in table if item.permission == AccessPermission.READ_WRITE]
    values_by_id = dl.tunables.read_many(writable)
```

Tunable 方法支持三种选择方式：精确名称、整数 ID、`discover()` 返回的描述符对象。
如果下位机有意注册重复名称，请使用 ID 或对象，因为名称选择器必须唯一匹配。
`read_all()` 返回以名称为键的完整值表；`write_many()` 会先检查只读权限，再按协议
允许的大小分批发送：

```python
dl.tunables.write_many({
    "sine_frequency_hz": 50.0,
    "sine_gain": 1.25,
    "sine_offset": 0.0,
})
```

API 根据下位机描述符自动编码 `U16`、`I16`、`U32`、`I32` 或 `F32`。Python
输入值必须在相应线协议类型的范围内。

## Memory Perspective

`discover()` 会取得下位机维护的访问白名单。`MemoryRegion` 只包含纯粹内存访问所需
的信息：`region_id`、字节地址 `address`、`byte_length`、`permission` 和可选
`name`。

```python
import struct

with GmpDatalinkClient("COM5", 256000) as dl:
    regions = dl.memory.discover()
    state = dl.memory.resolve("controller_state")
    raw = dl.memory.read_region(state)
    first_value = struct.unpack_from("<f", raw, 0)[0]

    if state.permission.name == "READ_WRITE":
        dl.memory.write_region(state, struct.pack("<f", 1.0), offset=0)
```

常用方法为：

- `read_region(selector, offset=0, byte_length=None)`；
- `write_region(selector, data, offset=0)`；
- `read(address, byte_length, chunk_size=240)`；
- `write(address, data, chunk_size=240)`。

线协议中的地址和长度始终以 8 位字节为单位，C28x 也不例外。仅当手工输入 C28x
链接器给出的原生字地址时才需要乘以 2；下位机上报的地址已经完成归一化。大块传输会
自动拆分。多分块写入不是原子操作，因此不能用于必须整体同时生效的实时控制状态。

完成发现后，Client 会在发送前校验访问范围和权限；下位机白名单仍是最终权限边界，
会独立拒绝非法访问。这使主机不能通过接口越过显式注册的内存区域。

## Data Link Scope

Scope 是独立于 Memory Perspective 的波形采集服务。发现结果 `ScopeResource` 包含
通道数、深度、采样类型、布局、标称采样率和名称。

```python
from apis import GmpDatalinkClient, ScopeConfiguration, ScopeTriggerMode

configuration = ScopeConfiguration(
    mode=ScopeTriggerMode.RISING_EDGE,
    trigger_channel=0,
    trigger_level=0.0,
    trigger_position_percent=25.0,
    auto_timeout_ms=1000,
    sample_divider=9,
)

with GmpDatalinkClient("COM5", 256000) as dl:
    scope = dl.scope.discover()[0]
    frame = dl.scope.capture(scope, configuration, timeout=5.0)
    print(frame.generation, frame.sample_rate_hz)
    print(frame.time_seconds[:5])
    print(frame.channels[0][:5])
    frame.save_csv("capture.csv")
```

触发模式包括 `IMMEDIATE`、`RISING_EDGE`、`FALLING_EDGE`、
`RISING_EDGE_AUTO` 和 `FALLING_EDGE_AUTO`。Auto 模式在没有满足条件的边沿时，
也会在 `auto_timeout_ms` 到达后完成一帧。

`trigger_position_percent` 决定触发点之前的数据占整帧的比例。例如 25% 会让触发点
位于横轴约四分之一处：约 25% 的样本记录边沿之前的历史，75% 记录边沿之后的变化。
下位机会持续维护预触发历史，再在触发后补齐剩余样本。

`sample_divider` 为非负整数，0 表示不分频。解码后的有效采样率为：

```text
effective_sample_rate = registered_sample_rate / (sample_divider + 1)
```

Scope 协议 v1 不支持非零分频，API 会明确抛出 `ValueError`，而不是静默忽略。

连续采集使用生成器；无人值守的测试应始终提供有限的 `count`：

```python
with GmpDatalinkClient("COM5", 256000) as dl:
    for index, frame in enumerate(
        dl.scope.iter_captures("control_scope", configuration, count=20)
    ):
        frame.save_csv(f"capture_{index:03d}.csv")
```

每次迭代只有在上一帧完整下载后才会重新布防，从而避免下位机在上传期间覆盖快照。
采集超时会抛出 `GmpDlTimeout`，但不会让主机传输层保持忙状态；调用者可以立刻发送
新的触发配置，或改用立即/自动触发。高级代码也可以显式调用 `configure()`、`arm()`、
`status()` 和 `read_snapshot()`。

## 处理器在环（PIL）

`PilConfiguration` 会加载目标工程本地的 SDPE requirement，并校验 PIL 开关、命令
分配、通道掩码与索引、UART 波特率和 UDP 端点。因此，固件和上位机共用 SDPE 这一
唯一连接配置源。

在 `tools/gmp_pil_server/gmp_debugger` 下启动标准桥接器：

```powershell
python -m apis.examples.pil_bridge `
  --sdpe ../../../ctl/suite/mcs_pmsm_nt/project/f280049c/sdpe_mgr/sdpe_requirement.json `
  --port COM5 `
  --trace ../../../ctl/suite/mcs_pmsm_nt/project/f280049c/pil/results/manual/bridge_trace.csv
```

Simulink 侧使用标准的 264-byte 输入向量和 200-byte 输出向量。每一个被接受的 UDP
输入严格对应一次 Data Link STEP 事务和一次控制器执行。STEP 不会自动重试：如果
目标已经执行但响应丢失，重试会让同一输入推进控制器两次。CSV 会记录仿真时间、所选
编码器输入、全部 ADC/PWM/Monitor 通道、输出使能状态和目标往返时间。

`ENABLE_GMP_DL_PIL_SIM` 必须是目标工程本地、独立的 SDPE 功能开关。关闭时，普通
ADC 中断和物理 PWM 路径应按原逻辑编译与运行；开启时，目标必须隔离全部物理输出，
并且只允许经过完整校验的 PIL STEP 请求调度控制器。PIL 固件不得用于带功率运行。

## 异常与底层扩展

公共异常层次如下：

- `GmpDlError`：API 与传输层基础异常；
- `GmpDlTimeout`：没有收到有效响应，或 Scope 未在限时内完成；
- `GmpDlProtocolError`：下位机响应格式错误或内容不一致；
- `GmpDlTargetError`：下位机明确拒绝操作；
- 标准 `ValueError`、`KeyError`、`PermissionError` 和 `struct.error`：调用参数
  非法或数值与目标类型不兼容。

新增 Data Link 子模块可以直接复用传输层，不必重复实现串口帧：

```python
response = dl.transact(command=0x70, payload=b"\x01\x02")
```

仿真和测试所用的替代传输只需实现同步的
`transact(command: int, payload: bytes) -> bytes`。如果工程修改了命令基址，可在
构造 `GmpDatalinkClient` 时传入 `tunable_command`、`memory_command` 和
`scope_command`。

## 推荐的自动化与 AI 调试流程

1. 首先发现全部资源并记录描述符。
2. 修改前读取并保存 Tunable 初值。
3. 确认目标状态与权限，只进行有界且物理意义明确的修改。
4. 每次写入后立即读回校验。
5. 使用有限超时采集 Scope 证据，并保存为 CSV。
6. 在 `finally` 中恢复临时修改的参数。
7. 出现保护事件、重复超时或描述符不一致时立即停止，不能通过任意内存写入进行补偿。

写操作会直接作用于正在运行的嵌入式系统。API 可以保证协议与白名单安全，但无法自行
判断一个控制值对于已连接的功率硬件是否安全。

## 验证

在 `tools/gmp_pil_server/gmp_debugger` 下执行：

```powershell
python -m unittest discover -s apis/tests -v
python -m py_compile apis/__init__.py apis/client.py apis/protocol.py apis/pil.py apis/examples/pil_bridge.py
```

完整可运行示例位于
[`examples/ai_debug_session.py`](examples/ai_debug_session.py)，下位机线协议参见
[`core/dev/readme_dl_protocol.md`](../../../../core/dev/readme_dl_protocol.md)。
