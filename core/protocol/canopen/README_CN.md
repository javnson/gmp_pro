# GMP CANopen 核心模块

本目录提供不使用动态内存、可跨嵌入式平台的 CANopen 核心：经典 CAN 报文模型、
NMT/心跳、基于红黑树的对象字典、SDO 服务端和 PDO 映射执行。CAN 控制器收发、
滤波、队列与调度仍由 CSP 或用户适配层负责。

## 模块与边界

- `can_if.h` 定义标准帧和归一化请求；`gmp_canopen_decode_frame()` 与
  `gmp_canopen_encode_request()` 按约定只保留适配接口，当前没有默认实现。
- `nmt_sm` 处理广播/定点 NMT 命令、Boot-up、生产者心跳和消费者超时。
- `od` 按 `(index, sub-index)` 将对象字典元素挂入侵入式红黑树。元素既可用
  实际值 Union 保存不超过 8 个报文字节的数据，也可用指针 Union 连接用户变量
  或更大的缓冲区。
- `sdo_engine` 实现服务端 expedited/segmented upload 和 download，默认最大分段
  事务为 128 字节，并产生标准 abort 报文。
- `pdo_engine` 先校核并编译映射，再在 NMT Operational 状态下构造 TPDO 或应用
  RPDO。目前只接受完整字节、宽度与 OD 元素完全一致且总长不超过 8 字节的映射。

所有 CAN 报文字节使用 `gmp_canopen_octet_t`（`uint_least8_t`），以兼容 C28x 上
C 语言字节宽度可能大于线上的 8 bit；所有多字节值都显式按小端序列化。

当前范围不包括 CAN FD、位级 PDO 映射、SYNC 调度、inhibit/event timer、EMCY、
TIME、LSS、SDO client 和 CiA 402 驱动状态机。应用应把硬件收到的标准帧交给
NMT/SDO/PDO，并发送这些引擎返回的帧。

## EDS 来源说明

EDS 按 CiA 306-1 描述的是“具体设备”，CiA 并没有公开发布每种 profile 唯一通用
的设备 EDS。CiA 301 v4.2.0、CiA 401/402 完整规范以及一致性工具中的对象数据库
属于受许可资料。因此三个目录里的 `.eds` 是依据 CiA 公共说明维护的 **GMP profile
seed**，不是 CiA 官方逐字文件、不是厂商设备 EDS，也不构成一致性声明。它们用于
验证对象字典生成和集成；产品化时必须替换 Identity，并按真实设备补齐对象。

官方入口：

- <https://www.can-cia.org/can-knowledge/cia-306-series-electronic-device-description-edd>
- <https://www.can-cia.org/can-knowledge/cia-401-series-i/o-device-profile>
- <https://can-cia.org/can-knowledge/cia-402-series-canopen-device-profile-for-drives-and-motion-control>
- <https://www.can-cia.org/services/where-is-the-eds-checker-gone>

## 生成与测试

生成器默认使用指针存储，便于用户变量直接出现在 CCS Expressions；
`--storage value` 可为小型私有对象生成实际值 Union，`--node-id` 用于展开 EDS 中的
`$NODEID` COB-ID 表达式。

在仓库根目录依次执行：

```powershell
python tools/protocol_mgr/canopen_eds_cc/canopen_eds_cc.py core/protocol/canopen/cia301/cia301.eds --output-dir core/protocol/canopen/cia301 --name gmp_cia301_od --storage pointer --node-id 1
python tools/protocol_mgr/canopen_eds_cc/canopen_eds_cc.py core/protocol/canopen/cia401/cia401.eds --output-dir core/protocol/canopen/cia401 --name gmp_cia401_od --storage pointer --node-id 1
python tools/protocol_mgr/canopen_eds_cc/canopen_eds_cc.py core/protocol/canopen/cia402/cia402.eds --output-dir core/protocol/canopen/cia402 --name gmp_cia402_od --storage pointer --node-id 1
python -m unittest discover -s tools/protocol_mgr/canopen_eds_cc/tests -v
& 'C:\Program Files\Microsoft Visual Studio\2022\Professional\MSBuild\Current\Bin\amd64\MSBuild.exe' core/unit_test/projects/core_unit_tests.vcxproj /m /p:Configuration=Debug /p:Platform=x64
& 'C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\IDE\CommonExtensions\Microsoft\TestWindow\vstest.console.exe' core/unit_test/projects/out/x64/Debug/core_unit_tests/core_unit_tests.dll
```

重点测试向量如下：

| 场景 | 输入 | 应观察到的结果 |
| --- | --- | --- |
| Boot-up/心跳 | 节点 5 初始化并 tick | 首帧 `0x705 [00]`，随后进入 Pre-operational；周期到达后发送当前状态 |
| NMT 接收 | `0x000 [01 05]` | 节点 5 进入 Operational；非法 node-id、扩展帧和远程帧被忽略 |
| 心跳消费 | `0x707 [05]` 后累计 250 ms | 记录 Operational，在超时时间到达时置位 `timed_out` |
| expedited SDO | 向节点 3 发送 `0x603 [23 00 20 00 EF BE AD DE]` | OD `0x2000:00` 写入 `0xDEADBEEF`，回复 `0x583 [60 00 20 00 ...]` |
| segmented SDO | 对 10 字节 Domain 执行 initiate 和两个 segment | toggle、末段长度和顺序正确；错误 toggle/out-of-sequence 产生 abort |
| PDO | 编译 16-bit + 32-bit 映射 | Pre-operational 拒绝上线；Operational 下 TPDO/RPDO 完成 6 字节双向搬运 |
| 生成 OD | 分别初始化 301/401/402 生成文件 | 三棵树校核通过并能查到 profile 关键对象；静态 entry 被第二棵树复用时被拒绝 |

原生测试必须显示 47/47 通过且 `/W4 /WX` 下为 0 warning。EDS 工具测试必须显示
6/6 通过，其中包含三套已提交生成文件的确定性再生成比较。C28x 可移植性检查使用
TI C2000 CGT 22.6.1.LTS 与 25.11.1.LTS 对 7 个 CANopen C 源文件执行 C99 编译；
当前两套编译器均已通过。
