# GMP CANopen 核心模块

本目录提供不使用动态内存、可跨嵌入式平台的 CANopen 核心：经典 CAN 报文模型、
NMT/心跳、基于红黑树的对象字典、SDO 服务端、预编译 PDO 和归一化 CoE 接口。
完整节点负责协议设施和中断/后台队列；CAN/EtherCAT 的实际搬运、滤波、mailbox
编码以及唤醒调度仍由 CSP 或用户适配层负责。

## 模块与边界

- `can_if.h` 定义标准帧和归一化请求；`gmp_canopen_decode_frame()` 与
  `gmp_canopen_encode_request()` 按约定只保留适配接口，当前没有默认实现。
- `nmt_sm` 处理广播/定点 NMT 命令、Boot-up、生产者心跳和消费者超时。
- `od` 按 `(index, sub-index)` 将对象字典元素挂入侵入式红黑树。元素既可用
  实际值 Union 保存不超过 8 个报文字节的数据，也可用指针 Union 连接用户变量
  或更大的缓冲区。
- `sdo_engine` 实现服务端 expedited/segmented upload 和 download，默认最大分段
  事务为 128 字节，并产生标准 abort 报文。
- `pdo_engine` 分别编译 TXPDO 与 RXPDO，并用独立的有序组管理多个 PDO。在线
  `_fast` 接口直接访问编译时解析好的存储地址，不再查红黑树或重复编译。
- `ethercat_if` 在同一 OD、SDO abort 模型和 PDO 计划之上提供归一化 CoE 接口。
- `canopen.h`/`src/canopen.c` 将以上模块组成一个完整节点，定义 CAN/CoE 公共报文、
  CiA 301 基础对象、有界输入/输出队列以及传输层和主循环使用的三个回调。

## 完整节点与公共接口

用户通常只需声明一个 `gmp_canopen_t`。`gmp_canopen_init()` 使 NMT、SDO、PDO 和
CoE 共用一棵 OD，并注册采用指针存储的 CiA 301 基础对象：`0x1000`、`0x1001`、
`0x1017`、`0x1018:00..04`，以及可选字符串 `0x1008..0x100A`。之后可继续向节点
的 OD 加入应用/profile 对象，并把编译完成的 RX/TX PDO 计划加入相应组。

`gmp_canopen_packet_t` 是 CAN 与 CoE 共用的传输边界：CAN 使用
`GMP_CANOPEN_PACKET_CAN_FRAME`，`key` 表示 CAN ID；CoE 使用归一化 SDO/PDO 类型，
通过 `index/subindex`、请求 `number`、PDO `key` 和节点自有的 `byte_gt` 缓冲区传递
语义。EtherCAT 原始 mailbox header 不属于该公共接口。

运行顺序固定如下：

1. CAN 接收中断或 EtherCAT 回调调用 `gmp_canopen_input_callback()`；该函数只校验
   并复制报文，不在中断内解析协议或访问 OD。
2. 主循环调用 `gmp_canopen_background_callback(elapsed_ms)`；它推进心跳、解析排队的
   CAN/CoE 请求、更新 NMT/RPDO，并把 SDO 回复、心跳或 TPDO 放入输出队列。
3. CAN 空闲中断、DMA feeder 或 EtherCAT 适配层反复调用
   `gmp_canopen_output_callback()`，直到返回 `GMP_CANOPEN_NODE_EMPTY`。

最小的经典 CAN 适配形式如下；外设驱动只负责把 `frame` 搬入或搬出：

```c
static gmp_canopen_t canopen_node;

void app_canopen_init(void)
{
    (void)gmp_canopen_init_default(&canopen_node, 5U);
}

void app_can_rx_isr(const gmp_canopen_frame_t* frame)
{
    gmp_canopen_packet_t packet;
    if (gmp_canopen_packet_from_can(frame, &packet))
        (void)gmp_canopen_input_callback(&canopen_node, &packet);
}

void app_canopen_background(uint32_t elapsed_ms)
{
    gmp_canopen_packet_t packet;
    gmp_canopen_frame_t frame;
    (void)gmp_canopen_background_callback(&canopen_node, elapsed_ms);
    while (gmp_canopen_output_callback(&canopen_node, &packet) ==
           GMP_CANOPEN_NODE_OK)
        if (gmp_canopen_packet_to_can(&packet, &frame))
            app_can_start_transmit(&frame);
}
```

实际工程应由 CAN 空闲中断或 DMA 消费输出队列；上例把这一动作写在后台函数中，只为
展示公共接口的闭环，`app_can_start_transmit()` 由目标 CSP 提供。

每个队列保留一个物理槽用于区分满/空，因此可用容量为
`GMP_CANOPEN_QUEUE_SLOTS - 1`。这是严格的 SPSC 契约：输入回调是 RX 唯一生产者，
后台回调是 RX 消费者和 TX 唯一生产者，输出回调是 TX 唯一消费者。
`gmp_canopen_publish_tpdo()` 也必须从后台/TX 生产者上下文调用，不能从另一个中断
并发生产。队列深度、公共报文容量和单次后台处理预算均可在编译期调整。可移植实现
面向单核 ISR/主循环；`volatile` 索引不是多核内存屏障，多核适配层必须额外串行化或
提供目标平台的内存栅栏。

## C28x 的 8 位线数据策略

本模块明确不定义 `gmp_canopen_octet_t`。C28x 的 `CHAR_BIT` 为 16，C 对象的
“byte”不能等同于协议的 8-bit octet，因此采用一套统一实现：

| 数据类别 | 表示 | 约束 |
| --- | --- | --- |
| CAN 帧、SDO/PDO 序列化缓冲区、字符串和 Domain | `byte_gt` 元素 | 每个元素承载一个线 octet；编译期保证 `byte_gt` 无符号，原生宽度可表示 `0xFF` 以上数值时仍拒绝越界值 |
| CiA `INTEGER8` / `UNSIGNED8` 标量 | `int_least8_t` / `byte_gt` | C28x 上允许占 16 C 位，但序列化和 EDS 仍执行 8 位范围约束 |
| 主机/驱动的紧凑字节缓冲区 | 条件化的 `uint8_t` 适配函数 | 仅在存在 `UINT8_MAX` 且 `CHAR_BIT == 8` 时提供；C28x 直接使用逻辑单元接口 |

因此 OD、NMT、SDO、PDO 不需要各维护一套 u8/u16 实现；表示转换只发生在硬件或
EtherCAT 协议栈适配层。所有多字节值均显式按小端序列化。

## PDO 分层与实时路径

编译阶段逐项检查映射、解析 OD 元素和实际存储地址、选择编解码器并计算偏移；
实时阶段只执行直接搬运。计划生存期内，OD 元素及其目标存储地址必须保持有效且
不变；若重新绑定指针，必须停止在线访问并重新编译该 PDO。

经典 CAN 的编译接口限制 8 octet 和 11-bit COB-ID；面向 CoE 的 buffer 编译接口
接受调用者指定的更大 process image。TX 组支持按排序槽快速构帧，RX 组使用二分
查找按 COB-ID 分发，收发状态不会混在一个对象中。

当前范围不包括 CAN FD、位级 PDO 映射、SYNC 调度、inhibit/event timer、EMCY、
TIME、LSS、SDO client 和 CiA 402 驱动状态机。`ethercat_if` 也不是原始 EtherCAT
mailbox 编解码器：mailbox header/counter、分片和 AL 状态门控由选用的 EtherCAT
协议栈适配；Complete Access 与 SDO Information 尚未实现。

ETG 公开资料说明 CoE 复用 CANopen 的 OD/SDO/PDO 模型，并允许 EtherCAT process
data 不受传统 8-octet PDO 长度约束：

- <https://www.ethercat.org/en/technology.html?trk=public_post_comment-text>
- <https://www.ethercat.org/download/documents/ETG1500_V1i0i2_D_R_MasterClasses.pdf>
- <https://www.ethercat.org/download/documents/ETG2200_V3i2i2_G_R_EtherCATImplementationGuide.pdf>

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
Push-Location core/protocol/canopen
& 'C:\Program Files\doxygen\bin\doxygen.exe' Doxyfile
Pop-Location
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
| PDO | 编译独立 TX/RX 计划并加入有序组 | Pre-operational 拒绝经典 CAN 上线；Operational 下 fast path 双向搬运且无需再次查 OD |
| CoE | 对同一 OD 执行 SDO，并编译 12-octet PDO | SDO upload/download 正确；CoE process buffer 成功，经典 CAN 编译拒绝超长映射 |
| 线 octet | 输入 `0x100` 并测试主机 u8 bridge | 非法逻辑单元被拒绝；8-bit 主机可无损导入/导出紧凑缓冲区 |
| 完整节点 | CAN SDO 写入后以 CoE SDO 读取同一对象，并经队列执行 NMT/RPDO/TPDO | 两种传输共用 OD；输入回调不直接解析；后台产生回复；输出回调按顺序取走结果 |
| 生成 OD | 分别初始化 301/401/402 生成文件 | 三棵树校核通过并能查到 profile 关键对象；静态 entry 被第二棵树复用时被拒绝 |

原生测试必须显示 53/53 通过且 `/W4 /WX` 下为 0 warning。EDS 工具测试必须显示
6/6 通过，其中包含三套已提交生成文件的确定性再生成比较。C28x 可移植性检查使用
TI C2000 CGT 22.6.1.LTS 与 25.11.1.LTS 对 9 个 CANopen C 源文件执行 C99 编译。
Doxygen 配置为遇到警告即失败。
