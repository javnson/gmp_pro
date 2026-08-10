# GMP SIL TCP Helper v2

[English](README.md) | **简体中文**

本目录提供一个独立的、带帧协议的 TCP 传输原型，供后续 GMP SIL/PIL 在跨电脑或较差
网络环境下使用。当前模块没有安装到 MATLAB，没有加入 Simulink 库，也没有被任何
Suite 工程引用。

## 为什么 TCP 仍然需要帧协议

TCP 在连接有效期间保证字节顺序，并自动重传丢失的 IP 包，但 TCP 本身是字节流，不保留
应用报文边界。一次 `send()` 可能被拆成多次接收，多次发送也可能合并。因此模块使用
定长精确读取，并在每个负载前加入24字节、网络字节序的帧头：

| 偏移 | 长度 | 字段 |
|---:|---:|---|
| 0 | 4 | 魔数 `GMPT` |
| 4 | 2 | 协议版本（`1`） |
| 6 | 2 | 帧类型 |
| 8 | 4 | 标志/保留字段 |
| 12 | 8 | 事务序号 |
| 20 | 4 | 负载字节数 |

一条全双工连接复用数据请求/响应、命令、心跳和错误帧。SIL 是小数据包的因果请求/响应
过程，因此默认开启 `TCP_NODELAY`，避免 Nagle 聚合引入额外延迟。

## 故障处理约定

- 连接和接受使用较短的连接超时。数据链路在收到指定数量的完整帧以前使用启动期短超时；
  确认链路已经运行后自动切换为长超时。默认收到1个完整帧后从5秒切换为2000秒，设计思想
  与UDP helper一致，允许在调试器中长时间卡住控制程序并调整参数。
- 只有完整帧（帧头和全部负载）才会推进切换计数；半帧不会把故障链路误判为已建立链路。
- `abort()`是无等待的主动中止入口，可由仿真停止回调或另一线程调用。它会打断正在进行的
  连接、接受、读取或写入，并返回专用的`error_code::aborted`，因此数据传输完成时不必等待
  长超时到期。主动中止不发送应用层报文，网络或对端卡死不会阻碍本地退出。
- 超时或只收到部分帧后立即关闭连接，因为继续读取将无法可靠恢复下一帧边界。
- 在分配内存前检查负载长度上限。
- 事务响应必须具有匹配的序号和帧类型。
- 不自动重试控制事务。TCP可以消除正常连接中的丢包，但连接中断后无法确认最后一个
  请求是否已经让控制器执行；自动重放可能让同一个控制周期执行两次。

当前每个 helper 对象串行执行网络操作，面向
`请求 -> 控制器执行一步 -> 响应`的因果 SIL 模型，不支持多个并发未完成请求。

## 配置格式

`parse_configuration()` 接受如下 JSON：

```json
{
  "transport": "tcp",
  "role": "client",
  "target_address": "192.168.1.20",
  "bind_address": "0.0.0.0",
  "port": 12510,
  "connect_timeout_ms": 5000,
  "startup_io_timeout_ms": 5000,
  "established_io_timeout_ms": 2000000,
  "established_after_frames": 1,
  "max_payload": 16777216,
  "no_delay": true,
  "keep_alive": true
}
```

服务端使用 `bind_address`，客户端使用 `target_address`。当前原型要求使用数值 IPv4 地址，
避免域名解析引入不可控的额外阻塞。`established_after_frames`设为`0`时会从连接建立后立即
使用长超时；通常保持默认值`1`，以便启动错误仍能快速暴露。

正常仿真结束时，控制线程应调用`helper.abort()`。如果另一个线程正阻塞在`receive()`中，
主动中止会直接唤醒它；调用方应识别`error_code::aborted`为预期停止，而不是网络故障。

## 编译和测试

除测试程序外，模块为纯头文件实现。在仓库根目录执行：

```powershell
cmake -S tools/gmp_sil/tcp_helper_v2 `
      -B tools/gmp_sil/tcp_helper_v2/build `
      -DCMAKE_TOOLCHAIN_FILE=bin/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build tools/gmp_sil/tcp_helper_v2/build --config Release
ctest --test-dir tools/gmp_sil/tcp_helper_v2/build -C Release --output-on-failure
```

测试覆盖帧头字节序、512次SIL尺寸二进制请求/响应、强制分片写入、启动期有限超时、
完整帧后的长超时、跨线程主动中止、半帧断线、超长帧拒绝、心跳和JSON解析。
测试还会拒绝事务序号不匹配和错误帧魔数。

## 当前集成边界

本次没有提供 MATLAB MEX/S-Function、Simulink mask、UDP兼容层、Suite控制程序接线或
安装器入口。后续集成需要明确客户端/服务端角色，把一个带序号的`data_request`严格映射
为一次控制器执行，保留major-step保护，并在不改变SIL负载ABI的前提下设计UDP/TCP选择方式。
