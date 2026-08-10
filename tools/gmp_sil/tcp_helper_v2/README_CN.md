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

- 连接、接受、读取和写入都有有限且可配置的超时。对端无响应或断线时返回明确错误，
  不会无限阻塞。
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
  "io_timeout_ms": 5000,
  "max_payload": 16777216,
  "no_delay": true,
  "keep_alive": true
}
```

服务端使用 `bind_address`，客户端使用 `target_address`。当前原型要求使用数值 IPv4 地址，
避免域名解析引入不可控的额外阻塞。

## 编译和测试

除测试程序外，模块为纯头文件实现。在仓库根目录执行：

```powershell
cmake -S tools/gmp_sil/tcp_helper_v2 `
      -B tools/gmp_sil/tcp_helper_v2/build `
      -DCMAKE_TOOLCHAIN_FILE=bin/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build tools/gmp_sil/tcp_helper_v2/build --config Release
ctest --test-dir tools/gmp_sil/tcp_helper_v2/build -C Release --output-on-failure
```

测试覆盖帧头字节序、512次SIL尺寸二进制请求/响应、强制分片写入、有限超时、半帧断线、
超长帧拒绝、心跳和JSON解析。
测试还会拒绝事务序号不匹配和错误帧魔数。

## 当前集成边界

本次没有提供 MATLAB MEX/S-Function、Simulink mask、UDP兼容层、Suite控制程序接线或
安装器入口。后续集成需要明确客户端/服务端角色，把一个带序号的`data_request`严格映射
为一次控制器执行，保留major-step保护，并在不改变SIL负载ABI的前提下设计UDP/TCP选择方式。
