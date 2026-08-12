# N32H474VEL7-STB GMP 示例

该工程由 N32Cube/NTFx 创建，并通过 `gmp_src_mgr` 接入 GMP。已打通：

- Cortex-M SysTick 提供 N32Cube 与 GMP 共用的 1 ms 时基，不占用 GTIM10；
- 板载 D1（PA3，高电平点亮）每秒翻转；
- USART1（PA9 TX、PA10 RX、115200-8-N-1）通过 J5/NS-LINK 接入 GMP Debug Tool；
- GMP datalink 的 echo、PIL、参数整定和内存观察后台任务。

N32H47x 常用外设和 GMP CSP 用法见 [N32H47x/N32H48x CSP 与常用外设快速指南](../README_CN.md)。

## 生成 GMP 源文件

先安装/配置 GMP，使 `GMP_PRO_LOCATION` 指向仓库根目录，然后执行：

```bat
cd gmp_src_mgr
gmp_generate_all.bat
```

选择模块的唯一来源是 `gmp_src_mgr/gmp_framework_config.json`。工程宏由 SDPE 文件 `xplt/xplt.config.h` 管理。

## Keil 编译

1. 安装 `Nations.N32H47x_DFP.1.2.0.pack`，并确认 Keil 可找到 N32H474 device。
2. 运行 `gmp_generate_all.bat`。
3. 打开 `MDK-ARM/N32Test.uvprojx`，选择 `N32Test` target 后 Build。
4. 下载后将 J5 的 MCU_TX/MCU_RX 跳帽连接到 NS-LINK，串口使用 115200-8-N-1。

N32Cube 下载地址：

https://www.nationstech.com/api/dowfilebefore?did=6616

也可以在 https://www.nationstech.com/support/down/ 搜索 N32Cube。
