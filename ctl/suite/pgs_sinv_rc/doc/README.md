# PGS SINV RC 文档索引

- [SINV UDP/SIL 联合仿真实验报告](SINV_UDP_SIL仿真实验报告.md)：BUILD_LEVEL 1–5 的配置、编译、启动、判据和已验证结果。
- [`images/`](images/)：五个控制层级及 FDRC 开/关对比的正式波形。
- [`results/`](results/)：与实验报告对应的原始 JSON 指标快照。
- `SINV 调试记录-20250726.docx`：早期硬件调试历史记录，保留供追溯；当前仿真操作以本实验报告和 SDPE 配置为准。

本目录保存已确认结果的归档快照。新仿真首先写入 `../project/simulate/validation/`，确认 BUILD_LEVEL、编译版本和指标有效后再同步到 `doc`。
