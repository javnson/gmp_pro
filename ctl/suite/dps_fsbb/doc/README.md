# DPS FSBB 文档索引

- [FSBB UDP/SIL 联合仿真实验报告](FSBB_UDP_SIL仿真实验报告.md)：从 SDPE 参数设置、控制器编译、模型初始化到 BUILD_LEVEL 1–3 验证的完整操作流程。
- [`images/`](images/)：模型接线图和三组正式验证波形。
- [`results/`](results/)：与报告对应的原始 JSON 指标快照。

本目录中的结果是已调通版本的归档快照。继续开发时，仿真脚本会把新结果写入 `../project/simulate/validation/`；确认结果有效后再同步到本目录，避免实验报告被临时调试数据覆盖。
