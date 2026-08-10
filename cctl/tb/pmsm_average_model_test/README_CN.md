# CCTL PMSM 平均值模型离线验证

本工程只验证 CCTL 数值内核和被控对象，不连接 UDP/TCP，也不运行 PMSM 参数辨识控制器。

## 构建与测试

```powershell
$build = Join-Path $env:TEMP 'gmp_cctl_pmsm_build'
cmake -S E:\lib\gmp_pro\cctl\tb\pmsm_average_model_test -B $build `
  -G "Visual Studio 17 2022" -A x64
cmake --build $build --config Release
ctest --test-dir $build -C Release --output-on-failure
```

性能基准：

```powershell
& "$build\Release\cctl_pmsm_average_model_benchmark.exe"
```

2026-08-10 的 MSVC Release 验证结果：所有测试通过。20 秒、20 kHz、400000 个 RK4 步的纯模型计算耗时约 0.059 s，约为实时速度的 336 倍。该数字只衡量 CCTL 数值内核，不包含控制器、通信、日志或操作系统调度开销。

## 已验证关系

- Euler 对指数衰减呈一阶收敛，RK4 呈四阶收敛。
- 锁轴 d 轴电压阶跃与 `V/R*(1-exp(-R/L*t))` 一致。
- 自由减速速度和角度与 `J*domega/dt=-B*omega-Tload` 的解析解一致。
- abc 瞬时功率与采用等幅 Clarke/Park 变换后的 `1.5*(vd*id+vq*iq)` 一致。
- 凸极式 PMSM 转矩、磁场储能、电磁转换功率和机械功率满足功率平衡。
- 平均死区线电压误差与配置公式一致。

PWM 关闭采用可配置的换流电流指数衰减，且关闭期间电磁转矩为零。使用 Euler 时，为得到单调稳定的电流衰减，应保持 `dt * open_circuit_current_decay_per_s <= 1`；RK4 的稳定范围更宽，但仍建议通过减小步长检查收敛性。
