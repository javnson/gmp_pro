# PMSM 离线参数辨识模块

`pmsm_offline_id` 是 PMSM 控制器使用的分阶段离线辨识状态机。它通过适配器访问外部 FOC、编码器和 PWM 安全接口；模块本身不拥有目标平台外设。

## 当前辨识阶段

状态和配置以 `pmsm_offline_id_sm.h` 为准：

- `PREPARE`：应用自定义准备/校准；
- `ENCODER_CALIB`：有感编码器偏置、极对数与故障检查；
- `RS_DT`：定子电阻与综合死区压降；
- `LD_LQ`：d/q 轴电感；
- `FLUX`：永磁体磁链；
- `MECH`：机械惯量、黏性摩擦和负载相关量；
- `COMPLETE` / `FAULT`：完成或故障保持。

各阶段由 `ctl_pmsm_offline_id_init_t::cfg_basic` 的 `flag_enable_*` 字段选择。高频路径调用 `ctl_step_pmsm_offline_id()`，后台路径调用 `ctl_loop_pmsm_offline_id()`。辨识刺激、等待时间、限值和应用适配不应在模块源码中写死；套件通过 SDPE 和 `pmsm_offline_id_if` 组装这些参数。

## 最小集成骨架

```c
ctl_pmsm_offline_id_t oid;
ctl_pmsm_offline_id_init_t cfg;
ctrl_gt capture_buffer[2048];

/* 填充 cfg 的基值、阶段配置与 flag_enable_*，并绑定适配器。 */
ctl_init_pmsm_offline_id_sm(&oid, &cfg, capture_buffer, 2048U);
ctl_enable_pmsm_offline_id(&oid);

/* 在快速控制路径、FOC 正常执行位置之前调用。 */
ctl_step_pmsm_offline_id(&oid);

/* 在后台任务中调用。 */
ctl_loop_pmsm_offline_id(&oid);
```

`ctl_init_pmsm_offline_id_sm()` 初始化后进入 `PMSM_OFFLINE_ID_READY`。`ctl_disable_pmsm_offline_id()` 立即请求安全停机；`ctl_clear_pmsm_offline_id()` 清除状态并返回就绪。应用必须在任何故障或停机路径上保证物理 PWM 被禁止，不能只依赖软件状态枚举。

## 结果

完成后从对象中读取：

```c
if (oid.sm == PMSM_OFFLINE_ID_COMPLETE)
{
    parameter_gt rs_ohm = oid.pmsm_param.Rs;
    parameter_gt ld_henry = oid.pmsm_param.Ld;
    parameter_gt lq_henry = oid.pmsm_param.Lq;
    parameter_gt flux_weber = oid.pmsm_param.flux_linkage;
    parameter_gt deadtime_comp_volts = oid.V_comp_volts;
    parameter_gt inertia = oid.pmsm_mech_param.J_total;
    parameter_gt viscous_friction = oid.pmsm_mech_param.B_viscous;
}
```

辨识会主动给电机施加电压、电流并可能使其旋转。上电前必须确认电流标度、相序、编码器方向、保护限值、转子自由度和急停。可重复的主机 SIL 工作流、故障注入及当前误差记录见 [`ctl/suite/mcs_pmsm_id`](../../../suite/mcs_pmsm_id/README.md)；这些软件结果不自动构成目标硬件验证。
