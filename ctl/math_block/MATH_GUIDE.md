# GMP 数学计算库（Math Block）使用指南

## 📚 目录

1. [概述](#概述)
2. [模块架构](#模块架构)
3. [ctrl_gt 类型系统](#ctrl_gt-类型系统)
4. [数学常数模块](#数学常数模块)
5. [坐标变换模块](#坐标变换模块)
6. [线性代数模块](#线性代数模块)
7. [使用示例](#使用示例)
8. [后端选择指南](#后端选择指南)

---

## 概述

GMP 数学计算库（Math Block）为电机驱动和数字控制算法提供全面的数学计算支持。该库的核心特性是**类型抽象**：通过 `ctrl_gt` 类型，可以在不修改算法代码的情况下，在浮点、定点、双精度等多种数值后端之间切换。

### 核心特性

- ✅ **类型抽象（ctrl_gt）**: 统一的控制类型，支持多种数值后端
- ✅ **坐标变换**: Clarke、Park、SVPWM 等电机控制核心变换
- ✅ **线性代数**: 向量、矩阵、复数、四元数运算
- ✅ **高精度常数**: 统一管理的数学和物理常数
- ✅ **平台优化**: 针对不同芯片（ARM、C28x）的优化实现
- ✅ **零开销抽象**: 内联函数和宏，运行时无额外开销

### 主要头文件

```c
// 总入口：包含所有数学模块
#include <gmp_math.h>

// 或单独包含特定模块
#include <ctl/math_block/coordinate/coord_trans.h>
#include <ctl/math_block/vector_lite/vector2.h>
#include <ctl/math_block/complex_lite/complex.h>
```

---

## 模块架构

```
ctl/math_block/
├── gmp_math.h              # 总入口头文件
│
├── ctrl_gt/                # ctrl_gt 类型的不同实现
│   ├── float_macros.h     # 单精度浮点（默认）
│   ├── double_macros.h    # 双精度浮点
│   ├── iqmath_macros.h    # TI IQmath 定点库
│   ├── arm_cmsis_macros.h # ARM CMSIS-DSP 定点库
│   ├── qfp_float_macros.h # QFP 软浮点库（无FPU平台）
│   └── cla_macros.h       # TI C2000 CLA 加速器
│
├── const/                  # 数学常数
│   ├── math_ctrl_const.h  # 控制用常数（π、√3等）
│   └── math_param_const.h # 物理参数常数
│
├── coordinate/             # 坐标变换
│   └── coord_trans.h      # Clarke/Park/SVPWM 变换
│
├── vector_lite/            # 向量运算
│   ├── vector2.h          # 2D 向量
│   ├── vector3.h          # 3D 向量
│   └── vector4.h          # 4D 向量
│
├── matrix_lite/            # 矩阵运算
│   ├── matrix2.h          # 2×2 矩阵
│   ├── matrix3.h          # 3×3 矩阵
│   └── matrix4.h          # 4×4 矩阵
│
└── complex_lite/           # 复数与四元数
    ├── complex.h          # 复数运算
    └── quaternion.h       # 四元数运算
```

---

## ctrl_gt 类型系统

### 核心思想

`ctrl_gt` 是一个**抽象的控制数据类型**，其底层实现可以在编译时选择。这使得同一套算法代码可以在不同的数值后端上运行。

```c
// 算法代码中统一使用 ctrl_gt
ctrl_gt voltage = float2ctrl(48.0);
ctrl_gt current = float2ctrl(10.5);
ctrl_gt power = ctl_mul(voltage, current);  // 电压 × 电流 = 功率
```

### 支持的后端

| 后端类型 | 宏定义 | 精度/范围 | 适用平台 | 性能 |
|---------|--------|----------|---------|------|
| **单精度浮点** | `USING_FLOAT_FPU` | 32位，±1.17×10⁻³⁸ ~ ±3.40×10³⁸ | 带FPU的MCU（默认） | ⭐⭐⭐⭐⭐ |
| **双精度浮点** | `USING_DOUBLE_FPU` | 64位，±2.23×10⁻³⁰⁸ ~ ±1.80×10³⁰⁸ | 高性能平台 | ⭐⭐⭐⭐ |
| **TI IQmath** | `USING_FIXED_TI_IQ_LIBRARY` | 定点，可配置Q格式 | TI C2000（无FPU） | ⭐⭐⭐⭐⭐ |
| **ARM CMSIS-DSP** | `USING_FIXED_ARM_CMSIS_Q_LIBRARY` | 定点Q15/Q31 | ARM Cortex-M（无FPU） | ⭐⭐⭐⭐ |
| **QFP 软浮点** | `USING_QFPLIB_FLOAT` | 软件模拟浮点 | 无FPU的低端MCU | ⭐⭐⭐ |
| **TI CLA** | `USING_FLOAT_CLA_LIBRARY` | 32位浮点 | TI C2000 CLA协处理器 | ⭐⭐⭐⭐⭐ |

### 后端选择

在项目配置文件（如 `csp.config.h` 或 `ctrl_settings.h`）中定义：

```c
// 选择单精度浮点（默认）
#define SPECIFY_CTRL_GT_TYPE  USING_FLOAT_FPU

// 或选择 TI IQmath 定点
// #define SPECIFY_CTRL_GT_TYPE  USING_FIXED_TI_IQ_LIBRARY
// #define GLOBAL_IQ  24  // Q24 格式

// 或选择双精度浮点
// #define SPECIFY_CTRL_GT_TYPE  USING_DOUBLE_FPU
```

### 核心 API

#### 类型转换

```c
// 字面量 → ctrl_gt
ctrl_gt val = float2ctrl(3.14);

// ctrl_gt → 标准浮点
float f = ctrl2float(val);

// 整数 → ctrl_gt
ctrl_gt i = int2ctrl(100);

// ctrl_gt → 整数（截断）
int n = ctrl2int(val);
```

#### 基本运算

```c
// 加减法（大多数平台直接支持 +/-）
ctrl_gt sum = a + b;
ctrl_gt diff = a - b;

// 乘法（某些平台需要用宏）
ctrl_gt prod = ctl_mul(a, b);

// 除法
ctrl_gt quot = ctl_div(a, b);

// 限幅
ctrl_gt limited = ctl_sat(val, max_limit, min_limit);

// 绝对值
ctrl_gt abs_val = ctl_abs(val);
```

#### 高级数学函数

```c
// 三角函数（角度单位为标幺值：1.0 = 360°）
ctrl_gt sin_val = ctl_sin(angle);
ctrl_gt cos_val = ctl_cos(angle);
ctrl_gt tan_val = ctl_tan(angle);

// 反三角函数
ctrl_gt angle = ctl_atan2(y, x);  // 返回 arctan(y/x)

// 指数和对数
ctrl_gt exp_val = ctl_exp(x);
ctrl_gt ln_val = ctl_ln(x);

// 平方根
ctrl_gt sqrt_val = ctl_sqrt(x);
```

---

## 数学常数模块

**位置**: `ctl/math_block/const/`

提供高精度的数学和物理常数，确保计算精度和一致性。

### 通用数学常数

```c
#include <ctl/math_block/const/math_ctrl_const.h>

// 圆周率
CTL_CTRL_CONST_PI           // π = 3.14159...
CTL_CTRL_CONST_2_PI         // 2π = 6.28318...
CTL_CTRL_CONST_1_OVER_2PI   // 1/(2π) = 0.15915...

// 根号相关
CTL_CTRL_CONST_SQRT_3       // √3 = 1.73205...
CTL_CTRL_CONST_1_OVER_SQRT3 // 1/√3 = 0.57735...
CTL_CTRL_CONST_SQRT_3_OVER_2// √3/2 = 0.86603...

// 基本常数
CTL_CTRL_CONST_1            // 1.0
CTL_CTRL_CONST_1_OVER_2     // 0.5
CTL_CTRL_CONST_3_OVER_2     // 1.5
```

### 坐标变换常数

```c
// Clarke 变换系数
CTL_CTRL_CONST_ABC2AB_ALPHA  // 2/3 = 0.66667
CTL_CTRL_CONST_ABC2AB_BETA   // 1/√3 = 0.57735
CTL_CTRL_CONST_ABC2AB_GAMMA  // 1/3 = 0.33333

// 反 Clarke 变换系数
CTL_CTRL_CONST_AB2ABC_ALPHA  // √3/2 = 0.86603

// 功率不变 Clarke 变换
CTL_CTRL_CONST_AB02AB_ALPHA  // 2/√3 = 1.15470
```

### 使用示例

```c
// 计算三相电压的 RMS 值
ctrl_gt v_rms = ctl_mul(v_phase, CTL_CTRL_CONST_SQRT_3);

// 角度归一化
ctrl_gt angle_normalized = ctl_mul(angle_rad, CTL_CTRL_CONST_1_OVER_2PI);
```

---

## 坐标变换模块

**位置**: `ctl/math_block/coordinate/coord_trans.h`

提供电机控制中核心的坐标系变换函数。

### 坐标系定义

```c
// 三相坐标系轴索引
enum ABC_ASIX_ENUM {
    phase_A = 0,  // A 相
    phase_B = 1,  // B 相
    phase_C = 2   // C 相
};

// 静止坐标系轴索引（αβ）
enum ALPHA_BETA_ENUM {
    phase_alpha = 0,  // α 轴
    phase_beta = 1    // β 轴
};

// 旋转坐标系轴索引（dq）
enum DQ_ASIC_ENUM {
    phase_d = 0,  // d 轴（直轴）
    phase_q = 1,  // q 轴（交轴）
    phase_0 = 2   // 0 轴（零序）
};
```

### 相量生成

```c
/**
 * @brief 根据角度生成相量（sin, cos）
 * @param angle 角度（标幺值：0~1 对应 0~2π）
 * @param[out] phasor 输出相量 [sin(θ), cos(θ)]
 */
void ctl_set_phasor_via_angle(ctrl_gt angle, ctl_vector2_t* phasor);

// 使用示例
ctl_vector2_t phasor;
ctrl_gt theta = float2ctrl(0.25);  // 90度 = π/2
ctl_set_phasor_via_angle(theta, &phasor);
// phasor.dat[0] = sin(90°) = 1.0
// phasor.dat[1] = cos(90°) = 0.0
```

### Clarke 变换（ABC → αβ0）

将三相静止坐标系转换为两相静止坐标系。

```c
/**
 * @brief ABC → αβ0 Clarke 变换
 * @param[in] abc 三相输入 [Ia, Ib, Ic]
 * @param[out] ab0 两相输出 [Iα, Iβ, I0]
 */
void ctl_clarke_abc2ab0(const ctl_vector3_t* abc, ctl_vector3_t* ab0);

// 使用示例（三相电流采样）
ctl_vector3_t i_abc = {
    .dat = {float2ctrl(10.0),   // Ia = 10A
            float2ctrl(-5.0),    // Ib = -5A
            float2ctrl(-5.0)}    // Ic = -5A
};
ctl_vector3_t i_ab0;
ctl_clarke_abc2ab0(&i_abc, &i_ab0);
// i_ab0.dat[0] = Iα
// i_ab0.dat[1] = Iβ
// i_ab0.dat[2] = I0（零序分量）
```

**变换公式**：
$$
\begin{aligned}
I_\alpha &= \frac{2}{3}(I_a - \frac{1}{2}I_b - \frac{1}{2}I_c) \\
I_\beta &= \frac{1}{\sqrt{3}}(I_b - I_c) \\
I_0 &= \frac{1}{3}(I_a + I_b + I_c)
\end{aligned}
$$

### Park 变换（αβ → dq）

将静止坐标系转换为旋转坐标系。

```c
/**
 * @brief αβ → dq Park 变换
 * @param[in] ab 静止坐标系输入 [Iα, Iβ]
 * @param[in] phasor 相量 [sin(θ), cos(θ)]
 * @param[out] dq 旋转坐标系输出 [Id, Iq]
 */
void ctl_park_ab2dq(const ctl_vector2_t* ab, 
                    const ctl_vector2_t* phasor, 
                    ctl_vector2_t* dq);

// 使用示例
ctl_vector2_t i_ab = {.dat = {iα, iβ}};
ctl_vector2_t phasor;
ctl_set_phasor_via_angle(theta_e, &phasor);  // θ_e 为电角度

ctl_vector2_t i_dq;
ctl_park_ab2dq(&i_ab, &phasor, &i_dq);
// i_dq.dat[0] = Id（励磁电流）
// i_dq.dat[1] = Iq（转矩电流）
```

**变换公式**：
$$
\begin{aligned}
I_d &= I_\alpha \cos\theta + I_\beta \sin\theta \\
I_q &= -I_\alpha \sin\theta + I_\beta \cos\theta
\end{aligned}
$$

### 反 Park 变换（dq → αβ）

```c
/**
 * @brief dq → αβ 反 Park 变换
 * @param[in] dq 旋转坐标系输入 [Vd, Vq]
 * @param[in] phasor 相量 [sin(θ), cos(θ)]
 * @param[out] ab 静止坐标系输出 [Vα, Vβ]
 */
void ctl_ipark_dq2ab(const ctl_vector2_t* dq, 
                     const ctl_vector2_t* phasor, 
                     ctl_vector2_t* ab);
```

**变换公式**：
$$
\begin{aligned}
V_\alpha &= V_d \cos\theta - V_q \sin\theta \\
V_\beta &= V_d \sin\theta + V_q \cos\theta
\end{aligned}
$$

### 反 Clarke 变换（αβ → ABC）

```c
/**
 * @brief αβ → ABC 反 Clarke 变换
 * @param[in] ab 两相输入 [Vα, Vβ]
 * @param[out] abc 三相输出 [Va, Vb, Vc]
 */
void ctl_iclarke_ab2abc(const ctl_vector2_t* ab, ctl_vector3_t* abc);
```

### SVPWM 计算

```c
/**
 * @brief 计算 SVPWM 占空比
 * @param[in] vab αβ坐标系电压指令
 * @param[in] vdc 直流母线电压
 * @param[out] svpwm 三相占空比输出
 */
void ctl_svpwm_calc(const ctl_vector2_t* vab, 
                    ctrl_gt vdc, 
                    ctl_vector3_t* svpwm);

// 使用示例
ctl_vector2_t v_ab = {.dat = {vα, vβ}};
ctrl_gt v_dc = float2ctrl(48.0);
ctl_vector3_t duty_abc;

ctl_svpwm_calc(&v_ab, v_dc, &duty_abc);
// duty_abc.dat[0] = Duty_A
// duty_abc.dat[1] = Duty_B
// duty_abc.dat[2] = Duty_C
```

---

## 线性代数模块

### 1. 向量运算

#### 2D 向量（vector2.h）

```c
#include <ctl/math_block/vector_lite/vector2.h>

// 定义向量
typedef struct {
    ctrl_gt dat[2];  // [x, y]
} ctl_vector2_t;

// 清零
ctl_vector2_t vec;
ctl_vector2_clear(&vec);

// 复制
ctl_vector2_t dup;
ctl_vector2_copy(&dup, &vec);

// 加法：result = a + b
ctl_vector2_add(&result, &a, &b);

// 减法：result = a - b
ctl_vector2_sub(&result, &a, &b);

// 数乘：result = scalar * vec
ctl_vector2_scalar_mul(&result, scalar, &vec);

// 点积：dot = a · b
ctrl_gt dot = ctl_vector2_dot(&a, &b);

// 模长
ctrl_gt magnitude = ctl_vector2_magnitude(&vec);

// 归一化
ctl_vector2_normalize(&vec);
```

#### 3D 向量（vector3.h）

```c
#include <ctl/math_block/vector_lite/vector3.h>

typedef struct {
    ctrl_gt dat[3];  // [x, y, z]
} ctl_vector3_t;

// 叉积：result = a × b
ctl_vector3_cross(&result, &a, &b);

// 其他操作与 2D 向量类似
```

### 2. 矩阵运算

#### 2×2 矩阵（matrix2.h）

```c
#include <ctl/math_block/matrix_lite/matrix2.h>

// 定义矩阵（行优先）
typedef struct {
    ctrl_gt dat[4];  // [m00, m01, m10, m11]
} ctl_matrix2_t;

// 获取元素
ctrl_gt element = ctl_matrix2_get(&mat, row, col);

// 设置元素
ctl_matrix2_set(&mat, row, col, value);

// 单位矩阵
ctl_matrix2_identity(&mat);

// 矩阵乘法：C = A × B
ctl_matrix2_mul(&C, &A, &B);

// 矩阵-向量乘法：result = mat × vec
ctl_matrix2_mul_vector(&result, &mat, &vec);

// 转置
ctl_matrix2_transpose(&result, &mat);

// 行列式
ctrl_gt det = ctl_matrix2_determinant(&mat);

// 逆矩阵
ctl_matrix2_inverse(&inv, &mat);
```

#### 3×3 和 4×4 矩阵

```c
#include <ctl/math_block/matrix_lite/matrix3.h>
#include <ctl/math_block/matrix_lite/matrix4.h>

// API 类似，支持 3×3 和 4×4 矩阵运算
```

### 3. 复数运算

```c
#include <ctl/math_block/complex_lite/complex.h>

// 定义复数
typedef struct {
    ctrl_gt real;  // 实部
    ctrl_gt imag;  // 虚部
} ctl_complex_t;

// 创建复数
ctl_complex_t z = {
    .real = float2ctrl(3.0),
    .imag = float2ctrl(4.0)
};

// 加法：result = a + b
ctl_complex_t result = ctl_cadd(a, b);

// 减法：result = a - b
result = ctl_csub(a, b);

// 乘法：result = a × b
result = ctl_cmul(a, b);

// 除法：result = a / b
result = ctl_cdiv(a, b);

// 共轭：result = a*
result = ctl_cconj(a);

// 模长：|z|
ctrl_gt magnitude = ctl_cabs(z);

// 幅角
ctrl_gt phase = ctl_carg(z);

// 极坐标形式：z = r·e^(jθ)
result = ctl_cpolar(radius, angle);
```

### 4. 四元数运算

```c
#include <ctl/math_block/complex_lite/quaternion.h>

// 定义四元数
typedef struct {
    ctrl_gt w;  // 实部
    ctrl_gt x;  // i 分量
    ctrl_gt y;  // j 分量
    ctrl_gt z;  // k 分量
} ctl_quaternion_t;

// 四元数乘法
ctl_quaternion_t result = ctl_qmul(q1, q2);

// 共轭
result = ctl_qconj(q);

// 归一化（用于旋转）
ctl_qnormalize(&q);

// 从欧拉角创建四元数
result = ctl_qfrom_euler(roll, pitch, yaw);
```

---

## 使用示例

### 场景1：PMSM FOC 电流控制

```c
#include <gmp_math.h>

void foc_current_control() {
    // 1. 采样三相电流
    ctl_vector3_t i_abc = {
        .dat = {adc_ia, adc_ib, adc_ic}
    };
    
    // 2. Clarke 变换：ABC → αβ
    ctl_vector3_t i_ab0;
    ctl_clarke_abc2ab0(&i_abc, &i_ab0);
    
    // 3. Park 变换：αβ → dq
    ctl_vector2_t i_ab = {
        .dat = {i_ab0.dat[0], i_ab0.dat[1]}
    };
    
    ctl_vector2_t phasor;
    ctl_set_phasor_via_angle(theta_elec, &phasor);
    
    ctl_vector2_t i_dq;
    ctl_park_ab2dq(&i_ab, &phasor, &i_dq);
    
    // 4. PI 控制（Id、Iq）
    ctrl_gt i_d = i_dq.dat[0];
    ctrl_gt i_q = i_dq.dat[1];
    
    ctrl_gt v_d = pi_control_d(id_ref - i_d);
    ctrl_gt v_q = pi_control_q(iq_ref - i_q);
    
    // 5. 反 Park 变换：dq → αβ
    ctl_vector2_t v_dq = {.dat = {v_d, v_q}};
    ctl_vector2_t v_ab;
    ctl_ipark_dq2ab(&v_dq, &phasor, &v_ab);
    
    // 6. SVPWM 调制
    ctl_vector3_t duty_abc;
    ctl_svpwm_calc(&v_ab, v_dc, &duty_abc);
    
    // 7. 更新 PWM
    set_pwm_duty_a(duty_abc.dat[0]);
    set_pwm_duty_b(duty_abc.dat[1]);
    set_pwm_duty_c(duty_abc.dat[2]);
}
```

### 场景2：电网电压相位检测（PLL）

```c
#include <gmp_math.h>

void grid_pll_update() {
    // 采样电网电压
    ctl_vector3_t v_abc = {
        .dat = {adc_va, adc_vb, adc_vc}
    };
    
    // Clarke 变换
    ctl_vector3_t v_ab0;
    ctl_clarke_abc2ab0(&v_abc, &v_ab0);
    
    // Park 变换（使用估计的角度）
    ctl_vector2_t v_ab = {
        .dat = {v_ab0.dat[0], v_ab0.dat[1]}
    };
    
    ctl_vector2_t phasor;
    ctl_set_phasor_via_angle(theta_pll, &phasor);
    
    ctl_vector2_t v_dq;
    ctl_park_ab2dq(&v_ab, &phasor, &v_dq);
    
    // PLL 控制（使 Vq → 0）
    ctrl_gt error = v_dq.dat[1];  // Vq 误差
    
    // PI 控制器输出频率偏差
    ctrl_gt delta_freq = pi_control_pll(error);
    
    // 积分得到角度
    theta_pll += ctl_mul(delta_freq, dt);
    
    // 角度归一化
    if (theta_pll > CTL_CTRL_CONST_1) {
        theta_pll -= CTL_CTRL_CONST_1;
    }
}
```

### 场景3：姿态解算（四元数）

```c
#include <gmp_math.h>

void attitude_update(ctrl_gt gx, ctrl_gt gy, ctrl_gt gz, ctrl_gt dt) {
    static ctl_quaternion_t q = {
        .w = float2ctrl(1.0),
        .x = 0, .y = 0, .z = 0
    };
    
    // 从陀螺仪角速度创建旋转四元数
    ctrl_gt half_dt = ctl_mul(dt, float2ctrl(0.5));
    
    ctl_quaternion_t dq = {
        .w = 0,
        .x = ctl_mul(gx, half_dt),
        .y = ctl_mul(gy, half_dt),
        .z = ctl_mul(gz, half_dt)
    };
    
    // 四元数微分方程
    ctl_quaternion_t q_dot = ctl_qmul(q, dq);
    
    // 更新姿态
    q.w += q_dot.w;
    q.x += q_dot.x;
    q.y += q_dot.y;
    q.z += q_dot.z;
    
    // 归一化
    ctl_qnormalize(&q);
    
    // 转换为欧拉角（如果需要）
    ctrl_gt roll, pitch, yaw;
    quaternion_to_euler(&q, &roll, &pitch, &yaw);
}
```

---

## 后端选择指南

### 选择决策树

```
是否有硬件 FPU？
├─ 是 → 使用 USING_FLOAT_FPU（推荐）
│      或 USING_DOUBLE_FPU（需要高精度时）
│
└─ 否 → 平台是？
       ├─ TI C2000 → USING_FIXED_TI_IQ_LIBRARY
       │             或 USING_FLOAT_CLA_LIBRARY（如有CLA）
       │
       ├─ ARM Cortex-M → USING_FIXED_ARM_CMSIS_Q_LIBRARY
       │                 或 USING_QFPLIB_FLOAT（软浮点）
       │
       └─ 其他 → USING_QFPLIB_FLOAT
```

### 性能对比

| 后端 | 加法/减法 | 乘法 | 除法 | 三角函数 | 内存占用 |
|------|----------|------|------|---------|---------|
| **Float (FPU)** | 1 周期 | 1-3 周期 | 7-14 周期 | 查表+插值 | 4 字节/变量 |
| **Double (FPU)** | 1 周期 | 1-3 周期 | 7-14 周期 | 查表+插值 | 8 字节/变量 |
| **IQmath** | 1-2 周期 | 15-30 周期 | 50-100 周期 | 查表 | 4 字节/变量 |
| **CMSIS-DSP** | 1-2 周期 | 10-20 周期 | 40-80 周期 | 查表 | 2-4 字节/变量 |
| **QFP Float** | 20-50 周期 | 50-100 周期 | 100-200 周期 | 软件模拟 | 4 字节/变量 |

### 推荐配置

**STM32F4/G4/H7 系列（有 FPU）**
```c
#define SPECIFY_CTRL_GT_TYPE  USING_FLOAT_FPU
```

**STM32F0/L0 系列（无 FPU）**
```c
#define SPECIFY_CTRL_GT_TYPE  USING_FIXED_ARM_CMSIS_Q_LIBRARY
// 或
#define SPECIFY_CTRL_GT_TYPE  USING_QFPLIB_FLOAT
```

**TI C2000 F28069（无 FPU）**
```c
#define SPECIFY_CTRL_GT_TYPE  USING_FIXED_TI_IQ_LIBRARY
#define GLOBAL_IQ  24  // Q24 格式
```

**TI C2000 F28379D（有 FPU + CLA）**
```c
#define SPECIFY_CTRL_GT_TYPE  USING_FLOAT_FPU
// CLA 代码自动使用 USING_FLOAT_CLA_LIBRARY
```

---

## 常见问题

### Q1: float 和 double 如何选择？

**A:**
- **float**（默认推荐）：
  - 精度：7 位有效数字（对电机控制足够）
  - 速度快（32 位 FPU 优化）
  - 内存占用小
  
- **double**：
  - 精度：15 位有效数字
  - 速度较慢（大多数 MCU 的 FPU 针对 float 优化）
  - 内存占用大（8字节）
  - 仅在需要极高精度时使用（如科学计算、天文导航）

### Q2: 定点数和浮点数如何选择？

**A:**
- **浮点（FPU 平台）**：
  - ✅ 编程简单，无需考虑定标
  - ✅ 动态范围大
  - ✅ 精度高
  - ❌ 需要硬件 FPU 支持
  
- **定点（无 FPU 平台）**：
  - ✅ 在无 FPU 的 MCU 上速度快
  - ✅ 功耗低
  - ❌ 需要手动管理定标（Q 格式）
  - ❌ 容易溢出

**建议**：
- 有 FPU → 优先使用浮点
- 无 FPU 但性能要求高 → 使用定点（IQmath/CMSIS-DSP）
- 无 FPU 且代码简洁优先 → 使用软浮点（QFP）

### Q3: 坐标变换中的角度单位是什么？

**A:**
所有角度使用**标幺值（per-unit）**：
- 0.0 = 0°
- 0.25 = 90° (π/2)
- 0.5 = 180° (π)
- 1.0 = 360° (2π)

转换公式：
```c
// 弧度 → 标幺值
ctrl_gt pu_angle = ctl_mul(radian, CTL_CTRL_CONST_1_OVER_2PI);

// 标幺值 → 弧度
ctrl_gt radian = ctl_mul(pu_angle, CTL_CTRL_CONST_2_PI);
```

### Q4: 三角函数查表精度如何？

**A:**
- 查表 + 线性插值，精度约 0.01%
- 对电机控制足够（电流谐波 < 1%）
- 如需更高精度，可使用 `math.h` 中的标准函数（需要 FPU）

### Q5: 矩阵运算中如何避免数值不稳定？

**A:**
- 使用条件数检查（`ctl_matrix_condition_number()`）
- 避免接近奇异的矩阵（行列式接近 0）
- 使用 SVD 分解（如有）
- 增加数值范围（考虑使用 double）

---

## 调试技巧

### 1. 类型转换调试

```c
// 打印 ctrl_gt 值
ctrl_gt val = float2ctrl(3.14);
gmp_base_print("Value: %f\r\n", ctrl2float(val));
```

### 2. 坐标变换验证

```c
// 验证 Clarke + 反 Clarke 是否可逆
ctl_vector3_t abc_in = {.dat = {1.0, -0.5, -0.5}};
ctl_vector3_t ab0;
ctl_vector3_t abc_out;

ctl_clarke_abc2ab0(&abc_in, &ab0);
ctl_iclarke_ab2abc(&ab0, &abc_out);

// abc_out 应该等于 abc_in
```

### 3. 数值精度测试

```c
// 测试定点数精度
ctrl_gt a = float2ctrl(0.1);
ctrl_gt sum = 0;
for (int i = 0; i < 10; i++) {
    sum = sum + a;
}
// sum 应该接近 1.0，检查累积误差
gmp_base_print("Sum = %f (expected 1.0)\r\n", ctrl2float(sum));
```

---

## 总结

GMP 数学计算库提供了电机控制和数字电源算法所需的全部数学工具：

| 模块 | 功能 | 典型应用 |
|------|------|---------|
| **ctrl_gt** | 类型抽象 | 所有控制算法 |
| **常数** | 高精度常数 | 坐标变换、功率计算 |
| **坐标变换** | Clarke/Park/SVPWM | PMSM/ACM FOC |
| **向量/矩阵** | 线性代数 | 状态空间控制、卡尔曼滤波 |
| **复数** | 复数运算 | PLL、频域分析 |
| **四元数** | 姿态表示 | 飞行器控制、机器人 |

**最佳实践：**
1. 根据平台选择合适的 `ctrl_gt` 后端
2. 使用标幺值（per-unit）简化计算
3. 利用预定义常数确保精度
4. 充分测试坐标变换的可逆性
5. 注意数值稳定性和溢出问题

---

**版本历史：**
- v1.0 (2026-01-27): 初始版本

**参考资料：**
- 各模块头文件中的 Doxygen 注释
- `ctl/component/` - 使用这些数学函数的控制器示例
