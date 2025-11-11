#include "pch.h"

#include <ctl/component/intrinsic/continuous/continuous_pid.h>

// 使用“测试夹具”(Test Fixture) 是一个好习惯，它可以帮助我们
// 在多个测试之间共享设置代码和数据，避免代码重复。
class PID_Parallel_Test : public ::testing::Test
{
  protected:
    // 在每个测试用例运行前，SetUp() 会被调用
    void SetUp() override
    {
        // 初始化控制器到一个已知的、干净的状态
        ctl_init_pid_par(&pid, Kp, Ti, Td, fs);
        ctl_set_pid_limit(&pid, 1.0f, -1.0f);
        ctl_clear_pid(&pid);
    }

    // 在这里声明测试中会用到的变量
    ctl_pid_t pid;
    const parameter_gt Kp = 1.0f;
    const parameter_gt Ti = 0.05f;
    const parameter_gt Td = 0.01f;
    const parameter_gt fs = 100.0f;
};

// --- 测试用例 1: 验证初始化函数 ---
// TEST_F 使用我们上面定义的 PID_Parallel_Test 夹具
TEST_F(PID_Parallel_Test, Initialization_CalculatesGainsCorrectly)
{
    // Arrange: 计算预期的内部增益
    const parameter_gt T = 1.0f / fs;
    const float expected_ki = Kp * T / Ti; // 10.0 * 0.01 / 0.5 = 0.2
    const float expected_kd = Kp * Td / T; // 10.0 * 0.001 / 0.01 = 1.0

    printf("PID prameters: %f, %f, %f.\r\n", pid.kp, pid.ki, pid.kd);

    // Act: SetUp() 函数已经替我们执行了初始化

    // Assert: 验证计算出的增益是否正确
    // 使用 EXPECT_FLOAT_EQ 来比较浮点数
    EXPECT_FLOAT_EQ(pid.kp, Kp);
    EXPECT_FLOAT_EQ(pid.ki, expected_ki);
    EXPECT_FLOAT_EQ(pid.kd, expected_kd);
}

// --- 测试用例 2: 验证状态清除函数 ---
TEST_F(PID_Parallel_Test, Clear_ResetsInternalStates)
{
    // Arrange: 手动弄脏状态
    pid.sn = 10.0f;
    pid.dn = 5.0f;
    pid.out = 20.0f;

    // Act: 调用清除函数
    ctl_clear_pid(&pid);

    // Assert: 验证所有状态是否都归零
    EXPECT_FLOAT_EQ(pid.sn, 0.0f);
    EXPECT_FLOAT_EQ(pid.dn, 0.0f);
    EXPECT_FLOAT_EQ(pid.out, 0.0f);
}

// --- 测试用例 3: 验证单步执行（正常情况） ---
TEST_F(PID_Parallel_Test, Step_FirstStepOutputIsCorrect)
{
    // Arrange
    ctrl_gt input_error = 0.2f;

    // 手动计算预期结果:
    // P_term = kp * input = 1 * 0.2 = 0.2
    // I_term_sum = sn_prev + ki * input = 0 + 0.2 * 0.2 = 0.04
    // D_term = kd * (input - dn_prev) = 1.0 * (0.2 - 0) = 0.2
    // Output = P + I + D = 0.2 + 0.04 + 0.2 = 0.44
    const ctrl_gt expected_output = 0.44f;
    const ctrl_gt expected_integrator_sum = 0.04f;

    // Act
    ctrl_gt actual_output = ctl_step_pid_par(&pid, input_error);

    // Assert
    EXPECT_FLOAT_EQ(actual_output, expected_output);
    EXPECT_FLOAT_EQ(pid.sn, expected_integrator_sum); // 验证积分状态
    EXPECT_FLOAT_EQ(pid.dn, input_error);             // 验证微分前一状态

    // Step to next position
    actual_output = ctl_step_pid_par(&pid, input_error);

    // 手动计算预期结果:
    // P_term = kp * input = 1 * 0.2 = 0.2
    // I_term_sum = sn_prev + ki * input =0.04 + 0.2 * 0.2 = 0.08
    // D_term = kd * (input - dn_prev) = 1.0 * (0.2 - 0.2) = 0
    // Output = P + I + D = 0.2 + 0.04 + 0.2 = 0.44
    const ctrl_gt expected_output2 = 0.28f;
    const ctrl_gt expected_integrator_sum2 = 0.08f;

    // Assert
    EXPECT_FLOAT_EQ(actual_output, expected_output2);
    EXPECT_FLOAT_EQ(pid.sn, expected_integrator_sum2); // 验证积分状态
    EXPECT_FLOAT_EQ(pid.dn, input_error);             // 验证微分前一状态

    // Step to next position
    input_error = 0.5f;
    actual_output = ctl_step_pid_par(&pid, input_error);

    // 手动计算预期结果:
    // P_term = kp * input = 1 * 0.5 = 0.5
    // I_term_sum = sn_prev + ki * input =0.08 + 0.2 * 0.5 = 0.18
    // D_term = kd * (input - dn_prev) = 1.0 * (0.5 - 0.2) = 0.3
    // Output = P + I + D = 0.5 + 0.18 + 0.3 = 0.98
    const ctrl_gt expected_output3 = 0.98f;
    const ctrl_gt expected_integrator_sum3 = 0.18f;

    // Assert
    EXPECT_FLOAT_EQ(actual_output, expected_output3);
    EXPECT_FLOAT_EQ(pid.sn, expected_integrator_sum3); // 验证积分状态
    EXPECT_FLOAT_EQ(pid.dn, input_error);              // 验证微分前一状态

    // step to next position
    actual_output = ctl_step_pid_par(&pid, input_error);

    // 手动计算预期结果:
    // P_term = kp * input = 1 * 0.5 = 0.5
    // I_term_sum = sn_prev + ki * input =0.18 + 0.2 * 0.5 = 0.28
    // D_term = kd * (input - dn_prev) = 1.0 * (0.5 - 0.5) = 0
    // Output = P + I + D = 0.5 + 0.28 + 0 = 0.78
    const ctrl_gt expected_output4 = 0.78f;
    const ctrl_gt expected_integrator_sum4 = 0.28f;

    // Assert
    EXPECT_FLOAT_EQ(actual_output, expected_output4);
    EXPECT_FLOAT_EQ(pid.sn, expected_integrator_sum4); // 验证积分状态
    EXPECT_FLOAT_EQ(pid.dn, input_error);              // 验证微分前一状态
}

// --- 测试用例 4: 验证输出上限饱和 ---
TEST_F(PID_Parallel_Test, Step_OutputSaturatesAtMaxLimit)
{
    // Arrange: 使用一个会产生巨大正向输出的输入
    const ctrl_gt large_input_error = 10.0f;

    // Act
    ctrl_gt actual_output = ctl_step_pid_par(&pid, large_input_error);

    // Assert: 输出应该被限制在 out_max
    EXPECT_FLOAT_EQ(actual_output, pid.out_max);
}

// --- 测试用例 5: 验证输出下限饱和 ---
TEST_F(PID_Parallel_Test, Step_OutputSaturatesAtMinLimit)
{
    // Arrange: 使用一个会产生巨大负向输出的输入
    const ctrl_gt large_negative_error = -10.0f;

    // Act
    ctrl_gt actual_output = ctl_step_pid_par(&pid, large_negative_error);

    // Assert: 输出应该被限制在 out_min
    EXPECT_FLOAT_EQ(actual_output, pid.out_min);
}

// --- 测试用例 6: 验证积分器上限饱和 ---
TEST_F(PID_Parallel_Test, Step_IntegratorSaturatesAtMaxLimit)
{
    // Arrange: 多次输入正误差，使积分累加
    ctl_step_pid_par(&pid, 5.0f); // sn = 1.0
    ctl_step_pid_par(&pid, 5.0f); // sn = 1.0 + 1.0 = 2.0
    // ...
    // 为了直接测试，我们手动设置一个接近饱和的积分值
    pid.sn = 49.5f;

    // Act: 再执行一步，理论上 sn 会变成 49.5 + 0.2*5.0 = 50.5
    ctl_step_pid_par(&pid, 5.0f);

    // Assert: 积分值 sn 应该被限制在 integral_max
    EXPECT_FLOAT_EQ(pid.sn, pid.integral_max);
}
