#include "pch.h"

#include <ctl/component/intrinsic/continuous/continuous_pid.h>

// ʹ�á����Լоߡ�(Test Fixture) ��һ����ϰ�ߣ������԰�������
// �ڶ������֮�乲�����ô�������ݣ���������ظ���
class PID_Parallel_Test : public ::testing::Test
{
  protected:
    // ��ÿ��������������ǰ��SetUp() �ᱻ����
    void SetUp() override
    {
        // ��ʼ����������һ����֪�ġ��ɾ���״̬
        ctl_init_pid_par(&pid, Kp, Ti, Td, fs);
        ctl_set_pid_limit(&pid, 1.0f, -1.0f);
        ctl_clear_pid(&pid);
    }

    // ���������������л��õ��ı���
    ctl_pid_t pid;
    const parameter_gt Kp = 1.0f;
    const parameter_gt Ti = 0.05f;
    const parameter_gt Td = 0.01f;
    const parameter_gt fs = 100.0f;
};

// --- �������� 1: ��֤��ʼ������ ---
// TEST_F ʹ���������涨��� PID_Parallel_Test �о�
TEST_F(PID_Parallel_Test, Initialization_CalculatesGainsCorrectly)
{
    // Arrange: ����Ԥ�ڵ��ڲ�����
    const parameter_gt T = 1.0f / fs;
    const float expected_ki = Kp * T / Ti; // 10.0 * 0.01 / 0.5 = 0.2
    const float expected_kd = Kp * Td / T; // 10.0 * 0.001 / 0.01 = 1.0

    printf("PID prameters: %f, %f, %f.\r\n", pid.kp, pid.ki, pid.kd);

    // Act: SetUp() �����Ѿ�������ִ���˳�ʼ��

    // Assert: ��֤������������Ƿ���ȷ
    // ʹ�� EXPECT_FLOAT_EQ ���Ƚϸ�����
    EXPECT_FLOAT_EQ(pid.kp, Kp);
    EXPECT_FLOAT_EQ(pid.ki, expected_ki);
    EXPECT_FLOAT_EQ(pid.kd, expected_kd);
}

// --- �������� 2: ��֤״̬������� ---
TEST_F(PID_Parallel_Test, Clear_ResetsInternalStates)
{
    // Arrange: �ֶ�Ū��״̬
    pid.sn = 10.0f;
    pid.dn = 5.0f;
    pid.out = 20.0f;

    // Act: �����������
    ctl_clear_pid(&pid);

    // Assert: ��֤����״̬�Ƿ񶼹���
    EXPECT_FLOAT_EQ(pid.sn, 0.0f);
    EXPECT_FLOAT_EQ(pid.dn, 0.0f);
    EXPECT_FLOAT_EQ(pid.out, 0.0f);
}

// --- �������� 3: ��֤����ִ�У���������� ---
TEST_F(PID_Parallel_Test, Step_FirstStepOutputIsCorrect)
{
    // Arrange
    ctrl_gt input_error = 0.2f;

    // �ֶ�����Ԥ�ڽ��:
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
    EXPECT_FLOAT_EQ(pid.sn, expected_integrator_sum); // ��֤����״̬
    EXPECT_FLOAT_EQ(pid.dn, input_error);             // ��֤΢��ǰһ״̬

    // Step to next position
    actual_output = ctl_step_pid_par(&pid, input_error);

    // �ֶ�����Ԥ�ڽ��:
    // P_term = kp * input = 1 * 0.2 = 0.2
    // I_term_sum = sn_prev + ki * input =0.04 + 0.2 * 0.2 = 0.08
    // D_term = kd * (input - dn_prev) = 1.0 * (0.2 - 0.2) = 0
    // Output = P + I + D = 0.2 + 0.04 + 0.2 = 0.44
    const ctrl_gt expected_output2 = 0.28f;
    const ctrl_gt expected_integrator_sum2 = 0.08f;

    // Assert
    EXPECT_FLOAT_EQ(actual_output, expected_output2);
    EXPECT_FLOAT_EQ(pid.sn, expected_integrator_sum2); // ��֤����״̬
    EXPECT_FLOAT_EQ(pid.dn, input_error);             // ��֤΢��ǰһ״̬

    // Step to next position
    input_error = 0.5f;
    actual_output = ctl_step_pid_par(&pid, input_error);

    // �ֶ�����Ԥ�ڽ��:
    // P_term = kp * input = 1 * 0.5 = 0.5
    // I_term_sum = sn_prev + ki * input =0.08 + 0.2 * 0.5 = 0.18
    // D_term = kd * (input - dn_prev) = 1.0 * (0.5 - 0.2) = 0.3
    // Output = P + I + D = 0.5 + 0.18 + 0.3 = 0.98
    const ctrl_gt expected_output3 = 0.98f;
    const ctrl_gt expected_integrator_sum3 = 0.18f;

    // Assert
    EXPECT_FLOAT_EQ(actual_output, expected_output3);
    EXPECT_FLOAT_EQ(pid.sn, expected_integrator_sum3); // ��֤����״̬
    EXPECT_FLOAT_EQ(pid.dn, input_error);              // ��֤΢��ǰһ״̬

    // step to next position
    actual_output = ctl_step_pid_par(&pid, input_error);

    // �ֶ�����Ԥ�ڽ��:
    // P_term = kp * input = 1 * 0.5 = 0.5
    // I_term_sum = sn_prev + ki * input =0.18 + 0.2 * 0.5 = 0.28
    // D_term = kd * (input - dn_prev) = 1.0 * (0.5 - 0.5) = 0
    // Output = P + I + D = 0.5 + 0.28 + 0 = 0.78
    const ctrl_gt expected_output4 = 0.78f;
    const ctrl_gt expected_integrator_sum4 = 0.28f;

    // Assert
    EXPECT_FLOAT_EQ(actual_output, expected_output4);
    EXPECT_FLOAT_EQ(pid.sn, expected_integrator_sum4); // ��֤����״̬
    EXPECT_FLOAT_EQ(pid.dn, input_error);              // ��֤΢��ǰһ״̬
}

// --- �������� 4: ��֤������ޱ��� ---
TEST_F(PID_Parallel_Test, Step_OutputSaturatesAtMaxLimit)
{
    // Arrange: ʹ��һ��������޴��������������
    const ctrl_gt large_input_error = 10.0f;

    // Act
    ctrl_gt actual_output = ctl_step_pid_par(&pid, large_input_error);

    // Assert: ���Ӧ�ñ������� out_max
    EXPECT_FLOAT_EQ(actual_output, pid.out_max);
}

// --- �������� 5: ��֤������ޱ��� ---
TEST_F(PID_Parallel_Test, Step_OutputSaturatesAtMinLimit)
{
    // Arrange: ʹ��һ��������޴������������
    const ctrl_gt large_negative_error = -10.0f;

    // Act
    ctrl_gt actual_output = ctl_step_pid_par(&pid, large_negative_error);

    // Assert: ���Ӧ�ñ������� out_min
    EXPECT_FLOAT_EQ(actual_output, pid.out_min);
}

// --- �������� 6: ��֤���������ޱ��� ---
TEST_F(PID_Parallel_Test, Step_IntegratorSaturatesAtMaxLimit)
{
    // Arrange: �����������ʹ�����ۼ�
    ctl_step_pid_par(&pid, 5.0f); // sn = 1.0
    ctl_step_pid_par(&pid, 5.0f); // sn = 1.0 + 1.0 = 2.0
    // ...
    // Ϊ��ֱ�Ӳ��ԣ������ֶ�����һ���ӽ����͵Ļ���ֵ
    pid.sn = 49.5f;

    // Act: ��ִ��һ���������� sn ���� 49.5 + 0.2*5.0 = 50.5
    ctl_step_pid_par(&pid, 5.0f);

    // Assert: ����ֵ sn Ӧ�ñ������� integral_max
    EXPECT_FLOAT_EQ(pid.sn, pid.integral_max);
}
