# CTL���ʹ��ָ�� (CTL Framework Quick Start)

CTL�ⲻ���ṩ�˷ḻ�ĵ�������㷨ģ�飬���ṩ��һ����Ϊ **CTL-Nano** ���ϲ�Ӧ�ÿ�ܣ����ڹ���������������������ں�������ȡ�����������ǳɹ�ʹ�ñ���Ĺؼ���

�ÿ����Ҫ�� `ctl_nano.h` �� `ctl_dispatch.h` �����ļ����壬�����˼���� **��״̬������** �� **����ʱ���ȡ�**��

### 1. ���ĸ��`ctl_object_nano_t`

�� `ctl_nano.h` �ж���ĺ��Ľṹ�� `ctl_object_nano_t` ����������ϵͳ�����ࡣ�����԰��������һ�������ذ塱���������ˣ�

- **ϵͳ״̬�� (`state_machine`)**: ������������ϵ絽�����ٵ����ϵ�ÿһ����
- **ʱ��� (`isr_tick`, `mainloop_tick`)**: ���ٸ�Ƶ�͵�Ƶ�����ִ�н��ġ�
- **ȫ�־�� (`ctl_nano_handle`)**: һ��ȫ��ָ�룬��ϵͳ���κβ��ֶ��ܷ��ʵ���������ذ塱��

### 2. �������ڣ�״̬�� (`ctl_nano_state_machine`)

������ӵ��һ����ȷ���������ڣ���״̬���ϸ����ȷ��ϵͳ��ȫ����������У�

1. **`CTL_SM_PENDING` (�ȴ�)**: �ϵ��ĳ�ʼ״̬��һ�ж�����ֹ��
2. **`CTL_SM_CALIBRATE` (У׼)**: �ڴ�״̬��ִ��Ӳ���Լ죬��ADC���У׼��
3. **`CTL_SM_READY` (����)**: У׼��ɣ��ȴ�����ָ�
4. **`CTL_SM_RUNUP` (����)**: ִ�е�����������������ʶ�򿪻�Ԥ��λ��
5. **`CTL_SM_ONLINE` (��������)**: ����������ģʽ����������ջ����С�
6. **`CTL_SM_FAULT` (����)**: ��⵽���ش�������ͣ�����ȴ�����

### 3. ������ȣ���Ƶ���Ƶ����

CTL��ܽ����������Ϊ���ࣺ

- **��Ƶ���� (ISR��ִ��)**: �� `ctl_dispatch.h` �У�`gmp_base_ctl_step()` ������Ҫ��**��ʱ���жϷ������ (ISR)** �е��õ�Ψһ��ڡ�������ִ�ж�ʵʱ��Ҫ����ߵ������磺
  - **���� (`ctl_fmif_input_stage_routine`)**: ��ȡADC������������λ�õȡ�
  - **���ļ��� (`ctl_fmif_core_stage_routine`)**: ִ��FOC��SMO�Ⱥ����㷨��
  - **��� (`ctl_fmif_output_stage_routine`)**: ����PWMռ�ձȡ�
- **��Ƶ���� (��ѭ����ִ��)**: �� `ctl_nano.h` �У�`ctl_fm_state_dispatch()` ������Ҫ��**��ѭ�� (`while(1)`)** �е��õĺ�����������ִ��ʵʱ��Ҫ�󲻸ߵ����񣬰�����
  - **״̬������**: ���ݵ�ǰ״̬�����ö�Ӧ�Ĵ��������� `ctl_fmif_sm_online_routine`����
  - **��� (`ctl_fmif_monitor_routine`)**: ������λ�����͵������ݡ�
  - **��ȫ��� (`ctl_fmif_security_routine`)**: ����������ѹ�ȹ��ϡ�

### 4. ���ʹ�ã��û�������

Ҫ��CTL��ܼ��ɵ�������Ŀ�У�����Ҫ�������£�

1. **����ȫ�־��**: ������һ�� `.c` �ļ��У����岢��ʼ�� `ctl_object_nano_t` �ṹ���ȫ��ָ�� `ctl_nano_handle`��

   ```
   ctl_object_nano_t g_my_controller_obj;
   ctl_object_nano_t *ctl_nano_handle = &g_my_controller_obj;
   ```

2. **ʵ��`ctl_fmif_\*`�ص�����**: ����Ѿ�����˹Ǽܣ�����Ҫ�������ľ���Ӳ����Ӧ���߼���ȥʵ�� `ctl_nano.h` ����������Щ `ctl_fmif_*` ���������磬�� `ctl_fmif_input_stage_routine` �б�д��ȡ��MCU��ADC�Ĵ����Ĵ��룻�� `ctl_fmif_sm_online_routine` �б�д�����û��������л��������ģʽ���߼���

ͨ�����ַ�ʽ��CTL��ܽ����ӵ�ʵʱ���Ⱥ�״̬���������ľ���Ӧ�ô��������ط��뿪�����������Ը�רע��ʵ�ֺ��ĵĿ����㷨��

### 5. ��ģʽ��ֱ�ӵ��� (Simplified Mode: Direct Dispatch)

���������Ŀ����Ҫ `CTL-Nano` �ṩ�ĸ���״̬������������ѡ��һ�ָ�ֱ�ӡ����򵥵ĵ���ģʽ��

**ǰ������**: �����Ĺ��������У�**��Ҫ**���� `SPECIFY_ENABLE_CTL_FRAMEWORK_NANO` �ꡣ

**�����߼�**: ������ģʽ�£�������ISR�е��� `gmp_base_ctl_step()` ʱ����������ִ�� `Nano` ��ܵ��߼������ǻ����ε����������������**ȫ�ֻص�����**��

1. `ctl_input_callback()`
2. `ctl_dispatch()`
3. `ctl_output_callback()`

**���ʹ�ã��û�������** ����Ҫ�����Ĵ�����ʵ�������������������������������طֵ��������׶��У�

1. **ʵ�� `ctl_input_callback()`**:
   - **����**: �������е�����**����**��**����**��
   - **����**: �������д��ȡADC����������������������Ӳ������Ĵ��룬��������ֵ���µ����Ķ���������ṹ���С�
2. **ʵ�� `ctl_dispatch()`**:
   - **����**: ������Ŀ����㷨��**����**��
   - **����**: �����������ѡ��Ķ��������ģ��� `step` ���������� `ctl_step_pmsm_ctrl()`�����������ִ��FOC���㲢��������յ�PWMռ�ձȡ�
3. **ʵ�� `ctl_output_callback()`**:
   - **����**: �����������ݵ�**���**��**ִ��**��
   - **����**: �������д�� `step` �����������PWMռ�ձȸ��µ�MCU�Ķ�ʱ���ȽϼĴ����Ĵ��롣

**ʾ������**: �����жϷ������������ǳ���ࣺ

```
// ������һ�� .c �ļ���ʵ���������ص�
void ctl_input_callback(void) {
    // ��ȡADC�ͱ�����ֵ...
    update_adc_inputs(&g_my_pmsm_controller.mtr_interface);
    update_encoder_inputs(&g_my_pmsm_controller.mtr_interface);
}

void ctl_dispatch(void) {
    // ִ��FOC���ļ���
    ctl_step_pmsm_ctrl(&g_my_pmsm_controller);
}

void ctl_output_callback(void) {
    // ��������д��PWM�Ĵ���
    update_pwm_outputs(g_my_pmsm_controller.pwm_out);
}

// �����Ķ�ʱ���жϷ��������
void TIM1_UP_IRQHandler(void) {
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        
        // ֻ�������һ������
        gmp_base_ctl_step();
    }
}
```

����ģʽ��Ȼû��״̬���ṩ�İ�ȫ���Ϻ��������ڹ��������ṹ�򵥣�ִ��·��ֱ�ӣ��ǳ��ʺϿ���ԭ�Ϳ������߼���Լ򵥵���Ŀ��