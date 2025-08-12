/**
 * @file flc.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a Fuzzy Logic controller with LUT-based  tuning.
 * @version 0.1
 * @date 2025-08-07
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FUZZY_PID_H_
#define _FUZZY_PID_H_

#include <ctl/component/intrinsic/advance/surf_search.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup fuzzy_pid_controller Fuzzy PID Controller
 * @brief A self-tuning PID controller using fuzzy logic look-up tables.
 * ���ģ���ڶ��׵�ͨ���ڣ�Ƿ����Ҳ���ã��ǳ����ã�����������sʱϵͳ���񵴡�
 * ��Ҫ���ھ�����Ե������棬��Ҫ����error���棬��Ҫ������Ӧʱ����Ե���error diff������
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Fuzzy Self-Tuning PID Controller                                          */
/*---------------------------------------------------------------------------*/

/**
 * @}
 */ // end of fuzzy_pid_controller group

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _FUZZY_PID_H_
