/**
 * @file ctl_common_init.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */
#include <gmp_core.h>

#include <math.h>

//////////////////////////////////////////////////////////////////////////
// FIR Filter
#include <ctl/component/intrinsic/discrete/fir_filter.h>
#include <stdlib.h> // Required for malloc and free

fast_gt ctl_init_fir_filter(ctl_fir_filter_t* fir, uint32_t order, const ctrl_gt* coeffs)
{
    fir->order = order;
    fir->coeffs = coeffs;
    fir->output = 0.0f;
    fir->buffer_index = 0;

    // 动态分配状态缓冲区
    // 滤波器的状态（即过去的输入样本）需要一个缓冲区。其大小等于滤波器的阶数。
    // 使用动态内存分配可以使模块适应任意阶数的滤波器。
    fir->buffer = (ctrl_gt*)malloc(order * sizeof(ctrl_gt));
    if (fir->buffer == NULL)
    {
        return 0; // 内存分配失败
    }

    // 将缓冲区初始化为零
    ctl_clear_fir_filter(fir);

    return 1; // 成功
}
