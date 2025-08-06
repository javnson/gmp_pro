/**
 * @defgroup CTL_LIB  Control Template Library
 * @brief The whole Digital Power Control Library
 */

// 把所有intrinsic的header全部引入进来

// TODO: 为每一个头文件(模块)补充一个简单的说明，方便用户可以快速在这个文件中找到需要的模块

// basic includes

// #include <ctl/ctl_core.h>

#ifndef GMP_CTL_DISABLE_COM_INTRINSIC

//////////////////////////////////////////////////////////////////////////
#ifndef GMP_CTL_DISABLE_COM_INTRINSIC_INTERFACE

#include <ctl/component/intrinsic/interface/adc_channel.h>

#include <ctl/component/intrinsic/interface/dac_channel.h>

#include <ctl/component/intrinsic/interface/pwm_channel.h>

#endif // GMP_CTL_DISABLE_COM_INTRINSIC_INTERFACE

//////////////////////////////////////////////////////////////////////////
#ifndef GMP_CTL_DISABLE_COM_INTRINSIC_DISCRETE

#include <ctl/component/intrinsic/discrete/divider.h>

#include <ctl/component/intrinsic/discrete/feed_forward.h>

#include <ctl/component/intrinsic/discrete/discrete_filter.h>

#include <ctl/component/intrinsic/discrete/hcc.h>

#include <ctl/component/intrinsic/discrete/pid.h>

#include <ctl/component/intrinsic/discrete/pll.h>

#include <ctl/component/intrinsic/discrete/pole_zero.h>

#include <ctl/component/intrinsic/discrete/saturation.h>

#include <ctl/component/intrinsic/discrete/slope_lim.h>

#include <ctl/component/intrinsic/discrete/stimulate.h>

// #include <ctl/component/intrinsic/discrete/z_function.h>

#endif // GMP_CTL_DISABLE_COM_INTRINSIC_DISCRETE

//////////////////////////////////////////////////////////////////////////
#ifndef GMP_CTL_DISABLE_COM_INTRINSIC_PROTECT

#include <ctl/component/intrinsic/protection/fusing.h>

#include <ctl/component/intrinsic/protection/protection.h>

#endif // GMP_CTL_DISABLE_COM_INTRINSIC_PROTECT

//////////////////////////////////////////////////////////////////////////
#ifndef GMP_CTL_DISABLE_COM_INTRINSIC_ADVANCE

#include <ctl/component/intrinsic/advance/fuzzy_pid.h>

#include <ctl/component/intrinsic/advance/surf_search.h>

#endif // GMP_CTL_DISABLE_COM_INTRINSIC_ADVANCE

//////////////////////////////////////////////////////////////////////////
#ifndef GMP_CTL_DISABLE_COM_INTRINSIC_COMBO

#include <ctl/component/intrinsic/combo/track_pid.h>

#endif // GMP_CTL_DISABLE_COM_INTRINSIC_COMBO

#endif // GMP_CTL_DISABLE_COM_INTRINSIC
