/** @file xplt.config.h @brief Windows/Simulink configuration for mcs_acim_nt. */

#define USER_SPECIFIED_PRINT_FUNCTION printf_s

#define SPECIFY_ENABLE_GMP_CTL
#define SPECIFY_CTRL_GT_TYPE USING_FLOAT_FPU

#define SPECIFY_PC_ENVIRONMENT
#define PC_ENV_MAX_ITERATION ((100000000))
#define GMP_ASIO_CONFIG_JSON "network.json"

#define gmp_pc_simulink_rx_buffer_t dp_sil_rx_buf_t
#define gmp_pc_simulink_tx_buffer_t dp_sil_tx_buf_t

#include <ctrl_settings.h>

