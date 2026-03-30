
#include <ctl/component/motor_control/pmsm_offline_id/pmsm_offline_id_sm.h>

extern ctl_pmsm_offline_id_t pmsm_oid;
#define DSA_BUFFER_SIZE 2048  // 分配 2K 的控制字内存给录波器
extern ctrl_gt dsa_buffer[DSA_BUFFER_SIZE];

void init_pmsm_offline_id(void);
void loop_pmsm_offline_id(void);
