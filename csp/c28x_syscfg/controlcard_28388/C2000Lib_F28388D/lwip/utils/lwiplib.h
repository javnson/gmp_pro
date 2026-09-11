//*****************************************************************************
//
// lwiplib.h - Prototypes for the lwIP library wrapper API.
//
//
// Copyright (c) 2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************


#ifndef __LWIPLIB_H__
#define __LWIPLIB_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// lwIP Options
//
//*****************************************************************************
#include "lwip/opt.h"

//*****************************************************************************
//
// Ensure that AUTOIP COOP option is configured correctly.
//
//*****************************************************************************
#undef LWIP_DHCP_AUTOIP_COOP
#define LWIP_DHCP_AUTOIP_COOP   ((LWIP_DHCP) && (LWIP_AUTOIP))

//*****************************************************************************
//
// lwIP API Header Files
//
//*****************************************************************************
#include <stdint.h>
#include "lwip/api.h"
#include "lwip/netifapi.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "lwip/tcpip.h"
#include "lwip/sockets.h"
#include "lwip/mem.h"
#include "lwip/stats.h"
#include "lwip/def.h"

//*****************************************************************************
//
// IP Address Acquisition Modes
//
//*****************************************************************************
#define IPADDR_USE_STATIC       0
#define IPADDR_USE_DHCP         1
#define IPADDR_USE_AUTOIP       2

//*****************************************************************************
//
// Hardware timer interrupt callback function type (available only when running
// on F2838X parts).  This function is called in interrupt context whenever the
// Ethernet MAC reports an interrupt from the IEEE-1588 timestamping
// timer.  The first parameter is the base address of the MAC and the second
// is the interrupt status as reported via EthMACTimestampIntStatus.
//
//*****************************************************************************
typedef void (* tHardwareTimerHandler)(uint32_t ui32Base,
                                       uint32_t ui32IntStatus);

//*****************************************************************************
//
// lwIP Abstraction Layer API
//
//*****************************************************************************
extern void lwIPInit(uint32_t ui32SysClkHz, const uint8_t *pui8Mac,
                     uint32_t ui32IPAddr, uint32_t ui32NetMask,
                     uint32_t ui32GWAddr, uint32_t ui32IPMode);
extern void lwIPTimerCallbackRegister(tHardwareTimerHandler pfnTimerFunc);
extern void lwIPTimer(uint32_t ui32TimeMS);
extern void EMAC0_IRQHandler(void);
extern uint32_t lwIPLocalIPAddrGet(void);
extern uint32_t lwIPLocalNetMaskGet(void);
extern uint32_t lwIPLocalGWAddrGet(void);
extern void lwIPLocalMACGet(uint8_t *pui8Mac);
extern void lwIPNetworkConfigChange(uint32_t ui32IPAddr, uint32_t ui32NetMask,
                                    uint32_t ui32GWAddr, uint32_t ui32IPMode);
extern uint32_t lwIPAcceptUDPPort(uint16_t ui16Port);
extern void lwIPHostGetTime(u32_t *time_s, u32_t *time_ns);
extern void lwIPLLDPSend(void);
extern void lwIPMulticastLoopbackPoll(void);
extern err_t lwIPJoinMulticastGroup(ip4_addr_t *multi_addr);
extern err_t lwIPLeaveMulticastGroup(ip4_addr_t *multi_addr);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __LWIPLIB_H__
