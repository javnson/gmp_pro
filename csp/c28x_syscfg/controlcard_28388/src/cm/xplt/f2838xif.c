//###########################################################################
//
// FILE:   f2838xif.c
//
// TITLE:  F2838x interface port file.
//
//###########################################################################
// $TI Release: $
// $Release Date: $
// 
// C2000Ware v5.04.00.00
//
// Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
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
// $
//###########################################################################

/**
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/**
 * Copyright (c) 2018 Texas Instruments Incorporated
 *
 * This file is dervied from the ``ethernetif.c'' skeleton Ethernet network
 * interface driver for lwIP.
 *
 */

#include <string.h>
/**
 * lwIP specific header files
 */
#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include <lwip/stats.h>
#include <lwip/snmp.h>
#include "netif/etharp.h"
#include "netif/ppp/pppoe.h"
#include "netif/f2838xif.h"

/**
 * f2838x device specific header files
 */
#include "inc/hw_emac.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "ethernet.h"
#include "interrupt.h"
#include "sysctl.h"

#include "utils/lwiplib.h"
/**
 * Sanity Check:  This interface driver will NOT work if the following defines
 * are incorrect.
 *
 */
#if (PBUF_LINK_HLEN != 16)
#error "PBUF_LINK_HLEN must be 16 for this interface driver!"
#endif
#if (ETH_PAD_SIZE != 0)
#error "ETH_PAD_SIZE must be 0 for this interface driver!"
#endif
#if (!SYS_LIGHTWEIGHT_PROT)
#error "SYS_LIGHTWEIGHT_PROT must be enabled for this interface driver!"
#endif

/**
 * Number of pbufs supported in low-level tx/rx pbuf queue.
 *
 */
#ifndef F2838X_NUM_PBUF_QUEUE
#define F2838X_NUM_PBUF_QUEUE    20
#endif

#define EMAC_BASE                   0x400C0000U //EMAC
#define EMAC_SS_BASE                0x400C2000U //EMACSS

/* Define those to better describe your network interface. */
#define IFNAME0 't'
#define IFNAME1 'i'

/* Helper struct to hold a queue of pbufs for transmit and receive. */
struct pbufq
{
    struct pbuf *pbuf[F2838X_NUM_PBUF_QUEUE];
    unsigned long qwrite;
    unsigned long qread;
    unsigned long overflow;
};

/* Helper macros for accessing pbuf queues. */
#define PBUF_QUEUE_EMPTY(q) \
    (((q)->qwrite == (q)->qread) ? true : false)

#define PBUF_QUEUE_FULL(q) \
    ((((((q)->qwrite + 1) % F2838X_NUM_PBUF_QUEUE)) == (q)->qread) ? \
    true : false )

/**
 * Helper struct to hold private data used to operate your ethernet interface.
 * Keeping the ethernet address of the MAC in this struct is not necessary
 * as it is already kept in the struct netif.
 * But this is only an example, anyway...
 */
struct f2838xif
{
    struct eth_addr *ethaddr;
    /* Add whatever per-interface state that is needed here. */
    struct pbufq txq;
    Ethernet_Pkt_Desc *pktDesc;
};

/**
 * A structure used to keep track of driver state and error counts.
 */
typedef struct {
    uint32_t ui32TXCount;
    uint32_t ui32TXCopyCount;
    uint32_t ui32TXCopyFailCount;
    uint32_t ui32TXNoDescCount;
    uint32_t ui32TXBufQueuedCount;
    uint32_t ui32TXBufFreedCount;
    uint32_t ui32RXBufReadCount;
    uint32_t ui32RXPacketReadCount;
    uint32_t ui32RXPacketErrCount;
    uint32_t ui32RXPacketCBErrCount;
    uint32_t ui32RXNoBufCount;
}
tDriverStats;

tDriverStats g_sDriverStats = {0};

#define DRIVER_STATS_INC(x) do{ g_sDriverStats.ui32##x++; } while(0)
#define DRIVER_STATS_DEC(x) do{ g_sDriverStats.ui32##x--; } while(0)
#define DRIVER_STATS_ADD(x, inc) do{ g_sDriverStats.ui32##x += inc; } while(0)
#define DRIVER_STATS_SUB(x, dec) do{ g_sDriverStats.ui32##x -= dec; } while(0)

/*
 * Creating a queue that maps an ethernet packet descriptor with
 * the corresponding pbuf. It is useful to free up the allocated pbuf memory
 * after the packet has been sent.
 */
#ifndef F2838X_INTERFACE_NUM_PKT_DESC_QUEUE
#define F2838X_INTERFACE_NUM_PKT_DESC_QUEUE    20
#endif

/**
 * Global variable for this interface's private data.  Needed to allow
 * the interrupt handlers access to this information outside of the
 * context of the lwIP netif.
 *
 */
static struct f2838xif f2838xif_data;

Ethernet_Handle emac_handle;

/**
 * A macro which determines whether a pointer is within the SRAM address
 * space and, hence, points to a buffer that the Ethernet MAC can directly
 * DMA from.
 */
#define PTR_SAFE_FOR_EMAC_DMA(ptr) (((uint32_t)(ptr) >= 0x2000800) &&   \
                                    ((uint32_t)(ptr) < 0x2000FFFF))

/**
 * In this function, the hardware should be initialized.
 * Called from f2838xif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void
f2838xif_hwinit(struct netif *netif)
{
    uint32_t mac_low,mac_high;
    uint8_t *pucTemp;

    /* set MAC hardware address length */
    netif->hwaddr_len = ETHARP_HWADDR_LEN;

    /* set MAC address */
    Ethernet_getMACAddr(EMAC_BASE, 0, &mac_high, &mac_low);

    pucTemp = (uint8_t *)&mac_low;
    netif->hwaddr[0] = pucTemp[0];
    netif->hwaddr[1] = pucTemp[1];
    netif->hwaddr[2] = pucTemp[2];
    netif->hwaddr[3] = pucTemp[3];

    pucTemp = (uint8_t *)&mac_high;
    netif->hwaddr[4] = pucTemp[0];
    netif->hwaddr[5] = pucTemp[1];

    /* maximum transfer unit */
    netif->mtu = 1500;

    /* device capabilities */
    /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP | NETIF_FLAG_IGMP;
}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf might be
 * chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 * @note This function MUST be called with interrupts disabled or with the
 *       F2838x Ethernet transmit fifo protected.
 */
static err_t
f2838xif_transmit(struct netif *netif, struct pbuf *p)
{
    struct pbuf *q;

    /* No of pbufs (if chained)*/
    int n=0;
    int i=0;
    Ethernet_Pkt_Desc *pktDescOrigPtr, *pktDescPtr;

    /* ENTER CRITICAL SECTION
     * This is to protect the forming of packetc descriptor chain using pbufs
     * passed to the function.
     */
    Interrupt_disable(INT_EMAC_TX0);
    Interrupt_disable(INT_EMAC_RX0);

    /*
     * Make sure we still have a valid buffer (it may have been copied)
     */
    if(!p)
    {
        LINK_STATS_INC(link.memerr);

        /* EXIT CRITICAL SECTION */
        Interrupt_enable(INT_EMAC_TX0);
        Interrupt_enable(INT_EMAC_RX0);

        return(ERR_MEM);
    }

    /*
     * Count the number of the pbufs in the chain that we are passed with.
     */
    for(q = p; q != NULL; q = q->next)
            n++;

    /*
     * Get the head of the packet descriptor chain that will be passed to
     * the driver for sending.
     */
    pktDescOrigPtr = mem_malloc(sizeof(Ethernet_Pkt_Desc));
    if(pktDescOrigPtr == NULL)
    {
        __asm("   bkpt #0");
    }

    /*
     * Go over the pbufs present in the chain and allot a packet descriptor for
     * each pbuf. The descriptors will also be chained in the same order as
     * pbufs before passing the head of the chain to the driver.
     */
    for(q = p, pktDescPtr = pktDescOrigPtr; // Initialization
        q != NULL; // Condition evaluation
        q = q->next, pktDescPtr = pktDescPtr->nextPacketDesc) // Manipulation
    {
        /*
         * Initializing this data structure, otherwise garbage values will
         * result in rogue results while running.
         */
        memset(pktDescPtr,0,sizeof(Ethernet_Pkt_Desc));

        pktDescPtr->dataOffset = 0;
        pktDescPtr->dataBuffer = q->payload;
        pktDescPtr->pktChannel = ETHERNET_DMA_CHANNEL_NUM_0;
        pktDescPtr->pktLength = q->tot_len;
        pktDescPtr->bufferLength = q->len;
        pktDescPtr->validLength = q->len;


          if(i==0)
          {
                if(n!=1)
                {
                    /*
                     * Prepare the next ethernet packet descriptor holder if
                     * there is a pbuf next in the chain.
                     */
                    pktDescPtr->nextPacketDesc =
                            mem_malloc(sizeof(Ethernet_Pkt_Desc));
                    if(pktDescPtr->nextPacketDesc == NULL)
                        {
                            __asm("   bkpt #0");
                        }
                    pktDescPtr->flags = ETHERNET_PKT_FLAG_SOP ;
                }
                else
                {
                    pktDescPtr->nextPacketDesc = 0;
                    pktDescPtr->flags = ETHERNET_PKT_FLAG_SOP |
                                        ETHERNET_PKT_FLAG_EOP;
                }
          }
          else
          {
              if(q->next != NULL)
              {
                  /*
                   * Prepare the next ethernet packet descriptor holder if
                   * there is a pbuf next in the chain.
                   */
                  pktDescPtr->nextPacketDesc =
                          mem_malloc(sizeof(Ethernet_Pkt_Desc));
                  if(pktDescPtr->nextPacketDesc == NULL)
                      {
                          __asm("   bkpt #0");
                      }
                  pktDescPtr->flags = 0;
              }
              else
              {
                  pktDescPtr->nextPacketDesc = 0;
                  pktDescPtr->flags = ETHERNET_PKT_FLAG_EOP;
              }
          }

          i++;
    }

    /* EXIT CRITICAL SECTION */
    Interrupt_enable(INT_EMAC_TX0);
    Interrupt_enable(INT_EMAC_RX0);

    /*
     * Hand over the packet descriptor to the driver.
     */
    pktDescOrigPtr->numPktFrags = n;
    Ethernet_sendPacket(emac_handle,pktDescOrigPtr);

    LINK_STATS_INC(link.xmit);

    return(ERR_OK);
}

/**
 * This function will process all transmit descriptors and free pbufs attached
 * to any that have been transmitted since we last checked.
 *
 * This function is called only from the Ethernet interrupt handler.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return None.
 */
int descriptorFreeErr=0;
static void
f2838xif_process_transmit(struct netif *netif, Ethernet_Pkt_Desc *pPacket)
{
    Ethernet_Pkt_Desc *pktDescPtr, *pktDescPtrShadow;

    /*
     * Free the packet descriptor memory.
     */
    if (pPacket == 0)
        return;

    pktDescPtr = pPacket;

    /* ENTER CRITICAL SECTION */
    Interrupt_disable(INT_EMAC_TX0);
    //Interrupt_disable(INT_EMAC_RX0);

    do
    {
        pktDescPtrShadow = pktDescPtr->nextPacketDesc;
        mem_free(pktDescPtr);
        pktDescPtr = pktDescPtrShadow;
    }
    while(pktDescPtr != 0);

    /* EXIT CRITICAL SECTION */
    Interrupt_enable(INT_EMAC_TX0);
    //Interrupt_enable(INT_EMAC_RX0);
}

/**
 * This function with either place the packet into the F2838x transmit fifo,
 * or will place the packet in the interface PBUF Queue for subsequent
 * transmission when the transmitter becomes idle.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 */

static err_t
f2838xif_output(struct netif *netif, struct pbuf *p)
{
    f2838xif_transmit(netif, p);

    return ERR_OK;
}

/**
 * This function will read a single packet from the F2838x ethernet
 * interface, if available, and return a pointer to a pbuf.  The timestamp
 * of the packet will be placed into the pbuf structure.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return pointer to pbuf packet if available, NULL otherswise.
 */
Ethernet_Pkt_Desc*
f2838xif_receive( struct netif *netif, Ethernet_Pkt_Desc *pPacket )
{
    struct pbuf *p;

#if LWIP_PTPD
    u32_t time_s, time_ns;
    /* Get the current timestamp if PTPD is enabled */
    lwIPHostGetTime(&time_s, &time_ns);
#endif

    p = pbuf_alloc(PBUF_RAW, sizeof(struct pbuf), PBUF_POOL);
    if(p)
    {
        p->payload = pPacket->dataBuffer;

        p->len = 1538U; //Max possible length of received packet.
        p->tot_len = p->len;

    #if LWIP_PTPD
        /* Place the timestamp in the PBUF */
        p->time_s = time_s;
        p->time_ns = time_ns;
    #endif

        if(ethernet_input(p, netif)!=ERR_OK)
        {
            /* drop the packet */
            LWIP_DEBUGF(NETIF_DEBUG, ("f2838xif_input: input error\n"));
            pbuf_free(p);
            p = NULL;
            /* Adjust the link statistics */
            LINK_STATS_INC(link.memerr);
            LINK_STATS_INC(link.drop);
        }
    }

    LINK_STATS_INC(link.recv);

    return(pPacket);
}

Ethernet_Pkt_Desc *f2838xif_newbuffcallback(void)
{
    Ethernet_Pkt_Desc *newPacket = mem_calloc(1, sizeof(Ethernet_Pkt_Desc));

    if(newPacket)
    {
        newPacket->dataBuffer= memp_malloc(MEMP_PBUF_POOL);
    }
    return newPacket;
}

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function f2838xif_hwinit() to do the
 * actual setup of the hardware.
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t
f2838xif_init(struct netif *netif)
{
    LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
    /* Initialize interface hostname */
    netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

    /*
     * Initialize the snmp variables and counters inside the struct netif.
     * The last argument should be replaced with your link speed, in units
     * of bits per second.
     */
    NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, 1000000);

    netif->state = &f2838xif_data;
    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;

    /* We directly use etharp_output() here to save a function call.
     * You can instead declare your own function an call etharp_output()
     * from it if you have to do some checks before sending (e.g. if link
     * is available...) */
    netif->output = etharp_output;
    netif->linkoutput = f2838xif_output;

    f2838xif_data.ethaddr = (struct eth_addr *)&(netif->hwaddr[0]);
    f2838xif_data.txq.qread = f2838xif_data.txq.qwrite = 0;
    f2838xif_data.txq.overflow = 0;

    /* initialize the hardware */
    f2838xif_hwinit(netif);

    return ERR_OK;
}

/**
 * Process tx and rx packets at the low-level interrupt.
 *
 * Should be called from the F2838x Ethernet Interrupt Handler.  This
 * function will read packets from the F2838x Ethernet fifo and place them
 * into a pbuf queue.  If the transmitter is idle and there is at least one packet
 * on the transmit queue, it will place it in the transmit fifo and start the
 * transmitter.
 *
 */

Ethernet_Pkt_Desc*
f2838xif_interrupt(struct netif *netif, Ethernet_Pkt_Desc *pPacket)
{
    Ethernet_Pkt_Desc *sPacket = NULL;

    /**
     * Based on the flags we get from pPacket, we should decide whether
     * to trasnmit or receive. Currently, it works for only incoming
     * ICMP ping requests.
     */

    /* ENTER CRITICAL SECTION
     * This is to protect the forming of packetc descriptor chain using pbufs
     * passed to the function.
     */
    Interrupt_disable(INT_EMAC_TX0);
    Interrupt_disable(INT_EMAC_RX0);

    if(pPacket->flags & ETHERNET_INTERRUPT_FLAG_RECEIVE)
        sPacket = f2838xif_receive(netif, pPacket);

    if(pPacket->flags & ETHERNET_INTERRUPT_FLAG_TRANSMIT)
        f2838xif_process_transmit(netif, pPacket);

    /* EXIT CRITICAL SECTION */
    Interrupt_enable(INT_EMAC_TX0);
    Interrupt_enable(INT_EMAC_RX0);

    return (sPacket);
}
