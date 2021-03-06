/*
 * Copyright (c) 2009, University of Colombo School of Computing
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 * @(#)$$
 */

/**
 * \file
 *         Kernel configuration for Arduino
 *
 * \author
 * 	   Ilya Dmitrichenko <errordeveloper@gmail.com>
 *         Kasun Hewage <kasun.ch@gmail.com>
 */

#ifndef __CONTIKI_CONF_H__
#define __CONTIKI_CONF_H__

#define HAVE_STDINT_H
#include "avrdef.h"

#ifndef WITH_UIP
#define WITH_UIP 1
#endif

#define PLATFORM_NAME  "DENNAO"
#define PLATFORM_TYPE  DENNAO
#ifndef F_CPU
#define F_CPU          16000000UL
#endif

/*
 * Definitions below are dictated by the hardware and not really
 * changeable!
 */
#define PLATFORM PLATFORM_AVR

/*
 * MCU and clock rate.
 */
#define MCU_MHZ 16

/* Clock ticks per second */
#define CLOCK_CONF_SECOND 128


/* LED ports */
#define LEDS_PxDIR DDRA /**< port direction register */
#define LEDS_PxOUT PORTA /**< port register */
#if 0
#define LEDS_CONF_RED    0x04 /**< red led */
#define LEDS_CONF_GREEN  0x02 /**< green led */
#define LEDS_CONF_YELLOW 0x01 /**< yellow led */
#endif

#include "dev/rs232.h"

/* USART port configuration for SLIP */

#define SLIP_PORT (RS232_PORT_0)
#define SLIP_BAUD (USART_BAUD_115200)

/* USART port configuration for serial I/O */

#define USART_PORT (RS232_PORT_0)
#define USART_BAUD (USART_BAUD_9600)

/* Pre-allocated memory for loadable modules heap space (in bytes)*/
#define MMEM_CONF_SIZE 256

/* Use the following address for code received via the codeprop
 * facility
 */
#define EEPROMFS_ADDR_CODEPROP 0x8000

#define EEPROM_NODE_ID_START 0x00




/*
 * SPI bus configuration.
 */

/* SPI input/output registers. */
#define SPI_TXBUF SPDR
#define SPI_RXBUF SPDR

#define BV(bitno) _BV(bitno)

#define SPI_WAITFOREOTx() do { while( (SPSR & 0x80)==0x00 ); } while (0)
#define SPI_WAITFOREORx() 

#define SCK            PB1  /* - Output: SPI Serial Clock (SCLK) - ATMEGA128 PORTB, PIN1 */
#define MOSI           PB2  /* - Output: SPI Master out - slave in (MOSI) - ATMEGA128 PORTB, PIN2 */
#define MISO           PB3  /* - Input:  SPI Master in - slave out (MISO) - ATMEGA128 PORTB, PIN3 */

#if WITH_UIP6

/* Network setup for IPv6 */
#define NETSTACK_CONF_NETWORK uip_driver
#define NETSTACK_CONF_MAC     csma_driver
#define NETSTACK_CONF_RDC     nullrdc_noframer_driver
#define NETSTACK_CONF_FRAMER  framer_ethernet
#define NETSTACK_CONF_RADIO   mrf24wb0ma_driver

#define CC2420_CONF_AUTOACK              1
#define MAC_CONF_CHANNEL_CHECK_RATE      8
#define RIME_CONF_NO_POLITE_ANNOUCEMENTS 0
#define CXMAC_CONF_ANNOUNCEMENTS         0

#else /* WITH_UIP6 */

/* Network setup for non-IPv6 (rime). */

#define NETSTACK_CONF_NETWORK uip_driver
#define NETSTACK_CONF_MAC     ethernet_mac_driver
#define NETSTACK_CONF_RDC     ethernet_rdc_driver
#define NETSTACK_CONF_FRAMER  framer_ethernet
#define NETSTACK_CONF_RADIO   mrf24wb0ma_driver

#define QUEUEBUF_CONF_NUM     0
#define QUEUEBUF_CONF_REF_NUM 0
#define ROUTE_CONF_ENTRIES    0

#define MAC_CONF_CHANNEL_CHECK_RATE      8

#define COLLECT_CONF_ANNOUNCEMENTS       1
#define RIME_CONF_NO_POLITE_ANNOUCEMENTS 1
#define CXMAC_CONF_ANNOUNCEMENTS         0
#define CXMAC_CONF_COMPOWER              1
#define CONTIKIMAC_CONF_ANNOUNCEMENTS    0
#define CONTIKIMAC_CONF_COMPOWER         1

#endif /* WITH_UIP6 */

#define PACKETBUF_CONF_ATTRS_INLINE 1

#define CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT 0

#define AODV_COMPLIANCE
#define AODV_NUM_RT_ENTRIES 32

#define WITH_ASCII 1

#define PROCESS_CONF_NUMEVENTS 8
#define PROCESS_CONF_STATS 1

#ifdef WITH_UIP6

#define RIMEADDR_CONF_SIZE              8

#define UIP_CONF_LL_802154              1
#define UIP_CONF_LLH_LEN                14

#define UIP_CONF_ROUTER                 1
#define UIP_CONF_IPV6_RPL               1

/* configure number of neighbors and routes */
#define UIP_CONF_DS6_NBR_NBU     5
#define UIP_CONF_DS6_ROUTE_NBU   5

#define RPL_CONF_MAX_PARENTS         4
#define NEIGHBOR_CONF_MAX_NEIGHBORS  8

#define UIP_CONF_ND6_SEND_RA		0
#define UIP_CONF_ND6_REACHABLE_TIME     600000
#define UIP_CONF_ND6_RETRANS_TIMER      10000

#define UIP_CONF_IPV6                   1
#define UIP_CONF_IPV6_QUEUE_PKT         0
#define UIP_CONF_IPV6_CHECKS            1
#define UIP_CONF_IPV6_REASSEMBLY        0
#define UIP_CONF_NETIF_MAX_ADDRESSES    3
#define UIP_CONF_ND6_MAX_PREFIXES       3
#define UIP_CONF_ND6_MAX_NEIGHBORS      4
#define UIP_CONF_ND6_MAX_DEFROUTERS     2
#define UIP_CONF_IP_FORWARD             0
#define UIP_CONF_BUFFER_SIZE		    UIP_LINK_MTU + UIP_LLH_LEN + 4

#define SICSLOWPAN_CONF_COMPRESSION_IPV6        0
#define SICSLOWPAN_CONF_COMPRESSION_HC1         1
#define SICSLOWPAN_CONF_COMPRESSION_HC01        2
#define SICSLOWPAN_CONF_COMPRESSION             SICSLOWPAN_COMPRESSION_HC06
#ifndef SICSLOWPAN_CONF_FRAG
#define SICSLOWPAN_CONF_FRAG                    1
#define SICSLOWPAN_CONF_MAXAGE                  8
#endif /* SICSLOWPAN_CONF_FRAG */
#define SICSLOWPAN_CONF_CONVENTIONAL_MAC	1
#define SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS       2
#else /* WITH_UIP6 */

#define UIP_CONF_IPV6            0
#define UIP_CONF_IP_FORWARD      0
#define UIP_CONF_BUFFER_SIZE     UIP_LINK_MTU + UIP_LLH_LEN + 4
#endif /* WITH_UIP6 */

#define UIP_CONF_ICMP_DEST_UNREACH 1

#define TIMESYNCH_CONF_ENABLED 1
#define PACKETBUF_CONF_SIZE 1000

#define WITH_NULLMAC 0

#define CCIF
#define CLIF

/* The process names are not used to save RAM */
#define PROCESS_CONF_NO_PROCESS_NAMES 1

#define UIP_CONF_ICMP_DEST_UNREACH 1

#define UIP_CONF_DHCP_LIGHT
#define UIP_CONF_LLH_LEN         14
#define UIP_CONF_RECEIVE_WINDOW  48
#define UIP_CONF_TCP_MSS         48
#define UIP_CONF_MAX_CONNECTIONS 4
#define UIP_CONF_MAX_LISTENPORTS 8
#define UIP_CONF_UDP_CONNS       12
#define UIP_CONF_FWCACHE_SIZE    15
#define UIP_CONF_BROADCAST       1
//#define UIP_ARCH_IPCHKSUM        1
#define UIP_CONF_TCP             1
#define UIP_CONF_UDP             1
#define UIP_CONF_UDP_CHECKSUMS   1
#define UIP_CONF_PINGADDRCONF    0
#define UIP_CONF_LOGGING         0

#define UIP_CONF_TCP_SPLIT       0

typedef unsigned short clock_time_t;
typedef unsigned short uip_stats_t;
typedef unsigned long off_t;

#endif /* __CONTIKI_CONF_H__ */