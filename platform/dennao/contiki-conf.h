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

#define PLATFORM_NAME  "DENNAO"
#define PLATFORM_TYPE  DENNAO
#ifndef F_CPU
#define F_CPU          8000000UL
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
#define CLOCK_CONF_SECOND 125


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


/* #define NETSTACK_CONF_RADIO   cc2420_driver */


/*
 * SPI bus configuration.
 */

/* SPI input/output registers. */
#define SPI_TXBUF SPDR
#define SPI_RXBUF SPDR

#define BV(bitno) _BV(bitno)

#define SPI_WAITFOREOTx() do { while (!(SPSR & BV(SPIF))); } while (0)
#define SPI_WAITFOREORx() do { while (!(SPSR & BV(SPIF))); } while (0)

#define SCK            1  /* - Output: SPI Serial Clock (SCLK) - ATMEGA128 PORTB, PIN1 */
#define MOSI           2  /* - Output: SPI Master out - slave in (MOSI) - ATMEGA128 PORTB, PIN2 */
#define MISO           3  /* - Input:  SPI Master in - slave out (MISO) - ATMEGA128 PORTB, PIN3 */

#if 0
/*
 * SPI bus - CC2420 pin configuration.
 */

#define CC2420_CONF_SYMBOL_LOOP_COUNT 500

/*
 * SPI bus - CC2420 pin configuration.
 */

#define FIFO_P         6
#define FIFO           7
#define CCA            6

#define SFD            4
#define CSN            0
#define VREG_EN        5
#define RESET_N        6

/* - Input: FIFOP from CC2420 - ATMEGA128 PORTE, PIN6 */
#define CC2420_FIFOP_PORT(type)   P##type##E
#define CC2420_FIFOP_PIN          6
/* - Input: FIFO from CC2420 - ATMEGA128 PORTB, PIN7 */
#define CC2420_FIFO_PORT(type)     P##type##B
#define CC2420_FIFO_PIN            7
/* - Input: CCA from CC2420 - ATMEGA128 PORTD, PIN6 */
#define CC2420_CCA_PORT(type)      P##type##D
#define CC2420_CCA_PIN             6
/* - Input:  SFD from CC2420 - ATMEGA128 PORTD, PIN4 */
#define CC2420_SFD_PORT(type)      P##type##D
#define CC2420_SFD_PIN             4
/* - Output: SPI Chip Select (CS_N) - ATMEGA128 PORTB, PIN0 */
#define CC2420_CSN_PORT(type)      P##type##B
#define CC2420_CSN_PIN             0
/* - Output: VREG_EN to CC2420 - ATMEGA128 PORTA, PIN5 */
#define CC2420_VREG_PORT(type)     P##type##A
#define CC2420_VREG_PIN            5
/* - Output: RESET_N to CC2420 - ATMEGA128 PORTA, PIN6 */
#define CC2420_RESET_PORT(type)    P##type##A
#define CC2420_RESET_PIN           6

#define CC2420_IRQ_VECTOR INT6_vect

/* Pin status. */
#define CC2420_FIFOP_IS_1 (!!(CC2420_FIFOP_PORT(IN) & BV(CC2420_FIFOP_PIN)))
#define CC2420_FIFO_IS_1  (!!(CC2420_FIFO_PORT(IN) & BV(CC2420_FIFO_PIN)))
#define CC2420_CCA_IS_1   (!!(CC2420_CCA_PORT(IN) & BV(CC2420_CCA_PIN)))
#define CC2420_SFD_IS_1   (!!(CC2420_SFD_PORT(IN) & BV(CC2420_SFD_PIN)))

/* The CC2420 reset pin. */
#define SET_RESET_INACTIVE()   (CC2420_RESET_PORT(ORT) |=  BV(CC2420_RESET_PIN))
#define SET_RESET_ACTIVE()     (CC2420_RESET_PORT(ORT) &= ~BV(CC2420_RESET_PIN))

/* CC2420 voltage regulator enable pin. */
#define SET_VREG_ACTIVE()       (CC2420_VREG_PORT(ORT) |=  BV(CC2420_VREG_PIN))
#define SET_VREG_INACTIVE()     (CC2420_VREG_PORT(ORT) &= ~BV(CC2420_VREG_PIN))

/* CC2420 rising edge trigger for external interrupt 6 (FIFOP).
 * Enable the external interrupt request for INT6.
 * See Atmega128 datasheet about EICRB Register
 */
#define CC2420_FIFOP_INT_INIT() do {\
  EICRB |= 0x30; \
  CC2420_CLEAR_FIFOP_INT(); \
} while (0)

/* FIFOP on external interrupt 6. */
#define CC2420_ENABLE_FIFOP_INT()          do { EIMSK |= 0x40; } while (0)
#define CC2420_DISABLE_FIFOP_INT()         do { EIMSK &= ~0x40; } while (0)
#define CC2420_CLEAR_FIFOP_INT()           do { EIFR = 0x40; } while (0)

/*
 * Enables/disables CC2420 access to the SPI bus (not the bus).
 * (Chip Select)
 */
#define CC2420_SPI_ENABLE() (PORTB &= ~BV(CSN)) /* ENABLE CSn (active low) */
#define CC2420_SPI_DISABLE() (PORTB |=  BV(CSN)) /* DISABLE CSn (active low) */

#endif

#if WITH_UIP6

/* Network setup for IPv6 */
#define NETSTACK_CONF_NETWORK sicslowpan_driver
#define NETSTACK_CONF_MAC     csma_driver
#define NETSTACK_CONF_RDC     nullrdc_driver
#define NETSTACK_CONF_FRAMER  framer_802154

#define CC2420_CONF_AUTOACK              1
#define MAC_CONF_CHANNEL_CHECK_RATE      8
#define RIME_CONF_NO_POLITE_ANNOUCEMENTS 0
#define CXMAC_CONF_ANNOUNCEMENTS         0

#else /* WITH_UIP6 */

/* Network setup for non-IPv6 (rime). */

#define NETSTACK_CONF_NETWORK rime_driver
#define NETSTACK_CONF_MAC     csma_driver
#define NETSTACK_CONF_RDC     cxmac_driver
#define NETSTACK_CONF_FRAMER  framer_802154

#define CC2420_CONF_AUTOACK              1
#define MAC_CONF_CHANNEL_CHECK_RATE      8

#define COLLECT_CONF_ANNOUNCEMENTS       1
#define RIME_CONF_NO_POLITE_ANNOUCEMENTS 1
#define CXMAC_CONF_ANNOUNCEMENTS         0
#define CXMAC_CONF_COMPOWER              1
#define CONTIKIMAC_CONF_ANNOUNCEMENTS    0
#define CONTIKIMAC_CONF_COMPOWER         1

#define COLLECT_NEIGHBOR_CONF_MAX_NEIGHBORS      32

#endif /* WITH_UIP6 */

#define PACKETBUF_CONF_ATTRS_INLINE 1

#ifndef RF_CHANNEL
#define RF_CHANNEL              26
#endif /* RF_CHANNEL */

#define CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT 0

#define IEEE802154_CONF_PANID       0xABCD


#define AODV_COMPLIANCE
#define AODV_NUM_RT_ENTRIES 32

#define WITH_ASCII 1

#define PROCESS_CONF_NUMEVENTS 8
#define PROCESS_CONF_STATS 1

#ifdef WITH_UIP6

#define RIMEADDR_CONF_SIZE              8

#define UIP_CONF_LL_802154              1
#define UIP_CONF_LLH_LEN                0

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
#define UIP_CONF_BUFFER_SIZE		    240

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
#define UIP_CONF_IP_FORWARD      1
#define UIP_CONF_BUFFER_SIZE     128
#endif /* WITH_UIP6 */

#define UIP_CONF_ICMP_DEST_UNREACH 1

#if !WITH_UIP && !WITH_UIP6
#define QUEUEBUF_CONF_NUM          8
#else
#define QUEUEBUF_CONF_NUM          4
#endif

#define TIMESYNCH_CONF_ENABLED 1
#define CC2420_CONF_TIMESTAMPS 1
#define CC2420_CONF_SYMBOL_LOOP_COUNT 500

#define WITH_NULLMAC 0

#define CCIF
#define CLIF

/* The process names are not used to save RAM */
#define PROCESS_CONF_NO_PROCESS_NAMES 1

#define UIP_CONF_ICMP_DEST_UNREACH 1

#define UIP_CONF_DHCP_LIGHT
#define UIP_CONF_LLH_LEN         0
#define UIP_CONF_RECEIVE_WINDOW  48
#define UIP_CONF_TCP_MSS         48
#define UIP_CONF_MAX_CONNECTIONS 4
#define UIP_CONF_MAX_LISTENPORTS 8
#define UIP_CONF_UDP_CONNS       12
#define UIP_CONF_FWCACHE_SIZE    15
#define UIP_CONF_BROADCAST       1
//#define UIP_ARCH_IPCHKSUM        1
#define UIP_CONF_UDP             1
#define UIP_CONF_UDP_CHECKSUMS   1
#define UIP_CONF_PINGADDRCONF    0
#define UIP_CONF_LOGGING         0

#define UIP_CONF_TCP_SPLIT       0


typedef unsigned short clock_time_t;
typedef unsigned short uip_stats_t;
typedef unsigned long off_t;

void clock_delay(unsigned int us2);
// void clock_wait(int ms10);
void clock_set_seconds(unsigned long s);
unsigned long clock_seconds(void);

#endif /* __CONTIKI_CONF_H__ */