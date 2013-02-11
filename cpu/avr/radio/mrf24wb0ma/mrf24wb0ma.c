/*
 * Copyright (c) 2007, Swedish Institute of Computer Science
 * All rights reserved.
 *
 *  Additional fixes for AVR contributed by:
 *
 *  David Kopf dak664@embarqmail.com
 *  Ivan Delamer delamer@ieee.com
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
 */
/*
 * This code is almost device independent and should be easy to port.
 * Ported to Atmel RF230 21Feb2010 by dak
 */

#include <stdio.h>
#include <string.h>

#include "contiki.h"

#if defined(__AVR__)
#include <avr/io.h>

//_delay_us has the potential to use floating point which brings the 256 byte clz table into RAM
//#include <util/delay.h>
//#define delay_us( us )   ( _delay_us( ( us ) ) )
//_delay_loop_2(uint16_t count) is 4 CPU cycles per iteration, up to 32 milliseconds at 8MHz
#include <util/delay_basic.h>
#define delay_us( us )   ( _delay_loop_2(1+((unsigned long long)us*F_CPU)/4000000UL) ) 

#include <avr/pgmspace.h>
#elif defined(__MSP430__)
#include <io.h>
#endif

#include "dev/leds.h"
#include "dev/spi.h"
#include "mrf24wb0ma.h"

#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "net/netstack.h"

#include "sys/timetable.h"

/* RF230 hardware delay times, from datasheet */
typedef enum{
    TIME_TO_ENTER_P_ON               = 510, /**<  Transition time from VCC is applied to P_ON - most favorable case! */
    TIME_P_ON_TO_TRX_OFF             = 510, /**<  Transition time from P_ON to TRX_OFF. */
    TIME_SLEEP_TO_TRX_OFF            = 880, /**<  Transition time from SLEEP to TRX_OFF. */
    TIME_RESET                       = 6,   /**<  Time to hold the RST pin low during reset */
    TIME_ED_MEASUREMENT              = 140, /**<  Time it takes to do a ED measurement. */
    TIME_CCA                         = 140, /**<  Time it takes to do a CCA. */
    TIME_PLL_LOCK                    = 150, /**<  Maximum time it should take for the PLL to lock. */
    TIME_FTN_TUNING                  = 25,  /**<  Maximum time it should take to do the filter tuning. */
    TIME_NOCLK_TO_WAKE               = 6,   /**<  Transition time from *_NOCLK to being awake. */
    TIME_CMD_FORCE_TRX_OFF           = 1,   /**<  Time it takes to execute the FORCE_TRX_OFF command. */
    TIME_TRX_OFF_TO_PLL_ACTIVE       = 180, /**<  Transition time from TRX_OFF to: RX_ON, PLL_ON, TX_ARET_ON and RX_AACK_ON. */
    TIME_STATE_TRANSITION_PLL_ACTIVE = 1,   /**<  Transition time from PLL active state to another. */
}radio_trx_timing_t;

static int MRF24WB0MA_on(void);
static int MRF24WB0MA_off(void);

static int MRF24WB0MA_read(void *buf, unsigned short bufsize);

static int MRF24WB0MA_prepare(const void *data, unsigned short len);
static int MRF24WB0MA_transmit(unsigned short len);
static int MRF24WB0MA_send(const void *data, unsigned short len);

static int MRF24WB0MA_receiving_packet(void);
static int MRF24WB0MA_pending_packet(void);
static int MRF24WB0MA_cca(void);

const struct radio_driver mrf24wb0ma_driver =
  {
    MRF24WB0MA_init,
    MRF24WB0MA_prepare,
    MRF24WB0MA_transmit,
    MRF24WB0MA_send,
    MRF24WB0MA_read,
    MRF24WB0MA_cca,
    MRF24WB0MA_receiving_packet,
    MRF24WB0MA_pending_packet,
    MRF24WB0MA_on,
    MRF24WB0MA_off
  };

/*---------------------------------------------------------------------------*/
int MRF24WB0MA_init(void)
{

  printf_P(PSTR("MRF24WB0MA_init\n"));
  /* Initialize Wifi Module */
  zg_init();
  printf_P(PSTR("MRF24WB0MA_init done!\n"));
  return 1;
}
/*---------------------------------------------------------------------------*/

static int MRF24WB0MA_transmit(unsigned short payload_len)
{
  printf_P(PSTR("wifi transmit\n"));
  wifi_send();
  return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
static int MRF24WB0MA_prepare(const void *payload, unsigned short payload_len)
{
  printf("preparing data %d\n", payload_len);

  {
    int i;
    for(i=0;i<payload_len;i++){
     printf("%02x ", ((uint8_t*)payload)[i]);
     if(i%20==19)
      printf("\n");
    }
    printf("\n");
  }
  int ret = 0;
  wifi_prepare_payload((char*)payload, payload_len);
  return ret;
}
/*---------------------------------------------------------------------------*/
static int MRF24WB0MA_send(const void *payload, unsigned short payload_len)
{
	int ret = 0;
  MRF24WB0MA_prepare(payload, payload_len);
  MRF24WB0MA_transmit(payload_len);
	return ret;
}
/*---------------------------------------------------------------------------*/
static int MRF24WB0MA_off(void)
{
  printf_P(PSTR("TODO:wifi turned off\n"));
  return 0;
}
/*---------------------------------------------------------------------------*/
static int MRF24WB0MA_on(void)
{
  printf_P(PSTR("TODO:wifi turned on\n"));
  return 1;
}

/* Read packet that was uploaded from Radio in ISR, else return zero.
 * The two-byte checksum is appended but the returned length does not include it.
 * Frames are buffered in the interrupt routine so this routine
 * does not access the hardware or change its status.
 * However, this routine must be called with interrupts disabled to avoid ISR
 * writing to the same buffer we are reading.
 * As a result, PRINTF cannot be used in here.
 */
/*---------------------------------------------------------------------------*/
static int MRF24WB0MA_read(void *buf, unsigned short bufsize)
{
  printf_P(PSTR("TODO:MRF24WB0MA_read\n"));
  return 0;
}
/*---------------------------------------------------------------------------*/
static int MRF24WB0MA_cca(void)
{
  printf_P(PSTR("TODO:MRF24WB0MA_cca\n"));
	 return 0;
}
/*---------------------------------------------------------------------------*/
int MRF24WB0MA_receiving_packet(void)
{
  printf_P(PSTR("TODO:MRF24WB0MA_receiving_packet\n"));
  return 0;
}
/*---------------------------------------------------------------------------*/
static int MRF24WB0MA_pending_packet(void)
{
  printf_P(PSTR("TODO:MRF24WB0MA_pending_packet\n"));
  return 0;
}
/*---------------------------------------------------------------------------*/

