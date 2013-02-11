/*
 * Copyright (c) 2006, Technical University of Munich
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

#include <stdio.h>
#include <avr/pgmspace.h>
#include <avr/fuse.h>
#include <avr/eeprom.h>

#include "lib/mmem.h"
#include "loader/symbols-def.h"
#include "loader/symtab.h"
#include <stdbool.h>
 
#include "contiki.h"
#include "contiki-net.h"
#include "contiki-lib.h"

#include "dev/rs232.h"
#include "dev/serial-line.h"
#include "dev/slip.h"

#include "radio/mrf24wb0ma/mrf24wb0ma.h"
#include "contiki-conf.h"
#include "dhcp.h"
#include "net/uip-driver.h"

#if 0
FUSES =
	{
		.low = 0xe2,
		.high = 0x99,
		.extended = 0xff,
	};
	

/* Put default MAC address in EEPROM */
uint8_t mac_address[8] EEMEM = {0x02, 0x11, 0x22, 0xff, 0xfe, 0x33, 0x44, 0x55};
#endif

PROCINIT(&etimer_process, &serial_line_process);

void
init_lowlevel(void)
{
  rs232_init(USART_PORT, USART_BAUD,
             USART_PARITY_NONE | USART_STOP_BITS_1 | USART_DATA_BITS_8);

//#if WITH_UIP || WITH_UIP6
  /* Initialise SPLIP on USART port */
//  slip_arch_init(SLIP_BAUD);
//#else
  /* Redirect stdout and stdin to USART port */
  rs232_redirect_stdout(USART_PORT);
  rs232_set_input(USART_PORT, serial_line_input_byte);
//#endif

}


int main(void)
{
  //calibrate_rc_osc_32k(); //CO: Had to comment this out

  /* Initialize hardware */
  init_lowlevel();

  /* Clock */
  clock_init();

  /* Process subsystem */
  process_init();

  /* Register initial processes */
  procinit_init();


  //Give ourselves a prefix
  //init_net();

    /* System timers */
  process_start(&etimer_process, NULL);
  ctimer_init();

  /* This line wasn't present in ProMini code */
  /* Make pin 5 on port B an input (PB5 SCK/PCINT5) */
  PORTB &= ~(1<<5);
  serial_line_init();

  printf_P(PSTR("\r\n********BOOTING CONTIKI*********\r\n"));
  printf("sizeof(zg_rx_data_ind_t): %d \n", sizeof(zg_rx_data_ind_t));

  wifi_set_ssid("Jimmy");
  wifi_set_passphrase("jimmy12345orz");
  wifi_set_mode(WIRELESS_MODE_INFRA);
  wifi_set_security_type(ZG_SECURITY_TYPE_WPA2);
  // wifi_set_ssid("JimmyTest");
  // wifi_set_passphrase("jimmy12345orz");
  // wifi_set_mode(WIRELESS_MODE_ADHOC);
  // wifi_set_security_type(ZG_SECURITY_TYPE_NONE);
  queuebuf_init();
  NETSTACK_RADIO.init();
  NETSTACK_RDC.init();
  NETSTACK_MAC.init();
  NETSTACK_NETWORK.init();
  wifi_connect();
  process_start(&mrf24wb0ma_process,NULL);
  struct uip_eth_addr addr;
  wifi_getMAC(addr.addr);
  uip_setethaddr(addr);
  uip_arp_init();

  printf("%s %lu %u\n",
         NETSTACK_RDC.name,
         CLOCK_SECOND / (NETSTACK_RDC.channel_check_interval() == 0? 1:
                         NETSTACK_RDC.channel_check_interval()),
         RF_CHANNEL);
  /* Autostart processes */
  autostart_start(autostart_processes);

  printf("MAC:%s\nRDC:%s\nNETWORK:%s\n", NETSTACK_MAC.name, NETSTACK_RDC.name, NETSTACK_NETWORK.name);
  #if WITH_UIP
  /* IPv4 CONFIGURATION */
    process_start(&tcpip_process, NULL);
    //process_start(&uip_fw_process, NULL);
    process_start(&dhcp_process, NULL);
  #endif /* WITH_UIP */
  printf_P(PSTR("System online.\r\n"));
  watchdog_start();
  do {
    process_run();
    watchdog_periodic();
    etimer_request_poll();
  } while (1);

  return 0;
}