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

#include "watchdog.h"
#include "contiki-conf.h"
#include "dhcp.h"
#include "net/uip-driver.h"
#include "sd-spi.h"

PROCINIT(&etimer_process, &serial_line_process);

void
init_lowlevel(void) {
	rs232_init(USART_PORT, USART_BAUD, USART_PARITY_NONE | USART_STOP_BITS_1 | USART_DATA_BITS_8);
	rs232_redirect_stdout(USART_PORT);
	rs232_set_input(USART_PORT, serial_line_input_byte);
}


int main(void) {
	//calibrate_rc_osc_32k(); //CO: Had to comment this out
	/* Initialize hardware */
	init_lowlevel();
	/* Clock */
	clock_init();

	/* Process subsystem */
	process_init();

	/* Register initial processes */
	procinit_init();

	/* System timers */
	process_start(&etimer_process, NULL);
	ctimer_init();

	serial_line_init();
  printf_P(PSTR("\n*******Booting %s*******\n"),CONTIKI_VERSION_STRING);

	efs_sdcard_init();

	autostart_start(autostart_processes);
	printf_P(PSTR("System online.\r\n"));
	watchdog_start();
	do {
		process_run();
		watchdog_periodic();
		etimer_request_poll();
	} while (1);

	return 0;
}