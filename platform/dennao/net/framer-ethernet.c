/*
 * Copyright (c) 2009, Swedish Institute of Computer Science.
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
 */

/**
 * \file
 *         MAC framer for nullmac
 * \author
 *         Niclas Finne <nfi@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 */
#include <string.h>
#include "net/framer-ethernet.h"
#include "net/packetbuf.h"
#include "net/uip_arp.h"

#define DEBUG 0

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINTADDR(addr) PRINTF(" %02x%02x:%02x%02x:%02x%02x:%02x%02x ", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7])
#else
#define PRINTF(...)
#define PRINTADDR(addr)
#endif
static const struct uip_eth_addr broadcast_ethaddr = {{0xff, 0xff, 0xff, 0xff, 0xff, 0xff}};
/*---------------------------------------------------------------------------*/
static int create(void) {
	struct uip_eth_hdr *hdr;

	if (packetbuf_hdralloc(sizeof(struct uip_eth_hdr))) {
		hdr = packetbuf_hdrptr();
		memcpy(&(hdr->src), &uip_lladdr, 6);
		memcpy(&(hdr->dest), &broadcast_ethaddr, 6);
		hdr->type = 0x08;
		return sizeof(struct uip_eth_hdr);
	}
	PRINTF("PNULLMAC-UT: too large header: %u\n", len);
	return FRAMER_FAILED;
}
/*---------------------------------------------------------------------------*/
static int parse(void) {
	struct uip_eth_hdr *hdr;
	hdr = packetbuf_dataptr();
	if (packetbuf_hdrreduce(sizeof(struct uip_eth_hdr))) {
		packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &(hdr->src));
		packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, &(hdr->dest));

		PRINTF("PNULLMAC-IN: ");
		PRINTADDR(packetbuf_addr(PACKETBUF_ADDR_SENDER));
		PRINTADDR(packetbuf_addr(PACKETBUF_ADDR_RECEIVER));
		PRINTF("%u (%u)\n", packetbuf_datalen(), len);

		return sizeof(struct uip_eth_hdr);
	}
	return FRAMER_FAILED;
}
/*---------------------------------------------------------------------------*/
const struct framer framer_ethernet = {
	create, parse
};
