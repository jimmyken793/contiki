/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
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
 *         A brief description of what this file is
 * \author
 *         Niclas Finne <nfi@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 */

#include "net/netstack.h"
#include "net/uip.h"
#include "net/uip_arp.h"
#include "net/tcpip.h"
#include "net/hc.h"
#include "net/packetbuf.h"
#include "net/uip-driver.h"
#include <string.h>

#define BUF ((struct uip_eth_hdr *)&uip_buf[0])
/*--------------------------------------------------------------------*/
uint8_t
uip_driver_send(void)
{
  uip_arp_out();
  printf("uip send packet %d!!\n", uip_len);
  packetbuf_copyfrom(&uip_buf, uip_len);
  packetbuf_set_datalen(uip_len);

    {
      int i;
      for(i=0;i<uip_len;i++){
       printf("%02x ", ((uint8_t*)uip_buf)[i]);
       if(i%20==19)
        printf("\n");
      }
      printf("\n");
    }
  /* XXX we should provide a callback function that is called when the
     packet is sent. For now, we just supply a NULL pointer. */
  NETSTACK_MAC.send(NULL, NULL);
  return 1;
}
/*--------------------------------------------------------------------*/
static void
init(void)
{
  printf("uip init!!\n");
  /*
   * Set out output function as the function to be called from uIP to
   * send a packet.
   */
  tcpip_set_outputfunc(uip_driver_send);
}
/*--------------------------------------------------------------------*/
static void
input(void)
{
  printf("uip input!!\n");
  if(packetbuf_datalen() > 0 && packetbuf_datalen() <= UIP_BUFSIZE - UIP_LLH_LEN) {
    memcpy(&uip_buf[UIP_LLH_LEN], packetbuf_dataptr(), packetbuf_datalen());
    uip_len = packetbuf_datalen();
    if(uip_len > 0) {
      if(BUF->type == uip_htons(UIP_ETHTYPE_IP)) {
        uip_len -= sizeof(struct uip_eth_hdr);
        tcpip_input();
      }else if(BUF->type == uip_htons(UIP_ETHTYPE_ARP)) {
        uip_arp_arpin();
        if(uip_len > 0) {
          uip_driver_send();
        }
      }
    }
  }
}
/*--------------------------------------------------------------------*/
const struct network_driver uip_driver = {
  "uip",
  init,
  input
};
/*--------------------------------------------------------------------*/
