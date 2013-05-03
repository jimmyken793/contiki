
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
uint8_t uip_driver_send(void) {
	uip_arp_out();
	packetbuf_copyfrom(&uip_buf, uip_len);
	packetbuf_set_datalen(uip_len);

	/* XXX we should provide a callback function that is called when the
	   packet is sent. For now, we just supply a NULL pointer. */
	NETSTACK_MAC.send(NULL, NULL);
	return 1;
}
/*--------------------------------------------------------------------*/
static void init(void) {
	/*
	 * Set out output function as the function to be called from uIP to
	 * send a packet.
	 */
	tcpip_set_outputfunc(uip_driver_send);
}
/*--------------------------------------------------------------------*/
static void input(void) {
	if (packetbuf_datalen() > 0 && packetbuf_datalen() <= UIP_BUFSIZE - UIP_LLH_LEN) {
		memcpy(uip_buf, packetbuf_dataptr(), packetbuf_datalen());
		uip_len = packetbuf_datalen();
		if (uip_len > 0) {
			if (BUF->type == uip_htons(UIP_ETHTYPE_IP)) {
				uip_len -= sizeof(struct uip_eth_hdr);
				tcpip_input();
			} else if (BUF->type == uip_htons(UIP_ETHTYPE_ARP)) {
				uip_arp_arpin();
				if (uip_len > 0) {
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
