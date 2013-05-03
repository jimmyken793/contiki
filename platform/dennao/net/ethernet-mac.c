#include "net/ethernet-mac.h"
#include "net/packetbuf.h"
#include "net/netstack.h"

/*---------------------------------------------------------------------------*/
static void send_packet(mac_callback_t sent, void *ptr) {
	NETSTACK_RDC.send(sent, ptr);
}
/*---------------------------------------------------------------------------*/
static void packet_input(void) {
	NETSTACK_NETWORK.input();
}
/*---------------------------------------------------------------------------*/
static int on(void) {
	return NETSTACK_RDC.on();
}
/*---------------------------------------------------------------------------*/
static int off(int keep_radio_on) {
	return NETSTACK_RDC.off(keep_radio_on);
}
/*---------------------------------------------------------------------------*/
static unsigned short channel_check_interval(void) {
	return 0;
}
/*---------------------------------------------------------------------------*/
static void init(void) {
}
/*---------------------------------------------------------------------------*/
const struct mac_driver ethernet_mac_driver = {
	"etnernet-mac",
	init,
	send_packet,
	packet_input,
	on,
	off,
	channel_check_interval,
};
/*---------------------------------------------------------------------------*/
