#include "contiki-net.h"
#include "ctk/ctk.h"
#include "net/dhcpc.h"
#include <stdio.h>



PROCESS(dhcp_process, "DHCP");

AUTOSTART_PROCESSES(&dhcp_process);

static char ipaddr[17];
static char *
makebyte(uint8_t byte, char *str)
{
  if(byte >= 100) {
    *str++ = (byte / 100 ) % 10 + '0';
  }
  if(byte >= 10) {
    *str++ = (byte / 10) % 10 + '0';
  }
  *str++ = (byte % 10) + '0';

  return str;
}

static void
makeaddr(uip_ipaddr_t *addr, char *str)
{
  str = makebyte(addr->u8[0], str);
  *str++ = '.';
  str = makebyte(addr->u8[1], str);
  *str++ = '.';
  str = makebyte(addr->u8[2], str);
  *str++ = '.';
  str = makebyte(addr->u8[3], str);
  *str++ = 0;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(dhcp_process, ev, data)
{
  static struct etimer et;
  PROCESS_BEGIN();
  dhcpc_init(uip_lladdr.addr, sizeof(uip_lladdr.addr));
  dhcpc_request();

  while(1) {
    PROCESS_WAIT_EVENT();
    if(ev == tcpip_event|| ev == PROCESS_EVENT_TIMER) {
      dhcpc_appcall(ev, data);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
dhcpc_configured(const struct dhcpc_state *s)
{
  uip_sethostaddr(&s->ipaddr);
  uip_setnetmask(&s->netmask);
  uip_setdraddr(&s->default_router);
  resolv_conf(&s->dnsaddr);
  makeaddr(&s->ipaddr, ipaddr);
  printf_P(PSTR("new ip address: %s\n"), ipaddr);
}
/*---------------------------------------------------------------------------*/
void
dhcpc_unconfigured(const struct dhcpc_state *s)
{
}
/*---------------------------------------------------------------------------*/
