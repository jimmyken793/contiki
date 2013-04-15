#include "contiki-net.h"
#include "ctk/ctk.h"
#include "net/dhcpc.h"



PROCESS(dhcp_process, "DHCP");

AUTOSTART_PROCESSES(&dhcp_process);

static struct ctk_window window;
static struct ctk_button getbutton =
  {CTK_BUTTON(0, 0, 16, "Request address")};
static struct ctk_label statuslabel =
  {CTK_LABEL(0, 1, 16, 1, "")};


static struct ctk_label ipaddrlabel =
  {CTK_LABEL(0, 3, 10, 1, "IP address")};
static char ipaddr[17];
static struct ctk_textentry ipaddrentry =
  {CTK_LABEL(11, 3, 16, 1, ipaddr)};
static struct ctk_label netmasklabel =
  {CTK_LABEL(0, 4, 10, 1, "Netmask")};
static char netmask[17];
static struct ctk_textentry netmaskentry =
  {CTK_LABEL(11, 4, 16, 1, netmask)};
static struct ctk_label gatewaylabel =
  {CTK_LABEL(0, 5, 10, 1, "Gateway")};
static char gateway[17];
static struct ctk_textentry gatewayentry =
  {CTK_LABEL(11, 5, 16, 1, gateway)};
static struct ctk_label dnsserverlabel =
  {CTK_LABEL(0, 6, 10, 1, "DNS server")};
static char dnsserver[17];
static struct ctk_textentry dnsserverentry =
  {CTK_LABEL(11, 6, 16, 1, dnsserver)};

enum {
  SHOWCONFIG
};
/*---------------------------------------------------------------------------*/
static void
set_statustext(char *text)
{
  ctk_label_set_text(&statuslabel, text);
  CTK_WIDGET_REDRAW(&statuslabel);
}
/*---------------------------------------------------------------------------*/
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
/*---------------------------------------------------------------------------*/
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
static void
makestrings(void)
{
  uip_ipaddr_t addr, *addrptr;

  uip_gethostaddr(&addr);
  makeaddr(&addr, ipaddr);
  
  uip_getnetmask(&addr);
  makeaddr(&addr, netmask);
  
  uip_getdraddr(&addr);
  makeaddr(&addr, gateway);

  addrptr = resolv_getserver();
  if(addrptr != NULL) {
    makeaddr(addrptr, dnsserver);
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(dhcp_process, ev, data)
{
  PROCESS_BEGIN();
  printf("%02x:%02x:%02x:%02x:%02x:%02x\n",uip_lladdr.addr[0],uip_lladdr.addr[1],uip_lladdr.addr[2],uip_lladdr.addr[3],uip_lladdr.addr[4],uip_lladdr.addr[5]);
  
  dhcpc_init(uip_lladdr.addr, sizeof(uip_lladdr.addr));
  dhcpc_request();
  printf("dhcpc started!\n");

  while(1) {
    PROCESS_WAIT_EVENT();
  printf("dhcpc event! %d\n",ev);
    if(ev == tcpip_event) {
  printf("dhcpc tcpip event!\n");
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
  printf("new ip address: %s\n", ipaddr);
}
/*---------------------------------------------------------------------------*/
void
dhcpc_unconfigured(const struct dhcpc_state *s)
{
}
/*---------------------------------------------------------------------------*/
