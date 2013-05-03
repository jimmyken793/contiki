#ifndef __UIP_DRIVER_H__
#define __UIP_DRIVER_H__

#include "net/netstack.h"

uint8_t uip_driver_send(void);

extern const struct network_driver uip_driver;

#endif /* __UIP_DRIVER_H__ */
