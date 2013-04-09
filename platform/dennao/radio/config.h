#ifndef RADIO_CONFIG_H
#define RADIO_CONFIG_H

#define RADIO_BUFFER_LEN 1000

#include <stdint.h>
#include <avr/pgmspace.h>

#define u8 uint8_t
#define u16 uint16_t
#define U8 uint8_t
#define U16 uint16_t

extern char ssid[];
extern U8 ssid_len;
extern char security_passphrase[];
extern U8 security_passphrase_len;
extern U8 security_type;
extern U8 wireless_mode;

extern unsigned char wep_keys[];

#define WIRELESS_MODE_INFRA	1
#define WIRELESS_MODE_ADHOC	2

#endif