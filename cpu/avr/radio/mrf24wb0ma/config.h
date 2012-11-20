#ifndef RADIO_CONFIG_H
#define RADIO_CONFIG_H

#define RADIO_BUFFER_LEN 400

#include <stdint.h>
#include <avr/pgmspace.h>

#define u8 uint8_t
#define u16 uint16_t
#define U8 uint8_t
#define U16 uint16_t

extern U8 local_ip[];
extern U8 gateway_ip[];
extern U8 subnet_mask[];
extern const prog_char ssid[];
extern U8 ssid_len;
extern const prog_char security_passphrase[];
extern U8 security_passphrase_len;
extern U8 security_type;
extern U8 wireless_mode;

extern prog_uchar wep_keys[];

extern const prog_char webpage[];
extern const prog_char twitter[];
extern unsigned char mfg_id[4];

#define WIRELESS_MODE_INFRA	1
#define WIRELESS_MODE_ADHOC	2

#endif