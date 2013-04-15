/*
 * Copyright (c) 2007, Swedish Institute of Computer Science
 * All rights reserved.
 *
 *  Additional fixes for AVR contributed by:
 *
 *  David Kopf dak664@embarqmail.com
 *  Ivan Delamer delamer@ieee.com
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
 */
/*
 * This code is almost device independent and should be easy to port.
 * Ported to Atmel RF230 21Feb2010 by dak
 */

#include <stdio.h>
#include <string.h>

#include "contiki.h"

#if defined(__AVR__)
#include <avr/io.h>

//_delay_us has the potential to use floating point which brings the 256 byte clz table into RAM
//#include <util/delay.h>
//#define delay_us( us )   ( _delay_us( ( us ) ) )
//_delay_loop_2(uint16_t count) is 4 CPU cycles per iteration, up to 32 milliseconds at 8MHz
#include <util/delay_basic.h>
#define delay_us( us )   ( _delay_loop_2(1+((unsigned long long)us*F_CPU)/4000000UL) ) 

#include <avr/pgmspace.h>
#elif defined(__MSP430__)
#include <io.h>
#endif

#include "dev/leds.h"
#include "dev/spi.h"
#include "mrf24wb0ma.h"

#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "net/netstack.h"

#include "sys/timetable.h"

#include "contiki-conf.h"
#include "uip.h"
#include "uip_arp.h"
#include "dhcp.h"
#include "queuebuf.h"
#include "dennao_interrupt.h"
#include "process.h"

#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

static uint8_t mac[6];
static uint8_t zg_conn_status;

static uint8_t hdr[5];
static uint8_t intr_occured;
static uint8_t intr_valid;
static uint8_t tx_ready;
static uint8_t rx_ready;
static uint8_t tx_confirm_pending;

static uint8_t drv_buf[RADIO_BUFFER_LEN];
static uint8_t tx_buf[RADIO_BUFFER_LEN];
static uint16_t drv_buf_len;
static uint16_t tx_buf_len;

static uint8_t wpa_psk_key[32];

// local state 
char ssid[MAX_SSID_LENGTH+1];
uint8_t ssid_len;
char security_passphrase[MAX_PASSPHRASE_LENGTH+1];
uint8_t security_passphrase_len;
uint8_t security_type;
uint8_t wireless_mode;
unsigned char wep_keys[];

void drv_spi_transfer(volatile uint8_t* buf, uint16_t len, uint8_t toggle_cs);
void drv_register_interrupt(uint8_t mask, uint8_t state);
void drv_register_interrupt2(uint16_t mask, uint8_t state);
void zg_isr();
void zg_write_wep_key(uint8_t* cmd_buf);
static void zg_calc_psk_key(uint8_t* cmd_buf);
static void zg_write_psk_key(uint8_t* cmd_buf);
void drv_assign_mac(uint8_t* mac_in);
void drv_start_connection();
void drv_install_psk();
void drv_process_rx();
void drv_enable_connection_manager();
void drv_setup_security();
void drv_request_mac();
void drv_process();
void wifi_write_16bit_register(uint8_t reg, uint16_t val);
uint16_t wifi_read_16bit_register(uint8_t reg);
void wifi_write_8bit_register(uint8_t reg, uint8_t val);
uint8_t wifi_read_8bit_register(uint8_t reg);

static int MRF24WB0MA_on(void);
static int MRF24WB0MA_off(void);

static int MRF24WB0MA_read(void *buf, unsigned short bufsize);

static int MRF24WB0MA_prepare(const void *data, unsigned short len);
static int MRF24WB0MA_transmit(unsigned short len);
static int MRF24WB0MA_send(const void *data, unsigned short len);

static int MRF24WB0MA_receiving_packet(void);
static int MRF24WB0MA_pending_packet(void);
static int MRF24WB0MA_cca(void);

const struct radio_driver mrf24wb0ma_driver =
  {
    MRF24WB0MA_init,
    MRF24WB0MA_prepare,
    MRF24WB0MA_transmit,
    MRF24WB0MA_send,
    MRF24WB0MA_read,
    MRF24WB0MA_cca,
    MRF24WB0MA_receiving_packet,
    MRF24WB0MA_pending_packet,
    MRF24WB0MA_on,
    MRF24WB0MA_off
  };
void drv_spi_transfer(volatile uint8_t* buf, uint16_t len, uint8_t toggle_cs){
  WIFI_SS_OFF();
  static uint16_t i;
  for (i = 0; i < len; i++) {
    SPI_WRITE(buf[i]);
    SPI_READ(buf[i]);
  }
  if (toggle_cs){
    WIFI_SS_ON();
  }
  return;
}

void drv_register_interrupt(uint8_t mask, uint8_t state){
  // read the interrupt register
  static uint8_t val;
  val = wifi_read_8bit_register(WF_HOST_MASK_REG);
  if (state == WF_INT_DISABLE){
    val = (val & ~mask);
  }else{
    val = (val & ~mask) | mask;
  }
  // write out new interrupt mask value 
  wifi_write_8bit_register(WF_HOST_MASK_REG, val);

  // ensure that pending interrupts from those updated interrupts are cleared 
  wifi_write_8bit_register(WF_HOST_INTR_REG, mask);
}

void drv_register_interrupt2(uint16_t mask, uint8_t state){
  static uint16_t val;
  val = wifi_read_16bit_register(WF_HOST_INTR2_MASK_REG);
  if (state == WF_INT_DISABLE){
    val &= ~mask;
  }else{
    val |= mask;
  }

  /* write out new interrupt mask value */
  wifi_write_16bit_register(WF_HOST_INTR2_MASK_REG, val);

  /* ensure that pending interrupts from those updated interrupts are cleared */
  wifi_write_16bit_register(WF_HOST_INTR2_REG, mask);
}

void zg_isr(){
  WIFI_ISR_DISABLE();
  intr_occured = 1;
  process_poll(&mrf24wb0ma_process);
}

void wifi_write_16bit_register(uint8_t reg, uint16_t val){
  drv_buf[0] = reg | WF_WRITE_REGISTER_MASK;
  drv_buf[1] = (uint8_t) (val >> 8);
  drv_buf[2] = (uint8_t) (val & 0x00FF);
  drv_spi_transfer(drv_buf, 3, 1);
}

uint16_t wifi_read_16bit_register(uint8_t reg){
  drv_buf[0] = reg | WF_READ_REGISTER_MASK;
  drv_spi_transfer(drv_buf, 3, 1);
  return (((uint16_t)drv_buf[1]) << 8) | ((uint16_t)(drv_buf[2]));
}

void wifi_write_8bit_register(uint8_t reg, uint8_t val){
  drv_buf[0] = reg | WF_WRITE_REGISTER_MASK;
  drv_buf[1] = val;
  drv_spi_transfer(drv_buf, 2, 1);
}

uint8_t wifi_read_8bit_register(uint8_t reg){
  drv_buf[0] = reg | WF_READ_REGISTER_MASK;
  drv_spi_transfer(drv_buf, 2, 1);
  return drv_buf[1];
}

int MRF24WB0MA_init(void){

  printf_P(PSTR("MRF24WB0MA_init\n"));
  /* Initialize Wifi Module */
  uint8_t clr;
  spi_init();
  clr = SPSR;
  clr = SPDR;

  intr_occured = 0;
  intr_valid = 0;
  zg_conn_status = 0;
  tx_ready = 0;
  rx_ready = 0;
  tx_confirm_pending = 0;
  drv_buf_len = RADIO_BUFFER_LEN;
  tx_buf_len = RADIO_BUFFER_LEN;

  wifi_write_16bit_register(WF_PSPOLL_H_REG, 0x0000);
  wifi_write_16bit_register(WF_HOST_RESET_REG, wifi_read_16bit_register(WF_HOST_RESET_REG) | WF_HOST_RESET_MASK);
  wifi_write_16bit_register(WF_HOST_RESET_REG, wifi_read_16bit_register(WF_HOST_RESET_REG) & ~WF_HOST_RESET_MASK);

  wifi_write_16bit_register(WF_INDEX_ADDR_REG, WF_HW_STATUS_REG);

  // TODO: check the return value is not 0xFFFF
  while((wifi_read_16bit_register(WF_INDEX_DATA_REG) & WF_HW_STATUS_NOT_IN_RESET_MASK) == 0);

  while((wifi_read_16bit_register(WF_HOST_WFIFO_BCNT0_REG) & 0x0fff) == 0);

  drv_register_interrupt2(WF_HOST_2_INT_MASK_ALL_INT, WF_INT_DISABLE);
  drv_register_interrupt(WF_HOST_INT_MASK_ALL_INT, WF_INT_DISABLE);
  drv_register_interrupt(WF_HOST_INT_MASK_FIFO_1_THRESHOLD | WF_HOST_INT_MASK_FIFO_0_THRESHOLD | WF_HOST_INT_MASK_RAW_0_INT_0 | WF_HOST_INT_MASK_RAW_1_INT_0 , WF_INT_ENABLE);
  attachInterrupt(2, zg_isr, 0);
  drv_request_mac();
  printf_P(PSTR("MRF24WB0MA_init done!\n"));
  return 1;
}
/*---------------------------------------------------------------------------*/

static int MRF24WB0MA_transmit(unsigned short payload_len){
  printf_P(PSTR("wifi transmit\n"));
  tx_ready = 1;
  process_poll(&mrf24wb0ma_process);
  return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
static int MRF24WB0MA_prepare(const void *payload, unsigned short payload_len)
{
  int ret = 0;
  memcpy(tx_buf, payload, payload_len);
  tx_buf_len = payload_len;
  return ret;
}
/*---------------------------------------------------------------------------*/
static int MRF24WB0MA_send(const void *payload, unsigned short payload_len)
{
	MRF24WB0MA_prepare(payload, payload_len);
  return MRF24WB0MA_transmit(payload_len);
}
/*---------------------------------------------------------------------------*/
static int MRF24WB0MA_off(void)
{
  printf_P(PSTR("TODO:wifi turned off\n"));
  return 0;
}
/*---------------------------------------------------------------------------*/
static int MRF24WB0MA_on(void)
{
  printf_P(PSTR("TODO:wifi turned on\n"));
  return 1;
}
/*---------------------------------------------------------------------------*/
static int MRF24WB0MA_read(void *buf, unsigned short bufsize){
  printf_P(PSTR("TODO:MRF24WB0MA_read\n"));
  return 0;
}
/*---------------------------------------------------------------------------*/
static int MRF24WB0MA_cca(void){
  printf_P(PSTR("TODO:MRF24WB0MA_cca\n"));
	 return 0;
}
/*---------------------------------------------------------------------------*/
int MRF24WB0MA_receiving_packet(void)
{
  printf_P(PSTR("TODO:MRF24WB0MA_receiving_packet\n"));
  return 0;
}
/*---------------------------------------------------------------------------*/
static int MRF24WB0MA_pending_packet(void)
{
  printf_P(PSTR("TODO:MRF24WB0MA_pending_packet\n"));
  return 0;
}
/*---------------------------------------------------------------------------*/

PROCESS(mrf24wb0ma_process, "mrf24wb0ma driver");

void wifi_set_security_type(u8 type){
  security_type = type;
}

void wifi_set_ssid(char* new_ssid){
  strlcpy(ssid, new_ssid, MAX_SSID_LENGTH+1);
  ssid_len = (uint8_t)strlen(ssid);
  if (ssid_len > MAX_SSID_LENGTH){
    ssid_len = MAX_SSID_LENGTH;
  }
}

void wifi_set_passphrase(char* new_passphrase){
  strlcpy(security_passphrase,new_passphrase, MAX_PASSPHRASE_LENGTH+1);
  security_passphrase_len = (uint8_t)strlen(security_passphrase);
  if (security_passphrase_len > MAX_PASSPHRASE_LENGTH){
    security_passphrase_len = MAX_PASSPHRASE_LENGTH;
  }
}

void wifi_connect(){
  queuebuf_init();
  NETSTACK_RADIO.init();
  NETSTACK_RDC.init();
  NETSTACK_MAC.init();
  NETSTACK_NETWORK.init();
  printf("waiting for wifi connection...\n");
  while(wifi_is_connected() != 1) {
    drv_process();
  }
  printf("wifi connection established\n");
  process_start(&mrf24wb0ma_process,NULL);
  struct uip_eth_addr addr;
  wifi_getMAC(addr.addr);
  uip_setethaddr(addr);
  uip_arp_init();
  process_start(&tcpip_process, NULL);
  process_start(&dhcp_process, NULL);
}

//TODO: elimate this function into driver definition
void wifi_prepare_payload(char* buf, uint16_t len){

}

void wifi_set_mode(uint8_t mode){
  wireless_mode = mode;
}

void wifi_getMAC(uint8_t* d){
  memcpy(d,mac,sizeof(mac));
}

// TODO: unknown usage
uint16_t zg_get_rx_status(){
  if (rx_ready) {
    rx_ready = 0;
    return drv_buf_len;
  }
  else {
    return 0;
  }
}

void zg_clear_rx_status(){
  rx_ready = 0;
}

uint8_t wifi_is_connected(){
  return zg_conn_status;
}

void zg_write_wep_key(uint8_t* cmd_buf){
  zg_wep_key_req_t* cmd = (zg_wep_key_req_t*)cmd_buf;

  cmd->slot = 3;    // WEP key slot
  cmd->keyLen = 13; // Key length: 5 bytes (64-bit WEP); 13 bytes (128-bit WEP)
  cmd->defID = 0;   // Default key ID: Key 0, 1, 2, 3
  cmd->ssidLen = ssid_len;
  memset(cmd->ssid, 0x00, 32);
  memcpy(cmd->ssid, ssid, ssid_len);
  memcpy(cmd->key, wep_keys, ZG_MAX_ENCRYPTION_KEYS * ZG_MAX_ENCRYPTION_KEY_SIZE);

  return;
}

static void zg_calc_psk_key(uint8_t* cmd_buf){
  zg_psk_calc_req_t* cmd = (zg_psk_calc_req_t*)cmd_buf;

  cmd->configBits = 0;
  cmd->phraseLen = security_passphrase_len;
  cmd->ssidLen = ssid_len;
  cmd->reserved = 0;
  memset(cmd->ssid, 0x00, 32);
  memcpy(cmd->ssid, ssid, ssid_len);
  memset(cmd->passPhrase, 0x00, 64);
  memcpy(cmd->passPhrase, security_passphrase, security_passphrase_len);

  return;
}

static void zg_write_psk_key(uint8_t* cmd_buf){
  zg_pmk_key_req_t* cmd = (zg_pmk_key_req_t*)cmd_buf;

  cmd->slot = 0;  // WPA/WPA2 PSK slot
  cmd->ssidLen = ssid_len;
  memset(cmd->ssid, 0x00, 32);
  memcpy(cmd->ssid, ssid, cmd->ssidLen);
  memcpy(cmd->keyData, wpa_psk_key, ZG_MAX_PMK_LEN);

  return;
}

PROCESS_THREAD(mrf24wb0ma_process, ev, data){
  PROCESS_BEGIN();
    DEBUG_PRINT("process started!\n");
  while(1) {
    PROCESS_WAIT_EVENT();
    drv_process();
  }
  PROCESS_END();
}

void drv_assign_mac(uint8_t* mac_in){
  memcpy(mac, mac_in, 6);
}

void drv_start_connection(){
  zg_connect_req_t* cmd = (zg_connect_req_t*)&drv_buf[3];

  // start connection to AP
  drv_buf[0] = ZG_CMD_WT_FIFO_MGMT;
  drv_buf[1] = ZG_MAC_TYPE_MGMT_REQ;
  drv_buf[2] = ZG_MAC_SUBTYPE_MGMT_REQ_CONNECT;

  cmd->secType = security_type;

  cmd->ssidLen = ssid_len;
  memcpy(cmd->ssid, ssid, ssid_len);
  memset(cmd->ssid + ssid_len, 0, ZG_MAX_SSID_LENGTH - ssid_len);
  DEBUG_PRINT("Connecting with SSID: %s\n", cmd->ssid);

  // units of 100 milliseconds
  cmd->sleepDuration = 0;

  if (wireless_mode == WIRELESS_MODE_INFRA){
    cmd->modeBss = 1;
  }else if(wireless_mode == WIRELESS_MODE_ADHOC){
    cmd->modeBss = 2;
  }

  drv_spi_transfer(drv_buf, ZG_CONNECT_REQ_SIZE+3, 1);

  drv_buf[0] = ZG_CMD_WT_FIFO_DONE;
  drv_spi_transfer(drv_buf, 1, 1);
}
void drv_install_psk(){
  // Install the PSK key on G2100
  drv_buf[0] = ZG_CMD_WT_FIFO_MGMT;
  drv_buf[1] = ZG_MAC_TYPE_MGMT_REQ;
  drv_buf[2] = ZG_MAC_SUBTYPE_MGMT_REQ_PMK_KEY;
  zg_write_psk_key(&drv_buf[3]);
  drv_spi_transfer(drv_buf, ZG_PMK_KEY_REQ_SIZE+3, 1);

  drv_buf[0] = ZG_CMD_WT_FIFO_DONE;
  drv_spi_transfer(drv_buf, 1, 1);
}

void drv_process_rx(){
    // TODO: mind buf size when copying.
  zg_rx_data_ind_t* ptr = (zg_rx_data_ind_t*)&(drv_buf[3]);
  drv_buf_len = ZGSTOHS( ptr->dataLen );

  memcpy(&drv_buf[0], &drv_buf[5], 6);
  memcpy(&drv_buf[6], &drv_buf[11], 6);
  memcpy(&drv_buf[12], &drv_buf[29], drv_buf_len);

  drv_buf_len += 12;
  rx_ready = 1;

  packetbuf_copyfrom(drv_buf, drv_buf_len);
  packetbuf_set_datalen(drv_buf_len);
  NETSTACK_RDC.input();
}

void drv_enable_connection_manager(){
  // enable connection manager
  drv_buf[0] = ZG_CMD_WT_FIFO_MGMT;
  drv_buf[1] = ZG_MAC_TYPE_MGMT_REQ;
  drv_buf[2] = ZG_MAC_SUBTYPE_MGMT_REQ_CONNECT_MANAGE;
  drv_buf[3] = 0x01;  // 0x01 - enable; 0x00 - disable
  drv_buf[4] = 10;    // num retries to reconnect
  drv_buf[5] = 0x10 | 0x02 | 0x01;  // 0x10 - enable start and stop indication messages
                  //      from G2100 during reconnection
                  // 0x02 - start reconnection on receiving a deauthentication
                  //      message from the AP
                  // 0x01 - start reconnection when the missed beacon count
                  //      exceeds the threshold. uses default value of
                  //      100 missed beacons if not set during initialization
  drv_buf[6] = 0;
  drv_spi_transfer(drv_buf, 7, 1);
  drv_buf[0] = ZG_CMD_WT_FIFO_DONE;
  drv_spi_transfer(drv_buf, 1, 1);
}
void drv_setup_security(){
  switch (security_type) {
    case ZG_SECURITY_TYPE_NONE:
      drv_enable_connection_manager();
      break;
    case ZG_SECURITY_TYPE_WEP:
      // Install all four WEP keys on G2100
      drv_buf[0] = ZG_CMD_WT_FIFO_MGMT;
      drv_buf[1] = ZG_MAC_TYPE_MGMT_REQ;
      drv_buf[2] = ZG_MAC_SUBTYPE_MGMT_REQ_WEP_KEY;
      zg_write_wep_key(&drv_buf[3]);
      drv_spi_transfer(drv_buf, ZG_WEP_KEY_REQ_SIZE+3, 1);

      drv_buf[0] = ZG_CMD_WT_FIFO_DONE;
      drv_spi_transfer(drv_buf, 1, 1);
      break;
    case ZG_SECURITY_TYPE_WPA:
    case ZG_SECURITY_TYPE_WPA2:
      // Initiate PSK calculation on G2100
      drv_buf[0] = ZG_CMD_WT_FIFO_MGMT;
      drv_buf[1] = ZG_MAC_TYPE_MGMT_REQ;
      drv_buf[2] = ZG_MAC_SUBTYPE_MGMT_REQ_CALC_PSK;
      zg_calc_psk_key(&drv_buf[3]);
      drv_spi_transfer(drv_buf, ZG_PSK_CALC_REQ_SIZE+3, 1);

      drv_buf[0] = ZG_CMD_WT_FIFO_DONE;
      drv_spi_transfer(drv_buf, 1, 1);
      break;
    default:
      break;
  }
}

void drv_request_mac(){
  drv_buf[0] = ZG_CMD_WT_FIFO_MGMT;
  drv_buf[1] = ZG_MAC_TYPE_MGMT_REQ;
  drv_buf[2] = ZG_MAC_SUBTYPE_MGMT_REQ_GET_PARAM;
  drv_buf[3] = 0;
  drv_buf[4] = ZG_PARAM_MAC_ADDRESS;
  drv_spi_transfer(drv_buf, 5, 1);
  drv_buf[0] = ZG_CMD_WT_FIFO_DONE;
  drv_spi_transfer(drv_buf, 1, 1);
}

void drv_process(){
  if (tx_ready && !tx_confirm_pending) {
    hdr[0] = ZG_CMD_WT_FIFO_DATA;
    hdr[1] = ZG_MAC_TYPE_TXDATA_REQ;
    hdr[2] = ZG_MAC_SUBTYPE_TXDATA_REQ_STD;
    hdr[3] = 0x00;
    hdr[4] = 0x00;
    drv_spi_transfer(hdr, 5, 0);

    tx_buf[6] = 0xaa;
    tx_buf[7] = 0xaa;
    tx_buf[8] = 0x03;
    tx_buf[9] = tx_buf[10] = tx_buf[11] = 0x00;
    drv_spi_transfer(tx_buf, tx_buf_len, 1);

    hdr[0] = ZG_CMD_WT_FIFO_DONE;
    drv_spi_transfer(hdr, 1, 1);
      tx_ready = 0;
      tx_confirm_pending = 1;
    }

  if (intr_occured) {
    uint8_t next_cmd = 0;

    hdr[0] = 0x40 | ZG_INTR_REG;
    hdr[1] = 0x00;
    hdr[2] = 0x00;
    drv_spi_transfer(hdr, 3, 1);

    uint8_t intr_val = hdr[1] & hdr[2];

    if ( (intr_val & ZG_INTR_MASK_FIFO1) == ZG_INTR_MASK_FIFO1) {
      hdr[0] = ZG_INTR_REG;
      hdr[1] = ZG_INTR_MASK_FIFO1;
      drv_spi_transfer(hdr, 2, 1);
      next_cmd = ZG_BYTE_COUNT_FIFO1_REG;
    } else if ( (intr_val & ZG_INTR_MASK_FIFO0) == ZG_INTR_MASK_FIFO0) {
      hdr[0] = ZG_INTR_REG;
      hdr[1] = ZG_INTR_MASK_FIFO0;
      drv_spi_transfer(hdr, 2, 1);
      next_cmd = ZG_BYTE_COUNT_FIFO0_REG;
    }

    if (next_cmd != 0){
      hdr[0] = 0x40 | next_cmd;
      hdr[1] = 0x00;
      hdr[2] = 0x00;
      drv_spi_transfer(hdr, 3, 1);
      uint16_t rx_byte_cnt = (0x0000 | (hdr[1] << 8) | hdr[2]) & 0x0fff;
      drv_buf[0] = ZG_CMD_RD_FIFO;
      drv_spi_transfer(drv_buf, rx_byte_cnt + 1, 1);
      hdr[0] = ZG_CMD_RD_FIFO_DONE;
      drv_spi_transfer(hdr, 1, 1);
      intr_valid = 1;
    }
    intr_occured = 0;
    WIFI_ISR_ENABLE();
  }

  if (intr_valid) {
    switch (drv_buf[1]) {
    case ZG_MAC_TYPE_TXDATA_CONFIRM:
      DEBUG_PRINT("Tx Confirm!\n");
      tx_confirm_pending = 0;
      break;
    case ZG_MAC_TYPE_MGMT_CONFIRM:
      if (drv_buf[3] == ZG_RESULT_SUCCESS) {
        switch (drv_buf[2]) {
        case ZG_MAC_SUBTYPE_MGMT_REQ_GET_PARAM:
          drv_assign_mac(drv_buf+7);
          drv_setup_security();
          break;
        case ZG_MAC_SUBTYPE_MGMT_REQ_WEP_KEY:
          drv_enable_connection_manager();
          break;
        case ZG_MAC_SUBTYPE_MGMT_REQ_CALC_PSK:
          memcpy(wpa_psk_key, ((zg_psk_calc_cnf_t*)&drv_buf[3])->psk, 32);
          drv_install_psk();
          break;
        case ZG_MAC_SUBTYPE_MGMT_REQ_PMK_KEY:
          drv_enable_connection_manager();
          break;
        case ZG_MAC_SUBTYPE_MGMT_REQ_CONNECT_MANAGE:
          drv_start_connection();
          break;
        case ZG_MAC_SUBTYPE_MGMT_REQ_CONNECT:
          zg_conn_status = 1;
          break;
        default:
          DEBUG_PRINT("Unknown MAC subtype %d\n", drv_buf[2]);
          break;
        }
      }else{
        drv_request_mac();
        DEBUG_PRINT("request failed!\n");
      }
      break;
    case ZG_MAC_TYPE_RXDATA_INDICATE:
      drv_process_rx();
      break;
    case ZG_MAC_TYPE_MGMT_INDICATE:
      switch (drv_buf[2]) {
      case ZG_MAC_SUBTYPE_MGMT_IND_DISASSOC:
      case ZG_MAC_SUBTYPE_MGMT_IND_DEAUTH:
        DEBUG_PRINT("Wifi Disconnected!\n");
        zg_conn_status = 0;
        drv_start_connection();
        break;
      case ZG_MAC_SUBTYPE_MGMT_IND_CONN_STATUS:
        {
          uint16_t status = (((uint16_t)(drv_buf[3]))<<8)|drv_buf[4];
          if(status == 1 || status == 5){
            DEBUG_PRINT("Wifi Disconnected!\n");
            zg_conn_status = 0;
          }else if(status == 2 || status == 6) {
            zg_conn_status = 1;
          }
        }
        break;
      }
      break;
    }
    intr_valid = 0;
  }
}
