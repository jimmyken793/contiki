
/*****************************************************************************

  Filename:		g2100.c
  Description:	Driver for the ZeroG Wireless G2100 series devices

 *****************************************************************************

  Driver for the WiShield 1.0 wireless devices

  Copyright(c) 2009 Async Labs Inc. All rights reserved.

  This program is free software; you can redistribute it and/or modify it
  under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc., 59
  Temple Place - Suite 330, Boston, MA  02111-1307, USA.

  Contact Information:
  <asynclabs@asynclabs.com>

   Author               Date        Comment
  ----------------------------------------------------------------------------
   AsyncLabs			02/25/2009	Initial port
   AsyncLabs			05/29/2009	Adding support for new library

 *****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <avr/io.h> 
#include "dev/spi.h"
#include "contiki-conf.h"
#include "uip.h"
#include "uip_arp.h"
#include "dhcp.h"
#include "queuebuf.h"
#include "dennao_interrupt.h"
#include "process.h"
#include "g2100.h"
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
static uint16_t drv_buf_len;

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
void drv_interrupt_register(uint8_t mask, uint8_t state);
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
	memcpy(drv_buf, buf, len);
	drv_buf_len = len;
}

void wifi_set_mode(uint8_t mode){
	wireless_mode = mode;
}

void wifi_send(){
	tx_ready = 1;
	process_poll(&mrf24wb0ma_process);
}

void wifi_getMAC(uint8_t* d){
	memcpy(d,mac,sizeof(mac));
}

void zg_init(){
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

	// reset chip
	// write reset register addr
	hdr[0] = ZG_INDEX_ADDR_REG;
	hdr[1] = 0x00;
	hdr[2] = ZG_RESET_REG;
	drv_spi_transfer(hdr, 3, 1);

	hdr[0] = ZG_INDEX_DATA_REG;
	hdr[1] = 0x80;
	hdr[2] = 0xff;
	drv_spi_transfer(hdr, 3, 1);

	// write reset register addr
	hdr[0] = ZG_INDEX_ADDR_REG;
	hdr[1] = 0x00;
	hdr[2] = ZG_RESET_REG;
	drv_spi_transfer(hdr, 3, 1);

	hdr[0] = ZG_INDEX_DATA_REG;
	hdr[1] = 0x0f;
	hdr[2] = 0xff;
	drv_spi_transfer(hdr, 3, 1);

	// write reset register data
	hdr[0] = ZG_INDEX_ADDR_REG;
	hdr[1] = 0x00;
	hdr[2] = ZG_RESET_STATUS_REG;
	drv_spi_transfer(hdr, 3, 1);
	do {
		hdr[0] = 0x40 | ZG_INDEX_DATA_REG;
		hdr[1] = 0x00;
		hdr[2] = 0x00;
		drv_spi_transfer(hdr, 3, 1);
	} while((hdr[1] & ZG_RESET_MASK) == 0);
	do {
		hdr[0] = 0x40 | ZG_BYTE_COUNT_REG;
		hdr[1] = 0x00;
		hdr[2] = 0x00;
		drv_spi_transfer(hdr, 3, 1);
	} while((hdr[1] == 0) && (hdr[2] == 0));
	//end reset chip

	// register interrupt2
	// read the interrupt2 mask register
	hdr[0] = 0x40 | ZG_INTR2_MASK_REG;
	hdr[1] = 0x00;
	hdr[2] = 0x00;
	drv_spi_transfer(hdr, 3, 1);

	// modify the interrupt mask value and re-write the value to the interrupt
	// mask register clearing the interrupt register first
	hdr[0] = ZG_INTR2_REG;
	hdr[1] = 0xff;
	hdr[2] = 0xff;
	hdr[3] = 0;
	hdr[4] = 0;
	drv_spi_transfer(hdr, 5, 1);
	// end register interrupt2

	drv_interrupt_register(0xff, 0);
	drv_interrupt_register(0x80|0x40, 1);
	attachInterrupt(2, zg_isr, 0);
	drv_request_mac();
}

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

void drv_interrupt_register(uint8_t mask, uint8_t state){
	// read the interrupt register
	hdr[0] = 0x40 | ZG_INTR_MASK_REG;
	hdr[1] = 0x00;
	drv_spi_transfer(hdr, 2, 1);
	// now regBuf[0] contains the current setting for the
	// interrupt mask register
	// this is to clear any currently set interrupts of interest
	hdr[0] = ZG_INTR_REG;
	hdr[2] = (hdr[1] & ~mask) | ( (state == 0)? 0 : mask );
	hdr[1] = mask;
	drv_spi_transfer(hdr, 3, 1);
}

void zg_isr(){
	//printf("interrupt!\n");
	WIFI_ISR_DISABLE();
	intr_occured = 1;
	process_poll(&mrf24wb0ma_process);
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

	cmd->slot = 3;		// WEP key slot
	cmd->keyLen = 13;	// Key length: 5 bytes (64-bit WEP); 13 bytes (128-bit WEP)
	cmd->defID = 0;		// Default key ID: Key 0, 1, 2, 3
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

	cmd->slot = 0;	// WPA/WPA2 PSK slot
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
	drv_buf[3] = 0x01;	// 0x01 - enable; 0x00 - disable
	drv_buf[4] = 10;		// num retries to reconnect
	drv_buf[5] = 0x10 | 0x02 | 0x01;	// 0x10 -	enable start and stop indication messages
									// 		 	from G2100 during reconnection
									// 0x02 -	start reconnection on receiving a deauthentication
									// 			message from the AP
									// 0x01 -	start reconnection when the missed beacon count
									// 			exceeds the threshold. uses default value of
									//			100 missed beacons if not set during initialization
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

		drv_buf[6] = 0xaa;
		drv_buf[7] = 0xaa;
		drv_buf[8] = 0x03;
		drv_buf[9] = drv_buf[10] = drv_buf[11] = 0x00;
		drv_spi_transfer(drv_buf, drv_buf_len, 1);

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
			DEBUG_PRINT("ZG_MAC_TYPE_TXDATA_CONFIRM\n");
			tx_confirm_pending = 0;
			break;
		case ZG_MAC_TYPE_MGMT_CONFIRM:
			DEBUG_PRINT("ZG_MAC_TYPE_MGMT_CONFIRM\n");
			if (drv_buf[3] == ZG_RESULT_SUCCESS) {
				switch (drv_buf[2]) {
				case ZG_MAC_SUBTYPE_MGMT_REQ_GET_PARAM:
					DEBUG_PRINT("ZG_MAC_SUBTYPE_MGMT_REQ_GET_PARAM\n");
					drv_assign_mac(drv_buf+7);
					drv_setup_security();
					break;
				case ZG_MAC_SUBTYPE_MGMT_REQ_WEP_KEY:
					DEBUG_PRINT("ZG_MAC_SUBTYPE_MGMT_REQ_WEP_KEY\n");
					drv_enable_connection_manager();
					break;
				case ZG_MAC_SUBTYPE_MGMT_REQ_CALC_PSK:
					DEBUG_PRINT("ZG_MAC_SUBTYPE_MGMT_REQ_CALC_PSK\n");
					memcpy(wpa_psk_key, ((zg_psk_calc_cnf_t*)&drv_buf[3])->psk, 32);
					drv_install_psk();
					break;
				case ZG_MAC_SUBTYPE_MGMT_REQ_PMK_KEY:
					DEBUG_PRINT("ZG_MAC_SUBTYPE_MGMT_REQ_PMK_KEY\n");
					drv_enable_connection_manager();
					break;
				case ZG_MAC_SUBTYPE_MGMT_REQ_CONNECT_MANAGE:
					DEBUG_PRINT("ZG_MAC_SUBTYPE_MGMT_REQ_CONNECT_MANAGE\n");
					drv_start_connection();
					break;
				case ZG_MAC_SUBTYPE_MGMT_REQ_CONNECT:
					DEBUG_PRINT("ZG_MAC_SUBTYPE_MGMT_REQ_CONNECT\n");
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
			DEBUG_PRINT("ZG_MAC_TYPE_RXDATA_INDICATE\n");
			drv_process_rx();
			break;
		case ZG_MAC_TYPE_MGMT_INDICATE:
			DEBUG_PRINT("ZG_MAC_TYPE_MGMT_INDICATE\n");
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
