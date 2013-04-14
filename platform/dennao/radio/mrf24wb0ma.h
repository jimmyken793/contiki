/*   Copyright (c) 2008, Swedish Institute of Computer Science
 *  All rights reserved.
 *
 *  Additional fixes for AVR contributed by:
 *
 *	Colin O'Flynn coflynn@newae.com
 *	Eric Gnoske egnoske@gmail.com
 *	Blake Leverett bleverett@gmail.com
 *	Mike Vidales mavida404@gmail.com
 *	Kevin Brown kbrown3@uccs.edu
 *	Nate Bohlmann nate@elfwerks.com
 *  David Kopf dak664@embarqmail.com
 *  Ivan Delamer delamer@ieee.com
 *
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of the copyright holders nor the names of
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
/**
 *    \addtogroup radiorf230
 *   @{
 */
/**
 *  \file
 *  \brief This file contains radio driver code.
 *
 */

#ifndef RADIO_H
#define RADIO_H
/*============================ INCLUDE =======================================*/
#include <stdint.h>
#include <stdbool.h>
#include "atmega128rfa1_registermap.h"
#include "dev/radio.h"
#include "dennao_interrupt.h"
#include "config.h"
#include "sys/process.h"
#include "net/netstack.h"

#define RADIO_STATUS_START_VALUE                  ( 0x40 )

/** \brief  This enumeration defines the possible return values for the TAT API
 *          functions.
 *
 *          These values are defined so that they should not collide with the
 *          return/status codes defined in the IEEE 802.15.4 standard.
 *
 */
typedef enum{
    RADIO_SUCCESS = RADIO_STATUS_START_VALUE,  /**< The requested service was performed successfully. */
    RADIO_UNSUPPORTED_DEVICE,         /**< The connected device is not an Atmel AT86RF230. */
    RADIO_INVALID_ARGUMENT,           /**< One or more of the supplied function arguments are invalid. */
    RADIO_TIMED_OUT,                  /**< The requested service timed out. */
    RADIO_WRONG_STATE,                /**< The end-user tried to do an invalid state transition. */
    RADIO_BUSY_STATE,                 /**< The radio transceiver is busy receiving or transmitting. */
    RADIO_STATE_TRANSITION_FAILED,    /**< The requested state transition could not be completed. */
    RADIO_CCA_IDLE,                   /**< Channel is clear, available to transmit a new frame. */
    RADIO_CCA_BUSY,                   /**< Channel busy. */
    RADIO_TRX_BUSY,                   /**< Transceiver is busy receiving or transmitting data. */
    RADIO_BAT_LOW,                    /**< Measured battery voltage is lower than voltage threshold. */
    RADIO_BAT_OK,                     /**< Measured battery voltage is above the voltage threshold. */
    RADIO_CRC_FAILED,                 /**< The CRC failed for the actual frame. */
    RADIO_CHANNEL_ACCESS_FAILURE,     /**< The channel access failed during the auto mode. */
    RADIO_NO_ACK,                     /**< No acknowledge frame was received. */
}radio_status_t;


/*============================ PROTOTYPES ====================================*/

const struct radio_driver rf230_driver;

int MRF24WB0MA_init(void);
void MRF24WB0MA_warm_reset(void);
void MRF24WB0MA_start_sneeze(void);
void MRF24WB0MA_set_channel(uint8_t channel);
void MRF24WB0MA_listen_channel(uint8_t channel);
uint8_t MRF24WB0MA_get_channel(void);
void MRF24WB0MA_set_pan_addr(unsigned pan,unsigned addr,const uint8_t ieee_addr[8]);
void MRF24WB0MA_set_txpower(uint8_t power);
uint8_t MRF24WB0MA_get_txpower(void);

void MRF24WB0MA_set_promiscuous_mode(bool isPromiscuous);
bool MRF24WB0MA_is_ready_to_send();

extern uint8_t rf230_last_correlation,rf230_last_rssi,rf230_smallest_rssi;

uint8_t MRF24WB0MA_get_raw_rssi(void);

PROCESS_NAME(mrf24wb0ma_process);

#define MAX_SSID_LENGTH 32
#define MAX_PASSPHRASE_LENGTH 64

#define WIFI_ISR_DISABLE() (EIMSK &= ~(0x01))
#define WIFI_ISR_ENABLE() (EIMSK |= 0x01)

#define WIFI_SS_ON() (PORTB |= BV(CSN))
#define WIFI_SS_OFF() (PORTB &= ~BV(CSN))

//Host to Zero G long
#define HTOZGL(a) ( ((a & 0x000000ff)<<24) \
                    |((a & 0x0000ff00)<<8)  \
                    |((a & 0x00ff0000)>>8)  \
                    |((a & 0xff000000)>>24) )
#define ZGTOHL(a) HTOZGL(a)

// Host to Zero G short
#define HSTOZGS(a) (u16)(((a)<<8) | ((a)>>8))
#define ZGSTOHS(a) HSTOZGS(a)
//#define HTONS(a) HSTOZGS(a)


// Command values which appear in ZG_PREAMBLE_CMD_IDX for each SPI message
#define ZG_CMD_FIFO_ACCESS          (0x80)
#define ZG_CMD_WT_FIFO_DATA         (ZG_CMD_FIFO_ACCESS | 0x20)
#define ZG_CMD_WT_FIFO_MGMT         (ZG_CMD_FIFO_ACCESS | 0x30)
#define ZG_CMD_RD_FIFO              (ZG_CMD_FIFO_ACCESS | 0x00)
#define ZG_CMD_WT_FIFO_DONE         (ZG_CMD_FIFO_ACCESS | 0x40)
#define ZG_CMD_RD_FIFO_DONE         (ZG_CMD_FIFO_ACCESS | 0x50)
#define ZG_CMD_WT_REG               (0x00)
#define ZG_CMD_RD_REG               (0x40)

// Type values which appear in ZG_PREAMBLE_TYPE_IDX for each SPI message
#define ZG_MAC_TYPE_TXDATA_REQ      ((u8)1)
#define ZG_MAC_TYPE_MGMT_REQ        ((u8)2)

#define ZG_MAC_TYPE_TXDATA_CONFIRM  ((u8)1)
#define ZG_MAC_TYPE_MGMT_CONFIRM    ((u8)2)
#define ZG_MAC_TYPE_RXDATA_INDICATE ((u8)3)
#define ZG_MAC_TYPE_MGMT_INDICATE   ((u8)4)

// Subtype values which appear in ZG_PREAMBLE_SUBTYPE_IDX for each SPI message
// Subtype for ZG_MAC_TYPE_TXDATA_REQ and ZG_MAC_TYPE_TXDATA_CONFIRM
#define ZG_MAC_SUBTYPE_TXDATA_REQ_STD           ((u8)1)

// Subtype for ZG_MAC_TYPE_MGMT_REQ and ZG_MAC_TYPE_MGMT_CONFIRM
#define ZG_MAC_SUBTYPE_MGMT_REQ_PMK_KEY         ((u8)8)
#define ZG_MAC_SUBTYPE_MGMT_REQ_WEP_KEY         ((u8)10)
#define ZG_MAC_SUBTYPE_MGMT_REQ_CALC_PSK        ((u8)12)
#define ZG_MAC_SUBTYPE_MGMT_REQ_SET_PARAM       ((u8)15)
#define ZG_MAC_SUBTYPE_MGMT_REQ_GET_PARAM       ((u8)16)
#define ZG_MAC_SUBTYPE_MGMT_REQ_ADHOC_START     ((u8)18)
#define ZG_MAC_SUBTYPE_MGMT_REQ_CONNECT         ((u8)19)
#define ZG_MAC_SUBTYPE_MGMT_REQ_CONNECT_MANAGE  ((u8)20)

// Subtype for ZG_MAC_TYPE_RXDATA_INDICATE
#define ZG_MAC_SUBTYPE_RXDATA_IND_STD           ((u8)1)

// Subtype for ZG_MAC_TYPE_MGMT_INDICATE
#define ZG_MAC_SUBTYPE_MGMT_IND_DISASSOC        ((u8)1)
#define ZG_MAC_SUBTYPE_MGMT_IND_DEAUTH          ((u8)2)
#define ZG_MAC_SUBTYPE_MGMT_IND_CONN_STATUS     ((u8)4)

// Parameter IDs for ZG_MAC_SUBTYPE_MGMT_REQ_SET_PARAM
#define ZG_PARAM_MAC_ADDRESS            (1)

// MAC result code
enum {
    ZG_RESULT_SUCCESS = 1,
    ZG_RESULT_INVALID_SUBTYPE,
    ZG_RESULT_CANCELLED,
    ZG_RESULT_FRAME_EOL,
    ZG_RESULT_FRAME_RETRY_LIMIT,
    ZG_RESULT_FRAME_NO_BSS,
    ZG_RESULT_FRAME_TOO_BIG,
    ZG_RESULT_FRAME_ENCRYPT_FAILURE,
    ZG_RESULT_INVALID_PARAMS,
    ZG_RESULT_ALREADY_AUTH,
    ZG_RESULT_ALREADY_ASSOC,
    ZG_RESULT_INSUFFICIENT_RSRCS,
    ZG_RESULT_TIMEOUT,
    ZG_RESULT_BAD_EXCHANGE,     // frame exchange problem with peer (AP or STA)
    ZG_RESULT_AUTH_REFUSED,     // authenticating node refused our request
    ZG_RESULT_ASSOC_REFUSED,    // associating node refused our request
    ZG_RESULT_REQ_IN_PROGRESS,  // only one mlme request at a time allowed
    ZG_RESULT_NOT_JOINED,       // operation requires that device be joined
                                // with target
    ZG_RESULT_NOT_ASSOC,        // operation requires that device be
                                // associated with target
    ZG_RESULT_NOT_AUTH,         // operation requires that device be
                                // authenticated with target
    ZG_RESULT_SUPPLICANT_FAILED,
    ZG_RESULT_UNSUPPORTED_FEATURE,
    ZG_RESULT_REQUEST_OUT_OF_SYNC   // Returned when a request is recognized
                                                  // but invalid given the current state
                                                  // of the MAC
};

#define WF_READ_REGISTER_MASK           ((uint8_t)(0x40))
#define WF_WRITE_REGISTER_MASK          ((uint8_t)(0x00))

/*--------------------------------*/
/* MRF24W 8-bit Host Registers */
/*--------------------------------*/
#define WF_HOST_INTR_REG            ((uint8_t)(0x01))  /* 8-bit register containing 1st level interrupt bits. */
#define WF_HOST_MASK_REG            ((uint8_t)(0x02))  /* 8-bit register containing 1st level interrupt mask. */

/*---------------------------------*/
/* MRF24W 16-bit Host Registers */
/*---------------------------------*/
#define WF_HOST_RAW0_CTRL1_REG      ((uint16_t)(0x26))
#define WF_HOST_RAW0_STATUS_REG     ((uint16_t)(0x28))
#define WF_HOST_RAW1_CTRL1_REG      ((uint16_t)(0x2a))
#define WF_HOST_INTR2_REG           ((uint16_t)(0x2d)) /* 16-bit register containing 2nd level interrupt bits */
#define WF_HOST_INTR2_MASK_REG      ((uint16_t)(0x2e))
#define WF_HOST_WFIFO_BCNT0_REG     ((uint16_t)(0x2f)) /* 16-bit register containing available write size for fifo 0 (data)   */
                                                     /* (LS 12 bits contain the length)                                     */
                                                       
#define WF_HOST_WFIFO_BCNT1_REG     ((uint16_t)(0x31)) /* 16-bit register containing available write size for fifo 1 (mgmt)   */
                                                     /* (LS 12 bits contain the length)                                     */

#define WF_HOST_RFIFO_BCNT0_REG     ((uint16_t)(0x33)) /* 16-bit register containing number of bytes in read fifo 0 (data rx) */
                                                     /* (LS 12 bits contain the length)                                     */
#define WF_HOST_RESET_REG           ((uint16_t)(0x3c))
#define WF_HOST_RESET_MASK          ((uint16_t)(0x0001))
                                                       
#define WF_PSPOLL_H_REG             ((uint16_t)(0x3d)) /* 16-bit register used to control low power mode                      */
#define WF_INDEX_ADDR_REG           ((uint16_t)(0x3e)) /* 16-bit register to move the data window                             */
#define WF_INDEX_DATA_REG           ((uint16_t)(0x3f)) /* 16-bit register to read or write address-indexed register           */

/*----------------------------------------------------------------------------------------*/
/* MRF24W registers accessed via the WF_INDEX_ADDR_REG and WF_INDEX_DATA_REG registers */
/*----------------------------------------------------------------------------------------*/
#define WF_HW_STATUS_REG            ((uint16_t)(0x2a)) /* 16-bit read only register providing hardware status bits */
#define WF_CONFIG_CTRL0_REG         ((uint16_t)(0x002e)) /* 16-bit register used to initiate Hard reset              */
#define WF_LOW_POWER_STATUS_REG     ((uint16_t)(0x3e)) /* 16-bit register read to determine when low power is done */

/* This bit mask is used in the HW_STATUS_REG to determine */
/* when the MRF24W has completed its hardware reset.       */
/*  0 : MRF24W is in reset                                 */
/*  1 : MRF24W is not in reset                             */
#define WF_HW_STATUS_NOT_IN_RESET_MASK ((uint16_t)(0x1000)) 

/* Definitions represent individual interrupt bits for the 8-bit host interrupt registers */
/*  WF_HOST_INTR_REG and WF_HOST_MASK_REG                                                 */
#define WF_HOST_INT_MASK_INT2               ((uint8_t)(0x01))
#define WF_HOST_INT_MASK_FIFO_1_THRESHOLD   ((uint8_t)(0x80))
#define WF_HOST_INT_MASK_FIFO_0_THRESHOLD   ((uint8_t)(0x40))
#define WF_HOST_INT_MASK_RAW_1_INT_0        ((uint8_t)(0x04))
#define WF_HOST_INT_MASK_RAW_0_INT_0        ((uint8_t)(0x02))
#define WF_HOST_INT_MASK_ALL_INT            ((uint8_t)(0xff))

/* Bit mask for all interrupts in the level 2 16-bit interrupt register */
#define WF_HOST_2_INT_MASK_ALL_INT          ((uint16_t)(0xffff))

/* these definitions are used in calls to enable and
 * disable interrupt bits. */
#define WF_INT_DISABLE ((uint8_t)0)
#define WF_INT_ENABLE  ((uint8_t)1)

/*
 * G2100 command registers
 */
#define ZG_INTR_REG                 (0x01)  // 8-bit register containing interrupt bits
#define ZG_INTR_MASK_REG            (0x02)  // 8-bit register containing interrupt mask
#define ZG_SYS_INFO_DATA_REG        (0x21)  // 8-bit register to read system info data window
#define ZG_SYS_INFO_IDX_REG         (0x2b)
#define ZG_INTR2_REG                (0x2d)  // 16-bit register containing interrupt bits
#define ZG_INTR2_MASK_REG           (0x2e)  // 16-bit register containing interrupt mask
#define ZG_BYTE_COUNT_REG           (0x2f)  // 16-bit register containing available write size for fifo0
#define ZG_BYTE_COUNT_FIFO0_REG     (0x33)  // 16-bit register containing bytes ready to read on fifo0
#define ZG_BYTE_COUNT_FIFO1_REG     (0x35)  // 16-bit register containing bytes ready to read on fifo1
#define ZG_PWR_CTRL_REG             (0x3d)  // 16-bit register used to control low power mode
#define ZG_INDEX_ADDR_REG           (0x3e)  // 16-bit register to move the data window
#define ZG_INDEX_DATA_REG           (0x3f)  // 16-bit register to read the address in the ZG_INDEX_ADDR_REG

#define ZG_INTR_REG_LEN             (1)
#define ZG_INTR_MASK_REG_LEN        (1)
#define ZG_SYS_INFO_DATA_REG_LEN    (1)
#define ZG_SYS_INFO_IDX_REG_LEN     (2)
#define ZG_INTR2_REG_LEN            (2)
#define ZG_INTR2_MASK_REG_LEN       (2)
#define ZG_BYTE_COUNT_REG_LEN       (2)
#define ZG_BYTE_COUNT_FIFO0_REG_LEN (2)
#define ZG_BYTE_COUNT_FIFO1_REG_LEN (2)
#define ZG_PWR_CTRL_REG_LEN         (2)
#define ZG_INDEX_ADDR_REG_LEN       (2)
#define ZG_INDEX_DATA_REG_LEN       (2)

// Registers accessed through ZG_INDEX_ADDR_REG
#define ZG_RESET_STATUS_REG         (0x2a)  // 16-bit read only register providing HW status bits
#define ZG_RESET_REG                (0x2e)  // 16-bit register used to initiate hard reset
#define ZG_PWR_STATUS_REG           (0x3e)  // 16-bit register read to determine when device
                                            // out of sleep state

#define ZG_RESET_MASK               (0x10)  // the first byte of the ZG_RESET_STATUS_REG
                                            // used to determine when the G2100 is in reset

#define ZG_ENABLE_LOW_PWR_MASK      (0x01)  // used by the Host to enable/disable sleep state
                                            // indicates to G2100 that the Host has completed
                                            // transactions and the device can go into sleep
                                            // state if possible

// states for interrupt state machine
#define ZG_INTR_ST_RD_INTR_REG  (1)
#define ZG_INTR_ST_WT_INTR_REG  (2)
#define ZG_INTR_ST_RD_CTRL_REG  (3)

// mask values for ZG_INTR_REG and ZG_INTR2_REG
#define ZG_INTR_MASK_FIFO1      (0x80)
#define ZG_INTR_MASK_FIFO0      (0x40)
#define ZG_INTR_MASK_ALL        (0xff)
#define ZG_INTR2_MASK_ALL       (0xffff)

// Buffer size
#define ZG_BUFFER_SIZE      450

// Types of networks
#define ZG_BSS_INFRA        (1)    // infrastructure only
#define ZG_BSS_ADHOC        (2)    // Ad-hoc only (ibss)

// Max characters in network SSID
#define ZG_MAX_SSID_LENGTH      MAX_SSID_LENGTH

// Security keys
#define ZG_MAX_ENCRYPTION_KEYS      4
#define ZG_MAX_ENCRYPTION_KEY_SIZE  13
#define ZG_MAX_WPA_PASSPHRASE_LEN   64
#define ZG_MAX_PMK_LEN              32

#define ZG_SECURITY_TYPE_NONE   0
#define ZG_SECURITY_TYPE_WEP    1
#define ZG_SECURITY_TYPE_WPA    2
#define ZG_SECURITY_TYPE_WPA2   3

typedef struct{
    u8 slot;    /* slot index */
    u8 keyLen;
    u8 defID;   /* the default wep key id */
    u8 ssidLen; /* num valid bytes in ssid */
    u8 ssid[ZG_MAX_SSID_LENGTH];    /* ssid of network */
    u8 key[ZG_MAX_ENCRYPTION_KEYS][ZG_MAX_ENCRYPTION_KEY_SIZE]; /* wep key data for 4 default keys */
} zg_wep_key_req_t;

#define ZG_WEP_KEY_REQ_SIZE     (4 + ZG_MAX_SSID_LENGTH + ZG_MAX_ENCRYPTION_KEYS*ZG_MAX_ENCRYPTION_KEY_SIZE)

typedef struct{
    u8 configBits;
    u8 phraseLen;   /* number of valid bytes in passphrase */
    u8 ssidLen;     /* number of valid bytes in ssid */
    u8 reserved;    /* alignment byte */
    u8 ssid[ZG_MAX_SSID_LENGTH];    /* the string of characters representing the ssid */
    u8 passPhrase[ZG_MAX_WPA_PASSPHRASE_LEN]; /* the string of characters representing the passphrase */
} zg_psk_calc_req_t;

#define ZG_PSK_CALC_REQ_SIZE    (4 + ZG_MAX_SSID_LENGTH + ZG_MAX_WPA_PASSPHRASE_LEN) /* 100 bytes */

typedef struct{
    u8 result;      /* indicating success or other */
    u8 macState;    /* current State of the on-chip MAC */
    u8 keyReturned; /* 1 if psk contains key data, 0 otherwise */
    u8 reserved;    /* pad byte */
    u8 psk[ZG_MAX_PMK_LEN]; /* the psk bytes */
} zg_psk_calc_cnf_t;

typedef struct{
    u8 slot;
    u8 ssidLen;
    u8 ssid[ZG_MAX_SSID_LENGTH];
    u8 keyData[ZG_MAX_PMK_LEN];
} zg_pmk_key_req_t;

#define ZG_PMK_KEY_REQ_SIZE     (2 + ZG_MAX_SSID_LENGTH + ZG_MAX_PMK_LEN)

typedef struct{
    u16        rssi;                      /* the value of the G1000 RSSI when the data frame was received */
    u8         dstAddr[6];    /* MAC Address to which the data frame was directed. */
    u8         srcAddr[6];    /* MAC Address of the Station that sent the Data frame. */
    u16        arrivalTime_th;               /* the value of the 32-bit G1000 system clock when the frame arrived */
    u16        arrivalTime_bh;
    u16        dataLen;                   /* the length in bytes of the payload which immediately follows this data structure */
} zg_rx_data_ind_t;

typedef struct{
    u8 secType;     /* security type : 0 - none; 1 - wep; 2 - wpa; 3 - wpa2; 0xff - best available */
    u8 ssidLen;     /* num valid bytes in ssid */
    u8 ssid[ZG_MAX_SSID_LENGTH];    /* the ssid of the target */
    u16 sleepDuration;  /* power save sleep duration in units of 100 milliseconds */
    u8 modeBss;         /* 1 - infra; 2 - adhoc */
    u8 reserved;
} zg_connect_req_t;

#define ZG_CONNECT_REQ_SIZE         (38)
uint16_t zg_get_rx_status();
void zg_clear_rx_status();

void wifi_set_ssid(char* new_ssid);
void wifi_set_passphrase(char* new_passphrase);
void wifi_set_security_type(u8 type);
void wifi_connect();
void wifi_send();
void wifi_set_mode(uint8_t mode);
void wifi_getMAC(uint8_t* d);
uint8_t wifi_is_connected();

#endif
/** @} */
/*EOF*/
