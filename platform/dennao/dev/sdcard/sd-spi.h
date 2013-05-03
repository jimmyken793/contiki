
#ifndef __SD_SPI_H__
#define __SD_SPI_H__

#define __AVR_ATmega256__ 1
#define FALSE   0X00
#define TRUE    0x01

#define DDR_SPI DDRB
#define DD_MOSI DDB2
#define DD_SCK  DDB1

#define SS_BIT 0x01

#define SPI_FULL_SPEED 0
#define SPI_HALF_SPEED 1
#define SPI_QUARTER_SPEED 2

#define BYTE_ALIGNMENT

#include <avr/io.h>
#include <compat/ina90.h>
#include <efs.h>
#include "debug.h"
#include "config.h"

// Based on the document:
//
// SD Specifications
// Part 1
// Physical Layer
// Simplified Specification
// Version 2.00
// September 25, 2006
//
// www.sdcard.org/developers/tech/sdcard/pls/Simplified_Physical_Layer_Spec.pdf
//------------------------------------------------------------------------------
// SD card commands
/** GO_IDLE_STATE - init card in spi mode if CS low */
#define CMD_GO_IDLE_STATE 0X00
/** SEND_IF_COND - verify SD Memory Card interface operating condition.*/
#define CMD_SEND_IF_COND 0X08
/** SEND_CSD - read the Card Specific Data (CSD register) */
#define CMD9 0X09
/** SEND_CID - read the card identification information (CID register) */
#define CMD10 0X0A
/** SEND_STATUS - read the card status register */
#define CMD13 0X0D
/** READ_BLOCK - read a single data block from the card */
#define CMD17 0X11
/** WRITE_BLOCK - write a single data block to the card */
#define CMD24 0X18
/** WRITE_MULTIPLE_BLOCK - write blocks of data until a STOP_TRANSMISSION */
#define CMD25 0X19
/** ERASE_WR_BLK_START - sets the address of the first block to be erased */
#define CMD32 0X20
/** ERASE_WR_BLK_END - sets the address of the last block of the continuous
    range to be erased*/
#define CMD33 0X21
/** ERASE - erase all previously selected blocks */
#define CMD38 0X26
/** APP_CMD - escape for application specific command */
#define CMD55 0X37
/** READ_OCR - read the OCR register of a card */
#define CMD58 0X3A
/** SET_WR_BLK_ERASE_COUNT - Set the number of write blocks to be
     pre-erased before writing */
#define ACMD23 0X17
/** SD_SEND_OP_COMD - Sends host capacity support information and
    activates the card's initialization process */
#define ACMD41 0X29
//------------------------------------------------------------------------------
/** status for card in the ready state */
#define R1_READY_STATE 0X00
/** status for card in the idle state */
#define R1_IDLE_STATE 0X01
/** status bit for illegal command */
#define R1_ILLEGAL_COMMAND 0X04
/** start data token for read or write single block*/
#define DATA_START_BLOCK 0XFE
/** stop token for write multiple blocks*/
#define STOP_TRAN_TOKEN 0XFD
/** start data token for write multiple blocks*/
#define WRITE_MULTIPLE_TOKEN 0XFC
/** mask for data response tokens after a write block operation */
#define DATA_RES_MASK 0X1F
/** write data accepted token */
#define DATA_RES_ACCEPTED 0X05
//------------------------------------------------------------------------------
/** Protect block zero from write if nonzero */
#define SD_PROTECT_BLOCK_ZERO 1
/** init timeout ms */
#define SD_INIT_TIMEOUT 2000
/** erase timeout ms */
#define SD_ERASE_TIMEOUT 10000
/** read timeout ms */
#define SD_READ_TIMEOUT 300
/** write time out ms */
#define SD_WRITE_TIMEOUT 600
//------------------------------------------------------------------------------
// SD card errors
/** timeout error for command CMD0 */
#define SD_CARD_ERROR_CMD_GO_IDLE_STATE 0X1
/** CMD8 was not accepted - not a valid SD card*/
#define SD_CARD_ERROR_CMD_SEND_IF_COND 0X2
/** card returned an error response for CMD17 (read block) */
#define SD_CARD_ERROR_CMD17 0X3
/** card returned an error response for CMD24 (write block) */
#define SD_CARD_ERROR_CMD24 0X4
/**  WRITE_MULTIPLE_BLOCKS command failed */
#define SD_CARD_ERROR_CMD25 0X05
/** card returned an error response for CMD58 (read OCR) */
#define SD_CARD_ERROR_CMD58 0X06
/** SET_WR_BLK_ERASE_COUNT failed */
#define SD_CARD_ERROR_ACMD23 0X07
/** card's ACMD41 initialization process timeout */
#define SD_CARD_ERROR_ACMD41 0X08
/** card returned a bad CSR version field */
#define SD_CARD_ERROR_BAD_CSD 0X09
/** erase block group command failed */
#define SD_CARD_ERROR_ERASE 0X0A
/** card not capable of single block erase */
#define SD_CARD_ERROR_ERASE_SINGLE_BLOCK 0X0B
/** Erase sequence timed out */
#define SD_CARD_ERROR_ERASE_TIMEOUT 0X0C
/** card returned an error token instead of read data */
#define SD_CARD_ERROR_READ 0X0D
/** read CID or CSD failed */
#define SD_CARD_ERROR_READ_REG 0X0E
/** timeout while waiting for start of read data */
#define SD_CARD_ERROR_READ_TIMEOUT 0X0F
/** card did not accept STOP_TRAN_TOKEN */
#define SD_CARD_ERROR_STOP_TRAN 0X10
/** card returned an error token as a response to a write operation */
#define SD_CARD_ERROR_WRITE 0X11
/** attempt to write protected block zero */
#define SD_CARD_ERROR_WRITE_BLOCK_ZERO 0X12
/** card did not go ready for a multiple block write */
#define SD_CARD_ERROR_WRITE_MULTIPLE 0X13
/** card returned an error to a CMD13 status check after a write */
#define SD_CARD_ERROR_WRITE_PROGRAMMING 0X14
/** timeout occurred during write programming */
#define SD_CARD_ERROR_WRITE_TIMEOUT 0X15
/** incorrect rate selected */
#define SD_CARD_ERROR_SCK_RATE 0X16
//------------------------------------------------------------------------------
// card types
/** Standard capacity V1 SD card */
#define SD_CARD_TYPE_SD1 1
/** Standard capacity V2 SD card */
#define SD_CARD_TYPE_SD2 2
/** High Capacity SD card */
#define SD_CARD_TYPE_SDHC 3

//------------------------------------------------------------------------------
typedef struct CID {
	// byte 0
	uint8_t mid;  // Manufacturer ID
	// byte 1-2
	char oid[2];  // OEM/Application ID
	// byte 3-7
	char pnm[5];  // Product name
	// byte 8
	unsigned prv_m : 4;  // Product revision n.m
	unsigned prv_n : 4;
	// byte 9-12
	uint32_t psn;  // Product serial number
	// byte 13
	unsigned mdt_year_high : 4;  // Manufacturing date
	unsigned reserved : 4;
	// byte 14
	unsigned mdt_month : 4;
	unsigned mdt_year_low : 4;
	// byte 15
	unsigned always1 : 1;
	unsigned crc : 7;
} cid_t;
//------------------------------------------------------------------------------
// CSD for version 1.00 cards
typedef struct CSDV1 {
	// byte 0
	unsigned reserved1 : 6;
	unsigned csd_ver : 2;
	// byte 1
	uint8_t taac;
	// byte 2
	uint8_t nsac;
	// byte 3
	uint8_t tran_speed;
	// byte 4
	uint8_t ccc_high;
	// byte 5
	unsigned read_bl_len : 4;
	unsigned ccc_low : 4;
	// byte 6
	unsigned c_size_high : 2;
	unsigned reserved2 : 2;
	unsigned dsr_imp : 1;
	unsigned read_blk_misalign : 1;
	unsigned write_blk_misalign : 1;
	unsigned read_bl_partial : 1;
	// byte 7
	uint8_t c_size_mid;
	// byte 8
	unsigned vdd_r_curr_max : 3;
	unsigned vdd_r_curr_min : 3;
	unsigned c_size_low : 2;
	// byte 9
	unsigned c_size_mult_high : 2;
	unsigned vdd_w_cur_max : 3;
	unsigned vdd_w_curr_min : 3;
	// byte 10
	unsigned sector_size_high : 6;
	unsigned erase_blk_en : 1;
	unsigned c_size_mult_low : 1;
	// byte 11
	unsigned wp_grp_size : 7;
	unsigned sector_size_low : 1;
	// byte 12
	unsigned write_bl_len_high : 2;
	unsigned r2w_factor : 3;
	unsigned reserved3 : 2;
	unsigned wp_grp_enable : 1;
	// byte 13
	unsigned reserved4 : 5;
	unsigned write_partial : 1;
	unsigned write_bl_len_low : 2;
	// byte 14
	unsigned reserved5: 2;
	unsigned file_format : 2;
	unsigned tmp_write_protect : 1;
	unsigned perm_write_protect : 1;
	unsigned copy : 1;
	unsigned file_format_grp : 1;
	// byte 15
	unsigned always1 : 1;
	unsigned crc : 7;
} csd1_t;
//------------------------------------------------------------------------------
// CSD for version 2.00 cards
typedef struct CSDV2 {
	// byte 0
	unsigned reserved1 : 6;
	unsigned csd_ver : 2;
	// byte 1
	uint8_t taac;
	// byte 2
	uint8_t nsac;
	// byte 3
	uint8_t tran_speed;
	// byte 4
	uint8_t ccc_high;
	// byte 5
	unsigned read_bl_len : 4;
	unsigned ccc_low : 4;
	// byte 6
	unsigned reserved2 : 4;
	unsigned dsr_imp : 1;
	unsigned read_blk_misalign : 1;
	unsigned write_blk_misalign : 1;
	unsigned read_bl_partial : 1;
	// byte 7
	unsigned reserved3 : 2;
	unsigned c_size_high : 6;
	// byte 8
	uint8_t c_size_mid;
	// byte 9
	uint8_t c_size_low;
	// byte 10
	unsigned sector_size_high : 6;
	unsigned erase_blk_en : 1;
	unsigned reserved4 : 1;
	// byte 11
	unsigned wp_grp_size : 7;
	unsigned sector_size_low : 1;
	// byte 12
	unsigned write_bl_len_high : 2;
	unsigned r2w_factor : 3;
	unsigned reserved5 : 2;
	unsigned wp_grp_enable : 1;
	// byte 13
	unsigned reserved6 : 5;
	unsigned write_partial : 1;
	unsigned write_bl_len_low : 2;
	// byte 14
	unsigned reserved7: 2;
	unsigned file_format : 2;
	unsigned tmp_write_protect : 1;
	unsigned perm_write_protect : 1;
	unsigned copy : 1;
	unsigned file_format_grp : 1;
	// byte 15
	unsigned always1 : 1;
	unsigned crc : 7;
} csd2_t;
//------------------------------------------------------------------------------
// union of old and new style CSD register
typedef union {
	csd1_t v1;
	csd2_t v2;
} csd_t;

FileSystem *efs_sdcard_get_fs();
void efs_sdcard_init();
int sdcard_ready();
esint8 if_initInterface(hwInterface *file, eint8 *opts);
esint8 if_readBuf(hwInterface *file, euint32 address, euint8 *buf);
esint8 if_writeBuf(hwInterface *file, euint32 address, euint8 *buf);
esint8 if_setPos(hwInterface *file, euint32 address);
void if_spiInit(hwInterface *iface);
euint8 if_spiSend(hwInterface *iface, euint8 outgoing);
uint8_t sd_card_command(uint8_t cmd, uint32_t arg);
uint32_t sd_card_size(void);
esint8 sd_init(hwInterface *iface);
uint8_t sd_read_block(uint32_t block, uint8_t *dst);
uint8_t sd_read_data(uint32_t block, uint16_t offset, uint16_t count, uint8_t *dst);
void sd_read_end(void);
uint8_t read_register(uint8_t cmd, void *buf);
uint8_t sd_set_sck_rate(uint8_t sckRateID);
uint8_t sd_wait_not_busy(uint16_t timeoutMillis);
uint8_t sd_wait_start_block(void);
uint8_t sd_write_block(uint32_t blockNumber, const uint8_t *src);
uint8_t sd_write_data(uint8_t token, const uint8_t *src);
uint8_t sd_write_start(uint32_t blockNumber, uint32_t eraseCount);
uint8_t sd_write_stop(void);

#endif
