#include "sd-spi.h"
#include <efs.h>
#include <stdio.h>
#include "interfaces/sd.h"
#include "clock.h"

#define SS_ON() PORTL &= 0xFE
#define SS_OFF() PORTL |= 0x01



EmbeddedFileSystem efs;
uint32_t block_;
uint8_t errorCode_;
uint8_t inBlock_;
uint16_t offset_;
uint8_t partialBlockRead_;
uint8_t status_;
uint8_t type_;

uint32_t millis() {
	return (clock_time() * 1000) / CLOCK_SECOND;
}

FileSystem *efs_sdcard_get_fs() {
	efs_sdcard_init();
	return &efs.myFs;
}

void efs_sdcard_init() {
	static uint8_t initialized = 0;
	if (!initialized) {
		esint8 i = efs_init(&efs, 0);
		initialized = 1;

        printf_P(PSTR("Initialization status code: %d\n"), i);
        printf_P(PSTR("Filesystem code: %u\n"), efs.myFs.type);
	}
}

int sdcard_ready() {
	return efs.myCard.sectorCount > (uint32_t)0;
}

esint8 if_initInterface(hwInterface *file, eint8 *opts) {
	SS_ON();
	if_spiInit(file);
	if (sd_init(file) < 0) {
		printf_P(PSTR("Card failed to init, breaking up...\n"));
		SS_OFF();
		return (-1);
	}

	if (sd_card_command(CMD13, 0) != 0) {
		printf_P(PSTR("Card didn't return the ready state, breaking up...\n"));
		SS_OFF();
		return (-2);
	}
	SS_OFF();
	file->sectorCount = sd_card_size(); /* FIXME ASAP!! */
	printf_P(PSTR("Init done...SD card size: %u\n"), file->sectorCount);
	return (0);
}

esint8 if_readBuf(hwInterface *file, euint32 address, euint8 *buf) {
	esint8 ret = sd_read_block(address, buf);
	return ret;
}

esint8 if_writeBuf(hwInterface *file, euint32 address, euint8 *buf) {
	esint8 ret = sd_write_block(address, buf);
	return ret;
}

esint8 if_setPos(hwInterface *file, euint32 address) {
	return (0);
}

void if_spiInit(hwInterface *iface) {
	errorCode_ = inBlock_ = partialBlockRead_ = type_ = 0;

	/* Unselect card */
	DDRL |= 0x01;
	SS_OFF();
	/* Set as master, clock and chip select output */
	DDR_SPI = (1 << DD_MOSI) | (1 << DD_SCK);

	// Enable SPI, Master, clock rate f_osc/128
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);
	// clear double speed
	SPSR &= ~(1 << SPI2X);

}
/*****************************************************************************/

euint8 if_spiSend(hwInterface *iface, euint8 outgoing) {
	euint8 incoming = 0;

	SPDR = outgoing;
	while (!(SPSR & (1 << SPIF)));
	incoming = SPDR;

	return (incoming);
}

uint8_t read_CID(cid_t *cid) {
	return read_register(CMD10, cid);
}

uint8_t read_CSD(csd_t *csd) {
	return read_register(CMD9, csd);
}

static void spi_tx(uint8_t b) {
	SPDR = b;
	while (!(SPSR & (1 << SPIF)));
}

static uint8_t spi_rx(void) {
	spi_tx(0XFF);
	return SPDR;
}

//------------------------------------------------------------------------------
// send command and return error code.  Return zero for OK
uint8_t sd_card_command(uint8_t cmd, uint32_t arg) {
	sd_read_end();
	SS_ON();
	sd_wait_not_busy(300);
	spi_tx(cmd | 0x40);
	int8_t s;
	for (s = 24; s >= 0; s -= 8) {
		spi_tx(arg >> s);
	}
	uint8_t crc = 0XFF;
	if (cmd == CMD_GO_IDLE_STATE) {
		crc = 0X95;
	}
	if (cmd == CMD_SEND_IF_COND) {
		crc = 0X87;
	}
	spi_tx(crc);
	uint8_t i;
	for (i = 0; ((status_ = spi_rx()) & 0X80) && i != 0XFF; i++);
	return status_;
}
//------------------------------------------------------------------------------
/**
 * Determine the size of an SD flash memory card.
 *
 * \return The number of 512 byte data blocks in the card
 *         or zero if an error occurs.
 */
uint32_t sd_card_size(void) {
	csd_t csd;
	if (!read_CSD(&csd)) return 0;
	if (csd.v1.csd_ver == 0) {
		uint8_t read_bl_len = csd.v1.read_bl_len;
		uint16_t c_size = (csd.v1.c_size_high << 10)
						  | (csd.v1.c_size_mid << 2) | csd.v1.c_size_low;
		uint8_t c_size_mult = (csd.v1.c_size_mult_high << 1)
							  | csd.v1.c_size_mult_low;
		return (uint32_t)(c_size + 1) << (c_size_mult + read_bl_len - 7);
	} else if (csd.v2.csd_ver == 1) {
		uint32_t c_size = ((uint32_t)csd.v2.c_size_high << 16)
						  | (csd.v2.c_size_mid << 8) | csd.v2.c_size_low;
		return (c_size + 1) << 10;
	} else {
		errorCode_ = SD_CARD_ERROR_BAD_CSD;
		return 0;
	}
}

uint8_t sd_card_Acmd(uint8_t cmd, uint32_t arg) {
	sd_card_command(CMD55, 0);
	return sd_card_command(cmd, arg);
}
//------------------------------------------------------------------------------
/**
 * Initialize an SD flash memory card.
 *
 * \param[in] sckRateID SPI clock rate selector. See sd_set_sck_rate().
 * \param[in] chipSelectPin SD chip select pin number.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.  The reason for failure
 * can be determined by calling errorCode() and errorData().
 */
esint8 sd_init(hwInterface *iface) {
	// 16-bit init start time allows over a minute
	uint16_t t0 = (uint16_t)millis();
	uint32_t arg;
	uint8_t sckRateID = SPI_HALF_SPEED;
	// must supply min of 74 clock cycles with CS high.
	uint8_t i;
	for (i = 0; i < 10; i++) {
		spi_tx(0XFF);
	}

	SS_ON();

	// command to go idle in SPI mode
	while ((status_ = sd_card_command(CMD_GO_IDLE_STATE, 0)) != R1_IDLE_STATE) {
		if (((uint16_t)millis() - t0) > SD_INIT_TIMEOUT) {
			errorCode_ = SD_CARD_ERROR_CMD_GO_IDLE_STATE;
			goto fail;
		}
	}
	// check SD version
	if ((sd_card_command(CMD_SEND_IF_COND, 0x1AA) & R1_ILLEGAL_COMMAND)) {
		type_ = SD_CARD_TYPE_SD1;
	} else {
		// only need last byte of r7 response
		for (i = 0; i < 4; i++) status_ = spi_rx();
		if (status_ != 0XAA) {
			errorCode_ = SD_CARD_ERROR_CMD_SEND_IF_COND;
			goto fail;
		}
		type_ = SD_CARD_TYPE_SD2;
	}
	// initialize card and send host supports SDHC if SD2
	arg = (type_ == SD_CARD_TYPE_SD2) ? 0X40000000 : 0;

	while ((status_ = sd_card_Acmd(ACMD41, arg)) != R1_READY_STATE) {
		// check for timeout
		if (((uint16_t)millis() - t0) > SD_INIT_TIMEOUT) {
			errorCode_ = SD_CARD_ERROR_ACMD41;
			goto fail;
		}
	}
	// if SD2 read OCR register to check for SDHC card
	if (type_ == SD_CARD_TYPE_SD2) {
		if (sd_card_command(CMD58, 0)) {
			errorCode_ = SD_CARD_ERROR_CMD58;
			goto fail;
		}
		if ((spi_rx() & 0XC0) == 0XC0) {
			type_ = SD_CARD_TYPE_SDHC;
		}
		// discard rest of ocr - contains allowed voltage range
		for (i = 0; i < 3; i++) {
			spi_rx();
		}
	}
	SS_OFF();
	return sd_set_sck_rate(sckRateID);

fail:
	SS_OFF();
	return FALSE;
}

//------------------------------------------------------------------------------
/**
 * Read a 512 byte block from an SD card device.
 *
 * \param[in] block Logical block to be read.
 * \param[out] dst Pointer to the location that will receive the data.

 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t sd_read_block(uint32_t block, uint8_t *dst) {
	return sd_read_data(block, 0, 512, dst);
}
//------------------------------------------------------------------------------
/**
 * Read part of a 512 byte block from an SD card.
 *
 * \param[in] block Logical block to be read.
 * \param[in] offset Number of bytes to skip at start of block
 * \param[out] dst Pointer to the location that will receive the data.
 * \param[in] count Number of bytes to read
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t sd_read_data(uint32_t block, uint16_t offset, uint16_t count, uint8_t *dst) {
	uint16_t n;
	if (count == 0) {
		return 0;
	}

	if ((count + offset) > 512) {
		goto fail;
	}
	if (!inBlock_ || block != block_ || offset < offset_) {
		block_ = block;
		// use address if not SDHC card
		if (type_ != SD_CARD_TYPE_SDHC) {
			block <<= 9;
		}
		if (sd_card_command(CMD17, block)) {
			errorCode_ = SD_CARD_ERROR_CMD17;
			goto fail;
		}
		if (!sd_wait_start_block()) {
			goto fail;
		}
		offset_ = 0;
		inBlock_ = 1;
	}

	// start first spi transfer
	SPDR = 0XFF;

	// skip data before offset
	for (; offset_ < offset; offset_++) {
		while (!(SPSR & (1 << SPIF)));
		SPDR = 0XFF;
	}
	// transfer data
	n = count - 1;
	uint16_t i;
	for (i = 0; i < n; i++) {
		while (!(SPSR & (1 << SPIF)));
		dst[i] = SPDR;
		SPDR = 0XFF;
	}
	// wait for last byte
	while (!(SPSR & (1 << SPIF)));
	dst[n] = SPDR;

	offset_ += count;
	if (!partialBlockRead_ || offset_ >= 512) {
		// read rest of data, checksum and set chip select high
		sd_read_end();
	}
	return 0;

fail:
	SS_OFF();
	return -1;
}
//------------------------------------------------------------------------------
/** Skip remaining data in a block when in partial block read mode. */
void sd_read_end(void) {
	if (inBlock_) {
		// skip data and crc
		SPDR = 0XFF;
		while (offset_++ < 513) {
			while (!(SPSR & (1 << SPIF)));
			SPDR = 0XFF;
		}
		// wait for last crc byte
		while (!(SPSR & (1 << SPIF)));
		SS_OFF();
		inBlock_ = 0;
	}
}
//------------------------------------------------------------------------------
/** read CID or CSR register */
uint8_t read_register(uint8_t cmd, void *buf) {
	uint8_t *dst = (uint8_t *)buf;
	if (sd_card_command(cmd, 0)) {
		errorCode_ = SD_CARD_ERROR_READ_REG;
		goto fail;
	}
	if (!sd_wait_start_block()) goto fail;
	// transfer data
	uint16_t i;
	for (i = 0; i < 16; i++) dst[i] = spi_rx();
	spi_rx();  // get first crc byte
	spi_rx();  // get second crc byte
	SS_OFF();
	return TRUE;

fail:
	SS_OFF();
	return FALSE;
}

uint8_t sd_set_sck_rate(uint8_t sckRateID) {
	if (sckRateID > 6) {
		errorCode_ = SD_CARD_ERROR_SCK_RATE;
		return FALSE;
	}
	// see avr processor datasheet for SPI register bit definitions
	if ((sckRateID & 1) || sckRateID == 6) {
		SPSR &= ~(1 << SPI2X);
	} else {
		SPSR |= (1 << SPI2X);
	}
	SPCR &= ~((1 << SPR1) | (1 << SPR0));
	SPCR |= (sckRateID & 4 ? (1 << SPR1) : 0)
			| (sckRateID & 2 ? (1 << SPR0) : 0);
	return TRUE;
}
//------------------------------------------------------------------------------
// wait for card to go not busy
uint8_t sd_wait_not_busy(uint16_t timeoutMillis) {
	uint16_t t0 = millis();
	do {
		if (spi_rx() == 0XFF) return TRUE;
	} while (((uint16_t)millis() - t0) < timeoutMillis);
	return FALSE;
}
//------------------------------------------------------------------------------
/** Wait for start block token */
uint8_t sd_wait_start_block(void) {
	uint16_t t0 = millis();
	while ((status_ = spi_rx()) == 0XFF) {
		if (((uint16_t)millis() - t0) > SD_READ_TIMEOUT) {
			errorCode_ = SD_CARD_ERROR_READ_TIMEOUT;
			goto fail;
		}
	}
	if (status_ != DATA_START_BLOCK) {
		errorCode_ = SD_CARD_ERROR_READ;
		goto fail;
	}
	return TRUE;

fail:
	SS_OFF();
	return FALSE;
}
//------------------------------------------------------------------------------
/**
 * Writes a 512 byte block to an SD card.
 *
 * \param[in] blockNumber Logical block to be written.
 * \param[in] src Pointer to the location of the data to be written.
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t sd_write_block(uint32_t blockNumber, const uint8_t *src) {
#if SD_PROTECT_BLOCK_ZERO
	// don't allow write to first block
	if (blockNumber == 0) {
		errorCode_ = SD_CARD_ERROR_WRITE_BLOCK_ZERO;
		goto fail;
	}
#endif  // SD_PROTECT_BLOCK_ZERO

	// use address if not SDHC card
	if (type_ != SD_CARD_TYPE_SDHC) blockNumber <<= 9;
	if (sd_card_command(CMD24, blockNumber)) {
		errorCode_ = SD_CARD_ERROR_CMD24;
		goto fail;
	}
	if (!sd_write_data(DATA_START_BLOCK, src)) goto fail;

	// wait for flash programming to complete
	if (!sd_wait_not_busy(SD_WRITE_TIMEOUT)) {
		errorCode_ = SD_CARD_ERROR_WRITE_TIMEOUT;
		goto fail;
	}
	// response is r2 so get and check two bytes for nonzero
	if (sd_card_command(CMD13, 0) || spi_rx()) {
		errorCode_ = SD_CARD_ERROR_WRITE_PROGRAMMING;
		goto fail;
	}
	SS_OFF();
	return 0;

fail:
	SS_OFF();
	return -1;
}
//------------------------------------------------------------------------------
// send one block of data for write block or write multiple blocks
uint8_t sd_write_data(uint8_t token, const uint8_t *src) {
	// send data - optimized loop
	SPDR = token;

	// send two byte per iteration
	uint16_t i;
	for (i = 0; i < 512; i += 2) {
		while (!(SPSR & (1 << SPIF)));
		SPDR = src[i];
		while (!(SPSR & (1 << SPIF)));
		SPDR = src[i + 1];
	}

	// wait for last data byte
	while (!(SPSR & (1 << SPIF)));

	spi_tx(0xff);  // dummy crc
	spi_tx(0xff);  // dummy crc

	status_ = spi_rx();

	if ((status_ & DATA_RES_MASK) != DATA_RES_ACCEPTED) {
		errorCode_ = SD_CARD_ERROR_WRITE;
		SS_OFF();
		return FALSE;
	}
	return TRUE;
}
//------------------------------------------------------------------------------
/** Start a write multiple blocks sequence.
 *
 * \param[in] blockNumber Address of first block in sequence.
 * \param[in] eraseCount The number of blocks to be pre-erased.
 *
 * \note This function is used with writeData() and sd_write_stop()
 * for optimized multiple block writes.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t sd_write_start(uint32_t blockNumber, uint32_t eraseCount) {
#if SD_PROTECT_BLOCK_ZERO
	// don't allow write to first block
	if (blockNumber == 0) {
		errorCode_ = SD_CARD_ERROR_WRITE_BLOCK_ZERO;
		goto fail;
	}
#endif  // SD_PROTECT_BLOCK_ZERO
	// send pre-erase count
	if (sd_card_Acmd(ACMD23, eraseCount)) {
		errorCode_ = SD_CARD_ERROR_ACMD23;
		goto fail;
	}
	// use address if not SDHC card
	if (type_ != SD_CARD_TYPE_SDHC) blockNumber <<= 9;
	if (sd_card_command(CMD25, blockNumber)) {
		errorCode_ = SD_CARD_ERROR_CMD25;
		goto fail;
	}
	return TRUE;

fail:
	SS_OFF();
	return FALSE;
}
//------------------------------------------------------------------------------
/** End a write multiple blocks sequence.
 *
* \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t sd_write_stop(void) {
	if (!sd_wait_not_busy(SD_WRITE_TIMEOUT)) goto fail;
	spi_tx(STOP_TRAN_TOKEN);
	if (!sd_wait_not_busy(SD_WRITE_TIMEOUT)) goto fail;
	SS_OFF();
	return TRUE;

fail:
	errorCode_ = SD_CARD_ERROR_STOP_TRAN;
	SS_OFF();
	return FALSE;
}
