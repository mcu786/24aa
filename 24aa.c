/*
Copyright 2012 Uladzimir Pylinski aka barthess.
You may use this work without restrictions, as long as this notice is included.
The work is provided "as is" without warranty of any kind, neither express nor implied.
*/

/*****************************************************************************
 * DATASHEET NOTES
 *****************************************************************************
Write cycle time (byte or page) — 5 ms

Note:
Page write operations are limited to writing
bytes within a single physical page,
regardless of the number of bytes
actually being written. Physical page
boundaries start at addresses that are
integer multiples of the page buffer size (or
‘page size’) and end at addresses that are
integer multiples of [page size – 1]. If a
Page Write command attempts to write
across a physical page boundary, the
result is that the data wraps around to the
beginning of the current page (overwriting
data previously stored there), instead of
being written to the next page as might be
expected.
*********************************************************************/

#include <string.h>

#include "ch.h"
#include "hal.h"

#include "24aa.h"
//#include "message.h"
#include "../i2c_local.h"
//#include "../main.h"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define eeprom_i2caddr    0b1010000

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */
/* semaphore for mutual access to EEPROM IC */
static BinarySemaphore eeprom_sem;

/* temporal local buffer */
static uint8_t localtxbuf[EEPROM_TX_DEPTH];

/*
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 */
/**
 * @brief   Split one uint16_t address to two uint8_t.
 *
 * @param[in] txbuf pointer to driver transmit buffer
 * @param[in] addr  uint16_t address
 */
#define eeprom_split_addr(txbuf, addr){                                       \
  (txbuf)[0] = ((uint8_t)((addr >> 8) & 0xFF));                               \
  (txbuf)[1] = ((uint8_t)(addr & 0xFF));                                      \
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 * @brief   EEPROM read routine.
 *
 * @param[in] addr      addres of 1-st byte to be read
 * @param[in] len       number of bytes to be write
 * @param[in] ext_rxbuf pointer to data buffer
 */
msg_t eeprom_read(uint32_t addr, uint8_t *buf, size_t len){
  msg_t status = RDY_OK;

  chBSemWait(&eeprom_sem);

  chDbgCheck(((len <= EEPROM_SIZE) && ((addr+len) <= EEPROM_SIZE)),
             "requested data out of device bounds");

  eeprom_split_addr(localtxbuf, addr);                /* write address bytes */
  i2c_transmit(eeprom_i2caddr, localtxbuf, 2, buf, len);

  chBSemSignal(&eeprom_sem);
  return status;
}

/**
 * @brief   EEPROM write routine.
 * @details Function writes data to EEPROM.
 * @pre     Data must be fit to single EEPROM page.
 *
 * @param[in] addr  addres of 1-st byte to be write
 * @param[in] buf   pointer to data
 * @param[in] len   number of bytes to be written
 */
msg_t eeprom_write(uint32_t addr, const uint8_t *buf, size_t len){
  msg_t status = RDY_OK;

  chBSemWait(&eeprom_sem);

  chDbgCheck(((len <= EEPROM_SIZE) && ((addr+len) <= EEPROM_SIZE)),
             "data can not be fitted in device");

  chDbgCheck(((addr / EEPROM_PAGE_SIZE) == ((addr + len - 1) / EEPROM_PAGE_SIZE)),
             "data can not be fitted in single page");

  eeprom_split_addr(localtxbuf, addr);              /* write address bytes */
  memcpy(&(localtxbuf[2]), buf, len);               /* write data bytes */
  i2c_transmit(eeprom_i2caddr, localtxbuf, (len + 2), NULL, 0);

  /* wait until EEPROM process data */
  chThdSleepMilliseconds(EEPROM_WRITE_TIME);
  chBSemSignal(&eeprom_sem);
  return status;
}

/**
 * Starter
 */
void init_eepromio(void){

  /* clear bufer just to be safe. */
  memset(localtxbuf, 0x55, EEPROM_TX_DEPTH);

  chBSemInit(&eeprom_sem, FALSE);
}
