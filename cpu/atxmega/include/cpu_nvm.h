/*
 * Copyright (C) 2021 Gerson Fernando Budke <nandojve@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_atxmega
 * @brief       Non Volatile Memory (NVM) internal API
 * @{
 *
 * @author      Gerson Fernando Budke <nandojve@gmail.com>
 *
 */

#ifndef CPU_NVM_H
#define CPU_NVM_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get offset of calibration bytes in the production signature row
 *
 * @note In some distributions may require most recent vendor headers, even
 *       more recent than the upstream avr-libc.  The headers can be download
 *       directly from Microchip web site.  In general, Microchip have a
 *       dedicated page at Tools and Software / AVR and SAM Downloads Archive.
 *       The most recent version is AVR 8-bit Toolchain (3.4.4) 6.2.0.334.
 *       http://ww1.microchip.com/downloads/archive/avr8-headers-6.2.0.334.zip
 *
 *       The official RIOT-OS docker container, Debian and Ubuntu are
 *       distributions that already had updated versions of those files.
 *
 * @param regname Name of register within the production signature row
 * @retval Offset of register into the production signature row
 */
#define nvm_get_production_signature_row_offset(regname) \
    offsetof(NVM_PROD_SIGNATURES_t, regname)

/**
 * @brief Read one byte from the production signature row
 *
 * This function reads one byte from the production signature row of the device
 * at the given address.
 *
 * @note This function is modifying the NVM.CMD register.
 *       If the application are using program space access in interrupts
 *       (__flash pointers in IAR EW or pgm_read_byte in GCC) interrupts
 *       needs to be disabled when running EEPROM access functions. If not
 *       the program space reads will be corrupted.
 *
 * @param address Byte offset into the signature row
 */
uint8_t nvm_read_production_signature_row(uint8_t address);

/**
 * @brief Wait for any NVM access to finish.
 *
 * This function is blocking and waits for any NVM access to finish.
 * Use this function before any NVM accesses, if you are not certain that
 * any previous operations are finished yet.
 */
static inline void nvm_wait_until_ready(void)
{
    while ((NVM.STATUS & NVM_NVMBUSY_bm) == NVM_NVMBUSY_bm) {}
}

/**
 * @brief Non-Volatile Memory Execute Command
 *
 * This function sets the CCP register before setting the CMDEX bit in the
 * NVM.CTRLA register.
 *
 * @note The correct NVM command must be set in the NVM.CMD register before
 *       calling this function.
 */
static inline void nvm_exec(void)
{
    _PROTECTED_WRITE_SPM(NVM_CTRLA, NVM_CMDEX_bm);
}

/**
 * @brief Non-Volatile Memory Execute Specific Command
 *
 * This function sets a command in the NVM.CMD register, then performs an
 * execute command by writing the CMDEX bit to the NVM.CTRLA register.
 *
 * @note The function saves and restores the NVM.CMD register, but if this
 *       function is called from an interrupt, interrupts must be disabled
 *       before this function is called.
 *
 * @param nvm_command NVM Command to execute.
 */
static inline void nvm_issue_command(NVM_CMD_t nvm_command)
{
    uint8_t old_cmd;

    old_cmd = NVM.CMD;
    NVM.CMD = nvm_command;
    nvm_exec();
    NVM.CMD = old_cmd;
}

/**
 * @brief NVM EEPROM api
 *
 * @{
 */

/** @brief Enable EEPROM mapping into data space.
 *
 *  This macro enables mapping of EEPROM into data space.
 *  EEPROM starts at EEPROM_START in data memory. Read access
 *  can be done similar to ordinary SRAM access.
 *
 *  @note This disables IO-mapped access to EEPROM, although page erase and
 *        write operations still needs to be done through IO register.
 */
static inline void eeprom_enable_mapping(void)
{
#if !(defined(__AVR_ATxmega8E5__)  || \
      defined(__AVR_ATxmega16E5__) || \
      defined(__AVR_ATxmega32E5__))
    NVM.CTRLB |= NVM_EEMAPEN_bm;
#endif
}

/** @brief Disable EEPROM mapping into data space.
 *
 *  This macro disables mapping of EEPROM into data space.
 *  IO mapped access is now enabled.
 */
static inline void eeprom_disable_mapping(void)
{
#if !(defined(__AVR_ATxmega8E5__)  || \
      defined(__AVR_ATxmega16E5__) || \
      defined(__AVR_ATxmega32E5__))
    NVM.CTRLB &= ~NVM_EEMAPEN_bm;
#endif
}

/** @brief Disable EEPROM mapping into data space.
 *
 *  This macro disables mapping of EEPROM into data space.
 *  IO mapped access is now enabled.
 */
static inline int8_t eeprom_is_mapping_enabled(void)
{
    int8_t ret = 1;
#if !(defined(__AVR_ATxmega8E5__)  || \
      defined(__AVR_ATxmega16E5__) || \
      defined(__AVR_ATxmega32E5__))
    ret = (NVM.CTRLB & NVM_EEMAPEN_bm);
#endif
    return ret;
}

/**
 * @brief Read one byte from EEPROM using mapped access.
 *
 * This function reads one byte from EEPROM using mapped access.
 *
 * @param  address   EEPROM address, between 0 and EEPROM_SIZE
 * @param  value     pointer to the data value
 *
 * @return           0 on success otherwise the error code
 */
int nvm_eeprom_read_byte(uint16_t address, uint8_t *value);

/**
 * @brief Read buffer within the eeprom
 *
 * @param  address   the address to where to read
 * @param  value     data value to be stored
 *
 * @return           0 on success otherwise the error code
 */
int nvm_eeprom_write_byte(uint16_t address, uint8_t value);

/**
 * @brief Read bytes from EEPROM using IO mapping.
 *
 * This function reads len bytes from EEPROM using IO-mapped access.
 * This function will cancel all ongoing EEPROM page buffer loading
 * operations, if any.
 *
 * @param  address   EEPROM address (max EEPROM_SIZE)
 * @param  buf       pointer to the data to read from EEPROM
 * @param  len       the number of bytes to read
 *
 * @return           0 on success otherwise the error code
 */
int nvm_eeprom_read_buffer(uint16_t address, void *buf, uint16_t len);

/**
 * @brief Write buffer within the eeprom
 *
 * @param  address   the address to where to write
 * @param  buf       pointer to the data
 * @param  len       the number of bytes to write
 *
 * @return           0 on success otherwise the error code
 */
int nvm_eeprom_erase_and_write_buffer(uint16_t address, const void *buf,
    uint16_t len);

/**
 * @brief Flush temporary EEPROM page buffer.
 *
 * This function flushes the EEPROM page buffers. This function will cancel
 * any ongoing EEPROM page buffer loading operations, if any.
 * This function also works for memory mapped EEPROM access.
 *
 * @note An EEPROM write operations will automatically flush the buffer for you.
 * @note The function does not preserve the value of the NVM.CMD register
 */
void nvm_eeprom_flush_buffer(void);

/**
 * @brief Load single byte into temporary page buffer.
 *
 * This function loads one byte into the temporary EEPROM page buffers.
 * If memory mapped EEPROM is enabled, this function will not work.
 * Make sure that the buffer is flushed before starting to load bytes.
 * Also, if multiple bytes are loaded into the same location, they will
 * be ANDed together, thus 0x55 and 0xAA will result in 0x00 in the buffer.
 *
 * @note Only one page buffer exist, thus only one page can be loaded with
 *       data and programmed into one page. If data needs to be written to
 *       different pages, the loading and writing needs to be repeated.
 *
 * @param  byte_addr EEPROM Byte address, between 0 and EEPROM_PAGE_SIZE.
 * @param  value     Byte value to write to buffer.
 */
void nvm_eeprom_load_byte_to_buffer(uint8_t byte_addr, uint8_t value);

/**
 * @brief Write (without erasing) EEPROM page.
 *
 * This function writes the contents of an already loaded EEPROM page
 * buffer into EEPROM memory.
 *
 * As this is a split write, the page in EEPROM will _not_ be erased
 * before writing.
 *
 * @param  page_addr EEPROM Page address, between 0 and
 *                   EEPROM_SIZE/EEPROM_PAGE_SIZE
 *
 * @return           0 on success otherwise the error code
 */
int nvm_eeprom_split_write_page(uint8_t page_addr);

/**
 * @brief Fill temporary EEPROM page buffer with value.
 *
 * This fills the the EEPROM page buffers with a given value.
 * If memory mapped EEPROM is enabled, this function will not work.
 *
 * @note Only the lower part of the address is used to address the buffer.
 *       Therefore, no address parameter is needed. In the end, the data
 *       is written to the EEPROM page given by the address parameter to the
 *       EEPROM write page operation.
 *
 * @param  value     Value to copy to the page buffer.
 */
void nvm_eeprom_fill_buffer_with_value(uint8_t value);

/**
 * @brief Erase EEPROM page.
 *
 * This function erases one EEPROM page, so that every location reads 0xff.
 *
 * @param  page_addr EEPROM Page address, between 0 and
 *                   EEPROM_SIZE/EEPROM_PAGE_SIZE
 *
 * @return           0 on success otherwise the error code
 */
int nvm_eeprom_erase_page(uint8_t page_addr);

/**
 * @brief Erase entire EEPROM memory.
 *
 * This function erases the entire EEPROM memory block to 0xff.
 */
void nvm_eeprom_erase_all(void);
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* CPU_NVM_H */
/** @} */
