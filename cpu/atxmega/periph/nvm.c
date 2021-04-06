/*
 * Copyright (C) 2021 Gerson Fernando Budke <nandojve@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_atxmega
 * @ingroup     cpu_atxmega_periph
 * @{
 *
 * @file
 * @brief       Low-level NVM driver implementation
 *
 * @author      Gerson Fernando Budke <nandojve@gmail.com>
 *
 * @}
 */
#include <avr/io.h>
#include <string.h>
#include <errno.h>

#include "cpu_nvm.h"

#define ENABLE_DEBUG 1
#include "debug.h"

/**
 * @brief Read one byte using the Load Program Memory (LPM) instruction
 * @internal
 *
 * This function sets the specified NVM_CMD, reads one byte using at the
 * specified byte address with the LPM instruction. NVM_CMD is restored after
 * use.
 *
 * @note This will disable interrupts during execution.
 *
 * @param nvm_cmd NVM command to load before running LPM
 * @param address Byte offset into the signature row
 */
static inline uint8_t _nvm_read_byte(uint8_t nvm_cmd, uint16_t address)
{
    uint8_t result;

    __asm__ volatile (
        "in  __tmp_reg__, __SREG__ \n\t"
        "cli                       \n\t"
        "lds __zero_reg__, %3      \n\t"
        "sts %3, %1                \n\t"
        "lpm %0, %a2               \n\t"
        "sts %3, __zero_reg__      \n\t"
        "clr __zero_reg__          \n\t"
        "out __SREG__, __tmp_reg__ \n\t"
        :
        "=&r" (result)
        :
        "r" (nvm_cmd), "e" (address), "m" (NVM_CMD)
        : /* no clobbers */
    );

    return result;
}

/**
 * @brief Read one byte from the production signature row
 *
 * This function reads one byte from the production signature row of the device
 * at the given address.
 *
 * @note The execution can take some time. It is recommended not call this
 * inside an interrupt service routine.
 *
 * @param address Byte offset into the signature row
 */
uint8_t nvm_read_production_signature_row(uint8_t address)
{
    return _nvm_read_byte(NVM_CMD_READ_CALIB_ROW_gc, address);
}

/**
 * @brief EEPROM api
 * @{
 */
static inline void _nvm_print_eeprom(uint16_t address, const void *buf,
                                     uint16_t len)
{
    int8_t is_mapped;
    uint8_t *ptr;
    uint16_t idx;

    if (buf) {
        ptr = (uint8_t*)buf;
        is_mapped = 0;
    }
    else {
        ptr = (uint8_t*)(address + MAPPED_EEPROM_START);
        nvm_wait_until_ready();
        is_mapped = 1;
    }

    if (!is_mapped) {
        eeprom_enable_mapping();
    }

    for (idx = 0; idx < len; ++idx) {
        if (idx % 16) {
            DEBUG("\n");
        }
        DEBUG("%02x ", *ptr++);
    }
    DEBUG("\n");

    if (!is_mapped) {
        eeprom_disable_mapping();
    }
}

static inline void nvm_eeprom_load_page_to_buffer(const uint8_t *values)
{
    uint8_t i;

    nvm_wait_until_ready();

    for (i = 0; i < EEPROM_PAGE_SIZE; ++i) {
        nvm_eeprom_load_byte_to_buffer(i, *values);
        ++values;
    }
}
static inline void nvm_eeprom_atomic_write_page(uint8_t page_addr)
{
    nvm_wait_until_ready();

    uint16_t address = (uint16_t)(page_addr * EEPROM_PAGE_SIZE);

    NVM.ADDR2 = 0x00;
    NVM.ADDR1 = (address >> 8) & 0xff;
    NVM.ADDR0 = address & 0xff;

    /* EEPROM Atomic Write (Erase & Write) */
    nvm_issue_command(NVM_CMD_ERASE_WRITE_EEPROM_PAGE_gc);
}
int nvm_eeprom_read_byte(uint16_t address, uint8_t* data)
{
    if (address >= EEPROM_SIZE) {
        return -ENXIO;
    }
    if (data == NULL) {
        return -EFAULT;
    }

    nvm_wait_until_ready();
    eeprom_enable_mapping();
    *data = *(uint8_t*)(address + MAPPED_EEPROM_START);
    if (IS_ACTIVE(ENABLE_DEBUG)) {
        _nvm_print_eeprom(address, NULL, 1);
    }
    eeprom_disable_mapping();

    return 0;
}
int nvm_eeprom_read_buffer(uint16_t address, void *buf, uint16_t len)
{
    if ((address + len) >= EEPROM_SIZE) {
        return -ENXIO;
    }
    if (buf == NULL) {
        return -EFAULT;
    }

    nvm_wait_until_ready();

    eeprom_enable_mapping();
    (void)memcpy(buf, (void*)(address + MAPPED_EEPROM_START), len);
    if (IS_ACTIVE(ENABLE_DEBUG)) {
        _nvm_print_eeprom(address, buf, len);
    }
    eeprom_disable_mapping();

    return 0;
}
int nvm_eeprom_write_byte(uint16_t address, uint8_t value)
{
    uint8_t old_cmd;

    if (address >= EEPROM_SIZE) {
        return -ENXIO;
    }

    /*  Flush buffer to make sure no unintentional data is written and load
     *  the "Page Load" command into the command register.
     */
    old_cmd = NVM.CMD;
    nvm_eeprom_flush_buffer();

    nvm_wait_until_ready();
    nvm_eeprom_load_byte_to_buffer(address, value);
    if (IS_ACTIVE(ENABLE_DEBUG)) {
        _nvm_print_eeprom(address, NULL, 1);
    }

    NVM.ADDR2 = 0x00;
    NVM.ADDR1 = (address >> 8) & 0xff;
    NVM.ADDR0 = address & 0xff;

    /*  Issue EEPROM Atomic Write (Erase&Write) command. Load command, write
     *  the protection signature and execute command.
     */
    NVM.CMD = NVM_CMD_ERASE_WRITE_EEPROM_PAGE_gc;
    nvm_exec();
    NVM.CMD = old_cmd;

    return 0;
}
int nvm_eeprom_erase_and_write_buffer(uint16_t address, const void *buf,
                                       uint16_t len)
{
    if ((address + len) >= EEPROM_SIZE) {
        return -ENXIO;
    }
    if (buf == NULL) {
        return -EFAULT;
    }

    if (IS_ACTIVE(ENABLE_DEBUG)) {
        _nvm_print_eeprom(address, buf, len);
    }

    while (len) {
        if (((address % EEPROM_PAGE_SIZE) == 0) && (len >= EEPROM_PAGE_SIZE)) {
            /* A full page can be written */
            nvm_eeprom_load_page_to_buffer((uint8_t*)buf);
            nvm_eeprom_atomic_write_page(address / EEPROM_PAGE_SIZE);
            address += EEPROM_PAGE_SIZE;
            buf = (uint8_t*)buf + EEPROM_PAGE_SIZE;
            len -= EEPROM_PAGE_SIZE;
        }
        else {
            nvm_eeprom_write_byte(address++, *(uint8_t*)buf);
            buf = (uint8_t*)buf + 1;
            len--;
        }
    }

    return 0;
}
void nvm_eeprom_flush_buffer(void)
{
    nvm_wait_until_ready();

    if ((NVM.STATUS & NVM_EELOAD_bm) != 0) {
        NVM.CMD = NVM_CMD_ERASE_EEPROM_BUFFER_gc;
        nvm_exec();
    }
}
void nvm_eeprom_load_byte_to_buffer(uint8_t byte_addr, uint8_t value)
{
    nvm_wait_until_ready();

    eeprom_enable_mapping();
    *(uint8_t*)(byte_addr + MAPPED_EEPROM_START) = value;
    eeprom_disable_mapping();
}
int nvm_eeprom_split_write_page(uint8_t page_addr)
{
    nvm_wait_until_ready();

    uint16_t address = (uint16_t)(page_addr * EEPROM_PAGE_SIZE);

    if (address >= EEPROM_SIZE) {
        return -ENXIO;
    }

    NVM.ADDR2 = 0x00;
    NVM.ADDR1 = (address >> 8) & 0xff;
    NVM.ADDR0 = address & 0xff;

    /* EEPROM Split Write */
    nvm_issue_command(NVM_CMD_WRITE_EEPROM_PAGE_gc);

    return 0;
}
void nvm_eeprom_fill_buffer_with_value(uint8_t value)
{
    uint8_t i;

    nvm_eeprom_flush_buffer();
    nvm_wait_until_ready();

    for (i = 0; i < EEPROM_PAGE_SIZE; ++i) {
        nvm_eeprom_load_byte_to_buffer(i, value);
    }
}
int nvm_eeprom_erase_page(uint8_t page_addr)
{
    nvm_eeprom_fill_buffer_with_value(0xff);

    nvm_wait_until_ready();

    uint16_t address = (uint16_t)(page_addr * EEPROM_PAGE_SIZE);

    if (address >= EEPROM_SIZE) {
        return -ENXIO;
    }

    NVM.ADDR2 = 0x00;
    NVM.ADDR1 = (address >> 8) & 0xff;
    NVM.ADDR0 = address & 0xff;

    nvm_issue_command(NVM_CMD_ERASE_EEPROM_PAGE_gc);

    return 0;
}
void nvm_eeprom_erase_all(void)
{
    nvm_eeprom_fill_buffer_with_value(0xff);

    nvm_wait_until_ready();

    nvm_issue_command(NVM_CMD_ERASE_EEPROM_gc);
}
/** @} */
