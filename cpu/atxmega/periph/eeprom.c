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
 * @brief       Low-level EEPROM driver implementation for ATxmega family
 *
 * @author      Gerson Fernando Budke <nandojve@gmail.com>
 * @}
 */
#include <stdint.h>

#include "cpu.h"
#include "cpu_nvm.h"
#include "periph/eeprom.h"

uint8_t eeprom_read_byte(uint32_t pos)
{
    uint8_t value = 0xff;

    (void)nvm_eeprom_read_byte(pos, &value);

    return value;
}
size_t eeprom_read(uint32_t pos, void *data, size_t len)
{
    return nvm_eeprom_read_buffer(pos, data, len) ? 0 : len;
}
void eeprom_write_byte(uint32_t pos, uint8_t data)
{
    (void)nvm_eeprom_write_byte(pos, data);
}
size_t eeprom_write(uint32_t pos, const void *data, size_t len)
{
    return nvm_eeprom_erase_and_write_buffer(pos, data, len) ? 0 : len;
}
size_t eeprom_set(uint32_t pos, uint8_t val, size_t len)
{
    (void)pos;
    (void)val;
    
    return len;
}
size_t eeprom_clear(uint32_t pos, size_t len)
{
    return eeprom_set(pos, EEPROM_CLEAR_BYTE, len);
}
size_t eeprom_erase(void)
{
    nvm_eeprom_erase_all();

    return EEPROM_SIZE;
}