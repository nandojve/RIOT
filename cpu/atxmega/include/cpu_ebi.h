/*
 * Copyright (C) 2021 Gerson Fernando Budke
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup         cpu_atxmega
 * @{
 *
 * @file
 * @brief           External Bus Interface API
 *
 * @author          Gerson Fernando Budke <nandojve@gmail.com>
 */

#include "periph_cpu.h"

#ifndef CPU_EBI_H
#define CPU_EBI_H

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t hugemem_ptr_t;

#define HUGEMEM_NULL        0

void ebi_init(void);

static inline uint_fast8_t hugemem_read8(const hugemem_ptr_t from)
{
    uint8_t value;

    __asm__ volatile (
        "movw r30, %A1                  \n\t"
        "out %2, %C1                    \n\t"
        "ld %0, Z                       \n\t"
        "out %2, __zero_reg__           \n\t"
        :
        "=r"(value)
        :
        "r"(from), "i"(&RAMPZ)
        :
        "r30", "r31"
    );

    return value;
}
static inline void hugemem_write8(hugemem_ptr_t to, uint_fast8_t val)
{
    __asm__ volatile (
        "movw r30, %A0                  \n\t"
        "out %2, %C0                    \n\t"
        "st Z, %1                       \n\t"
        "out %2, __zero_reg__           \n\t"
        :
        :
        "r"(to), "r"(val), "i"(&RAMPZ)
        :
        "r30", "r31"
    );
}
uint_fast16_t hugemem_read16(const hugemem_ptr_t from);
void hugemem_write16(hugemem_ptr_t to, uint_fast16_t val);
uint_fast32_t hugemem_read32(const hugemem_ptr_t from);
void hugemem_write32(hugemem_ptr_t to, uint_fast32_t val);

/**
 * Ensure that the address range to copy from is within 64 kB boundary.
 */
void hugemem_read_block(void *to, const hugemem_ptr_t from, size_t size);
void hugemem_write_block(hugemem_ptr_t to, const void *from, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* CPU_EBI_H */
/** @} */
