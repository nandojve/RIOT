/*
 * Copyright (C) 2021 Gerson Fernando Budke <nandojve@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_atxmega
 * @brief       Common implementations and headers for ATxmega family based micro-controllers
 * @{
 *
 * @file
 * @brief       Basic definitions for the ATxmega common clock module
 *
 * When ever you want to do something hardware related, that is accessing MCUs registers directly,
 * just include this file. It will then make sure that the MCU specific headers are included.
 *
 * @author      Gerson Fernando Budke <nandojve@gmail.com>
 *
 */

#ifndef CPU_CLOCK_H
#define CPU_CLOCK_H

#include <avr/io.h>
#include <stdint.h>
#include "cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   ATxmega system clock prescaler settings
 *
 * Some CPUs may not support the highest prescaler settings
 * {@
 */
enum {
    CPU_ATXMEGA_CLK_SCALE_DIV1      = 0,
    CPU_ATXMEGA_CLK_SCALE_DIV2      = 1,
    CPU_ATXMEGA_CLK_SCALE_DIV4      = 3,
    CPU_ATXMEGA_CLK_SCALE_DIV8      = 5,
    CPU_ATXMEGA_CLK_SCALE_DIV16     = 7,
    CPU_ATXMEGA_CLK_SCALE_DIV32     = 9,
    CPU_ATXMEGA_CLK_SCALE_DIV64     = 11,
    CPU_ATXMEGA_CLK_SCALE_DIV128    = 13,
    CPU_ATXMEGA_CLK_SCALE_DIV256    = 15,
    CPU_ATXMEGA_CLK_SCALE_DIV512    = 17,
};

enum {
    CPU_ATXMEGA_BUS_SCALE_DIV1_1    = 0,
    CPU_ATXMEGA_BUS_SCALE_DIV1_2    = 1,
    CPU_ATXMEGA_BUS_SCALE_DIV4_1    = 2,
    CPU_ATXMEGA_BUS_SCALE_DIV2_2    = 3,
};
/** @} */

/**
 * @brief   Initializes system clock prescaler
 */
static inline void atxmega_set_prescaler(uint8_t clk_scale, uint8_t bus_scale)
{
    /* Disable CCP for Protected IO register and set new value
     * Set system clock prescalers to zero. PSCTRL contains A Prescaler
     * Value and one value for and B and C Prescaler
     */
    _PROTECTED_WRITE(CLK.PSCTRL, clk_scale | bus_scale);
}

static inline uint32_t atxmega_get_sys_hz(void)
{
    uint32_t main_hz;

    switch (CLK.PSCTRL & CLK_SCLKSEL_gm) {
    case CLK_SCLKSEL_RC2M_gc:
    default:
        main_hz = 2000000UL;
        break;
    case CLK_SCLKSEL_RC32M_gc:
        main_hz = 32000000UL; /* TODO: Add config */
        break;
    case CLK_SCLKSEL_RC32K_gc:
        main_hz = 32768UL;
        break;
    case CLK_SCLKSEL_XOSC_gc:
        main_hz = 16000000UL; /* TODO: Add config */
        break;
    case CLK_SCLKSEL_PLL_gc:
        main_hz = 32000000UL; /* TODO: Add config */
        break;
#if defined (__AVR_ATxmega8e5__)  || \
    defined (__AVR_ATxmega16e5__) || \
    defined (__AVR_ATxmega32e5__)
    case CLK_SCLKSEL_RC8M_gc:
        main_hz = 8000000UL;
        break;
#endif
    }

    return main_hz;
}

static inline uint32_t atxmega_get_per4_hz(void)
{
    uint32_t per4 = atxmega_get_sys_hz();

#if defined (__AVR_ATxmega8e5__)  || \
    defined (__AVR_ATxmega16e5__) || \
    defined (__AVR_ATxmega32e5__)

    if ((clk_osc_config.psa_div & CLK_PSADIV_gm) > CLK_PSADIV_512_gc) {
        uint8_t div;
        switch (clk_osc_config.psa_div & CLK_PSADIV_gm) {
        case CLK_PSADIV_6_gc:
        default:
            div = 6;
	    break;
        case CLK_PSADIV_10_gc:
            div = 10;
	    break;
        case CLK_PSADIV_12_gc:
            div = 12;
	    break;
        case CLK_PSADIV_24_gc:
            div = 24;
	    break;
        case CLK_PSADIV_48_gc:
            div = 48;
	    break;
        }

        return per4 / div;
    }
#endif

    return per4 >> ((clk_osc_config.psa_div >> (CLK_PSADIV_gp + 1)) + 1);
}

static inline uint32_t atxmega_get_per2_hz(void)
{
    uint32_t per2 = atxmega_get_per4_hz();

    if (clk_osc_config.psbc_div & CLK_PSBCDIV1_bm) {
        per2 >>= (clk_osc_config.psbc_div & CLK_PSBCDIV0_bm) ? 2 : 4;
    }

    return per2;
}

static inline uint32_t atxmega_get_per_hz(void)
{
    uint32_t per = atxmega_get_per2_hz();

    if (clk_osc_config.psbc_div & CLK_PSBCDIV0_bm) {
        per >>= 2;
    }

    return per;
}

static inline uint32_t atxmega_get_cpu_hz(void)
{
    return atxmega_get_per_hz();
}

#ifdef __cplusplus
}
#endif

#endif /* CPU_CLOCK_H */
/** @} */
