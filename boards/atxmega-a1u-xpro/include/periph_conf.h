/*
 * Copyright (C) 2021 Gerson Fernando Budke
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_atxmega-a1u-xpro
 * @{
 *
 * @file
 * @brief       Peripheral MCU configuration for the ATxmegaA1U Xplained Pro board.
 *
 * @author      Gerson Fernando Budke <nandojve@gmail.com>
 */
#include "mutex.h"

#ifndef PERIPH_CONF_H
#define PERIPH_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <avr/io.h>

#include "periph_cpu.h"

/**
 * @name    Clocks and Oscilators configuration
 *
 * The sys_clk = 64MHz
 *     per4    = 64MHz
 *     per2    = 64MHz
 *     per     = 32MHz
 *     cpu     = 32MHz
 * @{
 */
static const cpu_clk_osc_t clk_osc_config = {
    .sys_src         = CLK_SCLKSEL_PLL_gc,
    .psa_div         = CLK_PSADIV_1_gc,
    .psbc_div        = CLK_PSBCDIV_1_2_gc,

    .xosc_en         = CPU_CLK_OFF,

    .dfll_2M_src     = OSC_RC2MCREF_RC32K_gc,
    .dfll_2M_en      = CPU_CLK_EN;
    .dfll_32M_src    = OSC_RC32MCREF_RC32K_gc,
    .dfll_32M_en     = CPU_CLK_EN;

    .int_32k_en      = CPU_CLK_EN;
    .int_2M_en       = CPU_CLK_EN;
    .int_8M_en       = CPU_CLK_OFF;
    .int_32M_en      = CPU_CLK_EN;
    .int_32M_freq    = 32000000U,

    .pll_src         = OSC_PLLSRC_RC32M_gc; /* src = 32M / 4 = 8MHz         */
    .pll_div_by2     = CPU_CLK_OFF;
    .pll_mul_factor  = 8;
    .pll_en          = CPU_CLK_EN;          /* pll_clk = 8 * 8 = 64MHz      */
    .pll_freq        = 64000000U,

    .usb_en          = CPU_CLK_OFF;
};
/** @} */

/**
 * @name    Timer peripheral configuration
 * @{
 */
static const timer_conf_t timer_config[] = {
    {
        .dev            = (void *)&TCC1,
        .type           = TC_TYPE_1,
        .int_lvl        = { CPU_INT_LVL_LOW,
                            CPU_INT_LVL_OFF,
                            CPU_INT_LVL_OFF,
                            CPU_INT_LVL_OFF },
    },
    {
        .dev            = (void *)&TCC0,
        .type           = TC_TYPE_0,
        .int_lvl        = { CPU_INT_LVL_LOW,
                            CPU_INT_LVL_LOW,
                            CPU_INT_LVL_LOW,
                            CPU_INT_LVL_LOW },
    }
};

#define TIMER_0_ISRA      TCC1_CCA_vect

#define TIMER_1_ISRA      TCC0_CCA_vect
#define TIMER_1_ISRB      TCC0_CCB_vect
#define TIMER_1_ISRC      TCC0_CCC_vect
#define TIMER_1_ISRD      TCC0_CCD_vect

#define TIMER_NUMOF       ARRAY_SIZE(timer_config)
/** @} */

/**
 * @name    UART configuration
 * @{
 */
static const uart_conf_t uart_config[] = {
    {   /* CDC-ACM */
        .dev            = &USARTE0,
        .rx_pin         = GPIO_PIN(PORT_E, 2),
        .tx_pin         = GPIO_PIN(PORT_E, 3),
#ifdef MODULE_PERIPH_UART_HW_FC
        .rts_pin        = GPIO_UNDEF,
        .cts_pin        = GPIO_UNDEF,
#endif
        .rx_int_lvl     = CPU_INT_LVL_LOW,
        .tx_int_lvl     = CPU_INT_LVL_LOW,
        .dre_int_lvl    = CPU_INT_LVL_OFF,
    },
};

/* interrupt function name mapping */
#define UART_0_RXC_ISR    USARTE0_RXC_vect    /* Reception Complete Interrupt */
#define UART_0_DRE_ISR    USARTE0_DRE_vect    /* Data Register Empty Interrupt */
#define UART_0_TXC_ISR    USARTE0_TXC_vect    /* Transmission Complete Interrupt */

#define UART_NUMOF        ARRAY_SIZE(uart_config)
/** @} */

/**
 * @name EBI configuration
 * @{
 */
static const ebi_conf_t ebi_config = {
    .addr_bits              = 19,
    .flags                  = (EBI_PORT_LPC | EBI_PORT_CS2),
    .sram_ale               = 0,
    .lpc_ale                = 2,
    .sdram                  = { 0 },
    .cs                     = { { EBI_CS_MODE_LPC_gc,
                                  EBI_CS_ASPACE_512KB_gc,
                                  EBI_CS_SRWS_1CLK_gc,
                                  0x0UL,
                                },
                                { EBI_CS_MODE_DISABLED_gc,
                                  0,
                                  EBI_CS_SRWS_0CLK_gc,
                                  0x0UL,
                                },
                                { EBI_CS_MODE_DISABLED_gc,
                                  0,
                                  EBI_CS_SRWS_0CLK_gc,
                                  0x0UL,
                                },
                                { EBI_CS_MODE_DISABLED_gc,
                                  0,
                                  EBI_CS_SRWS_0CLK_gc,
                                  0x0UL,
                                },
                              },
    .stack_size             = 0xffff,
    /**
     * Make sure you tell to compiler that heap_end at 0x800000 + stack_size as
     * "LDSCRIPT_EXTRA = -Wl,--defsym=__heap_end=0x80FFFF" at Makefile.include
     */
};
/** @} */

#ifdef __cplusplus
}
#endif

#include "periph_conf_common.h"

#endif /* PERIPH_CONF_H */
/** @} */
