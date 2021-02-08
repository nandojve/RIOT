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
 * @brief       Low-level CLOCK driver implementation
 *
 * @author      Gerson Fernando Budke <nandojve@gmail.com>
 *
 * @}
 */

#include <string.h>
#include <avr/io.h>

#include "cpu_clock.h"
#include "cpu.h"
#include "irq.h"

#define ENABLE_DEBUG 0
#include "debug.h"

/**
 * @brief   Initializes system clock prescaler
 */
void atxmega_clk_usb_enable(bool is_full_speed)
{
    uint8_t div;
    uint8_t fac;
    uint8_t irq_state;

    if (OSC.STATUS & OSC_PLLRDY_bm) {
        return;
    }

    /* f_OUT = ((f_IN / DIV) * PLL_FAC), where:
     *    DIV == 4 if OSC_PLLSRC_RC32M_gc or
     *    DIV == 1 for any other PLLSRC
     */
    fac = 48 >> 1; /* fac = (F_OUT / (F_IN / DIV) */
    div = is_full_speed ? CLK_USBPSDIV_1_gc : CLK_USBPSDIV_8_gc;

    irq_state = irq_disable();
    OSC.PLLCTRL = OSC_PLLSRC_RC2M_gc | (fac << OSC_PLLFAC_gp);
    OSC.CTRL |= OSC_PLLEN_bm;
    irq_restore(irq_state);

    while (!(OSC.STATUS & OSC_PLLRDY_bm)) {}

    _PROTECTED_WRITE(CLK_USBCTRL, div | CLK_USBSRC_PLL_gc | CLK_USBSEN_bm);
}
