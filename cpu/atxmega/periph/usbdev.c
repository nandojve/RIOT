/*
 * Copyright (C) 2021 Gerson Fernando Budke
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_atxmega
 * @ingroup     cpu_atxmega_periph
 * @{
 * @file
 * @brief       Low level USB interface functions for the xmega class devices
 *
 * @author      Gerson Fernando Budke <nandojve@gmail.com>
 * @}
 */

#define USB_H_USER_IS_RIOT_INTERNAL

#include <errno.h>

#include "cpu.h"
#include "cpu_nvm.h"
#include "cpu_pm.h"
#include "cpu_usb.h"
#include "periph/gpio.h"
#include "periph/pm.h"
#include "periph/usbdev.h"

/**
 * Be careful with enabling debug here. As with all timing critical systems it
 * is able to interfere with USB functionality and you might see different
 * errors than debug disabled
 */
#define ENABLE_DEBUG 0
#include "debug.h"

/**
 * USB SRAM data about fifo, endpoint descriptor table and frame number
 *
 * The content of the USB SRAM can be:
 * - modified by USB hardware by interface to signal endpoint status.
 *   Thereby, it is read by software.
 * - modified by USB software to control endpoint.
 *   Thereby, it is read by hardware.
 * This data section is volatile and the specific opcode read/modify/write
 * must be used.
 * @{
 */
struct usb_sram_data {
#if XMEGA_A1U
#  if (0!=((XMEGA_USB_NUM_EP) % 4))
        uint8_t padding_align[16 - ((XMEGA_USB_NUM_EP) * sizeof(uint32_t)) % 16];
#  endif
#endif
        uint32_t fifo[XMEGA_USB_NUM_EP];
        USB_EP_t ep_ctrl[2 * (XMEGA_USB_NUM_EP)];
        uint16_t frame_number;
};
#if XMEGA_A1U
__attribute__ ((aligned(16)))
#else
__attribute__ ((aligned(4)))
#endif
static volatile struct usb_sram_data usb_sram;
#define USB_EP_Desc_t USB_EP_t volatile
/** @} */

/* Instantiated USB peripheral */
static usb_t _usb_dev;

/* Forward declaration for the usb device driver */
const usbdev_driver_t driver;

static inline uint8_t _ep_num(uint8_t num, usb_ep_dir_t dir)
{
    return 2 * num + (dir == USB_EP_DIR_IN ? 1 : 0);
}

static inline usbdev_ep_t *_get_ep(usb_t *dev, uint8_t num, usb_ep_dir_t dir)
{
#if 0
    DEBUG("EP 0x%02x get\n",
        (num >> 1) | ((dir == USB_EP_DIR_IN) ? 0x80 : 0x00));
#endif

    return &dev->endpoints[_ep_num(num, dir)];
}

static inline USB_EP_Desc_t *_ep_reg_from_idx(uint8_t idx)
{
	return &usb_sram.ep_ctrl[idx];
}

static inline USB_EP_Desc_t *_ep_reg_from_ep(usbdev_ep_t *ep)
{
    return _ep_reg_from_idx(_ep_num(ep->num, ep->dir));
}

static inline uint8_t _decode_ep_idx_from_fifo(void)
{
    int8_t    rp;
    uint8_t   i_fifo;
    uint16_t  ad;
    uint16_t *p_ad;
    uint8_t   ep_idx;

    /* Compute ep addr */
    rp       = usb_get_fifo_rp();
    i_fifo   = 2 * (1 + ~rp);
    ad       = ((uint16_t) usb_sram.ep_ctrl) - i_fifo;
    p_ad     = (uint16_t *) ad;

    /* Compute ep index */
    ep_idx   = (((uint16_t) * p_ad - ((uint16_t) usb_sram.ep_ctrl)) >> 3);

    /* Make index inside endpoint range */
    ep_idx  %= 2 * XMEGA_USB_NUM_EP;

    return ep_idx;
}

static inline void _reset_ep_table(void)
{
    uint8_t i;

    for (i = 0; i < (XMEGA_USB_NUM_EP * 2); i++) {
        usb_sram.ep_ctrl[i].CTRL = 0;
    }
}

static inline void _enable_irq(void)
{
    uint8_t flags = USB_BUSEVIE_bm;

    if (_usb_dev.config->sof) {
        flags |= USB_SOFIE_bm;
    }

    USB_INTCTRLA |= flags;
}

static inline void _disable_irq(void)
{
    USB_INTCTRLA &= ~(USB_BUSEVIE_bm | USB_SOFIE_bm);
}

/**
 * Initializes the USB DP/DM buffers
 *
 * This functions initializes the USB buffer using the calibration value
 * stored in production raw. If the calibration value is not found (0xFF)
 * value, a default typical value is applied.
 *
 */
static inline void _usb_pad_init(void)
{
    uint8_t cal;

#ifdef USB_PAD_USER_CAL0
    USB_CAL0 = USB_PAD_USER_CAL0;
#else
    cal = nvm_read_production_signature_row
        (nvm_get_production_signature_row_offset(USBCAL0));
if (cal != 0xFF) {
    USB_CAL0 = cal;
}
else {
    USB_CAL0 = 0x1F;
}
#endif

#ifdef USB_PAD_USER_CAL1
    USB_CAL1 = USB_PAD_USER_CAL1;
#else
    cal = nvm_read_production_signature_row
        (nvm_get_production_signature_row_offset(USBCAL1));
if (cal != 0xFF) {
    USB_CAL1 = cal;
}
else {
    USB_CAL1 = 0x1F;
}
#endif
}

static void _bank_set_address(usbdev_ep_t *ep)
{
    USB_EP_Desc_t *ep_reg = _ep_reg_from_ep(ep);

    usb_ep_set_buf(ep_reg, ep->buf);
}

static int _ep_calc_size(usbdev_ep_t *ep)
{
   uint8_t val = 0x00;

    switch (ep->len) {
        case 8:
            val = 0x0;
            break;
        case 16:
            val = 0x1;
            break;
        case 32:
            val = 0x2;
            break;
        case 64:
            val = 0x3;
            break;
        case 128:
            val = 0x4;
            break;
        case 256:
            val = 0x5;
            break;
        case 512:
            val = 0x6;
            break;
        case 1023:
            val = 0x7;
            break;
        default:
            return -1;
    }

    if (ep->type != USB_EP_TYPE_ISOCHRONOUS && ep->len > 64) {
        return -1;
    }

    return val;
}

static int _bank_set_size(usbdev_ep_t *ep)
{
    int size = _ep_calc_size(ep);

//     if (size >= 0) {
//         USB_EP_Desc_t *ep_desc = _ep_reg_from_ep(ep);
//
//         /* Set size */
//         ep_desc->CTRL = size;
//     }

    return size;
}

static void _set_address(uint8_t addr)
{
#if 0
    DEBUG("Set USB Address to %d\n", addr);
#endif

    usb_set_device_address(addr);
}

static usbdev_ep_t *_usbdev_new_ep(usbdev_t *dev, usb_ep_type_t type,
                                   usb_ep_dir_t dir, size_t buf_len)
{
    usb_t *usbdev = (usb_t *)dev;

    /* The IP supports all types for all endpoints */
    usbdev_ep_t *res = NULL;

    /* Always return endpoint 0 for control types */
    if (type == USB_EP_TYPE_CONTROL) {
        res = _get_ep(usbdev, 0, dir);
        res->num = 0;
    }
    else {
        /* Find the first unassigned ep with proper dir */
        for (uint8_t idx = 1; idx < XMEGA_USB_NUM_EP; idx++) {
            usbdev_ep_t *ep = _get_ep(usbdev, idx, dir);

            if (ep->type == USB_EP_TYPE_NONE) {
                res = ep;
                res->num = idx;

		break;
            }
        }
    }

    if (res) {
        DEBUG("EP 0x%02x new at 0x%04x\n",
            res->num | ((dir == USB_EP_DIR_IN) ? 0x80 : 0x00), (uint16_t) res);

        res->dev = dev;
        res->dir = dir;
        if (usbdev->used + buf_len < XMEGA_USB_BUF_SPACE) {
            res->buf = usbdev->buffer + usbdev->used;
            res->len = buf_len;
            if (_bank_set_size(res) < 0) {
                return NULL;
            }
            usbdev->used += buf_len;
            _bank_set_address(res);
            res->type = type;
            res->dev = dev;
        }
    }
    return res;
}

/**
 * Power mode management
 *
 * The Sleep modes authorized :
 * - in USB IDLE state, the USB needs of USB clock and authorizes up to IDLE mode
 * - in USB SUSPEND state, the USB no needs USB clock but requests a minimum
 *   clock restart timing. Thus, it authorizes up to POWER_DOWN or STANDBY mode.
 *
 * The USB SUSPEND depends on USB clock startup timing:
 * | Clock Startup | Sleep mode authorized |
 * | <=10ms        | SLEEP_SMODE_PDOWN_gc  |
 */
static void _block_pm(void)
{
//    pm_block(3);
}

static void _unblock_pm(void)
{
//    pm_unblock(3);
}

void usbdev_init_lowlevel(void)
{
    _usb_dev.usbdev.driver = &driver;
    _usb_dev.config = &usb_config;
}

usbdev_t *usbdev_get_ctx(unsigned num)
{
    (void)num;

    return &_usb_dev.usbdev;
}

static void _usbdev_deinit(void)
{
    uint8_t irq_state;

    irq_state = irq_disable();

    usb_detach_device();

    /* Disable interface */
    USB_CTRLA = 0;
    USB_CTRLB = 0;

    irq_restore(irq_state);
}

static void _usbdev_init(usbdev_t *dev)
{
    DEBUG("Initializing xmega usb peripheral\n");

    /* Only one usb device exists */
    usb_t *usbdev = (usb_t *)dev;
    uint8_t irq_state;

    usbdev->used = 0;
    atxmega_clk_usb_enable(usbdev->config->speed == USB_SPEED_FULL);
    pm_periph_enable(usbdev->config->pwr);

    _usbdev_deinit();

    if (usbdev->config->speed == USB_SPEED_FULL) {
        usb_set_full_speed();
    }

    /* Enable USB device */
    irq_state = irq_disable();

    _reset_ep_table();

    _usb_pad_init();
    usb_set_nb_max_ep(XMEGA_USB_NUM_EP);
    usb_enable_interface();
    usb_enable_store_frame_number();

#if XMEGA_A1U
    assert(((uint16_t)(&usb_sram) & 0x0F) == 0); /* check align on 16bit */
#else
    assert(((uint16_t)(&usb_sram) & 0x01) == 0); /* check align on WORD */
#endif
    usb_set_ep_table_addr(usb_sram.ep_ctrl);

    usb_enable_fifo();
    usb_reset_fifo();

    usb_enable_interrupt(usbdev->config->int_lvl);
    _block_pm();

    irq_restore(irq_state);

    usbdev->usbdev.cb(&usbdev->usbdev, USBDEV_EVENT_HOST_CONNECT);
}

static void usb_attach(void)
{
    DEBUG("Attaching to host\n");

    uint8_t irq_state = irq_disable();

    usb_ack_suspend_event();
    usb_ack_resume_event();
    usb_attach_device();

    /* Enable main USB interrupts */
    _enable_irq();
    usb_enable_tc_interrupt();
    usb_enable_setup_interrupt();

    irq_restore(irq_state);
}

static void usb_detach(void)
{
    DEBUG("Detaching to host\n");

    usb_detach_device();
}

static int _usbdev_get(usbdev_t *usbdev, usbopt_t opt,
                       void *value, size_t max_len)
{
    (void)usbdev;
    (void)max_len;
    int res = -ENOTSUP;
    switch (opt) {
        case USBOPT_MAX_VERSION:
            assert(max_len == sizeof(usb_version_t));
            *(usb_version_t *)value = USB_VERSION_20;
            res = sizeof(usb_version_t);
            break;
        case USBOPT_MAX_SPEED:
            assert(max_len == sizeof(usb_speed_t));
            *(usb_speed_t *)value = USB_SPEED_FULL;
            res = sizeof(usb_speed_t);
            break;
        default:
            DEBUG("Unhandled get call: 0x%x\n", opt);
            break;
    }
    return res;
}

static int _usbdev_set(usbdev_t *dev, usbopt_t opt,
                       const void *value, size_t value_len)
{
    (void)dev;
    (void)value_len;
    int res = -ENOTSUP;
    switch (opt) {
        case USBOPT_ADDRESS:
            assert(value_len == sizeof(uint8_t));
            uint8_t addr = (*((uint8_t *)value));
            _set_address(addr);
            break;
        case USBOPT_ATTACH:
            assert(value_len == sizeof(usbopt_enable_t));
            if (*((usbopt_enable_t *)value)) {
                usb_attach();
            }
            else {
                usb_detach();
            }
            res = sizeof(usbopt_enable_t);
            break;
        default:
            DEBUG("Unhandled set call: 0x%x\n", opt);
            break;
    }
    return res;
}

static void _ep_disable(usbdev_ep_t *ep)
{
    USB_EP_Desc_t *ep_reg = _ep_reg_from_ep(ep);

#if 0
    DEBUG("EP 0x%02x dis\n",
        ep->num | ((ep->dir == USB_EP_DIR_IN) ? 0x80 : 0x00));
#endif

    /* Abort OUT (receive) transaction */
    usb_ep_set_NACK0(ep_reg);
    usb_ep_disable(ep_reg);
}

static void _ep_enable(usbdev_ep_t *ep)
{
#if 0
    DEBUG("EP 0x%02x en\n",
        ep->num | ((ep->dir == USB_EP_DIR_IN) ? 0x80 : 0x00));
#endif

    USB_EP_Desc_t *ep_reg = _ep_reg_from_ep(ep);
    int8_t size = _ep_calc_size(ep);
    uint8_t type = USB_EP_TYPE_NONE;

    if (size < 0) {
        DEBUG("EP en size error\n");
        return;
    }

    switch (ep->type) {
        case USB_EP_TYPE_CONTROL:
            type = USB_EP_TYPE_CONTROL_gc;
            size &= ~(USB_EP_BUFSIZE1_bm | USB_EP_BUFSIZE0_bm);
            break;
        case USB_EP_TYPE_ISOCHRONOUS:
            type = USB_EP_TYPE_ISOCHRONOUS_gc;
            break;
        case USB_EP_TYPE_BULK:
        case USB_EP_TYPE_INTERRUPT:
            type = USB_EP_TYPE_BULK_gc;
            size &= ~(USB_EP_BUFSIZE1_bm | USB_EP_BUFSIZE0_bm);
            break;
        default:
            /* Must never happen */
            assert(false);
    }

    usb_ep_disable(ep_reg);
    usb_ep_clear_status(ep_reg);
    usb_ep_set_control(ep_reg, type | size);

    /* Do not use multipacket mode with EP Control */
    if (type == USB_EP_TYPE_CONTROL_gc) {
        return;
    }
    /* Do not use multipacket mode with isochronous 1023 bytes endpoint */
    if (size == USB_EP_BUFSIZE_1023_gc) {
        return;
    }

    usb_ep_set_multipacket(ep_reg);
}

static void _usbdev_ep_init(usbdev_ep_t *ep)
{
    USB_EP_Desc_t *ep_reg = _ep_reg_from_ep(ep);

#if 0
    DEBUG("EP 0x%02x init\n",
        ep->num | ((ep->dir == USB_EP_DIR_IN) ? 0x80 : 0x00));
#endif

    usb_ep_set_NACK0(ep_reg);
    usb_ep_set_NACK1(ep_reg);

    usb_ep_in_set_bytecnt(ep_reg, 0);
    usb_ep_out_set_nbbyte(ep_reg, 0);

    usb_ep_ack_transfer_complete_bankO(ep_reg);
    usb_ep_ack_transfer_complete_bank1(ep_reg);

    usb_ep_TC_int_enable(ep_reg);
}

static void usb_ctrl_interrupt_error(usb_t *usbdev)
{
    usbdev_ep_t   *ep = NULL;
    USB_EP_Desc_t *ep_reg;

    /* Underflow only managed for control endpoint */
    if (usb_is_underflow_event()) {
        usb_ack_underflow_event();
        if (usb_ctrl_in_underflow()) {
            ep = _get_ep(usbdev, 0, USB_EP_DIR_IN);
            ep_reg = _ep_reg_from_idx(1);
        }
    }
    /* Overflow only managed for control endpoint */
    else {
        usb_ack_overflow_event();
        if (usb_ctrl_out_overflow()) {
            ep     = _get_ep(usbdev, 0, USB_EP_DIR_OUT);
            ep_reg = _ep_reg_from_idx(0);
        }
    }

    usb_disable_underflow_overflow_interrupt();

    if (ep == NULL) {
        /** ignored error if endpoint doesn't issued
         */
        return;
    }

    if (usb_is_tc_setup_event()) {
        /** ignored error if a transfer complete has been no processed
         */
        return;
    }

    usb_ep_TC_int_disable(ep_reg);

    /* Clear eventually previous stall events */
   usb_ctrl_out_ack_stall();
   usb_ctrl_in_ack_stall();
   usb_ack_stall_event();

#if 0
    DEBUG("EP 0x%02x int err sts: 0x%02x, ep sts: 0x%02x\n",
        ep->num | ((ep->dir == USB_EP_DIR_IN) ? 0x80 : 0x00),
	USB_INTFLAGSASET, usb_ep_get_status(ep_reg));
#endif

    usbdev->usbdev.epcb(ep, USBDEV_EVENT_ESR);
}

/**
 * @internal
 * @brief Function called by USB bus event interrupt
 *
 * USB bus event interrupt includes :
 * - USB line events SOF, reset, suspend, resume, wakeup
 * - endpoint control errors underflow, overflow, stall
 */
ISR(USB_BUSEVENT_vect, ISR_BLOCK)
{
    avr8_enter_isr();

    _disable_irq();

    _usb_dev.usbdev.cb(&_usb_dev.usbdev, USBDEV_EVENT_ESR);

    avr8_exit_isr();
}

/**
 * @internal
 * @brief Function called by USB transfer complete interrupt
 *
 * USB transfer complete interrupt includes events about endpoint transfer
 * on all endpoints.
 */
ISR(USB_TRNCOMPL_vect, ISR_BLOCK)
{
    avr8_enter_isr();

    usb_t         *dev    = &_usb_dev;
    usbdev_ep_t   *ep;
    USB_EP_Desc_t *ep_reg;
    uint8_t        ep_idx = 0;
    uint8_t        ep_dir = USB_EP_DIR_OUT;

    if (usb_is_tc_event()) {
        usb_ack_tc_event();

        ep_idx     = _decode_ep_idx_from_fifo();
        ep_dir     = (ep_idx & USB_EP_DIR_IN) ? USB_EP_DIR_IN : USB_EP_DIR_OUT;
        ep         = _get_ep(dev, ep_idx >> 1, ep_dir);
        ep_reg     = _ep_reg_from_idx(ep_idx);
    }
    else {
        usb_ack_setup_event();

        /* Clear eventually previous stall events */
        usb_ctrl_out_ack_stall();
        usb_ctrl_in_ack_stall();
        usb_ack_stall_event();

        //usb_ctrl_interrupt_tc_setup(_get_ep(dev, 0, USB_EP_DIR_OUT));

        ep         = _get_ep(dev, 0, USB_EP_DIR_OUT);
        ep_reg     = _ep_reg_from_idx(0);

        usb_enable_underflow_overflow_interrupt();
    }

#if 0
        DEBUG("num: 0x%02x, sts: %02x\n",
            (ep_idx >> 1) | ((ep_dir == USB_EP_DIR_IN) ? 0x80 : 0x00),
	    usb_ep_get_status(ep_reg));
#endif

    usb_ep_TC_int_disable(ep_reg);
    dev->usbdev.epcb(ep, USBDEV_EVENT_ESR);

    avr8_exit_isr();
}

static void _usbdev_esr(usbdev_t *dev)
{
    usb_t *usbdev = (usb_t *)dev;
    volatile uint8_t flags = USB_INTFLAGSASET;

#if 0
    DEBUG("ISR BUS INTFLAGSASET: 0x%02x\n", flags);
#endif

    if (usb_is_start_of_frame_event()) {
	usb_ack_start_of_frame_event();

        if (usb_config.sof) {
            usbdev->usbdev.cb(&usbdev->usbdev, USBDEV_EVENT_SOF);

            _enable_irq();

            return;
        }
    }

    if (usb_is_underflow_overflow_event()) {
        usb_ctrl_interrupt_error(usbdev);
    }
    else if (usb_is_reset_event()) {
        uint8_t       ep_idx;

	usb_ack_reset_event();

        for (ep_idx = 0; ep_idx < XMEGA_USB_NUM_EP; ep_idx++) {
            _ep_disable(_get_ep(usbdev, ep_idx, USB_EP_DIR_OUT));
	    _ep_disable(_get_ep(usbdev, ep_idx, USB_EP_DIR_IN));
        }

	usbdev->usbdev.cb(&usbdev->usbdev, USBDEV_EVENT_RESET);
    }
    else if (usb_is_suspend_event()) {
        usb_ack_suspend_event();

        if (!usbdev->suspended) {
	    usbdev->suspended = true;
	    usbdev->usbdev.cb(&usbdev->usbdev, USBDEV_EVENT_SUSPEND);

	    /* Low power modes are available while suspended */
	    _unblock_pm();
        }
    }
    else if (usb_is_resume_event()) {
	usb_ack_resume_event();

        if (usbdev->suspended) {
	    usbdev->suspended = false;
	    usbdev->usbdev.cb(&usbdev->usbdev, USBDEV_EVENT_RESUME);

	    /* Device wakeup detected, blocking low power modes */
	    _block_pm();
        }
    }
    else {
        DEBUG("Interrupt 0x%02x is not enabled.\n", flags);

	USB_INTFLAGSACLR = flags;
    }

    /* Re-enable the USB IRQ */
    _enable_irq();
}

static void _ep_set_stall(usbdev_ep_t *ep, usbopt_enable_t enable)
{
    USB_EP_Desc_t *ep_reg = _ep_reg_from_ep(ep);

#if 0
    DEBUG("EP 0x%02x set stall %c\n",
        ep->num | ((ep->dir == USB_EP_DIR_IN) ? 0x80 : 0x00),
	enable ? 'y' : 'n');
#endif

    if (enable == USBOPT_ENABLE) {
        usb_ep_enable_stall(ep_reg);
    }
    else {
        usb_ep_disable_stall(ep_reg);
    }
}

usbopt_enable_t _ep_get_stall(usbdev_ep_t *ep)
{
    USB_EP_Desc_t *ep_reg = _ep_reg_from_ep(ep);

#if 0
    DEBUG("EP 0x%02x is stall %c\n",
        ep->num | ((ep->dir == USB_EP_DIR_IN) ? 0x80 : 0x00),
	usb_ep_is_stall(ep_reg) ? 'y' : 'n');
#endif

    return usb_ep_is_stall(ep_reg) ? USBOPT_ENABLE : USBOPT_DISABLE;
}

static size_t _ep_get_available(usbdev_ep_t *ep)
{
    USB_EP_Desc_t *ep_reg = _ep_reg_from_ep(ep);

#if 0
    DEBUG("EP 0x%02x avail\n",
        ep->num | ((ep->dir == USB_EP_DIR_IN) ? 0x80 : 0x00));
#endif

    return (ep->dir == USB_EP_DIR_IN)
            ? usb_ep_in_nb_sent(ep_reg)
	    : usb_ep_out_nb_receiv(ep_reg);
}

static int _usbdev_ep_ready(usbdev_ep_t *ep, size_t len)
{
    USB_EP_Desc_t *ep_reg = _ep_reg_from_ep(ep);

#if 0
    DEBUG("EP 0x%02x ready with len %d@%02x and status %02x\n",
        ep->num | ((ep->dir == USB_EP_DIR_IN) ? 0x80 : 0x00),
	len, len, usb_ep_get_status(ep_reg));
#endif

    usb_ep_ack_underflow_overflow(ep_reg);

    if (ep->dir == USB_EP_DIR_IN) {
        usb_ep_in_reset_nb_sent(ep_reg);
        usb_ep_in_set_bytecnt(ep_reg, len);
    }
    else {
        usb_ep_out_reset_nb_received(ep_reg);
        usb_ep_out_set_nbbyte(ep_reg, len);
    }

    usb_ep_clear_NACK0(ep_reg);

    return 0;
}

static int _usbdev_ep_get(usbdev_ep_t *ep, usbopt_ep_t opt,
                          void *value, size_t max_len)
{
    (void)max_len;
    int res = -ENOTSUP;
    switch (opt) {
        case USBOPT_EP_STALL:
            assert(max_len == sizeof(usbopt_enable_t));
            *(usbopt_enable_t *)value = _ep_get_stall(ep);
            res = sizeof(usbopt_enable_t);
            break;
        case USBOPT_EP_AVAILABLE:
            assert(max_len == sizeof(size_t));
            *(size_t *)value = _ep_get_available(ep);
            res = sizeof(size_t);
            break;
        default:
            DEBUG("xmega_usb: Unhandled get call: 0x%x\n", opt);
            break;
    }
    return res;
}

static int _usbdev_ep_set(usbdev_ep_t *ep, usbopt_ep_t opt,
                          const void *value, size_t value_len)
{
    (void)value_len;
    int res = -ENOTSUP;
    switch (opt) {
        case USBOPT_EP_ENABLE:
            assert(value_len == sizeof(usbopt_enable_t));
            if (*((usbopt_enable_t *)value)) {
                _ep_enable(ep);
            }
            else {
                _ep_disable(ep);
            }
            res = sizeof(usbopt_enable_t);
            break;
        case USBOPT_EP_STALL:
            assert(value_len == sizeof(usbopt_enable_t));
            _ep_set_stall(ep, *(usbopt_enable_t *)value);
            res = sizeof(usbopt_enable_t);
            break;
        case USBOPT_EP_READY:
            assert(value_len == sizeof(usbopt_enable_t));
            if (*((usbopt_enable_t *)value)) {
                _usbdev_ep_ready(ep, 0);
                res = sizeof(usbopt_enable_t);
            }
            break;
        default:
            DEBUG("xmega_usb: Unhandled set call: 0x%x\n", opt);
            break;
    }
    return res;
}

/**
 * Endpoint event handler
 *
 * Calls the endpoint callback to report the event to the USB stack
 */
static void _usbdev_ep_esr(usbdev_ep_t *ep)
{
    USB_EP_Desc_t *ep_reg = _ep_reg_from_ep(ep);
    signed event = -1;

#if 0
    DEBUG("EP 0x%02x ESR 0x%02x\n",
        ep->num | ((ep->dir == USB_EP_DIR_IN) ? 0x80 : 0x00),
	usb_ep_get_status(ep_reg));
#endif

    if (usb_ep_transfer_complete(ep_reg)) {
        usb_ep_ack_transfer_complete_bankO(ep_reg);
        usb_ep_ack_transfer_complete_bank1(ep_reg);

	event = USBDEV_EVENT_TR_COMPLETE;
    }
    else if (usb_ep_is_stalled(ep_reg)) {               /* USB_EP_CRC_bm    */
        usb_ep_ack_stall(ep_reg);

	event = USBDEV_EVENT_TR_STALL;
    }
    else if (usb_ep_is_underflow_overflow(ep_reg)) {    /* USB_EP_OVF_bm    */
        usb_ep_ack_underflow_overflow(ep_reg);

	event = USBDEV_EVENT_TR_FAIL;
#if 1
    DEBUG("EP 0x%02x ESR un/ov\n",
        ep->num | ((ep->dir == USB_EP_DIR_IN) ? 0x80 : 0x00));
#endif
    }

    if (event >= 0) {
        ep->dev->epcb(ep, event);
    }

    usb_ep_TC_int_enable(ep_reg);
}

const usbdev_driver_t driver = {
    .init = _usbdev_init,
    .new_ep = _usbdev_new_ep,
    .get = _usbdev_get,
    .set = _usbdev_set,
    .esr = _usbdev_esr,
    .ep_init = _usbdev_ep_init,
    .ep_get = _usbdev_ep_get,
    .ep_set = _usbdev_ep_set,
    .ep_esr = _usbdev_ep_esr,
    .ready = _usbdev_ep_ready,
};
