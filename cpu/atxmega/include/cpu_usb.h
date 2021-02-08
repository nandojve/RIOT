/*
 * Copyright (C) 2021 Gerson Fernando Budke
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    cpu_atxmega_usb xmega USB peripheral
 * @ingroup     cpu_atxmega
 * @brief       USB interface functions for the xmega class devices
 *
 * @{
 *
 * @file
 * @brief       USB interface functions for the xmega class devices
 *
 * @author      Gerson Fernando Budke <nandojve@gmail.com>
 */

#ifndef CPU_USB_H
#define CPU_USB_H

#include <stdint.h>
#include <stdlib.h>
#include "periph_cpu.h"
#include "periph/usbdev.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * USB endpoint buffer space
 */
#define XMEGA_USB_BUF_SPACE     USBDEV_EP_BUF_SPACE

/**
 * Number of USB IN and OUT endpoints
 */
#define XMEGA_USB_NUM_EP        USBDEV_NUM_ENDPOINTS

/**
 * @brief USB peripheral device context
 */
typedef struct {
    usbdev_t                    usbdev;                          /**< Inherited usbdev struct */
    __attribute__ ((aligned(16)))
    uint8_t                     buffer[XMEGA_USB_BUF_SPACE];     /**< Buffer space, must be
                                                                      32-bit aligned          */
    const usb_config_t         *config;                          /**< USB peripheral config   */
    usbdev_ep_t                 endpoints[2 * XMEGA_USB_NUM_EP]; /**< Endpoints               */
    size_t                      used;                            /**< Number of bytes from the
                                                                      buffer that are used    */
    bool                        suspended;                       /**< Suspend active          */
} usb_t;

/**
 * @ingroup usb_group
 * @defgroup xmega_usb_group Xmega USB Device Driver
 * USB low-level driver for USB Device mode
 * @{
 */

/**
 * @name USB Device read/modify/write management
 * @{
 */
#ifndef USB_WORKAROUND_DO_NOT_USE_RMW
/*
 * Read modify write new instructions for Xmega
 * inline asm implementation with R16 register.
 */

/* Load and Clear */
#define LACR16(addr,msk) \
   __asm__ __volatile__ ( \
         "ldi r16, %1" "\n\t" \
         ".dc.w 0x9306" "\n\t"\
         ::"z" (addr), "M" (msk):"r16")

/* Load and Set */
#define LASR16(addr,msk) \
   __asm__ __volatile__ ( \
         "ldi r16, %1" "\n\t" \
         ".dc.w 0x9305" "\n\t"\
         ::"z" (addr), "M" (msk):"r16")

/* Exchange */
#define XCHR16(addr,msk) \
   __asm__ __volatile__ ( \
         "ldi r16, %1" "\n\t" \
         ".dc.w 0x9304" "\n\t"\
         ::"z" (addr), "M" (msk):"r16")

/* Load and toggle */
#define LATR16(addr,msk) \
   __asm__ __volatile__ ( \
         "ldi r16, %1" "\n\t" \
         ".dc.w 0x9307" "\n\t"\
         ::"z" (addr), "M" (msk):"r16")

#else

/* Load and Clear */
#define LACR16(addr,msk) (*addr &= ~msk)
/* Load and Set */
#define LASR16(addr,msk)(*addr |= msk)

#endif
/** @} */

/**
 * @name USB Device main management
 * @{
 */
#define  usb_enable_interface()                      (USB_CTRLA |= USB_ENABLE_bm)
#define  usb_disable_interface()                     (USB_CTRLA &= ~USB_ENABLE_bm)
#define  usb_attach_device()                         (USB_CTRLB |= USB_ATTACH_bm)
#define  usb_detach_device()                         (USB_CTRLB &= ~USB_ATTACH_bm)
#define  usb_gnak_disable()                          (USB_CTRLB &= ~USB_GNACK_bm)
#define  usb_gnak_enable()                           (USB_CTRLB |=  USB_GNACK_bm)
#define  usb_gnak_is_enable()                        (USB_CTRLB & USB_GNACK_bm)
#define  usb_set_nb_max_ep(n)                        (USB_CTRLA |= n)
#define  usb_enable_store_frame_number()             (USB_CTRLA |= USB_STFRNUM_bm)
#define  usb_disable_store_frame_number()            (USB_CTRLA &= ~USB_STFRNUM_bm)
#define  usb_set_full_speed()                        (USB_CTRLA |= USB_SPEED_bm)
#define  usb_set_low_speed()                         (USB_CTRLA &= ~USB_SPEED_bm)
#define  usb_set_ep_table_addr(n)                    (USB.EPPTR = (uint16_t)n)
#define  usb_get_ep_table_addr()                     (USB.EPPTR)
#define  usb_get_fifo_rp()                           (USB_FIFORP)
#define  usb_reset_fifo()                            (USB_FIFORP=0xFF)
#define  usb_enable_interrupt(level)                 (USB_INTCTRLA |= level&(USB_INTLVL1_bm|USB_INTLVL0_bm))

#define  usb_set_device_address(n)                   (USB_ADDR=n)
#define  usb_get_device_address()                    (USB_ADDR)
#define  usb_enable_fifo()                           (USB_CTRLA |= USB_FIFOEN_bm)
#define  usb_disable_fifo()                          (USB_CTRLA &= ~USB_FIFOEN_bm)

#define  usb_send_remote_wake_up()                   (USB_CTRLB &= ~USB_RWAKEUP_bm, USB_CTRLB |= USB_RWAKEUP_bm)
#define  usb_set_global_nack()                       (USB_CTRLB |= USB_GNACK_bm)
#define  usb_is_crc_event()                          (USB_INTFLAGSASET & USB_CRCIF_bm ? true : false)
#define  usb_ack_crc_event()                         (USB_INTFLAGSACLR = USB_CRCIF_bm)
#define  usb_set_crc_event()                         (USB_INTFLAGSASET = USB_CRCIF_bm)
#define  usb_enable_crc_interrupt()                  (USB_INTCTRLA |= USB_CRCIE_bm)
#define  usb_disable_crc_interrupt()                 (USB_INTCTRLA &= ~USB_CRCIE_bm)

#define  usb_is_start_of_frame_event()               (USB_INTFLAGSASET & USB_SOFIF_bm ? true : false)
#define  usb_ack_start_of_frame_event()              (USB_INTFLAGSACLR = USB_SOFIF_bm)
#define  usb_set_start_of_frame_event()              (USB_INTFLAGSASET = USB_SOFIF_bm)
#define  usb_enable_start_of_frame_interrupt()       (USB_INTCTRLA |= USB_SOFIE_bm)
#define  usb_disable_start_of_frame_interrupt()      (USB_INTCTRLA &= ~USB_SOFIE_bm)
#define  usb_is_enable_start_of_frame_interrupt()    (0!=(USB_INTCTRLA|USB_SOFIE_bm))

#define  usb_is_reset_event()                        (USB_INTFLAGSASET & USB_RSTIF_bm ? true : false)
#define  usb_ack_reset_event()                       (USB_INTFLAGSACLR = USB_RSTIF_bm)
#define  usb_set_reset_event()                       (USB_INTFLAGSASET = USB_RSTIF_bm)

#define  usb_is_suspend_event()                      (USB_INTFLAGSASET & USB_SUSPENDIF_bm ? true : false)
#define  usb_ack_suspend_event()                     (USB_INTFLAGSACLR = USB_SUSPENDIF_bm)
#define  usb_set_suspend_event()                     (USB_INTFLAGSASET = USB_SUSPENDIF_bm)

#define  usb_is_resume_event()                       (USB_INTFLAGSASET & USB_RESUMEIF_bm ? true : false)
#define  usb_ack_resume_event()                      (USB_INTFLAGSACLR = USB_RESUMEIF_bm)
#define  usb_set_resume_event()                      (USB_INTFLAGSASET = USB_RESUMEIF_bm)

#define  usb_enable_busevt_interrupt()               (USB_INTCTRLA |= USB_BUSEVIE_bm)
#define  usb_disable_busevt_interrupt()              (USB_INTCTRLA &= ~USB_BUSEVIE_bm)

#define  usb_is_setup_event()                        (USB_INTFLAGSBCLR & USB_SETUPIF_bm ? true : false)
#define  usb_ack_setup_event()                       (USB_INTFLAGSBCLR = USB_SETUPIF_bm)
#define  usb_set_setup_event()                       (USB_INTFLAGSBSET = USB_SETUPIF_bm)
#define  usb_enable_setup_interrupt()                (USB_INTCTRLB |= USB_SETUPIE_bm)
#define  usb_disable_setup_interrupt()               (USB_INTCTRLB &= ~USB_SETUPIE_bm)

#define  usb_is_tc_event()                           (USB_INTFLAGSBCLR & USB_TRNIF_bm ? true : false)
#define  usb_ack_tc_event()                          (USB_INTFLAGSBCLR = USB_TRNIF_bm)
#define  usb_set_tc_event()                          (USB_INTFLAGSBSET = USB_TRNIF_bm)
#define  usb_enable_tc_interrupt()                   (USB_INTCTRLB |= USB_TRNIE_bm)
#define  usb_disable_tc_interrupt()                  (USB_INTCTRLB &= ~USB_TRNIE_bm)

#define  usb_is_tc_setup_event()                     (USB_INTFLAGSBCLR & (USB_TRNIF_bm | USB_SETUPIF_bm) ? true : false)

#define  usb_is_overflow_event()                     (USB_INTFLAGSASET & USB_OVFIF_bm ? true : false)
#define  usb_ack_overflow_event()                    (USB_INTFLAGSACLR = USB_OVFIF_bm)
#define  usb_set_overflow_event()                    (USB_INTFLAGSASET = USB_OVFIF_bm)
#define  usb_enable_overflow_interrupt()             (USB_INTCTRLA |= USB_BUSERRIE_bm)
#define  usb_disable_overflow_interrupt()            (USB_INTCTRLA &= ~USB_BUSERRIE_bm)
#define  usb_is_enable_overflow_interrupt()          (USB_INTCTRLA&USB_BUSERRIE_bm ? true : false)

#define  usb_is_underflow_event()                    (USB_INTFLAGSASET & USB_UNFIF_bm ? true : false)
#define  usb_ack_underflow_event()                   (USB_INTFLAGSACLR = USB_UNFIF_bm)
#define  usb_set_underflow_event()                   (USB_INTFLAGSASET = USB_UNFIF_bm)
#define  usb_enable_underflow_interrupt()            (USB_INTCTRLA |= USB_BUSERRIE_bm)
#define  usb_disable_underflow_interrupt()           (USB_INTCTRLA &= ~USB_BUSERRIE_bm)
#define  usb_is_enable_underflow_interrupt()         (USB_INTCTRLA&USB_BUSERRIE_bm ? true : false)

#define  usb_is_underflow_overflow_event()           (USB_INTFLAGSASET & (USB_UNFIF_bm | USB_OVFIF_bm) ? true : false)
#define  usb_enable_underflow_overflow_interrupt()   (USB_INTCTRLA |= USB_BUSERRIE_bm)
#define  usb_disable_underflow_overflow_interrupt()  (USB_INTCTRLA &= ~USB_BUSERRIE_bm)

#define  usb_is_stall_event()                        (USB_INTFLAGSASET & USB_STALLIF_bm ? true : false)
#define  usb_ack_stall_event()                       (USB_INTFLAGSACLR = USB_STALLIF_bm)
#define  usb_set_stall_event()                       (USB_INTFLAGSASET = USB_STALLIF_bm)
#define  usb_enable_stall_interrupt()                (USB_INTCTRLA |= USB_STALLIE_bm)
#define  usb_disable_stall_interrupt()               (USB_INTCTRLA &= ~USB_STALLIE_bm)
#define  usb_is_enable_stall_interrupt()             (USB_INTCTRLA&USB_STALLIE_bm ? true : false)
/** @} */

/**
 * @name USB Device endpoints table management
 * @{
 */
#define  usb_ep_set_control(ep_ctrl,val)             (ep_ctrl->CTRL=val)
#define  usb_ep_get_control(ep_ctrl)                 (ep_ctrl->CTRL)

#define  usb_ep_disable(ep_ctrl)                     usb_ep_set_control(ep_ctrl,0)
#define  usb_ep_is_enable(ep_ctrl)                   (USB_EP_TYPE_DISABLE_gc!=usb_ep_get_type(ep_ctrl))


#define  usb_ep_enable_stall(ep_ctrl)                (ep_ctrl->CTRL |= USB_EP_STALL_bm)
#define  usb_ep_disable_stall(ep_ctrl)               (ep_ctrl->CTRL &= ~USB_EP_STALL_bm)
#define  usb_ep_is_stall(ep_ctrl)                    (ep_ctrl->CTRL &USB_EP_STALL_bm ? true : false)
#define  usb_ep_set_multipacket(ep_ctrl)             (ep_ctrl->CTRL |= USB_EP_MULTIPKT_bm)
#define  usb_ep_TC_int_enable(ep_ctrl)               (ep_ctrl->CTRL &= ~USB_EP_INTDSBL_bm)
#define  usb_ep_TC_int_disable(ep_ctrl)              (ep_ctrl->CTRL |= USB_EP_INTDSBL_bm)
#define  usb_ep_set_pingpong(ep_ctrl)                (ep_ctrl->CTRL |= USB_EP_PINGPONG_bm)
#define  usb_ep_get_size_field(ep_ctrl)              (ep_ctrl->CTRL & USB_EP_BUFSIZE_gm)
#define  usb_ep_get_type(ep_ctrl)                    (ep_ctrl->CTRL & USB_EP_TYPE_gm)

#define  usb_ep_get_status(ep_ctrl)                  (ep_ctrl->STATUS)
#define  usb_ep_clear_status(ep_ctrl)                (ep_ctrl->STATUS=USB_EP_BUSNACK0_bm|USB_EP_BUSNACK1_bm)

#define  usb_ep_is_stalled(ep_ctrl)                  (ep_ctrl->STATUS&USB_EP_STALLF_bm ? true : false)
#define  usb_ep_ack_stall(ep_ctrl)                   LACR16(&ep_ctrl->STATUS,USB_EP_STALLF_bm)

#define  usb_ep_setup_received(ep_ctrl)              (ep_ctrl->STATUS&USB_EP_SETUP_bm ? true : false)
#define  usb_ep_ack_setup_received(ep_ctrl)          LACR16(&ep_ctrl->STATUS, USB_EP_SETUP_bm)

#define  usb_ep_transfer_complete(ep_ctrl)           (ep_ctrl->STATUS&(USB_EP_TRNCOMPL0_bm | USB_EP_SETUP_bm) ? true : false)
#define  usb_ep_ack_transfer_complete(ep_ctrl)       LACR16(&(ep_ctrl->STATUS), USB_EP_TRNCOMPL0_bm)
#define  usb_ep_transfer_complete_bank0(ep_ctrl)     (ep_ctrl->STATUS&USB_EP_TRNCOMPL0_bm ? true : false)
#define  usb_ep_ack_transfer_complete_bankO(ep_ctrl) LACR16(&ep_ctrl->STATUS, USB_EP_TRNCOMPL0_bm)
#define  usb_ep_transfer_complete_bank1(ep_ctrl)     (ep_ctrl->STATUS&USB_EP_SETUP_bm ? true : false)
#define  usb_ep_ack_transfer_complete_bank1(ep_ctrl) LACR16(&ep_ctrl->STATUS, USB_EP_SETUP_bm)

#define  usb_ep_get_bank(ep_ctrl)                    (ep_ctrl->STATUS & USB_EP_BANK_bm ? true : false)
#define  usb_ep_set_bank(ep_ctrl)                    LASR16(&ep_ctrl->STATUS, USB_EP_BANK_bm)
#define  usb_ep_clear_bank(ep_ctrl)                  LACR16(&ep_ctrl->STATUS, USB_EP_BANK_bm)

#define  usb_ep_set_dtgl(ep_ctrl)                    LASR16(&ep_ctrl->STATUS,USB_EP_TOGGLE_bm)
#define  usb_ep_clear_dtgl(ep_ctrl)                  LACR16(&ep_ctrl->STATUS, USB_EP_TOGGLE_bm )
#define  usb_ep_get_dtgl(ep_ctrl)                    ((ep_ctrl->STATUS)&USB_EP_TOGGLE_bm ? true : false)
#define  usb_ep_toggle_dtgl(ep_ctrl)                 LATR16(&ep_ctrl->STATUS, USB_EP_TOGGLE_bm)

#define  usb_ep_set_NACK0(ep_ctrl)                   LASR16(&ep_ctrl->STATUS,USB_EP_BUSNACK0_bm)
#define  usb_ep_set_NACK1(ep_ctrl)                   LASR16(&ep_ctrl->STATUS,USB_EP_BUSNACK1_bm)
#define  usb_ep_clear_NACK0(ep_ctrl)                 LACR16(&ep_ctrl->STATUS, USB_EP_BUSNACK0_bm)
#define  usb_ep_clear_NACK1(ep_ctrl)                 LACR16(&ep_ctrl->STATUS, USB_EP_BUSNACK1_bm)
#define  usb_ep_get_NACK1(ep_ctrl)                   ((ep_ctrl->STATUS&USB_EP_BUSNACK1_bm) ? true : false)
#define  usb_ep_get_NACK0(ep_ctrl)                   ((ep_ctrl->STATUS&USB_EP_BUSNACK0_bm) ? true : false)
#define  usb_ep_overflow(ep_ctrl)                    (ep_ctrl->STATUS&USB_EP_OVF_bm ? true : false)
#define  usb_ep_underflow(ep_ctrl)                   (ep_ctrl->STATUS&USB_EP_UNF_bm ? true : false)
#define  usb_ep_is_underflow_overflow(ep_ctrl)       (ep_ctrl->STATUS&USB_EP_UNF_bm ? true : false)
#define  usb_ep_ack_underflow_overflow(ep_ctrl)      LACR16(&ep_ctrl->STATUS,USB_EP_UNF_bm)

#define  CPU_ATXMEGA_USB_ENDPOINT_MAX_TRANS                           (0x3FF)

#define  usb_ep_out_nb_receiv(ep_ctrl)               (ep_ctrl->CNT)
#define  usb_ep_out_reset_nb_received(ep_ctrl)       (ep_ctrl->CNT = 0)
#define  usb_ep_in_set_bytecnt(ep_ctrl,n)            (ep_ctrl->CNT = n)
#define  usb_ep_set_azlp(ep_ctrl)                    (ep_ctrl->CNT |= 0x8000)
#define  usb_ep_clear_azlp(ep_ctrl)                  (ep_ctrl->CNT &= ~0x8000)

#define  usb_ep_set_buf(ep_ctrl,buf)                 (ep_ctrl->DATAPTR = (uint16_t) buf)

#define  usb_ep_in_nb_sent(ep_ctrl)                  (ep_ctrl->AUXDATA)
#define  usb_ep_in_reset_nb_sent(ep_ctrl)            (ep_ctrl->AUXDATA = 0)
#define  usb_ep_out_set_nbbyte(ep_ctrl,nb)           (ep_ctrl->AUXDATA = nb)
#define  usb_ep_out_get_nbbyte_requested(ep_ctrl)    (ep_ctrl->AUXDATA)
#define  usb_ep_set_aux(ep_ctrl,buf)                 (ep_ctrl->AUXDATA = (uint16_t) buf)
/** @} */

/**
 * @name USB Device endpoint control setup field management
 * @{
 */
#define  usb_ctrl_setup()                            (usb_sram.ep_ctrl[0].STATUS&USB_EP_SETUP_bm ? true : false)
#define  usb_ctrl_ack_setup()                        LACR16(&usb_sram.ep_ctrl[0].STATUS,USB_EP_SETUP_bm)
/** @} */

/**
 * @name USB Device endpoint control OUT field management
 * @{
 */
#define  usb_ctrl_out_is_enable_stall()              (usb_sram.ep_ctrl[0].CTRL&USB_EP_STALL_bm ? true : false)
#define  usb_ctrl_out_enable_stall()                 LASR16(&usb_sram.ep_ctrl[0].CTRL,USB_EP_STALL_bm)
#define  usb_ctrl_out_disable_stall()                LACR16(&usb_sram.ep_ctrl[0].CTRL,USB_EP_STALL_bm)
#define  usb_ctrl_out_is_stalled()                   (usb_sram.ep_ctrl[0].STATUS&USB_EP_STALLF_bm ? true : false)
#define  usb_ctrl_out_ack_stall()                    LACR16(&usb_sram.ep_ctrl[0].STATUS,USB_EP_STALLF_bm)
#define  usb_ctrl_out_set_NACK0()                    LASR16(&usb_sram.ep_ctrl[0].STATUS,USB_EP_BUSNACK0_bm)
#define  usb_ctrl_out_clear_NACK0()                  LACR16(&usb_sram.ep_ctrl[0].STATUS,USB_EP_BUSNACK0_bm)

#define  usb_ctrl_out_overflow()                     (usb_sram.ep_ctrl[0].STATUS&USB_EP_OVF_bm ? true : false)
#define  usb_ctrl_ack_out_overflow()                 LACR16(&usb_sram.ep_ctrl[0].STATUS,USB_EP_OVF_bm)

#define  usb_ctrl_out_tc()                           (usb_sram.ep_ctrl[0].STATUS&USB_EP_TRNCOMPL0_bm ? true : false)
#define  usb_ctrl_out_ack_tc()                       LACR16(&usb_sram.ep_ctrl[0].STATUS,USB_EP_TRNCOMPL0_bm)
#define  usb_ctrl_out_set_tc()                       LASR16(&usb_sram.ep_ctrl[0].STATUS,USB_EP_TRNCOMPL0_bm)

#define  usb_ctrl_out_dt_get()                       (usb_sram.ep_ctrl[0].STATUS&USB_EP_TOGGLE_bm ? true : false)
#define  usb_ctrl_out_dt_set()                       LASR16(&usb_sram.ep_ctrl[0].STATUS,USB_EP_TOGGLE_bm )
#define  usb_ctrl_out_dt_clear()                     LACR16(&usb_sram.ep_ctrl[0].STATUS,USB_EP_TOGGLE_bm )
#define  usb_ctrl_out_dt_toggle()                    LATR16(&usb_sram.ep_ctrl[0].STATUS,USB_EP_TOGGLE_bm)

#define  usb_ctrl_out_set_buf(buf)                   (usb_sram.ep_ctrl[0].DATAPTR = (uint16_t) buf)

#define  usb_ctrl_out_get_bytecnt()                  (usb_sram.ep_ctrl[0].CNT)
/** @} */

/**
 * @name USB Device endpoint control IN field management
 * @{
 */
#define  usb_ctrl_in_is_enable_stall()               (usb_sram.ep_ctrl[1].CTRL&USB_EP_STALL_bm ? true : false)
#define  usb_ctrl_in_enable_stall()                  LASR16(&usb_sram.ep_ctrl[1].CTRL,USB_EP_STALL_bm)
#define  usb_ctrl_in_disable_stall()                 LACR16(&usb_sram.ep_ctrl[1].CTRL,USB_EP_STALL_bm)
#define  usb_ctrl_in_is_stalled()                    (usb_sram.ep_ctrl[1].STATUS&USB_EP_STALLF_bm ? true : false)
#define  usb_ctrl_in_ack_stall()                     LACR16(&usb_sram.ep_ctrl[1].STATUS,USB_EP_STALLF_bm)
#define  usb_ctrl_in_set_NACK0()                     LASR16(&usb_sram.ep_ctrl[1].STATUS,USB_EP_BUSNACK0_bm)
#define  usb_ctrl_in_clear_NACK0()                   LACR16(&usb_sram.ep_ctrl[1].STATUS,USB_EP_BUSNACK0_bm)

#define  usb_ctrl_in_underflow()                     (usb_sram.ep_ctrl[1].STATUS&USB_EP_UNF_bm ? true : false)
#define  usb_ctrl_ack_in_underflow()                 LACR16(&usb_sram.ep_ctrl[1].STATUS,USB_EP_UNF_bm)

#define  usb_ctrl_in_tc()                            (usb_sram.ep_ctrl[1].STATUS&USB_EP_TRNCOMPL0_bm ? true : false)
#define  usb_ctrl_in_ack_tc()                        LACR16(&usb_sram.ep_ctrl[1].STATUS,USB_EP_TRNCOMPL0_bm)
#define  usb_ctrl_in_set_tc()                        LASR16(&usb_sram.ep_ctrl[1].STATUS,USB_EP_TRNCOMPL0_bm)

#define  usb_ctrl_in_dt_get()                        (usb_sram.ep_ctrl[1].STATUS&USB_EP_TOGGLE_bm ? true : false)
#define  usb_ctrl_in_dt_set()                        LASR16(&usb_sram.ep_ctrl[1].STATUS,USB_EP_TOGGLE_bm )
#define  usb_ctrl_in_dt_clear()                      LACR16(&usb_sram.ep_ctrl[1].STATUS,USB_EP_TOGGLE_bm )
#define  usb_ctrl_in_dt_toggle()                     LATR16(&usb_sram.ep_ctrl[1].STATUS,USB_EP_TOGGLE_bm)

#define  usb_ctrl_in_set_buf(buf)                    (usb_sram.ep_ctrl[1].DATAPTR = (uint16_t) buf)

#define  usb_ctrl_in_set_bytecnt(n)                  (usb_sram.ep_ctrl[1].CNT = n)
/** @} */
/** @} */

/** @} */

#ifdef __cplusplus
}
#endif
#endif /* CPU_USB_H */
/** @} */
