/*
 * Copyright (C) 2021 Gerson Fernando Budke <nandojve@gmail.com>
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
 * @brief           CPU specific definitions for internal peripheral handling
 *
 * @author          Gerson Fernando Budke <nandojve@gmail.com>
 */

#ifndef PERIPH_CPU_H
#define PERIPH_CPU_H

#include <avr/io.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name   Length of the CPU_ID in octets
 * @{
 */
#define CPUID_LEN               (11U)
/** @} */

/**
 * @name   Interrupt level used to control nested interrupts
 * @{
 */
typedef enum {
    CPU_INT_LVL_OFF,    /**< Interrupt Disabled     */
    CPU_INT_LVL_LOW,    /**< Interrupt Low Level    */
    CPU_INT_LVL_MID,    /**< Interrupt Medium Level */
    CPU_INT_LVL_HIGH,   /**< Interrupt High Level   */
} cpu_int_lvl_t;
/** @} */

/**
 * @name   CPU Clocks and Oscilators configuration
 * @{
 */

/**
 * @brief  CPU Clock/Oscilator configuration Enable/Disable
 */
typedef enum {
    CPU_CLK_OFF,
    CPU_CLK_EN,
} cpu_clk_enable_t;

/**
 * @brief  CPU Clocks and Oscilators structure
 */
typedef struct {
    CLK_SCLKSEL_t    sys_src;
    CLK_PSADIV_t     psa_div;
    CLK_PSBCDIV_t    psbc_div;

    OSC_FRQRANGE_t   xosc_freq_range;
    OSC_XOSCSEL_t    xosc_src;
    cpu_clk_enable_t xosc_32k_ulp;   /**< Reduce the swing on the TOSC2 pin */
    cpu_clk_enable_t xosc_ulp;       /**< Reduce the swing on the XTAL2 pin */
    cpu_clk_enable_t xosc_pin;       /**< Ext. clock pin is port C, pin 4   */
    cpu_clk_enable_t xosc_en;
    uint32_t         xosc_freq;      /**< Ext. Oscilator frequency          */

    OSC_RC2MCREF_t   dfll_2M_src;
    cpu_clk_enable_t dfll_2M_en;
    OSC_RC32MCREF_t  dfll_32M_src;
    cpu_clk_enable_t dfll_32M_en;

    uint8_t          int_32k_cal;
    cpu_clk_enable_t int_32k_en;
    cpu_clk_enable_t int_2M_en;
    uint8_t          int_8M_cal;
    uint8_t          int_8M_ulp;
    cpu_clk_enable_t int_8M_en;
    cpu_clk_enable_t int_32M_en;
    uint32_t         int_32M_freq;   /**< RC32M calibrated frequency        */

    OSC_PLLSRC_t     pll_src;
    cpu_clk_enable_t pll_div_by2;
    uint8_t          pll_mul_factor;
    cpu_clk_enable_t pll_en;
    uint32_t         pll_freq;       /**< PLL output frequency              */

    CLK_USBPSDIV_t   usb_div;
    CLK_USBSRC_t     usb_src;
    cpu_clk_enable_t usb_en;
} cpu_clk_osc_t;
/** @} */

/**
 * @brief   Available ports on the ATxmega family
 */
enum {
    PORT_A,     /**< port A - 600 - 0 */
    PORT_B,     /**< port B - 620 - 1 */
    PORT_C,     /**< port C - 640 - 2 */
    PORT_D,     /**< port D - 660 - 3 */
    PORT_E,     /**< port E - 680 - 4 */
    PORT_F,     /**< port F - 6A0 - 5 */
    PORT_G,     /**< port G - 6C0 - 6 */
    PORT_H,     /**< port H - 6E0 - 7 */
    PORT_J,     /**< port J - 700 - 8 */
    PORT_K,     /**< port K - 720 - 9 */
    PORT_L,     /**< port L - 740 - A */
    PORT_M,     /**< port M - 760 - B */
    PORT_N,     /**< port N - 780 - C */
    PORT_P,     /**< port P - 7A0 - D */
    PORT_Q,     /**< port Q - 7C0 - E */
    PORT_R,     /**< port R - 7E0 - F */
    /* ... */
    PORT_MAX,
};

/**
 * @name    Power management configuration
 * @{
 */
#define PM_NUM_MODES            (4)
/** @} */

/**
 * @brief   Define the number of GPIO interrupts vectors for ATxmega CPU.
 * @{
 */
#define GPIO_EXT_INT_NUMOF      (2 * PORT_MAX)
/** @} */

/**
 * @brief   Override GPIO type
 * @{
 */
#define HAVE_GPIO_T
typedef uint16_t gpio_t;
/** @} */

/**
 * @brief   Definition of a fitting UNDEF value
 */
#define GPIO_UNDEF              (0xffff)

/**
 * @brief   Define a CPU specific GPIO pin generator macro
 *
 * The ATxmega internally uses pin mask to manipulate all gpio functions.
 * This allows simultaneous pin actions at any method call.  ATxmega specific
 * applications can use ATXMEGA_GPIO_PIN macro to define pins and generic
 * RIOT-OS application should continue to use GPIO_PIN API for compatibility.
 *
 * @{
 */
#define ATXMEGA_GPIO_PIN(x, y)  (((x & 0x0f) << 8) | (y & 0xff))
#define GPIO_PIN(x, y)          ATXMEGA_GPIO_PIN(x, (1U << (y & 0x07)))
/** @} */

/**
 * @brief   Available pin modes
 *
 * Generally, a pin can be configured to be input or output. In output mode, a
 * pin can further be put into push-pull or open drain configuration. Though
 * this is supported by most platforms, this is not always the case, so driver
 * implementations may return an error code if a mode is not supported.
 * @{
 */
#define HAVE_GPIO_MODE_T
typedef enum GPIO_MODE {
    GPIO_SLEW_RATE            = (1 << 7), /**< enable slew rate */
    GPIO_INVERTED             = (1 << 6), /**< enable inverted signal */

    GPIO_OPC_TOTEN            = (0 << 3), /**< select no pull resistor (TOTEM) */
    GPIO_OPC_BSKPR            = (1 << 3), /**< push-pull mode (BUSKEEPER) */
    GPIO_OPC_PD               = (2 << 3), /**< pull-down resistor */
    GPIO_OPC_PU               = (3 << 3), /**< pull-up resistor */
    GPIO_OPC_WRD_OR           = (4 << 3), /**< enable wired OR  */
    GPIO_OPC_WRD_AND          = (5 << 3), /**< enable wired AND */
    GPIO_OPC_WRD_OR_PULL      = (6 << 3), /**< enable wired OR and pull-down resistor */
    GPIO_OPC_WRD_AND_PULL     = (7 << 3), /**< enable wired AND and pull-up resistor */

    GPIO_ANALOG               = (1 << 1), /**< select GPIO for analog function */

    GPIO_IN                   = (0 << 0), /**< select GPIO MASK as input */
    GPIO_OUT                  = (1 << 0), /**< select GPIO MASK as output */

    /* Compatibility Mode */
    GPIO_IN_PU                = GPIO_IN  | GPIO_OPC_PU,
    GPIO_IN_PD                = GPIO_IN  | GPIO_OPC_PD,
    GPIO_OD                   = GPIO_OUT | GPIO_OPC_WRD_OR,
    GPIO_OD_PU                = GPIO_OUT | GPIO_OPC_WRD_OR_PULL,
} gpio_mode_t;
/** @} */

/**
 * @brief   Definition of possible active flanks for external interrupt mode
 * @{
 */
#define HAVE_GPIO_FLANK_T
typedef enum {
    GPIO_ISC_BOTH             = (0 << 4), /**< emit interrupt on both flanks (default) */
    GPIO_ISC_RISING           = (1 << 4), /**< emit interrupt on rising flank */
    GPIO_ISC_FALLING          = (2 << 4), /**< emit interrupt on falling flank */
    GPIO_ISC_LOW_LEVEL        = (3 << 4), /**< emit interrupt on low level */

    GPIO_INT_DISABLED_ALL     = (1 << 3), /**< disable all interrupts */

    GPIO_INT0_VCT             = (0 << 2), /**< enable interrupt on Vector 0 (default) */
    GPIO_INT1_VCT             = (1 << 2), /**< enable interrupt on Vector 1 */

    GPIO_LVL_OFF              = (0 << 0), /**< interrupt disabled (default) */
    GPIO_LVL_LOW              = (1 << 0), /**< interrupt low level */
    GPIO_LVL_MID              = (2 << 0), /**< interrupt medium level */
    GPIO_LVL_HIGH             = (3 << 0), /**< interrupt higher */

    /* Compatibility Mode */
    GPIO_FALLING              = GPIO_ISC_FALLING | GPIO_LVL_LOW,
    GPIO_RISING               = GPIO_ISC_RISING  | GPIO_LVL_LOW,
    GPIO_BOTH                 = GPIO_ISC_BOTH    | GPIO_LVL_LOW,
} gpio_flank_t;
/** @} */

/**
 * @brief   Max number of available UARTs
 */
#define UART_MAX_NUMOF          (7)

/**
 * @brief   Size of the UART TX buffer for non-blocking mode.
 */
#ifndef UART_TXBUF_SIZE
#define UART_TXBUF_SIZE         (64)
#endif

/**
 * @brief   UART device configuration
 */
typedef struct {
    USART_t *dev;                   /**< pointer to the used UART device */
    gpio_t rx_pin;                  /**< pin used for RX */
    gpio_t tx_pin;                  /**< pin used for TX */
#ifdef MODULE_PERIPH_UART_HW_FC
    gpio_t rts_pin;                 /**< RTS pin */
    gpio_t cts_pin;                 /**< CTS pin */
#endif
    cpu_int_lvl_t rx_int_lvl;       /**< RX Complete Interrupt Level */
    cpu_int_lvl_t tx_int_lvl;       /**< TX Complete Interrupt Level */
    cpu_int_lvl_t dre_int_lvl;      /**< Data Registry Empty Interrupt Level */
} uart_conf_t;

/**
 * @brief   Max number of available timer channels
 */
#define TIMER_CH_MAX_NUMOF      (4)

/**
 * @brief   A low-level timer_set() implementation is provided
 */
#define PERIPH_TIMER_PROVIDES_SET

/**
 * @brief   Timer Type
 *
 * Timer Type 1 is equal to Type 0 (two channels instead four)
 * Timer Type 2 is Type 0 configured as two 8 bit timers instead one 16 bit
 * Timer Type 2 won't be available as a standard timer
 * Timer Type 5 is equal to Type 4 (two channels instead four)
 */
typedef enum {
    TC_TYPE_0                 = 0,
    TC_TYPE_1                 = 1,
    TC_TYPE_2                 = 2,
    TC_TYPE_4                 = 4,
    TC_TYPE_5                 = 5,
} timer_type_t;

/**
 * @brief   Timer device configuration
 *
 * All timers can be derived from TC0_t struct. Need check at runtime the
 * type and number of channels to perform all operations.
 */
typedef struct {
    TC0_t *dev;                                 /**< Pointer to the used as Timer device */
    timer_type_t type;                          /**< Timer Type */
    cpu_int_lvl_t int_lvl[TIMER_CH_MAX_NUMOF];  /**< Interrupt channels level */
} timer_conf_t;

/**
 * @brief   EBI (External Bus Interface)
 * {@
 */

/**
 * @brief   EBI Low Pin Count (LPC) Mode ALE config
 */
typedef enum {
    EBI_LPC_MODE_ALE1       = 0x01,     /**< Data multiplexed with Address byte 0 */
    EBI_LPC_MODE_ALE12      = 0x03,     /**< Data multiplexed with Address byte 0 and 1 */
} ebi_lpc_mode_t;

/**
 * @brief   EBI Port Access Flags
 *
 * Indicate what should be configured
 */
typedef enum {
    EBI_PORT_3PORT          = 0x01,     /**< Three Port Config */
    EBI_PORT_SDRAM          = 0x02,     /**< SDRAM Port Config */
    EBI_PORT_SRAM           = 0x04,     /**< SRAM Port Config */
    EBI_PORT_LPC            = 0x08,     /**< Low Pin Count Port Config */
    EBI_PORT_CS0            = 0x10,     /**< Chip Select 0 Config */
    EBI_PORT_CS1            = 0x20,     /**< Chip Select 1 Config */
    EBI_PORT_CS2            = 0x40,     /**< Chip Select 2 Config */
    EBI_PORT_CS3            = 0x80,     /**< Chip Select 3 Config */
    EBI_PORT_CS_ALL         = 0xF0,     /**< Chip Select 0-3 Config */
} ebi_port_mask_t;

/**
 * @brief   SDRAM Column Address Strobe latency
 */
typedef enum {
    EBI_SDRAM_CAS_LAT_2CLK  = 0x00,     /**< 2 Clk PER2 cycles delay */
    EBI_SDRAM_CAS_LAT_3CLK  = 0x01,     /**< 3 Clk PER2 cycles delay */
} ebi_sdram_cas_latency_t;

/**
 * @brief   SDRAM number of Row Bits
 */
typedef enum {
    EBI_SDRAM_ROW_BITS_11  = 0x00,     /**< 11 row bits */
    EBI_SDRAM_ROW_BITS_12  = 0x01,     /**< 12 row bits */
} ebi_sdram_row_bits_t;

/**
 * @brief   EBI maximum Chip Select
 */
#define PERIPH_EBI_MAX_CS   (4)

/**
 * @brief   EBI SDRAM Chip Select
 */
#define PERIPH_EBI_SDRAM_CS (3)

/**
 * @brief   EBI Chip Select configuration structure
 */
typedef struct {
    EBI_CS_MODE_t   mode;               /**< Chip Select address mode */
#if defined (__AVR_ATxmega64A1U__) || defined (__AVR_ATxmega128A1U__)
    EBI_CS_ASPACE_t space;              /**< Chip Select address space */
#else
    EBI_CS_ASIZE_t  space;              /**< Chip Select address space */
#endif
    EBI_CS_SRWS_t   wait;               /**< SRAM Wait State Selection */
    uint32_t        address;            /**< Chip Select Base Address */
} ebi_cs_t;

/**
 * @brief   EBI SDRAM configuration structure
 */
typedef struct {
    uint8_t refresh;                     /**< Self-Refresh Enabled */
    uint16_t refresh_period;             /**< microseconds */
    uint16_t init_dly;                   /**< microseconds */
    EBI_CS_SDMODE_t mode;                /**< Access Mode */
    ebi_sdram_cas_latency_t cas_latency; /**< CAS Latency */
    ebi_sdram_row_bits_t row_bits;       /**< ROW bits */
    EBI_SDCOL_t column_bits;             /**< COLUMN bits */
    EBI_MRDLY_t ld_mode_dly;             /**< Number of Clk PER2 cycles */
    EBI_ROWCYCDLY_t row_cycle_dly;       /**< Number of Clk PER2 cycles */
    EBI_RPDLY_t row_prechage_dly;        /**< Number of Clk PER2 cycles */
    EBI_WRDLY_t write_recovery_dly;      /**< Number of Clk PER2 cycles */
    EBI_ESRDLY_t exit_self_rfsh_dly;     /**< Number of Clk PER2 cycles */
    EBI_ROWCOLDLY_t row_to_column_dly;   /**< Number of Clk PER2 cycles */
} ebi_sd_t;

/**
 * @brief   EBI configuration structure
 *
 * This is a mandatory configuration. Whowever, to complete disable module
 * just configure it as:
 *
 * static const ebi_conf_t ebi_config = { 0 };
 *
 * or, for a temporary disable, set addr_bits to 0 at periph_conf.h:
 *
 * .addr_bits = 0,
 *
 */
typedef struct {
    uint8_t addr_bits;                  /**< EBI port address lines */
    ebi_port_mask_t flags;              /**< EBI port flags */
    uint8_t sram_ale;                   /**< Number of ALE for SRAM mode */
    uint8_t lpc_ale;                    /**< Number of ALE for LPC mode */
    ebi_sd_t sdram;                     /**< SDRAM configuration */
    ebi_cs_t cs[PERIPH_EBI_MAX_CS];     /**< Chip Select configuration */
    uint16_t stack_size;                /**< Max 0xFFFF addressable value */
} ebi_conf_t;
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CPU_H */
/** @} */
