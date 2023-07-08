/*
 *  ======== ti_drivers_config.h ========
 *  Configured TI-Drivers module declarations
 *
 *  The macros defines herein are intended for use by applications which
 *  directly include this header. These macros should NOT be hard coded or
 *  copied into library source code.
 *
 *  Symbols declared as const are intended for use with libraries.
 *  Library source code must extern the correct symbol--which is resolved
 *  when the application is linked.
 *
 *  DO NOT EDIT - This file is generated for the LP_CC2652R7
 *  by the SysConfig tool.
 */
#ifndef ti_drivers_config_h
#define ti_drivers_config_h

#define CONFIG_SYSCONFIG_PREVIEW

#define CONFIG_LP_CC2652R7
#ifndef DeviceFamily_CC26X2X7
#define DeviceFamily_CC26X2X7
#endif

#include <ti/devices/DeviceFamily.h>

#include <stdint.h>

/* support C++ sources */
#ifdef __cplusplus
extern "C" {
#endif


/*
 *  ======== CCFG ========
 */


/*
 *  ======== AESCCM ========
 */

extern const uint_least8_t                  CONFIG_AESCCM0_CONST;
#define CONFIG_AESCCM0                      0
#define CONFIG_TI_DRIVERS_AESCCM_COUNT      1


/*
 *  ======== AESCTR ========
 */

extern const uint_least8_t                  CONFIG_AESCTR_0_CONST;
#define CONFIG_AESCTR_0                     0
#define CONFIG_TI_DRIVERS_AESCTR_COUNT      1


/*
 *  ======== AESCTRDRBG ========
 */

extern const uint_least8_t                      CONFIG_AESCTRDRBG_0_CONST;
#define CONFIG_AESCTRDRBG_0                     0
#define CONFIG_TI_DRIVERS_AESCTRDRBG_COUNT      1


/*
 *  ======== AESECB ========
 */

extern const uint_least8_t                  CONFIG_AESECB0_CONST;
#define CONFIG_AESECB0                      0
#define CONFIG_TI_DRIVERS_AESECB_COUNT      1


/*
 *  ======== ECDH ========
 */

extern const uint_least8_t              CONFIG_ECDH0_CONST;
#define CONFIG_ECDH0                    0
#define CONFIG_TI_DRIVERS_ECDH_COUNT    1


/*
 *  ======== GPIO ========
 */
extern const uint_least8_t CONFIG_GPIO_BTN1_CONST;
#define CONFIG_GPIO_BTN1 13

extern const uint_least8_t CONFIG_GPIO_BTN2_CONST;
#define CONFIG_GPIO_BTN2 14

extern const uint_least8_t CONFIG_GPIO_RLED_CONST;
#define CONFIG_GPIO_RLED 6

extern const uint_least8_t CONFIG_GPIO_GLED_CONST;
#define CONFIG_GPIO_GLED 7

extern const uint_least8_t CONFIG_GPIO_WAKEUP_CONST;
#define CONFIG_GPIO_WAKEUP 11

extern const uint_least8_t CONFIG_GPIO_1_CONST;
#define CONFIG_GPIO_1 0

/* Owned by CONFIG_DISPLAY_UART as  */
extern const uint_least8_t CONFIG_GPIO_UART_TX_CONST;
#define CONFIG_GPIO_UART_TX 3

/* Owned by CONFIG_DISPLAY_UART as  */
extern const uint_least8_t CONFIG_GPIO_UART_RX_CONST;
#define CONFIG_GPIO_UART_RX 2

/* Owned by CONFIG_NVS_SPI_0 as  */
extern const uint_least8_t CONFIG_GPIO_0_CONST;
#define CONFIG_GPIO_0 20

/* Owned by CONFIG_SPI_0 as  */
extern const uint_least8_t CONFIG_GPIO_SPI_0_SCLK_CONST;
#define CONFIG_GPIO_SPI_0_SCLK 10

/* Owned by CONFIG_SPI_0 as  */
extern const uint_least8_t CONFIG_GPIO_SPI_0_MISO_CONST;
#define CONFIG_GPIO_SPI_0_MISO 8

/* Owned by CONFIG_SPI_0 as  */
extern const uint_least8_t CONFIG_GPIO_SPI_0_MOSI_CONST;
#define CONFIG_GPIO_SPI_0_MOSI 9

/* The range of pins available on this device */
extern const uint_least8_t GPIO_pinLowerBound;
extern const uint_least8_t GPIO_pinUpperBound;

/* LEDs are active high */
#define CONFIG_GPIO_LED_ON  (1)
#define CONFIG_GPIO_LED_OFF (0)

#define CONFIG_LED_ON  (CONFIG_GPIO_LED_ON)
#define CONFIG_LED_OFF (CONFIG_GPIO_LED_OFF)


/*
 *  ======== NVS ========
 */

extern const uint_least8_t              CONFIG_NVSINTERNAL_CONST;
#define CONFIG_NVSINTERNAL              0
/*
 *  MOSI: DIO9
 *  MISO: DIO8
 *  SCLK: DIO10
 *  LaunchPad SPI Bus
 *  SS: undefined
 */
extern const uint_least8_t              CONFIG_NVSEXTERNAL_CONST;
#define CONFIG_NVSEXTERNAL              1
#define CONFIG_TI_DRIVERS_NVS_COUNT     2




/*
 *  ======== SPI ========
 */

/*
 *  MOSI: DIO9
 *  MISO: DIO8
 *  SCLK: DIO10
 *  LaunchPad SPI Bus
 */
extern const uint_least8_t              CONFIG_SPI_0_CONST;
#define CONFIG_SPI_0                    0
#define CONFIG_TI_DRIVERS_SPI_COUNT     1


/*
 *  ======== TRNG ========
 */

extern const uint_least8_t              CONFIG_TRNG_0_CONST;
#define CONFIG_TRNG_0                   0
#define CONFIG_TI_DRIVERS_TRNG_COUNT    1


/*
 *  ======== UART2 ========
 */

/*
 *  TX: DIO3
 *  RX: DIO2
 *  XDS110 UART
 */
extern const uint_least8_t                  CONFIG_DISPLAY_UART_CONST;
#define CONFIG_DISPLAY_UART                 0
#define CONFIG_TI_DRIVERS_UART2_COUNT       1


/*
 *  ======== Board_init ========
 *  Perform all required TI-Drivers initialization
 *
 *  This function should be called once at a point before any use of
 *  TI-Drivers.
 */
extern void Board_init(void);

/*
 *  ======== Board_initGeneral ========
 *  (deprecated)
 *
 *  Board_initGeneral() is defined purely for backward compatibility.
 *
 *  All new code should use Board_init() to do any required TI-Drivers
 *  initialization _and_ use <Driver>_init() for only where specific drivers
 *  are explicitly referenced by the application.  <Driver>_init() functions
 *  are idempotent.
 */
#define Board_initGeneral Board_init

#ifdef __cplusplus
}
#endif

#endif /* include guard */
