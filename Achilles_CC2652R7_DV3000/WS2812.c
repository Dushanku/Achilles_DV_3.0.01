/******************************************************************************
 * @file    WS2812.c
 *
 * So this implementation uses SPI configured at 3 x Neopixel Freq = 2.4MHz.
 * In this case a bit duration = 1/2.4MHz = 416ns. As 3 bits can be transferred in 1/NeopixelFreq
 * we can get a precise signal with :
 *   * 1 bit to 1 during 416ns and 2 other bits to 0. In this case a 1 bit is transferred to WS2812. We follow datasheet reqs
 *  350ns - 150ns < T0H = 416ns < 350ns + 150ns
 *  800ns - 150ns < T0L = 1250ns - T0H = 834 ns < 800ns + 150ns
 *
 *   * 2 bits to 1 during 833ns and 1 other bit to 0. In this case a 1 bit is transferred to WS2812. We do not follow datasheet reqs for T1L but
 *   after testing it works
 *  700ns - 150ns < T1H = 833ns < 700ns + 150ns
 *  600ns - 150ns < T1L = 1250ns - T1H = 417 ns < 600ns + 150ns
 *
 * 24 bit (GRB) color must be send for each neopixel => a buffer of 3x24bit = 72 bits = 9 bytes must be send to SPI for each neopixel.
 *
 * SPI SPH must be set in order to have back to back transfers like single word transfer.

 *****************************************************************************/

/**************************************************************************
 * Include Files
 **************************************************************************/
#include "WS2812.h"
#include <ti/drivers/GPIO.h>
/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>

/* Driver configuration */
#include "ti_drivers_config.h"


/**************************************************************************
 * Manifest Constants
 **************************************************************************/

//TODO : ADD limitation on NB_PIXELS
#ifndef NB_PIXELS
#define NB_PIXELS 3U
#endif

#define NB_SPI_BYTES_PER_PIXEL 9U

/** Get SPI value corresponding to a bit at index n in a grb color on 24 bits
 *  1 bit is 0b110
 *  0 bit is 0b100
 */
#define GRB_BIT_TO_SPI_BITS(val, bitPos) ((1 << bitPos & val) ? 0x06 : 0x04)

/**************************************************************************
 * Type Definitions
 **************************************************************************/

/**************************************************************************
 * Variables
 **************************************************************************/
/** buffer written to SPI */
static uint8_t _au8_spiLedBuffer[NB_SPI_BYTES_PER_PIXEL*NB_PIXELS] = {0};
static SPI_Handle      _spiHandle;


/**************************************************************************
 * Macros
 **************************************************************************/

/**************************************************************************
 * Local Functions Declarations
 **************************************************************************/

/**************************************************************************
 * Global Functions Defintions
 **************************************************************************/
void WS2812_begin(void)
{
    WS2812_beginSPI();
}
void clear()
{
    for (int i = 0; i < 28; i++) {
            _au8_spiLedBuffer[i] = 0;
        }
}

void WS2812_beginSPI()
{
    SPI_Params loc_spiParams;
    uint16_t loc_u16_pixelIndex;
    SPI_init();
    SPI_Params_init(&loc_spiParams);
    loc_spiParams.bitRate = 4000000;
    loc_spiParams.dataSize = 8;
    loc_spiParams.frameFormat = SPI_POL0_PHA1;
    _spiHandle = SPI_open(CONFIG_SPI_1, &loc_spiParams);
//    Task_sleep (500 * (1000 / Clock_tickPeriod));
//
//               WS2812_setPixelColor(0, 0, 0, 0);
//               WS2812_setPixelColor(1, 0, 0, 0);
//               WS2812_setPixelColor(2, 0, 0, 0);
//               WS2812_show();
}

void WS2812_close(void)
{
    SPI_close(_spiHandle);
}

bool WS2812_show(void)
{
    /** Make SPI transfer */
    SPI_Transaction spiTransaction;
    spiTransaction.count = sizeof(_au8_spiLedBuffer);
    spiTransaction.txBuf = _au8_spiLedBuffer;
    spiTransaction.rxBuf = NULL;

    return SPI_transfer(_spiHandle, &spiTransaction);

    for (int i = 0; i < 28; i++) {
        _au8_spiLedBuffer[i] = 0;
    }

}

void WS2812_setPin(uint8_t p)
{

}

/**
 * Slow function but few memory consumer, quicker implementation could be done mapping some grb bits to predefined
 * SPI data
 */
void WS2812_setPixelColor(uint16_t arg_u16_ledIndex, uint8_t arg_u8_red, uint8_t arg_u8_green, uint8_t arg_u8_blue)
{
    uint8_t loc_u8_currIndex = 3;

    /** Position of current led data in SPI buffer */
    uint16_t loc_u16_ledOffset = arg_u16_ledIndex*9;

    /** Concatenate color on a 32bit word */
    uint32_t loc_u32_grb = arg_u8_green << 16 | arg_u8_red << 8 | arg_u8_blue;

    /** Concatenate two bytes of SPI buffer in order to always transfer blocks of 3 bits
     * to SPI buffer corresponding to a single grb bit*/
    uint16_t loc_u16_currVal = 0;

    int8_t loc_u8_bitIndex;

    for(loc_u8_bitIndex = 23; loc_u8_bitIndex >= 0; loc_u8_bitIndex--)
    {
        loc_u16_currVal |= GRB_BIT_TO_SPI_BITS(loc_u32_grb, loc_u8_bitIndex) << (16 + 8*((loc_u8_currIndex - 3)/8) - loc_u8_currIndex);

        if((loc_u8_currIndex)/8 > (loc_u8_currIndex-3)/8) /** some bits have been written to byte at index 1 in  loc_u16_currVal*/
        {
            /** it's time to shift buffers */
            _au8_spiLedBuffer[loc_u16_ledOffset + loc_u8_currIndex /8 - 1] = loc_u16_currVal >> 8;
            loc_u16_currVal = loc_u16_currVal << 8;
        }
        loc_u8_currIndex += 3;
    }
}

/**************************************************************************
 * Local Functions Definitions
 **************************************************************************/
