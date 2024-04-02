
/**
 * @file SDCardSoure.cpp
 * @author Leon Farchau (leon2225)
 * @brief 
 * @version 0.1
 * @date 03.01.2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

/************************************************************
 *  INCLUDES
 ************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "hardware/gpio.h"
#include "debug_pins.h"

// For SD Card
#define FF_FS_READONLY 1
#include "ff.h"
#include "diskio.h"
#include "hw_config.h"
#include "f_util.h"
#include "sd_card.h"
//
#include "diskio.h" /* Declarations of disk functions */


#include "specificRegisters.h"
#include "comInterface.h"

/************************************************************
 *  DEFINES
 ************************************************************/
#define I2C_ADDR 0x28
#define SDA_PIN 26
#define SCL_PIN 27
#define I2C_UNIT i2c1

#define SAMPLES_PER_CHANNEL 15
#define CHANNEL_NUMBER 6
#define SAMPLE_SIZE 4
#define READ_SIZE ( SAMPLES_PER_CHANNEL * CHANNEL_NUMBER * SAMPLE_SIZE)


/************************************************************
 * Type definitions
 * **********************************************************/


/************************************************************
 * Variables
 * **********************************************************/
DeviceSpecificConfiguration_t* g_config;
/* SDIO Interface */
static sd_sdio_if_t sdio_if = {
    /*
    Pins CLK_gpio, D1_gpio, D2_gpio, and D3_gpio are at offsets from pin D0_gpio.
    The offsets are determined by sd_driver\SDIO\rp2040_sdio.pio.
        CLK_gpio = (D0_gpio + SDIO_CLK_PIN_D0_OFFSET) % 32;
        As of this writing, SDIO_CLK_PIN_D0_OFFSET is 18,
            which is -14 in mod32 arithmetic, so:
        CLK_gpio = D0_gpio -14.
        D1_gpio = D0_gpio + 1;
        D2_gpio = D0_gpio + 2;
        D3_gpio = D0_gpio + 3;
    */
    .CMD_gpio = 18,
    .D0_gpio = 19,
    .DMA_IRQ_num = DMA_IRQ_1,
    .use_exclusive_DMA_IRQ_handler = true,
    .baud_rate = 15 * 1000 * 1000  // 15 MHz
};

/* Hardware Configuration of the SD Card socket "object" */
static sd_card_t sd_card = {
    .type = SD_IF_SDIO,
    .sdio_if_p = &sdio_if
};

volatile bool g_sync = 0;
volatile bool g_cardAvailable = 0;
FIL g_file;
uint32_t data[READ_SIZE / 4];

/************************************************************
 * Function prototypes
 * **********************************************************/
void setup();
void asyncLoop();
void dataCallback(void* data, uint32_t length);
void configCallback(DeviceSpecificConfiguration_t* config, DeviceSpecificConfiguration_t* oldConfig);
size_t sd_get_num();
sd_card_t *sd_get_by_num(size_t num);
void sync_callback();

/************************************************************
 * Functions
 * **********************************************************/

int main()
{
    setup();
    // See FatFs - Generic FAT Filesystem Module, "Application Interface",
    // http://elm-chan.org/fsw/ff/00index_e.html
    FATFS fs;
    const char* const filename = "binaryfile2.bin";

    FRESULT fr = f_mount(&fs, "", 1);
    if (FR_OK == fr)
    {
        fr = f_open(&g_file, filename, FA_READ);
        if (FR_OK != fr && FR_EXIST != fr)
        {
            panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
        }
        g_cardAvailable = 1;
    }

    while (true)
    {
        asyncLoop();
    }

    printf("done\n");

    return 0;
}

void setup()
{
    stdio_init_all();

    printf("Start SD-Card Source\n");

    gpio_init(DEBUG_PIN1);
    gpio_init(DEBUG_PIN2);
    gpio_init(DEBUG_PIN3);
    gpio_set_dir(DEBUG_PIN1, GPIO_OUT);
    gpio_set_dir(DEBUG_PIN2, GPIO_OUT);
    gpio_set_dir(DEBUG_PIN3, GPIO_OUT);
    gpio_put(DEBUG_PIN1, 0);
    gpio_put(DEBUG_PIN2, 0);
    gpio_put(DEBUG_PIN3, 0);

    cominterfaceConfiguration config;
    config.g_i2c = I2C_UNIT;
    config.g_i2cAddr = I2C_ADDR;
    config.g_sdaPin = SDA_PIN;
    config.g_sclPin = SCL_PIN;
    config.HOut_Callback = dataCallback;
    config.UpdateConfig_Callback = configCallback;
    config.sync_callback = sync_callback;

    comInterfaceInit(&config);
}

void asyncLoop()
{    
    static int minValue = 0;
    static int maxValue = 0;
    if(g_sync)
    {
        UINT bytesRead = 0;

        if (g_cardAvailable)
        {
            if (f_read(&g_file, data, READ_SIZE, &bytesRead) < 0) {
                printf("f_printf failed\n");
            }

            if(READ_SIZE != bytesRead)
            {
                // Set file pointer to beginning
                f_lseek(&g_file, 0);

                // Read the rest of the data
                uint32_t bytesLeft = READ_SIZE - bytesRead;
                if (f_read(&g_file, data + bytesRead / 4, bytesLeft, &bytesRead) < 0) {
                    printf("f_printf failed\n");
                }

                // now all data should be read!
                assert(bytesLeft == bytesRead);
            }
        }

        for (size_t sample = 0; sample < SAMPLES_PER_CHANNEL; sample++)
        {
            for (size_t channel = 0; channel < CHANNEL_NUMBER; channel++)
            {
                int32_t value = data[sample * CHANNEL_NUMBER + channel];
                
                if (value < minValue)
                {
                    minValue = value;
                }
                if (value > maxValue)
                {
                    maxValue = value;
                }

                comInterfaceAddSample(&value, channel);
            }
            
        }
        g_sync = 0;
    }
}

void dataCallback(void* data, uint32_t length)
{
    // not implemented
}

void sync_callback()
{
    g_sync = 1;
}

void configCallback(DeviceSpecificConfiguration_t* config, DeviceSpecificConfiguration_t* oldConfig)
{
    g_config = config;
}

// SD Card callbacks
size_t sd_get_num() { return 1; }

sd_card_t *sd_get_by_num(size_t num) {
    if (0 == num)
        return &sd_card;
    else
        return NULL;
}