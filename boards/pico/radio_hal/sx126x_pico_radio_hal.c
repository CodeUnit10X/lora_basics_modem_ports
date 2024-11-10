
#include "sx126x_hal.h"
#include "hal.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/structs/scb.h"
#include "hardware/gpio.h"
#include <stdlib.h>
#include <stdio.h>

typedef enum
{
    RADIO_SLEEP,
    RADIO_AWAKE
} radio_sleep_mode_t;

static radio_sleep_mode_t radio_mode = RADIO_AWAKE;

static void sx126x_hal_check_device_ready(void);
static void sx126x_hal_wait_on_busy(void);

sx126x_hal_status_t sx126x_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length ) {
    //pull nss low
    gpio_put(PICO_LORA_SX1262_PIN_CS, 0); 
     
    //send command
    uint8_t* data_in = malloc(sizeof(uint8_t)*(command_length+data_length));
    spi_write_read_blocking(spi1, command, data_in, command_length);

    spi_write_read_blocking(spi1, data, data_in, data_length);

    //pull nss high
    gpio_put(PICO_LORA_SX1262_PIN_CS, 1); 

    free(data_in);

    // 0x84 - SX126x_SET_SLEEP opcode. In sleep mode the radio dio is struck to 1 => do not test it
    if( command[0] != 0x84 ) {
        sx126x_hal_check_device_ready();
    } else {
        radio_mode = RADIO_SLEEP;
    }

    return SX126X_HAL_STATUS_OK;    
}

sx126x_hal_status_t sx126x_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length ) {

    sx126x_hal_check_device_ready();

    //pull nss low
    gpio_put(PICO_LORA_SX1262_PIN_CS, 0); 

    //send command
    uint8_t* data_out = malloc(sizeof(uint8_t) * (command_length + data_length));
    spi_write_read_blocking(spi1, command, data_out, command_length);

    spi_write_read_blocking(spi1, data_out, data, data_length);

    //pull nss high
    gpio_put(PICO_LORA_SX1262_PIN_CS, 1); 

    sx126x_hal_check_device_ready();

    free(data_out);

    return SX126X_HAL_STATUS_OK;  
}

sx126x_hal_status_t sx126x_hal_reset( const void* context ) {
    hal_gpio_set(PICO_LORA_SX1262_PIN_RESET, false);
    sleep_us(10000);
    hal_gpio_set(PICO_LORA_SX1262_PIN_RESET, true);    
    sleep_us(10000);
    radio_mode = RADIO_AWAKE;
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup( const void* context ) {
    sx126x_hal_check_device_ready();
    
    return SX126X_HAL_STATUS_OK;
}


static void sx126x_hal_wait_on_busy( void )
{
    while(hal_gpio_get(PICO_LORA_SX1262_PIN_BUSY) == true)
    {
    };
}

/*
*  Lower CS(NSS) to bring the device out of sleep mode
*/
static void sx126x_hal_check_device_ready( void )
{
    if( radio_mode != RADIO_SLEEP ) {
        sx126x_hal_wait_on_busy();
    } else {
        hal_gpio_set(PICO_LORA_SX1262_PIN_CS, false);
        sx126x_hal_wait_on_busy( );
        hal_gpio_set(PICO_LORA_SX1262_PIN_CS, true);
        radio_mode = RADIO_AWAKE;
    }
}