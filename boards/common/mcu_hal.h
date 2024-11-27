/*!
 *   @file mcu_hal.h
 *   @brief mcu level hardware abstraction layer 
 *   @author Jeffrey Coffman <jeff.coffman@protonmail.com>
 *  
 *   Common mcu operations used to support ports of Lora Basics Modem API/LIB.
 *  
 */ 

#if !defined(MCU_HAL_H_)
#define MCU_HAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*!  @struct user_data_tm
 *   @brief wrapper for callbacks
 * 
 *   @var user_data_tm::cb
 *   callback funtion
 *   @var user_data_tm::context
 *   opaque context object
 */ 
struct user_data_tm {
    void (*cb)(void* context);
    void* context;
};

/*! Index for each Lora Context into a flash page 
 * 
 */
enum hal_flash_ctx {
    hal_flash_ctx_modem,
    hal_flash_ctx_key_modem,
    hal_flash_ctx_lora_stack,
    hal_flash_ctx_sc,
    hal_flash_ctx_fuota,
    hal_flash_ctx_str_fwd
};

/*!
 * @brief initialize lowlevel mcu hardware
 * 
 * Must be called first
 */ 
void mcu_hal_init();

/*!
 * @brief cleanup lowlevel mcu hardware
 */  
void mcu_hal_exit();

/*!
 * @brief  initialize the SPI interface
 */ 
void mcu_hal_spi_init();

/*!
 * @brief  initialize the I2C interface
 */ 
void mcu_hal_i2c_init();

/*!
 * @brief  Read from I2C device
 */ 
void mcu_hal_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint32_t len);

/*!
 * @brief  Write to I2C device
 */ 
void mcu_hal_i2c_write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *reg_data, uint32_t len);

/*!
 * @brief sleep/suspend execution for ms milliseconds
 * 
 * @param ms number of milliseconds to sleep for
 * 
 */ 
void mcu_hal_sleep_ms(uint32_t ms);

/*!
 * @brief get value for the gpio number
 * 
 * gpio numbers are mapped in the implementation
 * 
 * @param gpio the number of the gpio to query
 * 
 * @return int 0 if low, 1 if high, -1 if invalid gpio number
 */ 
int mcu_hal_gpio_get(unsigned int gpio);

/*!
 * @brief set the value for the gpio number
 * 
 *  gpio number and state to set the to
 * 
 *  @param unsigned int gpio the gpio number to set
 * 
 *  @param bool state, true for high, false for low
 * 
 *  @return 0 if successful, -1 if invalid gpio number
 */ 
int mcu_hal_gpio_set(unsigned int gpio, bool state);

/*!
 * @brief start a one shot high resolution timer
 * 
 * @param uint32_t milliseconds the deadline time
 * 
 * @param user_data_tm user_data wrapper around modem callback to be invoked 
 *        when timer expires
 */ 
void mcu_hal_start_timer(uint32_t milliseconds, struct user_data_tm user_data);

/*!
 * @brief stop a previously started one shot timer
 */ 
void mcu_hal_stop_timer();

/*!
 * @brief disable modem irq and timer irqs
 */ 
void mcu_hal_disable_irqs();

/*!
 * @brief enabled modem irq and timer irqs
 */ 
void mcu_hal_enable_irqs();

/*!
 * @brief registers callback with the radio irq
 */ 
void mcu_hal_config_radio_irq(void ( *callback )( void* context ), void* context);

/*!
 * @brief unregister callback from the radio irq
 */ 
void mcu_hal_clear_radio_irq(void);

/*!
 * @brief restore a saved context from persistent storage
 */ 
void mcu_hal_context_restore( const enum hal_flash_ctx ctx_type, uint32_t offset, uint8_t* buffer, const uint32_t size );

/*!
 * @brief store a context to persistent storage
 */ 
void mcu_hal_context_store( const enum hal_flash_ctx ctx_type, uint32_t offset, const uint8_t* buffer, const uint32_t size );

/*!
 * @brief erase a complete flash page
 */ 
void mcu_hal_erase_flash_page(uint32_t flash_offset, uint32_t nb_pages);

/*!
 * @brief  return a random number in a given range
 */ 
uint32_t mcu_hal_rng_get_random_in_range(const uint32_t val_1, const uint32_t val_2);

/*!
 * @brief read an adc channel 
 */ 
uint16_t mcu_hal_adc_read(unsigned int channel);

/*!
 * @brief read board temp
 */ 
int8_t mcu_hal_read_temp();

/*!
 * @brief read board battery voltage
 */ 
float mcu_hal_read_batt_voltage();

uint8_t mcu_hal_read_battery_level();

/*!
 * @brief get time stamp in milliseconds
 */ 
uint32_t mcu_hal_get_time_in_ms();

#ifdef __cplusplus
}
#endif

#endif