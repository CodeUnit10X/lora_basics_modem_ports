#if !defined(PICO_HAL_H_)
#define PICO_HAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define PICO_LORA_SX1262_PIN_CS 3
#define PICO_LORA_SX1262_PIN_SPI1_MISO 12
#define PICO_LORA_SX1262_PIN_SPI1_MOSI 11
#define PICO_LORA_SX1262_PIN_SPI1_SCLK 10
#define PICO_LORA_SX1262_PIN_BUSY  2
#define PICO_LORA_SX1262_PIN_RESET 15
#define PICO_LORA_SX1262_PIN_DIO1  20
#define PICO_LORA_SX1262_PIN_ADC 26

struct user_data_tm {
    void (*cb)(void* context);
    void* context;
};

enum pico_hal_flash_ctx {
    pico_hal_flash_ctx_modem,
    pico_hal_flash_ctx_key_modem,
    pico_hal_flash_ctx_lora_stack,
    pico_hal_flash_ctx_sc,
    pico_hal_flash_ctx_fuota,
    pico_hal_flash_ctx_str_fwd
};

void hal_init();

void hal_spi_init();

void hal_sleep_ms(uint32_t ms);

bool hal_gpio_get(unsigned int gpio);

void hal_gpio_set(unsigned int gpio, bool state);

void hal_start_timer(uint32_t milliseconds, struct user_data_tm user_data);

void hal_stop_timer();

void hal_disable_irqs();

void hal_enable_irqs();

void hal_config_radio_irq(void ( *callback )( void* context ), void* context);

void hal_clear_radio_irq(void);

void hal_context_restore( const enum pico_hal_flash_ctx ctx_type, uint32_t offset, uint8_t* buffer, const uint32_t size );

void hal_context_store( const enum pico_hal_flash_ctx ctx_type, uint32_t offset, const uint8_t* buffer, const uint32_t size );

void hal_erase_flash_page(uint32_t flash_offset, uint32_t nb_pages);

uint32_t hal_rng_get_random_in_range(const uint32_t val_1, const uint32_t val_2);

uint16_t hal_adc_read(unsigned int channel);

int8_t hal_read_temp();

uint16_t hal_read_batt_voltage();

uint32_t hal_get_time_in_ms();

#ifdef __cplusplus
}
#endif

#endif