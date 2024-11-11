#if !defined(MCU_PICO_HAL_H_)
#define MCU_PICO_HAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

constexpr uint8_t PICO_LORA_SX1262_PIN_CS = 3;
constexpr uint8_t PICO_LORA_SX1262_PIN_SPI1_MISO = 12;
constexpr uint8_t PICO_LORA_SX1262_PIN_SPI1_MOSI = 11;
constexpr uint8_t PICO_LORA_SX1262_PIN_SPI1_SCLK = 10;
constexpr uint8_t PICO_LORA_SX1262_PIN_BUSY = 2;
constexpr uint8_t PICO_LORA_SX1262_PIN_RESET = 15;
constexpr uint8_t PICO_LORA_SX1262_PIN_DIO1 = 20;
constexpr uint8_t PICO_LORA_SX1262_PIN_ADC = 26;

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


void mcu_hal_context_restore( const enum pico_hal_flash_ctx ctx_type, uint32_t offset, uint8_t* buffer, const uint32_t size );

void mcu_hal_context_store( const enum pico_hal_flash_ctx ctx_type, uint32_t offset, const uint8_t* buffer, const uint32_t size );

void mcu_hal_erase_flash_page(uint32_t flash_offset, uint32_t nb_pages);

uint16_t mcu_hal_adc_read(unsigned int channel);

int8_t mcu_hal_read_temp();

uint16_t mcu_hal_read_batt_voltage();

#ifdef __cplusplus
}
#endif

#endif