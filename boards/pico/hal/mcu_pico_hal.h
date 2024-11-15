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

#ifdef __cplusplus
}
#endif

#endif