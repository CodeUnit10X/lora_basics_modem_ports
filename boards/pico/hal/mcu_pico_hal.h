#if !defined(MCU_PICO_HAL_H_)
#define MCU_PICO_HAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

constexpr uint8_t PICO_LORA_SX1262_PIN_UNUSED = 255;

#if defined(PICO_LORA)
constexpr uint8_t PICO_LORA_SX1262_PIN_CS = 3;
constexpr uint8_t PICO_LORA_SX1262_PIN_SPI1_MISO = 12;
constexpr uint8_t PICO_LORA_SX1262_PIN_SPI1_MOSI = 11;
constexpr uint8_t PICO_LORA_SX1262_PIN_SPI1_SCLK = 10;
constexpr uint8_t PICO_LORA_SX1262_PIN_BUSY = 2;
constexpr uint8_t PICO_LORA_SX1262_PIN_RESET = 15;
constexpr uint8_t PICO_LORA_SX1262_PIN_DIO1 = 20;
constexpr uint8_t PICO_LORA_SX1262_PIN_ANT_SW = PICO_LORA_SX1262_PIN_UNUSED;
constexpr uint8_t PICO_LORA_SX1262_PIN_ADC = 26;
#elif defined(RP2040_LORA)
//Just got the new LORA-RP3040  it's a bit different and wip
constexpr uint8_t PICO_LORA_SX1262_PIN_CS = 13;
constexpr uint8_t PICO_LORA_SX1262_PIN_SPI1_MISO = 24;
constexpr uint8_t PICO_LORA_SX1262_PIN_SPI1_MOSI = 15;
constexpr uint8_t PICO_LORA_SX1262_PIN_SPI1_SCLK = 14;
constexpr uint8_t PICO_LORA_SX1262_PIN_BUSY = 18;
constexpr uint8_t PICO_LORA_SX1262_PIN_RESET = 23;
constexpr uint8_t PICO_LORA_SX1262_PIN_DIO1 = 16;
constexpr uint8_t PICO_LORA_SX1262_PIN_ANT_SW = 17;
constexpr uint8_t PICO_LORA_SX1262_PIN_ADC = PICO_LORA_SX1262_PIN_UNUSED;
#else
    #error "You must define the type of LORA Pico board, either a RP2040-LORA or PICO-LORA"
#endif

#ifdef __cplusplus
}
#endif

#endif