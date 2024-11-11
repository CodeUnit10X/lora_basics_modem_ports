#if !defined(MCU_HAL_H_)
#define MCU_HAL_H_

#ifdef __cplusplus
extern "C" {
#include <cstdint>
#else
#include <stdint.h>
#endif


struct user_data_tm {
    void (*cb)(void* context);
    void* context;
};

void mcu_hal_init();

void mcu_hal_spi_init();

void mcu_hal_sleep_ms(uint32_t ms);

bool mcu_hal_gpio_get(unsigned int gpio);

void mcu_hal_gpio_set(unsigned int gpio, bool state);

void mcu_hal_start_timer(uint32_t milliseconds, struct user_data_tm user_data);

void mcu_hal_stop_timer();

void mcu_hal_disable_irqs();

void mcu_hal_enable_irqs();

void mcu_hal_config_radio_irq(void ( *callback )( void* context ), void* context);

void mcu_hal_clear_radio_irq(void);

uint32_t mcu_hal_rng_get_random_in_range(const uint32_t val_1, const uint32_t val_2);

uint16_t mcu_hal_adc_read(unsigned int channel);

uint32_t mcu_hal_get_time_in_ms();

#ifdef __cplusplus
}
#endif

#endif