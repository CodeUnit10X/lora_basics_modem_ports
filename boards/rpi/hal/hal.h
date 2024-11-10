#if !defined(LINUX_HAL_H_)
#define LINUX_HAL_H_

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

uint32_t hal_rng_get_random_in_range(const uint32_t val_1, const uint32_t val_2);

uint16_t hal_adc_read(unsigned int channel);

uint32_t hal_get_time_in_ms();

#ifdef __cplusplus
}
#endif

#endif