#include "mcu_hal.h"
#include "mcu_pico_hal.h"

#include "pico/stdlib.h"
#include "string.h"
#include "stdio.h"
#include <math.h>

#include "hardware/structs/timer.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/spi.h"
#include "hardware/adc.h"
#include "pico/rand.h"

constexpr uint32_t CTX_MODEM_START          = PICO_FLASH_SIZE_BYTES - (FLASH_SECTOR_SIZE);
constexpr uint32_t CTX_KEY_MODEM_START      = PICO_FLASH_SIZE_BYTES - (FLASH_SECTOR_SIZE*2);
constexpr uint32_t CTX_LORAWAN_STACK_START  = PICO_FLASH_SIZE_BYTES - (FLASH_SECTOR_SIZE*3);
constexpr uint32_t CTX_SECURE_ELEMENT_START = PICO_FLASH_SIZE_BYTES - (FLASH_SECTOR_SIZE*4);
constexpr uint32_t CTX_FUOTA_STACK_START    = PICO_FLASH_SIZE_BYTES - (FLASH_SECTOR_SIZE*11);
constexpr uint32_t CTX_FOUTA_NUM_SECTORS    = 7;
constexpr uint32_t CTX_STORE_AND_FWD_START  = PICO_FLASH_SIZE_BYTES - (FLASH_SECTOR_SIZE*19);
constexpr uint32_t CTX_STORE_AND_FWD_NUM_SECTORS = 8;  

//measured accross voltage divider R1 = 200k, R2 = 100k, Vref 3.3v, 1<<12 ADC
constexpr float ADC_VOLT_CONV = .00241758f;
//derived from 1s lipo voltage-charge table
constexpr float LIPO_1S_COE_1 = 1.765289931f;
constexpr float LIPO_1S_COE_2 = 6.326972870f; 

/*
 *  We will use 4kb flash sector for each 
 * context in the modem api.  We could pack
 * things more effiecently and maintain an
 * in ram cache off the flash layout.  We
 * have ample flash space especially on the
 * pico 2.  For simplicity we, operate in this
 * mode.
 */
struct context_wr_cache {
    uint8_t ctx_modem[16];
    uint8_t ctx_key_modem[20];
    uint8_t ctx_sc[480];
    uint8_t ctx_lora_stack[32];
    uint8_t ctx_fuota[FLASH_SECTOR_SIZE * CTX_FOUTA_NUM_SECTORS];
    uint8_t ctx_str_fwd[FLASH_SECTOR_SIZE * CTX_STORE_AND_FWD_NUM_SECTORS]; 
};

constexpr unsigned int MAX_TIMERS = 4;

struct pico_hal_state {
    bool timer_started_;
    alarm_pool_t* pool_;
    uint alarm_num_;
    repeating_timer_t timer_;
    uint32_t timer_ms_;
    struct user_data_tm user_data_;
    struct context_wr_cache ctx_cache_;
    void (*radio_cb_)(void* context);
    void* radio_cb_ctx_;
    uint32_t radio_cb_events_;  
};

static struct pico_hal_state state_ = {false, nullptr, 0, {}, 0, {}, {}, nullptr, nullptr};


bool main_timer_callback(repeating_timer_t* rt) {
    //printf("repeating timer called\n");
    if(state_.user_data_.cb) {
        state_.user_data_.cb(state_.user_data_.context);
    }
}

void dio_gpio_callback(uint gpio, uint32_t events)
{
    if(state_.radio_cb_) {
        state_.radio_cb_(state_.radio_cb_ctx_);
        state_.radio_cb_events_ = events;
    }    
}

void dio_raw_gpio_callback(void) {
    if(gpio_get_irq_event_mask(PICO_LORA_SX1262_PIN_DIO1) & (GPIO_IRQ_EDGE_RISE)) { 
        gpio_acknowledge_irq(PICO_LORA_SX1262_PIN_DIO1, GPIO_IRQ_EDGE_RISE);
        if(state_.radio_cb_) {
            state_.radio_cb_(state_.radio_cb_ctx_);
            state_.radio_cb_events_ = gpio_get_irq_event_mask(PICO_LORA_SX1262_PIN_DIO1);
        }          
    }
}

void mcu_hal_context_restore(const enum hal_flash_ctx ctx_type, uint32_t offset, uint8_t* buffer, const uint32_t size ) {
    //read flash and update cache
    switch(ctx_type) {
        case hal_flash_ctx_modem:
        {
            char *p = (char *)XIP_BASE + CTX_MODEM_START;
            memcpy(buffer, p, size);
            break;
        }
        case hal_flash_ctx_key_modem:
        {
            char *p = (char *)XIP_BASE + CTX_KEY_MODEM_START;
            memcpy(buffer, p, size);            
            break;
        }
        case hal_flash_ctx_lora_stack:
        {
            char *p = (char *)XIP_BASE + CTX_LORAWAN_STACK_START + offset;
            memcpy(buffer, p, size);            
            break;
        }
        case hal_flash_ctx_sc:
        {
            char *p = (char *)XIP_BASE + CTX_SECURE_ELEMENT_START + offset;
            memcpy(buffer, p, size);              
            break;
        }
        case hal_flash_ctx_fuota:
        {
            char *p = (char *)XIP_BASE + CTX_FUOTA_STACK_START + offset;
            memcpy(buffer, p, size);              
            break;
        }
        case hal_flash_ctx_str_fwd:
        {
            char *p = (char *)XIP_BASE + CTX_STORE_AND_FWD_START + offset;
            memcpy(buffer, p, size);              
            break;
        }
        default:
            break;        
    };  
}

void mcu_hal_context_store(const enum hal_flash_ctx ctx_type, uint32_t offset, const uint8_t* buffer, const uint32_t size ) {
    uint32_t flash_offset = 0;
    uint8_t* cache_offset = nullptr;
    switch(ctx_type) {
        case hal_flash_ctx_modem:
        {
            memcpy(&state_.ctx_cache_.ctx_modem, buffer, size);
            flash_offset = CTX_MODEM_START;
            cache_offset = (uint8_t*)(&state_.ctx_cache_.ctx_modem);
            break;
        }
        case hal_flash_ctx_key_modem:
        {
            memcpy(&state_.ctx_cache_.ctx_key_modem, buffer, size);
            flash_offset = CTX_KEY_MODEM_START;
            cache_offset = (uint8_t*)(&state_.ctx_cache_.ctx_key_modem);
            break;
        }
        case hal_flash_ctx_lora_stack:
        {
            memcpy(&state_.ctx_cache_.ctx_lora_stack + offset, buffer, size);
            flash_offset = CTX_LORAWAN_STACK_START;
            cache_offset = (uint8_t*)(&state_.ctx_cache_.ctx_lora_stack);           
            break;
        }
        case hal_flash_ctx_sc:
        {
            memcpy(&state_.ctx_cache_.ctx_sc, buffer, size);
            flash_offset = CTX_SECURE_ELEMENT_START;
            cache_offset = (uint8_t*)(&state_.ctx_cache_.ctx_sc);
            break;
        }
        case hal_flash_ctx_fuota:
        {
            memcpy(&state_.ctx_cache_.ctx_fuota + offset, buffer, size);
            //compute the sector
            uint32_t page = offset / FLASH_SECTOR_SIZE;
            flash_offset = CTX_FUOTA_STACK_START + page;
            cache_offset = (uint8_t*)(&state_.ctx_cache_.ctx_fuota);                            
            break;
        }
        case hal_flash_ctx_str_fwd:
        {
            memcpy(&state_.ctx_cache_.ctx_str_fwd + offset, buffer, size);
            //compute the sector
            uint32_t page = offset / FLASH_SECTOR_SIZE;
            flash_offset = CTX_STORE_AND_FWD_START + page;
            cache_offset = (uint8_t*)(&state_.ctx_cache_.ctx_str_fwd);                             
            break;
        }
        default:
            break;        
    };

    //update flash
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(flash_offset, 1);
    flash_range_program (flash_offset, cache_offset, size);
    restore_interrupts (ints);
}

void mcu_hal_erase_flash_page(uint32_t flash_offset, uint32_t nb_pages) {
    uint32_t ints = save_and_disable_interrupts();
    uint32_t page = flash_offset / FLASH_SECTOR_SIZE;
    flash_range_erase(CTX_STORE_AND_FWD_START + page, nb_pages);
    restore_interrupts (ints);    
}

void mcu_hal_init() {

    stdio_init_all();

    gpio_init(PICO_LORA_SX1262_PIN_BUSY);
    gpio_set_dir(PICO_LORA_SX1262_PIN_BUSY, GPIO_IN);
    gpio_disable_pulls(PICO_LORA_SX1262_PIN_BUSY);

    //setup the dio callback, leave it disabled until the api sets the callback
    gpio_init(PICO_LORA_SX1262_PIN_DIO1);
    gpio_set_dir(PICO_LORA_SX1262_PIN_DIO1, GPIO_IN);
    gpio_disable_pulls(PICO_LORA_SX1262_PIN_DIO1);    
    gpio_set_irq_enabled_with_callback(PICO_LORA_SX1262_PIN_DIO1, GPIO_IRQ_EDGE_RISE, true, &dio_gpio_callback);


    //get an unused hardware alarm for our timers
    state_.pool_ = alarm_pool_create(2, 16);
    state_.alarm_num_ = alarm_pool_hardware_alarm_num(state_.pool_);

    mcu_hal_spi_init();     

    //ADC for battery voltage
    adc_init();
    adc_gpio_init(PICO_LORA_SX1262_PIN_ADC);

    adc_set_temp_sensor_enabled(true);

    //reset the radio
    gpio_init(PICO_LORA_SX1262_PIN_RESET);
    gpio_set_dir(PICO_LORA_SX1262_PIN_RESET, GPIO_OUT);
    gpio_disable_pulls(PICO_LORA_SX1262_PIN_RESET);
    gpio_put(PICO_LORA_SX1262_PIN_RESET, 0);
}

uint32_t mcu_hal_get_time_in_ms() {
    absolute_time_t t = get_absolute_time();
    return to_ms_since_boot(t);	    
}

void mcu_hal_sleep_ms(uint32_t ms) {
    sleep_ms(ms);
}

int mcu_hal_gpio_get(unsigned int gpio) {
    return gpio_get(gpio);
}

int mcu_hal_gpio_set(unsigned int gpio, bool state) {
    gpio_put(gpio, state);
    return 0;
}

//we only have one timer/alarm
static int64_t alarm_callback(alarm_id_t id, void *user_data) {
    if(state_.user_data_.cb) {
        state_.user_data_.cb(state_.user_data_.context);
    }
    return 0;
}


void mcu_hal_start_timer(uint32_t milliseconds, struct user_data_tm user_data) {
    state_.user_data_ = user_data;
    state_.timer_ms_ = milliseconds;
    if(!state_.timer_started_) {
        state_.timer_started_ = true;        
    } else {
        alarm_pool_cancel_alarm(state_.pool_, state_.alarm_num_);
    }
    
    state_.alarm_num_ = alarm_pool_add_alarm_at(state_.pool_, delayed_by_ms(get_absolute_time(), milliseconds), alarm_callback, NULL, true); 
}

void mcu_hal_stop_timer() {
    if(state_.timer_started_) {
        alarm_pool_cancel_alarm(state_.pool_, state_.alarm_num_);
        state_.timer_started_ = false;
    }
}

/**
 *   Disable irqs used by the mode, timer/alarm and dio
 * 
 */ 
void mcu_hal_disable_irqs() {
    //disable the alarm used as a repeating timer
    hw_clear_bits(&timer_hw->inte, 1u << state_.alarm_num_); 
    irq_set_enabled(IO_IRQ_BANK0, false);
}

void mcu_hal_enable_irqs() {
    hw_set_bits(&timer_hw->inte, 1u <<  state_.alarm_num_);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

void mcu_hal_spi_init() {
	//spi 1 defaults
    spi_init(spi1, 10 * 1000 * 1000);
    spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PICO_LORA_SX1262_PIN_SPI1_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PICO_LORA_SX1262_PIN_SPI1_SCLK, GPIO_FUNC_SPI);
    gpio_set_function(PICO_LORA_SX1262_PIN_SPI1_MOSI, GPIO_FUNC_SPI);

    //enable the GP3 for chipselect
    gpio_init(PICO_LORA_SX1262_PIN_CS);
    gpio_set_dir(PICO_LORA_SX1262_PIN_CS, GPIO_OUT);
    gpio_put(PICO_LORA_SX1262_PIN_CS, 1);    
}

uint32_t mcu_hal_rng_get_random_in_range(const uint32_t val_1, const uint32_t val_2) {
    if( val_1 <= val_2 ) {
        return ( uint32_t )( ( get_rand_32() % ( val_2 - val_1 + 1 ) ) + val_1 );
    } else {
        return ( uint32_t )( ( get_rand_32() % ( val_1 - val_2 + 1 ) ) + val_2 );
    }    
}

uint16_t mcu_hal_adc_read(unsigned int channel) {
    adc_select_input(channel);
    return adc_read();
}

int8_t mcu_hal_read_temp() {
    adc_select_input(ADC_TEMPERATURE_CHANNEL_NUM);
    uint16_t v_raw = adc_read() * (3.3 / (1<<12));
    float temp_celcius = 27 - (v_raw - 0.706) / 0.001721;
    return (int8_t)temp_celcius;
}

uint16_t mcu_hal_read_batt_voltage() {
    adc_select_input(0);
    float v_raw = adc_read() * ADC_VOLT_CONV;
    uint16_t capacity = floorf((LIPO_1S_COE_1 * v_raw) - LIPO_1S_COE_2) * 254;
    return capacity;    
}

void mcu_hal_config_radio_irq(void ( *callback )( void* context ), void* context) {
    //disable interrupts in case we're changing handle
    irq_set_enabled(IO_IRQ_BANK0, false);
    state_.radio_cb_ = callback;
    state_.radio_cb_ctx_ = context;
    gpio_set_irq_enabled(PICO_LORA_SX1262_PIN_DIO1, GPIO_IRQ_EDGE_RISE, true);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

void mcu_hal_clear_radio_irq(void) {
    //This is handled by pico sdk
}
