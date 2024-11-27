#include "smtc_modem_hal.h"

#include "mcu_hal.h"
#include "mcu_pico_hal.h"

#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "hardware/structs/scb.h"
#include "hardware/gpio.h"
#include "hardware/flash.h"

#include <stdio.h>

#define MS_TO_SECOND 1000

void smtc_modem_hal_reset_mcu( void ) {
    scb_hw->aircr = 0x5FA0004;
}

void smtc_modem_hal_reload_wdog( void ) {
	//watchdog_update();
}

uint32_t smtc_modem_hal_get_time_in_s( void ) {
    return mcu_hal_get_time_in_ms()/MS_TO_SECOND;	
}

uint32_t smtc_modem_hal_get_time_in_ms( void ) {
    return mcu_hal_get_time_in_ms();
}

void smtc_modem_hal_set_offset_to_test_wrapping( const uint32_t offset_to_test_wrapping ) {
    return;
}

void smtc_modem_hal_start_timer( const uint32_t milliseconds, void ( *callback )( void* context ), void* context ) {
    struct user_data_tm user_data = {callback, context};
    mcu_hal_start_timer(milliseconds, user_data); 
}

void smtc_modem_hal_stop_timer( void ) {
    mcu_hal_stop_timer();
}

void smtc_modem_hal_disable_modem_irq( void ) {
    mcu_hal_disable_irqs();
}

void smtc_modem_hal_enable_modem_irq( void ) {
    mcu_hal_enable_irqs();
}

void smtc_modem_hal_context_restore( const modem_context_type_t ctx_type, uint32_t offset, uint8_t* buffer,
                                     const uint32_t size ) {
    switch(ctx_type) {
    case CONTEXT_MODEM:
        mcu_hal_context_restore(hal_flash_ctx_modem, offset, buffer, size);        
        break;
    case CONTEXT_KEY_MODEM:
        mcu_hal_context_restore(hal_flash_ctx_key_modem, offset, buffer, size);           
        break;
    case CONTEXT_LORAWAN_STACK:
        mcu_hal_context_restore(hal_flash_ctx_lora_stack, offset, buffer, size);           
        break;
    case CONTEXT_FUOTA:
        mcu_hal_context_restore(hal_flash_ctx_fuota, offset, buffer, size);           
        break;
    case CONTEXT_SECURE_ELEMENT:
        mcu_hal_context_restore(hal_flash_ctx_sc, offset, buffer, size);           
        break;
    case CONTEXT_STORE_AND_FORWARD:
        mcu_hal_context_restore(hal_flash_ctx_str_fwd, offset, buffer, size);           
        break;
    default:
        break;
    };
}

void smtc_modem_hal_context_store( const modem_context_type_t ctx_type, uint32_t offset, const uint8_t* buffer,
                                   const uint32_t size ) {
    switch(ctx_type) {
    case CONTEXT_MODEM:
        mcu_hal_context_store(hal_flash_ctx_modem, offset, buffer, size);
        break;
    case CONTEXT_KEY_MODEM:
        mcu_hal_context_store(hal_flash_ctx_key_modem, offset, buffer, size);        
        break;
    case CONTEXT_LORAWAN_STACK:
        mcu_hal_context_store(hal_flash_ctx_lora_stack, offset, buffer, size);   
        break;
    case CONTEXT_FUOTA:
        mcu_hal_context_store(hal_flash_ctx_fuota, offset, buffer, size);           
        break;
    case CONTEXT_SECURE_ELEMENT:
        mcu_hal_context_store(hal_flash_ctx_sc, offset, buffer, size);           
        break;
    case CONTEXT_STORE_AND_FORWARD:
        mcu_hal_context_store(hal_flash_ctx_str_fwd, offset, buffer, size);           
        break;
    default:
        break;
    };    
}

void smtc_modem_hal_context_flash_pages_erase( const modem_context_type_t ctx_type, uint32_t offset, uint8_t nb_page ) {
    switch( ctx_type )
    {
    case CONTEXT_STORE_AND_FORWARD:
        mcu_hal_erase_flash_page(offset, nb_page); 
        break;
    default:
        break;
    };    
    
}

void smtc_modem_hal_on_panic( uint8_t* func, uint32_t line, const char* fmt, ... ) {
    //crash log
    smtc_modem_hal_reset_mcu();
}

uint32_t smtc_modem_hal_get_random_nb_in_range( const uint32_t val_1, const uint32_t val_2 ) {
    return mcu_hal_rng_get_random_in_range(val_1, val_2);
}

/*
  Not used on sharewave 1262
*/
void smtc_modem_hal_start_radio_tcxo( void ) {
    return;
}

/*
  Not used on sharewave 1262
*/
void smtc_modem_hal_stop_radio_tcxo( void ) {
    return;
}

/*
  Pico Lora Hat versions have TCXO
*/
uint32_t smtc_modem_hal_get_radio_tcxo_startup_delay_ms( void ) {
#if defined(RP2040_LORA)
    return 0;
#else
    return 5;
#endif    
}

/*
    The Waveshare LORA-2040 has an antenna swtich
*/
void smtc_modem_hal_set_ant_switch( bool is_tx_on ) {
#if defined(RP2040_LORA)
    mcu_hal_gpio_set(PICO_LORA_SX1262_PIN_ANT_SW, !is_tx_on);
#endif
}

uint8_t smtc_modem_hal_get_battery_level( void ) {
    return mcu_hal_read_battery_level();
}

int8_t smtc_modem_hal_get_board_delay_ms( void ) {
    //This can be fine tuned see section 8 of porting guide
    return 5;
}

void smtc_modem_hal_print_trace( const char* fmt, ... ) {
    va_list args;
    va_start( args, fmt );
    vprintf( fmt, args );
    va_end( args );
}

#if defined(USE_FUOTA)

#error FUOTA is not supported for this platform

uint32_t smtc_modem_hal_get_hw_version_for_fuota( void ) {
    return 0xDEADBEEF;
}

uint32_t smtc_modem_hal_get_fw_version_for_fuota( void ) {
    return 0xBEEFBADD;
}

uint8_t smtc_modem_hal_get_fw_status_available_for_fuota( void ) {
    return 0x00;
}

uint8_t smtc_modem_hal_get_fw_delete_status_for_fuota( uint32_t fw_to_delete_version ) {
    return 0x00;
}

uint32_t smtc_modem_hal_get_next_fw_version_for_fuota( void ) {
    return 0xFEEDBEEF;
}

#endif

int8_t smtc_modem_hal_get_temperature( void ) {
    return mcu_hal_read_temp(); 
}

uint16_t smtc_modem_hal_get_voltage_mv( void ) {
    return mcu_hal_read_batt_voltage()*1000;
}

void smtc_modem_hal_crashlog_store( const uint8_t* crash_string, uint8_t crash_string_length ) {
    //TBD
    return;
}

void smtc_modem_hal_crashlog_restore( uint8_t* crash_string, uint8_t* crash_string_length ) {
    //TBD
    return;
}

void smtc_modem_hal_crashlog_set_status( bool available ) {
    //TBD
    return;
}

bool smtc_modem_hal_crashlog_get_status( void ) {
    //TBD
    return false;
}

uint16_t smtc_modem_hal_store_and_forward_get_number_of_pages( void ) {
    return (FLASH_SECTOR_SIZE*8) / FLASH_PAGE_SIZE;
}

uint16_t smtc_modem_hal_flash_get_page_size( void ) {
    return FLASH_PAGE_SIZE;
}

void smtc_modem_hal_user_lbm_irq( void ) {
    //used in rtos ports
    return;
}

void smtc_modem_hal_irq_config_radio_irq( void ( *callback )( void* context ), void* context ) {
    mcu_hal_config_radio_irq(callback, context);
}

void smtc_modem_hal_radio_irq_clear_pending( void ) {
    mcu_hal_clear_radio_irq();
}