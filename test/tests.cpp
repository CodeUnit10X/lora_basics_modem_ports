#include "tests.h"

#include <cstddef>
#include <format>
#include <iostream>
#include <print>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "smtc_modem_utilities.h"
#include "smtc_modem_api.h"
#include "smtc_modem_test_api.h"

#include "sx126x.h"

#include "hal.h"

#define NB_LOOP_TEST_SPI 2

#define FREQ_NO_RADIO 800000000
#define FREQ_US_RADIO 915000000
#define SYNC_WORD_NO_RADIO 0x21
#define MARGIN_GET_TIME_IN_MS 1

constexpr uint8_t CONTEXT_MODEM_DATA[16] = { 0xAA, 0XDE, 0xAD, 0xBE,\
                                             0xEF, 0xBA, 0xAD, 0XCC,\
                                             0xFE, 0xFE, 0XAA, 0xBB,\
                                             0xDF, 0x01, 0x02, 0x3 };

constexpr uint8_t CONTEXT_KEY_MODEM_DATA[20] = { 0xBB, 0XED, 0xDA, 0xBC,\
                                                 0xEC, 0xFA, 0xEE, 0XCC,\
                                                 0xFE, 0xFE, 0XAA, 0xBB,\
                                                 0x12, 0x56, 0x12, 0xAC,\
                                                 0xDF, 0x01, 0x02, 0xAA};

constexpr uint8_t CONTEXT_LORAWAN_STACK_DATA[32] = { 0xCC, 0XED, 0xDA, 0xBC,\
                                                     0xEC, 0xFA, 0xEE, 0XCC,\
                                                     0xFE, 0xFE, 0XAA, 0xBB,\
                                                     0x12, 0x56, 0x12, 0xAC,\
                                                     0xDF, 0x01, 0x02, 0xAA,\ 
                                                     0xFE, 0xFE, 0XAA, 0xBB,\
                                                     0x12, 0x56, 0x12, 0xAC,\
                                                     0xDF, 0x01, 0xDD, 0xEE};

// LoRa configurations TO NOT receive or transmit

static ralf_params_lora_t rx_lora_param = { {RAL_LORA_SF12, RAL_LORA_BW_125_KHZ, RAL_LORA_CR_4_5, 0},
                                            {8, RAL_LORA_PKT_EXPLICIT, 255, false, true},
                                            FREQ_NO_RADIO,
                                            0, 
                                            SYNC_WORD_NO_RADIO,
                                            0 };

static ralf_params_lora_t tx_lora_param = { {RAL_LORA_SF12, RAL_LORA_BW_125_KHZ, RAL_LORA_CR_4_5, 0},
                                            {8, RAL_LORA_PKT_EXPLICIT, 50, true, false},
                                            FREQ_US_RADIO,
                                            1,
                                            0x34,
                                            0 };


bool test::radio_irq_raised_ = false;
bool test::irq_rx_timeout_raised_ = false;
bool test::timer_irq_raised_ = false;
uint32_t test::irq_time_ms_ = 0;
uint32_t test::irq_time_s_ = 0;
int test::radio_irq_cnt_ = 0;
int test::radio_rx_irq_cnt_ = 0;

test::test()
{

    uint32_t rx_timeout_in_ms = 500;

    // Reset, init radio and put it in sleep mode
    auto ret = reset_init_radio();
    if( ret == false ) {
        std::print("reset_init_radio failed!\n");
    }
    
    std::print("1\n");

    // Setup radio and relative irq
    smtc_modem_hal_irq_config_radio_irq(radio_irq_callback, this);
    smtc_modem_hal_start_radio_tcxo();
    smtc_modem_hal_set_ant_switch(false);
    
    std::print("2\n");

    if( ralf_setup_lora( &modem_radio_, &rx_lora_param ) != RAL_STATUS_OK )
    {
        std::print( " ralf_setup_lora() function failed \n" );
    }

    std::print("3\n");

    if( ral_set_dio_irq_params( &( modem_radio_.ral ), RAL_IRQ_ALL ) != RAL_STATUS_OK ) {
        std::print( " ral_set_dio_irq_params() function failed \n" );
    }

    std::print("4\n"); 
}

test::~test() {

}

bool test::porting_test_spi() {
    uint16_t counter_nok = 0;

    // Reset radio (prerequisite)
   // ral_reset( &( modem_radio_.ral ) );

    for( uint16_t i = 0; i < NB_LOOP_TEST_SPI; i++ )
    {

        sx126x_chip_status_t chip_status;
        sx126x_status_t      status;

        status = sx126x_get_status( nullptr, &chip_status );

        if( status == SX126X_STATUS_OK )
        {
            if( chip_status.chip_mode == SX126X_CHIP_MODE_UNUSED )
            {
                std::print( " Wrong SX126X chip mode, get SX126X_CHIP_MODE_UNUSED \n" );
                counter_nok++;
            }
        }
        else
        {
            std::print( " Failed to get SX126X status \n" );
            counter_nok++;
        }
    }

    if( counter_nok == 0 )
    {
        std::print( "SPI TEST - OK\n");
    }
    else
    {
        std::print( " Failed test = %u / %u \n", counter_nok, NB_LOOP_TEST_SPI );
        return false;
    }

    return true;    
}


bool test::porting_test_radio_irq() {
    //std::print( "----------------------------------------\n porting_test_radio_irq : " );

    bool     ret              = true;
    uint32_t rx_timeout_in_ms = 500;
    radio_irq_raised_         = false;
    irq_rx_timeout_raised_    = false;

    // Reset, init radio and put it in sleep mode
    //ret = reset_init_radio();
    //if( ret == false ) {
    //    std::print("call failed\n");
    //    return ret;
    //}

    // Setup radio and relative irq
    //smtc_modem_hal_irq_config_radio_irq( radio_rx_irq_callback, this);
    //smtc_modem_hal_start_radio_tcxo();
    //smtc_modem_hal_set_ant_switch(false);
    
    //if( ralf_setup_lora( &modem_radio_, &rx_lora_param ) != RAL_STATUS_OK )
    //{
    //    std::print( " ralf_setup_lora() function failed \n" );
    //   return false;
    //}
    
   // if( ral_set_dio_irq_params( &( modem_radio_.ral ), RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_HDR_ERROR |
   //                                                       RAL_IRQ_RX_CRC_ERROR ) != RAL_STATUS_OK )
   // {
   //     std::print( " ral_set_dio_irq_params() function failed \n" );
   //     return false;
   // }

    // Configure radio in reception mode
    if( ral_set_rx( &( modem_radio_.ral ), rx_timeout_in_ms ) != RAL_STATUS_OK ) {
        std::print( " ral_set_rx() function failed \n" );
        return false;
    }

    int32_t end_t = smtc_modem_hal_get_time_in_s()+1;
    while(!radio_irq_raised_ && (smtc_modem_hal_get_time_in_s() < end_t)) {

    }

    if( radio_irq_raised_ == true ) {
        return true;
        std::print( " OK \n" );
    } else {
        std::print( " Timeout, radio irq not received \n" );
        return false;
    }

    //smtc_modem_hal_stop_radio_tcxo();


}

bool test::porting_test_get_time() {
    int ret = RC_PORTING_TEST_OK;

    do
    {
        ret = test_get_time_in_s( );
        if( ret == RC_PORTING_TEST_NOK )
            return false;
    } while( ret == RC_PORTING_TEST_RELAUNCH );

    do
    {
        ret = test_get_time_in_ms( );
        if( ret == RC_PORTING_TEST_NOK )
            return false;
    } while( ret == RC_PORTING_TEST_RELAUNCH );

    return true;
}

/**
 * @brief Test get time in s
 *
 *
 * @remark
 *  Test processing:
 * - Reset, init and configure radio
 * - Configure radio in reception mode with a timeout
 * - Get start time
 * - Wait radio irq (get stop time in irq callback)
 * - Check if time is coherent with the configured timeout radio irq
 * Note: if radio irq received different of rx timeout irq -> relaunch test
 *
 * Ported functions:
 * smtc_modem_hal_get_time_in_s
 *      hal_rtc_get_time_s
 *
 * @return return_code_test_t   RC_PORTING_TEST_OK
 *                              RC_PORTING_TEST_NOK
 *                              RC_PORTING_TEST_RELAUNCH
 */
return_code_test_t test::test_get_time_in_s()
{
    bool     ret              = true;
    uint32_t rx_timeout_in_ms = 5000;

    radio_irq_raised_      = false;
    irq_rx_timeout_raised_ = false;

    rx_lora_param.symb_nb_timeout = 0;

    //std::print( " * Get time in second: " );

    // Reset, init radio and put it in sleep mode
    ret = reset_init_radio();
    if( ret == false )
        return RC_PORTING_TEST_NOK;

    //std::print( " * Get time in second: 1\n" );

    // Setup radio and relative irq
    smtc_modem_hal_irq_config_radio_irq(radio_irq_callback_get_time_in_s, this);
    smtc_modem_hal_start_radio_tcxo();
    smtc_modem_hal_set_ant_switch(false);

    if( ralf_setup_lora( &modem_radio_, &rx_lora_param ) != RAL_STATUS_OK )
    {
        std::print( " ralf_setup_lora() function failed \n" );
        return RC_PORTING_TEST_NOK;
    }

    //std::print( " * Get time in second: 5\n" );

    if( ral_set_dio_irq_params( &( modem_radio_.ral ), RAL_IRQ_ALL) != RAL_STATUS_OK ) //RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_HDR_ERROR |
                                                        //  RAL_IRQ_RX_CRC_ERROR ) != RAL_STATUS_OK )
    {
        std::print( " ral_set_dio_irq_params() function failed \n" );
        return RC_PORTING_TEST_NOK;
    }

    // Configure radio in reception mode
    if( ral_set_rx( &( modem_radio_.ral ), rx_timeout_in_ms ) != RAL_STATUS_OK )
    {
        std::print( " ral_set_rx() function failed \n" );
        return RC_PORTING_TEST_NOK;
    }

    //std::print( " * Get time in second: 7\n");

    uint32_t start_time_s = smtc_modem_hal_get_time_in_s( );

    std::print( " * Get time in second: 7a\n");

    while( radio_irq_raised_ == false )
    {
        // Do nothing
#if 0        
        busy_wait_ms(2000);

        ral_irq_t radio_irq = 0;
        if( ral_get_irq_status( &( modem_radio_.ral ), &radio_irq ) != RAL_STATUS_OK ) {
            std::print(" ral_get_irq_status() function failed \n" );
        }
         std::print("radio_irq = %d\n", radio_irq);
 
        busy_wait_ms(500);
       

        sx126x_chip_status_t chip_status;
        sx126x_status_t      status;
        status = sx126x_get_status( nullptr, &chip_status );

        if( status == SX126X_STATUS_OK )
        {
            std::print("chip mode = %d\n", chip_status.chip_mode);
        }

        if( ral_set_rx( &( modem_radio_.ral ), rx_timeout_in_ms ) != RAL_STATUS_OK ) {
            std::print( " ral_set_rx() function failed \n" );
        } 
      #endif        
    }

    std::print( " * Get time in second: 8\n" );

    if( irq_rx_timeout_raised_ == false )
    {
        std::print( "\n Radio irq received but not RAL_IRQ_RX_TIMEOUT -> relaunched test \n " );
        return RC_PORTING_TEST_RELAUNCH;
    }

    //std::print( " * Get time in second: 9\n" );

    uint32_t time = irq_time_s_ - start_time_s;
    if( time == ( rx_timeout_in_ms / 1000 ) )
    {
        std::print("");
        std::print( " Time expected %us / get %us (no margin) \n", ( rx_timeout_in_ms / 1000 ), time );
    }
    else
    {
        std::print( " Time is not coherent: expected %us / get %us (no margin) \n",
                              ( rx_timeout_in_ms / 1000 ), time );
        return RC_PORTING_TEST_NOK;
    }

    //std::print( " * Get time in second: 10\n" );

    return RC_PORTING_TEST_OK;
}

/**
 * @brief Test get time in ms
 *
 *
 * @remark
 *  Test processing:
 * - Reset, init and configure radio (with a timeout symbol number)
 * - Get start time
 * - Configure radio in reception mode
 * - Wait radio irq (get stop time in irq callback)
 * - Check if time is coherent with the configured timeout symbol number
 * Note: if radio irq received different of rx timeout irq -> relaunch test
 *
 * Ported functions:
 * smtc_modem_hal_get_time_in_ms
 *      hal_rtc_get_time_ms
 *
 * @return return_code_test_t   RC_PORTING_TEST_OK
 *                              RC_PORTING_TEST_NOK
 *                              RC_PORTING_TEST_RELAUNCH
 */
return_code_test_t test::test_get_time_in_ms( void )
{
    std::print( " * Get time in millisecond: " );

    bool ret               = true;
    radio_irq_raised_      = false;
    irq_rx_timeout_raised_ = false;
    uint8_t wait_start_ms  = 5;

    // To avoid misalignment between symb timeout and real timeout for all radio, a number of symbols smaller than 63 is
    // to be used.
    rx_lora_param.symb_nb_timeout = 62;
    rx_lora_param.mod_params.sf   = RAL_LORA_SF12;
    rx_lora_param.mod_params.bw   = RAL_LORA_BW_125_KHZ;

    // Warning: to be updated if previous parameters (SF and BW) are changed
    uint32_t symb_time_ms =
        ( uint32_t ) ( rx_lora_param.symb_nb_timeout * ( ( 1 << 12 ) / 125.0 ) );  // 2^(SF) / BW * symb_nb_timeout

    // Reset, init radio and put it in sleep mode
    ret = reset_init_radio( );
    if( ret == false )
        return RC_PORTING_TEST_NOK;

    // Setup radio and relative irq
    smtc_modem_hal_irq_config_radio_irq(radio_rx_irq_callback, this);

    smtc_modem_hal_start_radio_tcxo();
    smtc_modem_hal_set_ant_switch(false);
    if( ralf_setup_lora( &modem_radio_, &rx_lora_param ) != RAL_STATUS_OK )
    {
        std::print( " ralf_setup_lora() function failed \n" );
        return RC_PORTING_TEST_NOK;
    }
    if( ral_set_dio_irq_params( &( modem_radio_.ral ), RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_HDR_ERROR |
                                                          RAL_IRQ_RX_CRC_ERROR ) != RAL_STATUS_OK )
    {
        std::print( " ral_set_dio_irq_params() function failed \n" );
        return RC_PORTING_TEST_NOK;
    }

    // Wait 5ms to start
    uint32_t start_time_ms = smtc_modem_hal_get_time_in_ms( ) + wait_start_ms;
    while( smtc_modem_hal_get_time_in_ms( ) < start_time_ms )
    {
        // Do nothing
    }

    // Configure radio in reception mode
    if( ral_set_rx( &( modem_radio_.ral ), 0 ) != RAL_STATUS_OK )
    {
        std::print( " ral_set_rx() function failed \n" );
        return RC_PORTING_TEST_NOK;
    }

    while( radio_irq_raised_ == false )
    {
        // Do nothing
    }

    if( irq_rx_timeout_raised_ == false )
    {
        std::print( "\n Radio irq received but not RAL_IRQ_RX_TIMEOUT -> relaunched test \n" );
        return RC_PORTING_TEST_RELAUNCH;
    }

    uint32_t time = irq_time_ms_ - start_time_ms - smtc_modem_hal_get_radio_tcxo_startup_delay_ms( );
    if( abs( time - symb_time_ms ) <= MARGIN_GET_TIME_IN_MS )
    {
        std::print("OK 2");
        std::print( " Time expected %ums / get %ums (margin +/-%ums) \n", ( uint32_t ) symb_time_ms, time,
                               MARGIN_GET_TIME_IN_MS );
    }
    else
    {
        std::print( " Time is not coherent with radio irq : expected %ums / get %ums (margin +/-%ums) \n",
                              ( uint32_t ) symb_time_ms, time, MARGIN_GET_TIME_IN_MS );
        return RC_PORTING_TEST_NOK;
    }

    return RC_PORTING_TEST_OK;
}


bool test::porting_test_timer_irq( void )
{
    std::print( "----------------------------------------\n porting_test_timer_irq : " );

    uint32_t timer_ms      = 3000;
    uint8_t  wait_start_ms = 5;
    uint16_t timeout_ms    = 2000;
    timer_irq_raised_       = false;

    smtc_modem_hal_stop_timer( );

    // Wait 5ms to start
    uint32_t start_time_ms = smtc_modem_hal_get_time_in_ms( ) + wait_start_ms;

    volatile uint32_t current_time_ms = start_time_ms;
    uint32_t cnt = 0;
    do {
        current_time_ms = smtc_modem_hal_get_time_in_ms();
        cnt++;
    } while(current_time_ms < start_time_ms);

    //std::print(" porting_test_timer_irq::current_time_ms %d[%d]%d\n", 5, cnt, smtc_modem_hal_get_time_in_ms());

    smtc_modem_hal_start_timer( timer_ms, timer_irq_callback,
                                this );  // Warning this function takes ~3,69 ms for STM32L4

    // Timeout if irq not raised
    while( ( timer_irq_raised_ == false ) &&
           ( ( smtc_modem_hal_get_time_in_ms( ) - start_time_ms ) < ( timer_ms + timeout_ms ) ) )
    {
        // Do nothing
    }

    if( timer_irq_raised_ == false )
    {
        std::print( " Timeout: timer irq not received \n" );
        return false;
    }

    uint32_t time = irq_time_ms_ - start_time_ms;

    if( ( time >= timer_ms ) && ( time <= timer_ms) )
    {
        std::print(" OK " );
        std::print( " Timer irq configured with {} ms / get {} ms (margin + {} ms) \n", timer_ms, time,
                               0);
    }
    else
    {
        std::print( " Timer irq delay is not coherent: expected {} ms / get {} ms (margin + {}ms) \n", timer_ms,
                              time, 4 );
        return false;
    }
    return true;
}

bool test::porting_test_stop_timer()
{
    std::print( "----------------------------------------\n porting_test_stop_timer : " );

    uint32_t timer_ms = 1000;
    timer_irq_raised_  = false;

    smtc_modem_hal_start_timer( timer_ms, timer_irq_callback, this );

    // Wait half of timer
    uint32_t time = smtc_modem_hal_get_time_in_ms( );
    while( ( smtc_modem_hal_get_time_in_ms( ) - time ) < ( timer_ms / 2 ) )
    {
        // Do nothing
    }

    smtc_modem_hal_stop_timer( );

    // Wait a little more than the end of timer
    time = smtc_modem_hal_get_time_in_ms( );
    while( ( smtc_modem_hal_get_time_in_ms( ) - time ) < ( timer_ms + 500 ) )
    {
        // Do nothing
    }

    if( timer_irq_raised_ == false )
    {
        std::print(" OK ");
    }
    else
    {
        std::print( " Timer irq raised while timer is stopped \n" );
        return false;
    }
    return true;
}

bool test::porting_test_disable_enable_irq()
{
    std::print( "----------------------------------------\n porting_test_disable_enable_irq : " );

    uint32_t timer_ms = 3000;
    timer_irq_raised_  = false;

    smtc_modem_hal_start_timer( timer_ms, timer_irq_callback, this );

    smtc_modem_hal_disable_modem_irq( );

    // Wait a little more than the end of timer
    uint32_t time = smtc_modem_hal_get_time_in_ms( );
    while( ( smtc_modem_hal_get_time_in_ms( ) - time ) < ( timer_ms + 1000 ) ) {
        // Do nothing
    }

    if( timer_irq_raised_ == true ) {
        std::print( " Timer irq raised while irq is disabled\n" );
        return false;
    }

    smtc_modem_hal_enable_modem_irq( );

    smtc_modem_hal_start_timer( timer_ms, timer_irq_callback, this );

    if( timer_irq_raised_ == true ) {
        std::print( "OK" );
    } else {
        std::print( " Timer irq not received while irq is reenabled \n" );
        return false;
    }

    return true;
}

bool test::porting_test_random() {

    bool ret = true;

   std::print( "----------------------------------------\n porting_test_random : \n" );

    std::print( " * Get random nb : " );
    uint32_t rdom1 = smtc_modem_hal_get_random_nb_in_range( 0, 0xFFFFFFFF );
    uint32_t rdom2 = smtc_modem_hal_get_random_nb_in_range( 0, 0xFFFFFFFF );

    if( ( rdom1 != 0 ) && ( rdom2 != 0 ) && ( rdom1 != rdom2 ) )
    {
        std::print("OK");
        std::print( " random1 = {}, random2 = {}\n", rdom1, rdom2 );
    }
    else
    {
        std::print( "\n => random1 = {}, random2 = {}\n", rdom1, rdom2 );
        ret = false;
    }

    std::print( " * Get random nb in range : " );
    uint32_t range_min = 1;
    uint32_t range_max = 42;

    rdom1 = smtc_modem_hal_get_random_nb_in_range( range_min, range_max );
    rdom2 = smtc_modem_hal_get_random_nb_in_range( range_min, range_max );

    if( ( rdom1 >= range_min ) && ( rdom1 <= range_max ) && ( rdom2 >= range_min ) && ( rdom2 <= range_max ) &&
        ( rdom1 != rdom2 ) )
    {
        std::print( "OK" );
        std::print( " random1 = {}, random2 = {} in range [{};{}]\n", rdom1, rdom2, range_min, range_max );
    }
    else
    {
        std::print( "\n => random1 = {}, random2 = {}, expected range [{};{}]\n", rdom1, rdom2, range_min,
                               range_max );
        ret = false;
    }

    std::print( " * Get random draw : " );
    range_min                       = 1;
    range_max                       = 10;
    uint32_t tab_counter_random[10] = { 0 };
    uint32_t nb_draw                = 100000;
    uint32_t probability_draw       = nb_draw / ( range_max - range_min + 1 );
    // Warning to be update if probability_draw is changed
    int16_t margin = ( probability_draw * 5 ) / 100;  // error margin = 5% of probability_draw

    for( uint32_t i = 0; i < nb_draw; i++ )
    {
        rdom1 = smtc_modem_hal_get_random_nb_in_range( range_min, range_max );
        tab_counter_random[rdom1 - 1]++;
    }

    uint8_t tab_size = sizeof( tab_counter_random ) / sizeof( uint32_t );
    for( uint16_t i = 0; i < tab_size; i++ )
    {
        if( abs( probability_draw - tab_counter_random[i] ) > margin )
        {
            std::print( "\n => The number {} has been drawned {} times, Expected [{};{}] times \n",
                                   ( i + 1 ), tab_counter_random[i], ( probability_draw - margin ),
                                   ( probability_draw + margin ) );
            ret = false;
        }
    }

    if( ret == true )
    {
        std::print("OK" );
    }
    else
    {
        std::print( " TODO Warning smtc_modem_hal_get_random_nb_in_range error margin > 5%% \n" );
    }

    std::print( " Random draw of {} numbers between [{};{}] range \n", nb_draw, range_min, range_max );

    return ret;
}

bool test::porting_test_config_rx_radio() {

    std::print( "----------------------------------------\n porting_test_config_rx_radio :" );

    bool ret = true;
    // uint32_t rx_timeout_in_ms = 500;
    uint16_t counter_nok = 0;
    radio_irq_raised_     = false;

    // Reset, init and put it in sleep mode radio
    // Setup radio and relative irq
    //ret = reset_init_radio( );
    //if( ret == false )
    //    return ret;

    //smtc_modem_hal_irq_config_radio_irq( radio_rx_irq_callback, NULL );

    for( uint16_t i = 0; i < 2; i++ )
    {
        radio_irq_raised_ = false;

        uint32_t start_time_ms = smtc_modem_hal_get_time_in_ms( );
        // Setup radio and relative irq
        smtc_modem_hal_start_radio_tcxo( );
        smtc_modem_hal_set_ant_switch( false );
        //if( ralf_setup_lora( &modem_radio_, &rx_lora_param ) != RAL_STATUS_OK )
        //{
        //    std::print( " ralf_setup_lora() function failed \n" );
        //    return false;
        //}
        //if( ral_set_dio_irq_params( &( modem_radio_.ral ), RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_HDR_ERROR |
        //                                                      RAL_IRQ_RX_CRC_ERROR ) != RAL_STATUS_OK )
        //{
         //   std::print( " ral_set_dio_irq_params() function failed \n" );
         //   return false;
        //}

        // Configure radio in reception mode
        // if( ral_set_rx( &( modem_radio.ral ), rx_timeout_in_ms ) !=
        //     RAL_STATUS_OK )
        // {
        //     PORTING_TEST_MSG_NOK( " ral_set_rx() function failed \n" );
        //     return false;
        // }

        uint32_t time = smtc_modem_hal_get_time_in_ms( ) - start_time_ms;
        if( time >= 8 )
        {
            std::print( " Configuration of rx radio is too long: %ums (margin +%ums) \n", time,
                                 8 );
            counter_nok++;
        }
        // else
        // {
        //     SMTC_HAL_TRACE_std::print( " Configuration of rx radio is: %ums  \n", time );
        // }

        smtc_modem_hal_stop_radio_tcxo( );
    }

    if( counter_nok == 0 )
    {
        std::print( "OK" );
    }
    else
    {
        std::print( " => Failed test = %u / %u \n", counter_nok, 2 );
    }

    return true;
}

bool test::porting_test_config_tx_radio() {

    std::print( "----------------------------------------\n porting_test_config_tx_radio :" );

    uint16_t payload_size = 50;
    uint8_t  payload[50]  = { 0 };
    uint16_t counter_nok  = 0;
    radio_irq_raised_      = false;

    // Reset, init and put it in sleep mode radio
    //bool ret = reset_init_radio( );
    //if( ret == false )
    //    return ret;

    // Setup radio and relative irq
    //smtc_modem_hal_irq_config_radio_irq( radio_tx_irq_callback, NULL );

    for( uint16_t i = 0; i < 2; i++ )
    {
        //radio_irq_raised_ = false;

        uint32_t start_time_ms = smtc_modem_hal_get_time_in_ms( );

        smtc_modem_hal_start_radio_tcxo();
        smtc_modem_hal_set_ant_switch(true);
        if( ralf_setup_lora(&modem_radio_, &tx_lora_param ) != RAL_STATUS_OK )
        {
            std::print( " ralf_setup_lora() function failed \n" );
            return false;
        }
        
        if( ral_set_dio_irq_params( &( modem_radio_.ral ), RAL_IRQ_TX_DONE ) != RAL_STATUS_OK )
        {
            std::print( " ral_set_dio_irq_params() function failed \n" );
            return false;
        }

        if( ral_set_pkt_payload( &( modem_radio_.ral ), payload, payload_size ) != RAL_STATUS_OK )
        {
            std::print( " ral_set_pkt_payload() function failed \n" );
            return false;
        }

        if( ral_set_tx( &( modem_radio_.ral ) ) != RAL_STATUS_OK )
        {
            std::print( " ral_set_tx() function failed \n" );
            return false;
        }

        uint32_t time = smtc_modem_hal_get_time_in_ms( ) - start_time_ms;
        if( time >= 8 )
        {
            std::print( " Configuration of tx radio is too long: %ums (margin +%ums) \n", time,
                                  8 );
            counter_nok++;
        }
        // else
        // {
        //     SMTC_HAL_TRACE_std::print( " Configuration of tx radio is: %ums  \n", time );
        // }

        smtc_modem_hal_stop_radio_tcxo( );
    }

    if( counter_nok == 0 )
    {
        std::print( "OK" );
    }
    else
    {
        std::print( " => Failed test = %u / %u \n", counter_nok, 2 );
    }

    return true;
}

bool test::porting_test_sleep_ms() {

    std::print( "----------------------------------------\n porting_test_sleep_ms :" );

    bool    ret           = true;
    int32_t sleep_ms      = 2000;
    uint8_t wait_start_ms = 5;

    // Wait 5ms to start
    uint32_t start_time_ms = smtc_modem_hal_get_time_in_ms( ) + wait_start_ms;
    while( smtc_modem_hal_get_time_in_ms( ) < start_time_ms )
    {
        // Do nothing
    }

    hal_sleep_ms( sleep_ms );

    uint32_t stop_time_ms = smtc_modem_hal_get_time_in_ms( );
    uint32_t time         = stop_time_ms - start_time_ms;

    if( abs( time - sleep_ms ) <= 2 ) {
        std::print( "OK" );
        std::print( " Sleep time expected {} ms / get {} ms (margin +/- {} ms) \n", sleep_ms, time,
                               2 );
    } else {
        std::print( "\n => Sleep time is not coherent: expected {} ms / get {} ms (margin +/- {} ms) \n",
                               sleep_ms, time, 2 );
    }

    return ret;
}


bool test::porting_test_timer_irq_low_power() {

    std::print( "----------------------------------------\n porting_test_timer_irq_low_power : " );

    uint32_t timer_ms      = 3000;
    int32_t  sleep_ms1      = timer_ms + 3000;
    uint8_t  wait_start_ms = 5;
    timer_irq_raised_       = false;

    smtc_modem_hal_stop_timer();

    // Wait 5ms to start
    uint32_t start_time_ms = smtc_modem_hal_get_time_in_ms( ) + wait_start_ms;
    while( smtc_modem_hal_get_time_in_ms( ) < start_time_ms )
    {
        // Do nothing
    }

    smtc_modem_hal_start_timer( timer_ms, timer_irq_callback,
                                this );  // Warning this function takes ~3,69 ms for STM32L4

    hal_sleep_ms( sleep_ms1 );

    if( timer_irq_raised_ == false )
    {
        std::print( " Timeout: timer irq not received \n" );
        return false;
    }

    uint32_t time =
        irq_time_ms_ - start_time_ms - 0;  // TODO Warning to compensate delay introduced by
                                                                 // smtc_modem_hal_start_timer for STM32L4
    if( ( time >= timer_ms ) && ( time <= timer_ms + 2 ) )
    {
        std::print( "OK" );
        std::print( " Timer irq configured with {}ms / get {}ms (margin +{}ms) \n", timer_ms, time,
                               2 );
    }
    else
    {
        std::print( " Timer irq delay is not coherent: expected {}ms / get {}ms (margin +{}ms) \n", timer_ms,
                              time, 2 );
        return false;
    }
    return true;
}


bool test::porting_context_test(modem_context_type_t context_type) {

    switch(context_type) {
    case CONTEXT_MODEM:
    {   
        uint8_t read_buffer[16] = {0};
        smtc_modem_hal_context_restore( context_type, 0, read_buffer, sizeof( read_buffer ) );
        std::printf( " Read:  { " );
        for( uint8_t i = 0; i < sizeof( read_buffer ); i++ )
        {
            std::print( "{}", read_buffer[i] );
            if( i != ( sizeof( read_buffer ) - 1 ) )
                std::print( ", " );
        }
        std::printf( " }\n" );        

        if(memcmp(read_buffer, CONTEXT_MODEM_DATA, sizeof(CONTEXT_MODEM_DATA)) != 0) {
            std::print("CONTEXT_MODEM doesnt not match writing\n");
            smtc_modem_hal_context_store( context_type, 0, CONTEXT_MODEM_DATA, sizeof( CONTEXT_MODEM_DATA ) );
        } else {
            std::print("CONTEXT_MODEM matches test pattern\n");
        }
        break;
    }
    case CONTEXT_KEY_MODEM:
    {
        uint8_t read_buffer[20] = {0};
        smtc_modem_hal_context_restore( context_type, 0, read_buffer, sizeof( read_buffer ) );
        std::printf( " Read:  { " );
        for( uint8_t i = 0; i < sizeof( read_buffer ); i++ )
        {
            std::print( "{}", read_buffer[i] );
            if( i != ( sizeof( read_buffer ) - 1 ) )
                std::print( ", " );
        }
        std::printf( " }\n" );        

        if(memcmp(read_buffer, CONTEXT_KEY_MODEM_DATA, sizeof(CONTEXT_KEY_MODEM_DATA)) != 0) {
            std::print("CONTEXT_KEY_MODEM doesnt not match writing\n");
            smtc_modem_hal_context_store( context_type, 0, CONTEXT_KEY_MODEM_DATA, sizeof( CONTEXT_KEY_MODEM_DATA ) );
        } else {
            std::print("CONTEXT_KEY_MODEM matches test pattern\n");
        }
        break;
    }
    case CONTEXT_LORAWAN_STACK:
    {
        uint8_t read_buffer[32] = {0};
        smtc_modem_hal_context_restore( context_type, 0, read_buffer, sizeof( read_buffer ) );
        std::printf( " Read:  { " );
        for( uint8_t i = 0; i < sizeof( read_buffer ); i++ )
        {
            std::print( "{}", read_buffer[i] );
            if( i != ( sizeof( read_buffer ) - 1 ) )
                std::print( ", " );
        }
        std::printf( " }\n" );        

        if(memcmp(read_buffer, CONTEXT_LORAWAN_STACK_DATA, sizeof(CONTEXT_LORAWAN_STACK_DATA)) != 0) {
            std::print("CONTEXT_LORAWAN_STACK_DATA doesnt not match writing\n");
            smtc_modem_hal_context_store( context_type, 0, CONTEXT_LORAWAN_STACK_DATA, sizeof( CONTEXT_LORAWAN_STACK_DATA ) );
        } else {
            std::print("CONTEXT_LORAWAN_STACK_DATA matches test pattern\n");
        }
        break;            
        break;
    }
    case CONTEXT_FUOTA:
        break;
    case CONTEXT_SECURE_ELEMENT:
        break;
    case CONTEXT_STORE_AND_FORWARD:
        break;
    default:
        return false;
    };

    return true;
}

bool test::test_context_store_restore( modem_context_type_t context_type )
{
    bool    ret             = true;
    uint8_t read_buffer[8]  = { 0 };
    uint8_t write_buffer[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    bool    cmp             = true;

    static const char* name_context_type[] = { "MODEM", "KEY_MODEM", "LORAWAN_STACK", "FUOTA", "SECURE_ELEMENT", "STORE_AND_FORWARD" };

    std::print( "\n * Context %s : \n", name_context_type[context_type] );

    smtc_modem_hal_context_restore( context_type, 0, read_buffer, sizeof( read_buffer ) );

    std::printf( " Read:  { " );
    for( uint8_t i = 0; i < sizeof( read_buffer ); i++ )
    {
        std::print( "{}", read_buffer[i] );
        if( i != ( sizeof( read_buffer ) - 1 ) )
            std::print( ", " );
    }
    std::printf( " }\n" );

    for( uint8_t i = 0; i < sizeof( write_buffer ); i++ )
    {
        if( read_buffer[i] == write_buffer[i] )
        {
            write_buffer[i] = ( read_buffer[i] + 1 ) % 256;
        }
    }

    std::printf( " Write: { " );
    for( uint8_t i = 0; i < sizeof( write_buffer ); i++ )
    {
        std::print( "{}", write_buffer[i] );
        if( i != ( sizeof( write_buffer ) - 1 ) )
            std::print( ", " );
    }
    std::printf( " }\n" );

    smtc_modem_hal_context_store( context_type, 0, write_buffer, sizeof( write_buffer ) );

    memset( read_buffer, 0, sizeof( read_buffer ) );
    smtc_modem_hal_context_restore( context_type, 0, read_buffer, sizeof( read_buffer ) );

    std::printf( " Read:  { " );
    for( uint8_t i = 0; i < sizeof( read_buffer ); i++ )
    {
        std::print( "{}", read_buffer[i] );
        if( i != ( sizeof( read_buffer ) - 1 ) )
            std::print( ", " );
    }
    std::printf( " }\n" );

    for( uint8_t i = 0; i < sizeof( write_buffer ); i++ )
    {
        if( read_buffer[i] != write_buffer[i] )
        {
            cmp = false;
        }
    }
    if( cmp == true )
    {
        std::print( " Store/restore without MCU reset :" );
        std::print("OK");
    }
    else
    {
        std::print( " Store or restore context failed (without MCU reset) \n\n" );
        return false;
    }

    return ret;
}

bool test::reset_init_radio() {
    ral_status_t status = RAL_STATUS_ERROR;

    // Reset, init radio and put it in sleep mode
    ral_reset( &( modem_radio_.ral ) );

    status = ral_init( &( modem_radio_.ral ) );
    if( status != RAL_STATUS_OK )
    {
        std::print( " ral_init() function failed \n" );
        return false;
    }

    status = ral_set_sleep( &( modem_radio_.ral ), true );
    smtc_modem_hal_set_ant_switch( false );
    if( status != RAL_STATUS_OK )
    {
        std::print( " ral_set_sleep() function failed \n" );
        return false;
    }

    return true;    
}
    
void test::radio_irq_callback_get_time_in_s(void* obj) {
    test* t = reinterpret_cast<test*>(obj);
    if(!t) {
        std::print("called no context obj\n");
        return;
    }

    ral_irq_t radio_irq = 0;
    t->irq_time_s_          = smtc_modem_hal_get_time_in_s();
    t->radio_irq_raised_    = true;

    if( ral_get_irq_status( &( t->modem_radio_.ral ), &radio_irq ) != RAL_STATUS_OK )
    {
        std::print( " ral_get_irq_status() function failed \n" );
    }

    std::print("radio irq = {}\n", radio_irq);

    if( ( radio_irq & RAL_IRQ_RX_TIMEOUT ) == RAL_IRQ_RX_TIMEOUT )
    {
         t->irq_rx_timeout_raised_ = true;
    }

    if( ral_clear_irq_status( &( t->modem_radio_.ral ), RAL_IRQ_ALL ) != RAL_STATUS_OK )
    {
        std::print( " ral_clear_irq_status() function failed \n" );
    }

    // Shut Down the TCXO
    smtc_modem_hal_stop_radio_tcxo( );    
}

void  test::radio_irq_callback(void* obj) {
    if(test* t = reinterpret_cast<test*>(obj); t) {
        ral_irq_t radio_irq = 0;
        t->irq_time_ms_         = smtc_modem_hal_get_time_in_ms();
        t->radio_irq_raised_    = true;

        radio_irq_cnt_++;

        if( ral_get_irq_status( &( t->modem_radio_.ral ), &radio_irq ) != RAL_STATUS_OK ) {
            std::print(" ral_get_irq_status() function failed \n" );
        }

        if( ( radio_irq & RAL_IRQ_RX_TIMEOUT ) == RAL_IRQ_RX_TIMEOUT ) {
            t->radio_rx_irq_cnt_++;
        }

        if( ral_clear_irq_status( &( t->modem_radio_.ral ), RAL_IRQ_ALL ) != RAL_STATUS_OK ) {
            std::print(" ral_clear_irq_status() function failed \n" );
        }        

    }
}


void test::radio_tx_irq_callback( void* obj )
{
    if(test* t = reinterpret_cast<test*>(obj); t) {
        // ral_irq_t radio_irq = 0;

        t->irq_time_ms_ = smtc_modem_hal_get_time_in_ms( );

        t->radio_irq_raised_ = true;

        // if( ral_get_irq_status( &( modem_radio.ral ), &radio_irq ) != RAL_STATUS_OK )
        // {
        //     SMTC_HAL_TRACE_MSG_COLOR( "NOK\n ral_get_irq_status() function failed \n", HAL_DBG_TRACE_COLOR_RED );
        // }
        // SMTC_HAL_TRACE_INFO( " RP: IRQ source - 0x%04x\n", radio_irq );
        if( ral_clear_irq_status( &( t->modem_radio_.ral ), RAL_IRQ_ALL ) != RAL_STATUS_OK )
        {
            std::print( " ral_clear_irq_status() function failed \n" );
        }
    }
}

void test::radio_rx_irq_callback(void* obj) {

    if(test* t = reinterpret_cast<test*>(obj); t) {
        ral_irq_t radio_irq = 0;
        t->irq_time_ms_         = smtc_modem_hal_get_time_in_ms();
        t->radio_irq_raised_    = true;

        if( ral_get_irq_status( &( t->modem_radio_.ral ), &radio_irq ) != RAL_STATUS_OK ) {
            std::print(" ral_get_irq_status() function failed \n" );
        }

        //std::print("radio_irq = %d\n", radio_irq);
        
        if( ( radio_irq & RAL_IRQ_RX_TIMEOUT ) == RAL_IRQ_RX_TIMEOUT ) {
            t->irq_rx_timeout_raised_ = true;
        }

        if( ral_clear_irq_status( &( t->modem_radio_.ral ), RAL_IRQ_ALL ) != RAL_STATUS_OK ) {
            std::print(" ral_clear_irq_status() function failed \n" );
        }

        // Shut Down the TCXO
        smtc_modem_hal_stop_radio_tcxo();
    }
}

void test::timer_irq_callback( void* obj )
{
    //std::print("timer_irq_callback called!!!\n");
    test* t = reinterpret_cast<test*>(obj);
    if(!t) {
        std::print("called no context obj\n");
        return;
    }
    t->irq_time_ms_      = smtc_modem_hal_get_time_in_ms( );
    t->timer_irq_raised_ = true;

}