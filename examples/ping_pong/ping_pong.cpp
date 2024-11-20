
/*!  This is based off Semtech's main_pinp_pong.c example found in SWSD003, see original license below.  
 *   Use "ral_xxx* calls in Lora Basics Modem library, were original used radio driver directly.  Good
 *   for checking you have basic TX and RX working without getting into LoRaWAN timing stuff.
 */


/*!
 * @file      main_ping_pong.c
 *
 * @brief     Ping-pong example for Sx126x chip
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <cstring>
#include <print>
#include <string>
#include <unistd.h>
#include <fcntl.h>

#include "mcu_hal.h"

#include "smtc_modem_utilities.h"
#include "smtc_modem_api.h"
#include "smtc_modem_test_api.h"
#include "ralf_sx126x.h"
#include "smtc_modem_hal.h"
#include "sx126x.h"

#include "ral_sx126x_bsp.h"

using namespace std;
                                   
constexpr uint32_t RF_FREQ_IN_HZ = 903300000;
static bool    is_master = false;
static uint16_t iteration       = 0;
static uint16_t packets_to_sync = 0;
#define PING_PONG_PREFIX_SIZE 5
#define ITERATION_INDEX ( PING_PONG_PREFIX_SIZE + 1 )
#define PAYLOAD_LENGTH 7
#define DELAY_PING_PONG_PACE_MS 200
#define RX_TIMEOUT_VALUE 1000
#define DELAY_BEFORE_TX_MS 20
#define SYNC_PACKET_THRESHOLD 64

static const uint8_t ping_msg[PING_PONG_PREFIX_SIZE] = "PING";
static const uint8_t pong_msg[PING_PONG_PREFIX_SIZE] = "PONG";

static uint8_t buffer_tx[PAYLOAD_LENGTH];

void on_tx_done(ralf_t* context);
void on_rx_done(ralf_t* context);
void on_rx_timeout(ralf_t* context);


//!IMPORTANT:  These are just example parameters, make sure your settings are in compliance, with your region
static ralf_params_lora_t rx_lora_param = { {RAL_LORA_SF7, RAL_LORA_BW_125_KHZ, RAL_LORA_CR_4_5, 0},
                                            {8, RAL_LORA_PKT_EXPLICIT, PAYLOAD_LENGTH, false, false},
                                            RF_FREQ_IN_HZ,
                                            14, 
                                            0x34,
                                            0 };

void  radio_irq_callback(void* obj) {
    if(ralf_t* modem = reinterpret_cast<ralf_t*>(obj); modem) {
        ral_irq_t radio_irq = 0;
        if( ral_get_and_clear_irq_status( &( modem->ral ), &radio_irq ) != RAL_STATUS_OK ) {
            print(" ral_get_and_clear_irq_status() function failed \n" );
        }   

        if(radio_irq != 0) {
            if((radio_irq & RAL_IRQ_TX_DONE) == RAL_IRQ_TX_DONE) {
                print("TX Done!\n");
                on_tx_done(modem);
            }

            if((radio_irq & RAL_IRQ_RX_DONE) == RAL_IRQ_RX_DONE) {
                print("RX DONE\n");
                ral_sx126x_handle_rx_done(modem);
                on_rx_done(modem);
            }

            if((radio_irq & RAL_IRQ_RX_TIMEOUT ) == RAL_IRQ_RX_TIMEOUT) {
                print( "RX TIMEOUT\n" );
                on_rx_timeout(modem);
            }

        }    
    }
}

int main(int argc, char** argv) {
	
    mcu_hal_init();

    mcu_hal_sleep_ms(2000);

    print("mcu_hal_init complete!\n");

    ralf_t modem_radio_ = RALF_SX126X_INSTANTIATE(0);

    ral_reset( &( modem_radio_.ral ) );

    ral_status_t status = RAL_STATUS_ERROR;

    status = ral_init( &( modem_radio_.ral ) );
    if( status != RAL_STATUS_OK )
    {
        print( " ral_init() function failed \n" );
        return false;
    }

    status = ral_sx126x_set_standby(&( modem_radio_.ral ), RAL_STANDBY_CFG_RC);

    //sx126x_chip_status_t radio_status; 
    //if(sx126x_get_status(nullptr, &radio_status) != SX126X_STATUS_OK) {
    //    std::print("failed to get status\n");
    //}   

    //if(radio_status.chip_mode == SX126X_CHIP_MODE_STBY_RC) {
    //    std::print("sx1262 is in standby mode!\n");
    //} else {
     //   std::print("failed to put chip in standby mode!\n");
    //}

    smtc_modem_hal_irq_config_radio_irq(radio_irq_callback, (&( modem_radio_.ral )));

    ral_sx126x_set_rx_tx_fallback_mode( &( modem_radio_.ral ), RAL_FALLBACK_STDBY_RC);

    if( ralf_setup_lora( &modem_radio_, &rx_lora_param ) != RAL_STATUS_OK ) {
        print( " ralf_setup_lora() function failed \n" );
    }

    if( ral_set_dio_irq_params( &( modem_radio_.ral ), RAL_IRQ_ALL ) != RAL_STATUS_OK ) {
        print( " ral_set_dio_irq_params() function failed \n" );
    } 

    ral_sx126x_clear_irq_status( &( modem_radio_.ral ), RAL_IRQ_ALL);    

    memcpy( buffer_tx, ping_msg, PING_PONG_PREFIX_SIZE );
    buffer_tx[PING_PONG_PREFIX_SIZE] = 0;
    buffer_tx[ITERATION_INDEX]       = ( uint8_t ) ( iteration );
    for( int i = PING_PONG_PREFIX_SIZE + 1 + 1; i < PAYLOAD_LENGTH; i++ )
    {
        buffer_tx[i] = i;
    }


    smtc_modem_hal_set_ant_switch(true);

    if(ral_sx126x_set_pkt_payload( &( modem_radio_.ral ), buffer_tx, PAYLOAD_LENGTH ) == RAL_STATUS_OK) {
        print("Set pkt payload!!\n");
    }

   if(ral_sx126x_set_tx(&( modem_radio_.ral )) == RAL_STATUS_OK) {
        print("Set tx\n");
   }

    //gpio_init(PICO_DEFAULT_LED_PIN);
    //gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    bool led_on = false;
    while(true) {

        //sx126x_chip_status_t rs;
        //sx126x_get_status( &( modem_radio_.ral ), &rs);
        //print("rs chipmode = %d[%d]\n", rs.chip_mode, rs.cmd_status);

        //sx126x_errors_mask_t de;
        //sx126x_get_device_errors(&( modem_radio_.ral ), &de);
        //print("device errors = 0x%X\n", de);

        if(led_on) {
            led_on = false;
            //gpio_put(PICO_DEFAULT_LED_PIN, 0);
        } else {
            led_on = true;
            //gpio_put(PICO_DEFAULT_LED_PIN, 1);
        }        
        print("main loop....\n");
        mcu_hal_sleep_ms(2000);
    }

  	return 0;
}

void on_tx_done( ralf_t* context )
{
	string msg{(const char*)buffer_tx, sizeof(buffer_tx)-1};
    print("Sent message {}, iteration {}\n", msg, iteration);

    mcu_hal_sleep_ms(DELAY_PING_PONG_PACE_MS);

    if(RAL_STATUS_OK != ral_sx126x_set_rx(context, ral_sx126x_get_lora_time_on_air_in_ms( &rx_lora_param.pkt_params, &rx_lora_param.mod_params ) + RX_TIMEOUT_VALUE + rand( ) % 500)) {
        print("failed to set ral_sx126x_set_rx\n");
    }

    smtc_modem_hal_set_ant_switch(false);
}

void on_rx_done(ralf_t* context)
{
    uint8_t buffer_rx[PAYLOAD_LENGTH];
    uint8_t size;

    packets_to_sync = 0;

    uint16_t payload_byte = 0;
    if(ral_sx126x_get_pkt_payload(context, PAYLOAD_LENGTH, buffer_rx, &payload_byte) == RAL_STATUS_ERROR) {
        print( "Received more bytes than expected ({} vs {}), reception in buffer cancelled.\n",
                             payload_byte, PAYLOAD_LENGTH);
        size = 0;
    } 

    ral_lora_rx_pkt_status_t  pkt_status_lora;
    ral_sx126x_get_lora_rx_pkt_status(context, &pkt_status_lora);

    print( "  - RSSI packet = {} dBm\n", pkt_status_lora.rssi_pkt_in_dbm );
    print( "  - Signal RSSI packet = {} dBm\n", pkt_status_lora.signal_rssi_pkt_in_dbm );
    print( "  - SNR packet = {} dB\n", pkt_status_lora.snr_pkt_in_db );

    iteration = buffer_rx[ITERATION_INDEX];

    iteration++;

   	string msg{(const char*)buffer_rx, sizeof(buffer_rx)-1};
    print("Received message {}, iteration {}\n", msg, iteration);

    if( is_master == true )
    {
        if( memcmp( buffer_rx, ping_msg, PING_PONG_PREFIX_SIZE ) == 0 )
        {
            is_master = false;
            memcpy( buffer_tx, pong_msg, PING_PONG_PREFIX_SIZE );
        }
        else if( memcmp( buffer_rx, pong_msg, PING_PONG_PREFIX_SIZE ) != 0 )
        {
            print( "Unexpected message - PONG expected\n" );
        }
    }
    else
    {
        if( memcmp( buffer_rx, ping_msg, PING_PONG_PREFIX_SIZE ) != 0 )
        {
            print( "Unexpected message\n" );

            is_master = true;
            memcpy( buffer_tx, ping_msg, PING_PONG_PREFIX_SIZE );
        }
    }

    mcu_hal_sleep_ms(DELAY_PING_PONG_PACE_MS + DELAY_BEFORE_TX_MS);

    buffer_tx[ITERATION_INDEX] = ( uint8_t ) ( iteration );


    smtc_modem_hal_set_ant_switch(true);

    ral_sx126x_set_pkt_payload(context, buffer_tx, PAYLOAD_LENGTH );
    ral_sx126x_set_tx(context);
}


void on_rx_timeout(ralf_t* context)
{
    packets_to_sync++;
    if( packets_to_sync > SYNC_PACKET_THRESHOLD ) {
        print("It looks like synchronisation is still not done, consider resetting one of the board\n");
    }

    is_master = true;
    iteration = 0;
    memcpy( buffer_tx, ping_msg, PING_PONG_PREFIX_SIZE );
    buffer_tx[ITERATION_INDEX] = ( uint8_t ) ( iteration );

    ral_sx126x_set_pkt_payload(context, buffer_tx, PAYLOAD_LENGTH );
    ral_sx126x_set_tx(context);
}
