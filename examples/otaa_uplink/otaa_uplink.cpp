/*!
 *  Based on Semtech's main_periodical_uplink example.  See original license information below
*/


/*!
 * \file      main_periodical_uplink.c
 *
 * \brief     main program for periodical example
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
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

#include "hal.h"

#include "smtc_modem_api.h"
#include "smtc_modem_utilities.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_relay_api.h"

//On the server side this is the Application key for 1.0.X servers
#define USER_LORAWAN_DEVICE_EUI                        \
    {                                                  \
        0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x01, 0x05 \
    }

#define USER_LORAWAN_APP_KEY                                                                           \
    {                                                                                                  \
        0xE9, 0x31, 0xD0, 0x88, 0xB9, 0xA5, 0x75, 0x46, 0x97, 0x72, 0xD8, 0xC3, 0xB4, 0xFB, 0x31, 0x93 \
    }                                                                                                  \

#define USER_LORAWAN_NETWORK_KEY                                                                           \
    {                                                                                                  \
        0xE9, 0x31, 0xD0, 0x88, 0xB9, 0xA5, 0x75, 0x46, 0x97, 0x72, 0xD8, 0xC3, 0xB4, 0xFB, 0x31, 0x93 \
    }       

#define USER_LORAWAN_JOIN_EUI                          \
    {                                                  \
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 \
    }

#define STACK_ID 0

/**
 * @brief Stack credentials
 */
static const uint8_t user_dev_eui[8]  = USER_LORAWAN_DEVICE_EUI;
static const uint8_t user_join_eui[8] = USER_LORAWAN_JOIN_EUI;
static const uint8_t user_app_key[16] = USER_LORAWAN_APP_KEY;

/**
 * @brief Periodical uplink alarm delay in seconds
 */
#ifndef PERIODICAL_UPLINK_DELAY_S
#define PERIODICAL_UPLINK_DELAY_S 60
#endif

#ifndef DELAY_FIRST_MSG_AFTER_JOIN
#define DELAY_FIRST_MSG_AFTER_JOIN 60
#endif

uint8_t                  rx_payload[SMTC_MODEM_MAX_LORAWAN_PAYLOAD_LENGTH] = { 0 };  // Buffer for rx payload
uint8_t                  rx_payload_size = 0;      // Size of the payload in the rx_payload buffer
smtc_modem_dl_metadata_t rx_metadata     = { 0 };  // Metadata of downlink
uint8_t                  rx_remaining    = 0;      // Remaining downlink payload in modem
uint32_t                 uplink_counter  = 0;      // uplink raising counter
int join_fail_cnt = 0;

void modem_event_callback( void );
void send_uplink_counter_on_port( uint8_t port );
void send_voltage_reading_on_port( uint8_t port );

using namespace std;
int main(int argc, char** argv)
{
    uint32_t sleep_time_ms = 0;

    hal_init();

    hal_sleep_ms(1000);

    uint32_t error_ppm = 0;
    smtc_modem_get_crystal_error_ppm(&error_ppm);
    println("crystal error ppm currently set to = {}", error_ppm);

    if(smtc_modem_set_crystal_error_ppm(error_ppm*2) == SMTC_MODEM_RC_OK) {
        println("setting crystal error ppm");
    }

    hal_sleep_ms(1000);
    
    smtc_modem_init(&modem_event_callback);

    int uplink_counter = 0;
    while(true)
    {
        // Modem process launch
        sleep_time_ms = smtc_modem_run_engine();
        //hal_sleep_ms(sleep_time_ms);
    }

    return 0;
}

void modem_event_callback( void )
{
    printf("Modem event callback\n");

    smtc_modem_event_t current_event;
    uint8_t            event_pending_count;
    uint8_t            stack_id = STACK_ID;

    // Continue to read modem event until all event has been processed
    do
    {
        // Read modem event
        smtc_modem_get_event( &current_event, &event_pending_count );

        //printf("current modem event %d\n", current_event.event_type);

        switch( current_event.event_type )
        {
        case SMTC_MODEM_EVENT_RESET:
            println( "Event received: RESET" );

            smtc_modem_set_network_type(stack_id, true);

            // Set user credentials
           
            // Set user credentials
            smtc_modem_set_deveui(stack_id, user_dev_eui);
            smtc_modem_set_joineui(stack_id, user_join_eui);

            //This Application Key in LoRaWAN v1.0.x
            smtc_modem_set_nwkkey(stack_id, user_app_key);

            // Set user region
            smtc_modem_set_region( stack_id, SMTC_MODEM_REGION_US_915 );

            smtc_modem_join_network( stack_id );
            break;

        case SMTC_MODEM_EVENT_ALARM:
            println( "Event received: ALARM" );
            // Send periodical uplink on port 101
            send_uplink_counter_on_port( 101 );
            send_voltage_reading_on_port( 103 );
            // Restart periodical uplink alarm
            smtc_modem_alarm_start_timer( PERIODICAL_UPLINK_DELAY_S );
            break;

        case SMTC_MODEM_EVENT_JOINED:
            println( "Event received: JOINED" );
            println( "Modem is now joined" );

            // Send first periodical uplink on port 101
            send_uplink_counter_on_port( 101 );
            // start periodical uplink alarm
            smtc_modem_alarm_start_timer( DELAY_FIRST_MSG_AFTER_JOIN );
            break;

        case SMTC_MODEM_EVENT_TXDONE:
            println( "Event received: TXDONE" );
            println( "Transmission done" );
            break;

        case SMTC_MODEM_EVENT_DOWNDATA:
            println( "Event received: DOWNDATA" );
            // Get downlink data
            smtc_modem_get_downlink_data( rx_payload, &rx_payload_size, &rx_metadata, &rx_remaining );
            println( "Data received on port {}", rx_metadata.fport );
            println( "Received payload");
            break;

        case SMTC_MODEM_EVENT_JOINFAIL:
            println( "Event received: JOINFAIL {}", ++join_fail_cnt );
            break;

        case SMTC_MODEM_EVENT_ALCSYNC_TIME:
            println( "Event received: ALCSync service TIME" );
            break;

        case SMTC_MODEM_EVENT_LINK_CHECK:
            println( "Event received: LINK_CHECK" );
            break;

        case SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_INFO:
            println( "Event received: CLASS_B_PING_SLOT_INFO" );
            break;

        case SMTC_MODEM_EVENT_CLASS_B_STATUS:
            println( "Event received: CLASS_B_STATUS" );
            break;

        case SMTC_MODEM_EVENT_LORAWAN_MAC_TIME:
            println( "Event received: LORAWAN MAC TIME" );
            break;

        case SMTC_MODEM_EVENT_LORAWAN_FUOTA_DONE:
        {
            bool status = current_event.event_data.fuota_status.successful;
            if( status == true )
            {
                println( "Event received: FUOTA SUCCESSFUL" );
            }
            else
            {
                println( "Event received: FUOTA FAIL" );
            }
            break;
        }

        case SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_C:
            println( "Event received: MULTICAST CLASS_C STOP" );
            break;

        case SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_B:
            println( "Event received: MULTICAST CLASS_B STOP" );
            break;

        case SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_C:
            println( "Event received: New MULTICAST CLASS_C" );
            break;

        case SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_B:
            println( "Event received: New MULTICAST CLASS_B" );
            break;

        case SMTC_MODEM_EVENT_FIRMWARE_MANAGEMENT:
            println( "Event received: FIRMWARE_MANAGEMENT" );
            if( current_event.event_data.fmp.status == SMTC_MODEM_EVENT_FMP_REBOOT_IMMEDIATELY )
            {
                smtc_modem_hal_reset_mcu( );
            }
            break;

        case SMTC_MODEM_EVENT_STREAM_DONE:
            println( "Event received: STREAM_DONE" );
            break;

        case SMTC_MODEM_EVENT_UPLOAD_DONE:
            println( "Event received: UPLOAD_DONE" );
            break;

        case SMTC_MODEM_EVENT_DM_SET_CONF:
            println( "Event received: DM_SET_CONF" );
            break;

        case SMTC_MODEM_EVENT_MUTE:
            println( "Event received: MUTE" );
            break;
        case SMTC_MODEM_EVENT_RELAY_TX_DYNAMIC:  //!< Relay TX dynamic mode has enable or disable the WOR protocol
            println( "Event received: RELAY_TX_DYNAMIC");
            break;
        case SMTC_MODEM_EVENT_RELAY_TX_MODE:  //!< Relay TX activation has been updated
            println( "Event received: RELAY_TX_MODE" );
            break;
        case SMTC_MODEM_EVENT_RELAY_TX_SYNC:  //!< Relay TX synchronisation has changed
            println( "Event received: RELAY_TX_SYNC" );
            break;
        default:
            println( "Unknown event");
            break;
        }
    } while( event_pending_count > 0 );
}

void send_uplink_counter_on_port( uint8_t port )
{
    // Send uplink counter on port 102
    uint8_t buff[4] = { 0 };
    buff[0]         = ( uplink_counter >> 24 ) & 0xFF;
    buff[1]         = ( uplink_counter >> 16 ) & 0xFF;
    buff[2]         = ( uplink_counter >> 8 ) & 0xFF;
    buff[3]         = ( uplink_counter & 0xFF );

    //printf("uplink counter %x:%x:%x:%x\n", buff[0], buff[1], buff[2], buff[3]);

    smtc_modem_request_uplink( STACK_ID, port, false, buff, 4 );

    // Increment uplink counter
    uplink_counter++;
}

void send_voltage_reading_on_port( uint8_t port )
{
    auto level = smtc_modem_hal_get_battery_level();

    // Send uplink counter on port 102
    uint8_t buff[2] = { 0 };
    buff[0]         = ( level >> 8 ) & 0xFF;
    buff[1]         = ( level & 0xFF );

    smtc_modem_request_uplink( STACK_ID, port, false, buff, 2 );
}

/* --- EOF ------------------------------------------------------------------ */
