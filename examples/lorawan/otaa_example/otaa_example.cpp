/*!
 *   @file otaa_example.cpp
 *   @brief Simple OTAA LoraWan example for Semtech's Lora Basics Modem API 
 *   @author Jeffrey Coffman <jeff.coffman@protonmail.com>
 *  
 *   @date 11/13/2024
 *
 *   This uses my port of Semtech's Lora Basic Modem to do OTAA and 
 *   send board temp in a periodic uplink message.
 *  
 *   This was tested on PICO/PICO2/Raspberry Pi Waveshare Hats, the gateway
 *   was a Waveshare Raspberry Pi sx1303 Hat running ChirpStack v4.6.0
 *  
 */ 
#include <print>
#include <csignal>

#include "mcu_hal.h"
#include "smtc_modem_api.h"
#include "smtc_modem_utilities.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_relay_api.h"
#include "hardware/i2c.h"


using namespace std;

constexpr uint32_t UPLINK_PERIOD = 30;

//! Update these accordingly for your chirpstack setup
constexpr uint8_t joineui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
constexpr uint8_t deveui[]  = { 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0xBE, 0xEF };
constexpr uint8_t appkey[]  = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x88 };

void signal_handler(int sig);

void modem_event_cb();


auto done = false;

int main(int argc, char** argv) {

    //Register all our signal handlers
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);
    signal(SIGQUIT, signal_handler);

	//initialize platform board hal
	mcu_hal_init();

	smtc_modem_version_t fw_ver;
	smtc_modem_get_modem_version(&fw_ver);
	println("SX1262 FW REV = {}.{}.{}", fw_ver.major, fw_ver.minor, fw_ver.patch);

	//Initialize the modem
	smtc_modem_init(modem_event_cb);

	//setup join parameters, for 1.0.x

	//!Make sure this is correct for your region
	if(smtc_modem_set_region(0, SMTC_MODEM_REGION_US_915) != SMTC_MODEM_RC_OK) {
		println("failed to set region");
	}

	if(smtc_modem_set_joineui(0, joineui) != SMTC_MODEM_RC_OK) {
		println("failed to set joineui");
	}

	if(smtc_modem_set_deveui(0, deveui) != SMTC_MODEM_RC_OK) {
		println("failed to set deveui");
	}

	if(smtc_modem_set_nwkkey(0, appkey) != SMTC_MODEM_RC_OK) {
		println("failed to set app key");
	}

	
	mcu_hal_sleep_ms(1000);

	while(!done) {

		smtc_modem_status_mask_t modem_status;
		smtc_modem_get_status(0, &modem_status);

		auto sleepy_time = smtc_modem_run_engine();
		//if(modem_status != SMTC_MODEM_STATUS_JOINING) {
		//	if(sleepy_time != 0) {
		//		println("modem status {}, sleeping for {}", modem_status, sleepy_time);
		//		mcu_hal_sleep_ms(sleepy_time);				
		//	} 
		//}
	}

	println("cleaning up and exiting");

	mcu_hal_exit();

	return 0;
}

void modem_event_cb() {

	smtc_modem_event_t event;
	uint8_t pending_cnt = 0;

	do {
		if(smtc_modem_get_event(&event, &pending_cnt) == SMTC_MODEM_RC_OK) {
			switch(event.event_type) {
			case SMTC_MODEM_EVENT_RESET: 
			{
				println("\033[32m SMTC_MODEM_EVENT_RESET \033[0m");
				smtc_modem_join_network(0);
				break;
			}
	    	case SMTC_MODEM_EVENT_ALARM:
	    	{
				println("\033[32m SMTC_MODEM_EVENT_ALARM\033[0m");
				uint8_t data_up[4];
				data_up[0] = 0xFF;
				data_up[1] = mcu_hal_read_temp();
				
				uint16_t batt = mcu_hal_read_batt_voltage() * 1000;
				data_up[2] = batt>>8;
				data_up[3] = batt&0xFF;				
				smtc_modem_request_uplink(0, 1, false, data_up, 4);  
				smtc_modem_alarm_start_timer(UPLINK_PERIOD);
	    		break;
	    	}
	    	case SMTC_MODEM_EVENT_JOINED:
	    	{
				println("\033[32m SMTC_MODEM_EVENT_JOINED\033[0m");
				smtc_modem_alarm_start_timer(UPLINK_PERIOD);
	    		break;
	    	}
	    	case SMTC_MODEM_EVENT_TXDONE:
	    	{
				println("\033[32m SMTC_MODEM_EVENT_TXDONE\033[0m");	    	
	    		break;
	    	}
	    	case SMTC_MODEM_EVENT_DOWNDATA:
	    	{
				println("\033[32m SMTC_MODEM_EVENT_DOWNDATA\033[0m");	    	
	    		break;
	    	}
	    	case SMTC_MODEM_EVENT_JOINFAIL:
	    	{
				println("\033[31m SMTC_MODEM_EVENT_JOINFAIL\033[0m");	    	
	    		break;    		
	    	}
	    	default:
	    		println("UNHANDLED/UNKNOWN Event");
	    		break;
			}
		} else {
			println("failed to get modem event");
		}
	} while(pending_cnt && !done);
}


void signal_handler(int sig) {
	println("signal received");
    switch(sig) {
        case SIGTERM:
        case SIGINT:
            done = true;
            break;
        default:
            break;
    }
}
