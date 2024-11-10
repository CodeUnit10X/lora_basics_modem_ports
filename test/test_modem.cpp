#include <cstddef>
#include <cstdint>
#include <print>

#include "hal.h"
#include "tests.h"

int main(int argc, char** argv) {

    //Initialize the pico specific hal functions
    hal_init();

    hal_sleep_ms(3000);

    std::print("Starting modem porting test...\n");

    test t_modem{};

    std::print("---Testing SPI interface: ");

    if(t_modem.porting_test_spi()) {
        std::print(" OK \n");
    } else {
        std::print(" FAIL \n");
    }

    test::radio_irq_cnt_ = 0;
    int ok_cnt = 0;
    for(int i = 0; i < 10; i++) {
        if(t_modem.porting_test_radio_irq()) {
            ok_cnt++;
        }
    }

    std::print(" irq test count = {}\n", ok_cnt);

    t_modem.porting_test_config_rx_radio();

    //t_modem.porting_test_get_time();
 
    t_modem.porting_context_test(CONTEXT_MODEM);
    t_modem.porting_context_test(CONTEXT_KEY_MODEM);
    t_modem.porting_context_test(CONTEXT_LORAWAN_STACK);

    t_modem.porting_test_timer_irq();
    t_modem.porting_test_stop_timer();
    t_modem.porting_test_disable_enable_irq();
    t_modem.porting_test_random();
    t_modem.porting_test_sleep_ms();

    t_modem.porting_test_timer_irq_low_power();

    std::print("test modem complete\n");

    return 0;
}
