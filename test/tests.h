#if !defined(_TESTS_H_)
#define _TESTS_H_


#include "ralf_sx126x.h"

#include "smtc_modem_hal.h"

typedef enum return_code_test_e
{
    RC_PORTING_TEST_OK       = 0x00,  // Test OK
    RC_PORTING_TEST_NOK      = 0x01,  // Test NOK
    RC_PORTING_TEST_RELAUNCH = 0x02,  // Relaunch test
} return_code_test_t;


class test {
public:
    test();
    ~test();
    bool porting_test_spi();
    bool porting_test_radio_irq();
    bool porting_test_get_time();
    bool porting_test_timer_irq();
    bool porting_test_stop_timer();
    bool porting_test_disable_enable_irq();
    bool porting_test_random();
    bool porting_test_config_rx_radio();
    bool porting_test_config_tx_radio();
    bool porting_test_sleep_ms();
    bool porting_test_timer_irq_low_power();
    bool test_context_store_restore(modem_context_type_t context_type);
    bool porting_context_test(modem_context_type_t context_type);

    static void radio_irq_callback(void* obj);

    static void radio_rx_irq_callback(void* obj);
    static void radio_tx_irq_callback(void* obj);
    static void radio_irq_callback_get_time_in_s(void* obj);
    static void timer_irq_callback(void* obj);

    return_code_test_t test_get_time_in_s();
    return_code_test_t test_get_time_in_ms();

    static int radio_irq_cnt_;
    static int radio_rx_irq_cnt_;
private:
    
    bool reset_init_radio();

    static bool  radio_irq_raised_;
    static constexpr ralf_t modem_radio_ = RALF_SX126X_INSTANTIATE(0);
    static uint32_t irq_time_ms_;
    static uint32_t irq_time_s_;
    static bool     irq_rx_timeout_raised_;
    static bool     timer_irq_raised_;
};

#endif