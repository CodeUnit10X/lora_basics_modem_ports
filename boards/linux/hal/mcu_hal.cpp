#include "mcu_hal.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <sys/mman.h>
#include <poll.h>

#include <csignal>
#include <ctime>
#include <chrono>
#include <atomic>
#include <iostream>
#include <print>
#include <filesystem>
#include <thread>
#include <stdexcept>

#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <thread>

#include "sx126x_hal.h"

#include <gpiod.h>

namespace fs = std::filesystem;
using namespace std::chrono_literals;
using namespace std::chrono;

constexpr char TMP_LOCAL_PATH[]   = "/sys/class/hwmon/hwmon0/temp1_input";

typedef enum
{
    RADIO_SLEEP,
    RADIO_AWAKE
} radio_sleep_mode_t;

static radio_sleep_mode_t radio_mode = RADIO_AWAKE;

static void sx126x_hal_check_device_ready(void);
static void sx126x_hal_wait_on_busy(void);

struct gpio_info_t {
    char name_[255];
    struct gpiod_line* line_;
};

enum GPIO_NUMS {
    GPIO_DIO1,
    GPIO_DIO4,
    GPIO_RESET,
    GPIO_BUSY,
    GPIO_CS   
};


gpio_info_t gpio_info[5] = { { "DIO1", nullptr},
                             { "DIO4", nullptr},
                             { "RESET", nullptr},
                             { "BUSY",  nullptr},
                             { "CS", nullptr}};

struct context_wr_cache {
    uint8_t ctx_modem[16];
    uint8_t ctx_key_modem[20];
    uint8_t ctx_sc[480];
    uint8_t ctx_lora_stack[32];
    uint8_t ctx_fuota[4096 * 7];
    uint8_t ctx_str_fwd[4096 * 8]; 
};

/*!
 *  The Lora Basics Modem API assumes it working with flash.  We'll provide a mmap
 *  file for this on Linux.  This mimics the layout used on pico boards and thus
 *  uses the notion of sectors for each context.  Keep the same simplified system
 *  of just using a flash page for each context.
 */
constexpr uint32_t FAUX_FLASH_SIZE_BYTES    =  (4 * 1024 * 1024);
constexpr uint32_t FAUX_FLASH_SECTOR_SIZE   = 4096;
constexpr uint32_t CTX_MODEM_START          = FAUX_FLASH_SIZE_BYTES - (FAUX_FLASH_SECTOR_SIZE);
constexpr uint32_t CTX_KEY_MODEM_START      = FAUX_FLASH_SIZE_BYTES - (FAUX_FLASH_SECTOR_SIZE*2);
constexpr uint32_t CTX_LORAWAN_STACK_START  = FAUX_FLASH_SIZE_BYTES - (FAUX_FLASH_SECTOR_SIZE*3);
constexpr uint32_t CTX_SECURE_ELEMENT_START = FAUX_FLASH_SIZE_BYTES - (FAUX_FLASH_SECTOR_SIZE*4);
constexpr uint32_t CTX_FUOTA_STACK_START    = FAUX_FLASH_SIZE_BYTES - (FAUX_FLASH_SECTOR_SIZE*11);
constexpr uint32_t CTX_FOUTA_NUM_SECTORS    = 7;
constexpr uint32_t CTX_STORE_AND_FWD_START  = FAUX_FLASH_SIZE_BYTES - (FAUX_FLASH_SECTOR_SIZE*19);
constexpr uint32_t CTX_STORE_AND_FWD_NUM_SECTORS = 8;  

struct linux_hal_state {
    bool shutdown_;
    bool timer_started_;
    timer_t timer_id_;
    uint32_t timer_ms_;
    bool timer_enabled_;
    std::atomic_bool timer_pending_;
    struct user_data_tm user_data_;
    struct context_wr_cache ctx_cache_;
    void (*radio_cb_)(void* context);
    void* radio_cb_ctx_;
    uint32_t radio_cb_events_;
    int fd_;
    int fd_irq_;
    int fd_mmap_;
    int fd_temp_;
    std::thread irq_thread_;
    char* flash_addr_;
    struct gpiod_chip* gpio_chip_;
    struct gpiod_line* gpio_dio1_;
    struct gpiod_line* gpio_dio4_;
    struct gpiod_line* gpio_reset_;
    struct gpiod_line* gpio_busy_;
    struct gpiod_line* gpio_cs_;      
};

static struct linux_hal_state state_ = {false,
                                        false, 
                                        nullptr, 
                                        0, 
                                        false, 
                                        false, 
                                        {}, 
                                        {}, 
                                        nullptr, 
                                        nullptr, 
                                        0, 
                                        -1, 
                                        -1, 
                                        -1, 
                                        -1, 
                                        {}, 
                                        nullptr, 
                                        nullptr, 
                                        nullptr, 
                                        nullptr, 
                                        nullptr, 
                                        nullptr, 
                                        nullptr};

static void main_timer_callback(union sigval sv) {
    if(state_.user_data_.cb) {
        if(state_.timer_enabled_) {
            state_.user_data_.cb(state_.user_data_.context);
        } else {
            state_.timer_pending_ = true;
        }
    }    
}

static void set_uio_irq(bool enabled) {
    //enable/disable interrupts in case we're changing handle
    uint32_t info;

    if(enabled) {
        info = 1;
    } else {
        info = 0;
    }
    if(auto nb = write(state_.fd_irq_, &info, sizeof(info)); nb != sizeof(info)) {
        std::print("hal_config_radio_irq: falled to set uio irq = {}\n", enabled);
    }
}

void mcu_hal_init() {

    srand(time(nullptr));

    //setup gpio
    
    if(state_.gpio_chip_ = gpiod_chip_open_by_name("gpiochip0"); !state_.gpio_chip_) {
        std::print("failed to open gpio chip\n");
    }

    if(state_.gpio_cs_ = gpiod_chip_get_line(state_.gpio_chip_, 21); !state_.gpio_cs_) {
       perror("cs line:");
       std::print("failed to get chip select line[%d]\n", 21);
    }

    if(state_.gpio_busy_ = gpiod_chip_get_line(state_.gpio_chip_, 20); !state_.gpio_busy_) {
       std::print("failed to get busy line\n");
    }

    if(state_.gpio_reset_ = gpiod_chip_get_line(state_.gpio_chip_, 18); !state_.gpio_reset_) {
        std::print("failed to get reset line\n");
    }

    if(state_.gpio_dio1_ = gpiod_chip_get_line(state_.gpio_chip_, 16); !state_.gpio_dio1_) {
        std::print("failed to get dio1 line\n");
    }

    if(state_.gpio_dio4_ = gpiod_chip_get_line(state_.gpio_chip_, 6); !state_.gpio_dio4_) {
        std::print("failed to get dio4 line\n");
    }

    auto ret = gpiod_line_request_output(state_.gpio_cs_, "CS", 0);
    if (ret < 0) {
        std::print("Request line as output failed\n");
    }

    ret = gpiod_line_request_input(state_.gpio_busy_, "BUSY");
    if (ret < 0) {
        std::print("Request line as input failed\n");
    }

    ret = gpiod_line_request_input(state_.gpio_dio1_, "DIO1");
    if (ret < 0) {
        std::print("Request line as input failed\n");
   }

    ret = gpiod_line_request_output(state_.gpio_reset_, "RESET", 0);
    if (ret < 0) {
        std::print("Request line as input failed\n");
    }

    ret = gpiod_line_request_output(state_.gpio_dio4_, "DIO4", 1);
    if (ret < 0) {
        std::print("Request line as input failed\n");
    }    

    gpio_info[GPIO_DIO1].line_  = state_.gpio_dio1_;
    gpio_info[GPIO_DIO4].line_  = state_.gpio_dio4_;
    gpio_info[GPIO_RESET].line_ = state_.gpio_reset_;
    gpio_info[GPIO_BUSY].line_  = state_.gpio_busy_;
    gpio_info[GPIO_CS].line_    = state_.gpio_cs_;
                
    mcu_hal_spi_init();

    //uio irq
    state_.fd_irq_  = open("/dev/uio0", O_RDWR);
    if(state_.fd_irq_ < 0) {
        std::print("failed to open uio device\n");
    }
    
    state_.irq_thread_ = std::thread([](){
        pollfd poll_item;
        poll_item.fd = state_.fd_irq_;
        poll_item.events = POLLIN|POLLPRI;

        while(!state_.shutdown_) {
            uint32_t info = 1;
            ssize_t nb = write(state_.fd_irq_, &info, sizeof(info));
            if (nb != (ssize_t)sizeof(info)) {
                perror("write");
            }

            if(auto r = poll(&poll_item, (unsigned long)1, 1000); r > 0) {
                 /* Wait for interrupt */
                nb = read(state_.fd_irq_, &info, sizeof(info));
                if (nb == (ssize_t)sizeof(info)) {
                    if(state_.radio_cb_) {
                        state_.radio_cb_(state_.radio_cb_ctx_);
                    }                
                }                
            }       
        }           
    });

    //setup interval timer
    struct sigevent sev;

    sev.sigev_notify = SIGEV_THREAD;
    sev.sigev_notify_function = main_timer_callback;
    sev.sigev_notify_attributes = nullptr;
    sev.sigev_value.sival_ptr = &state_.timer_id_;

    if(timer_create(CLOCK_REALTIME, &sev, &state_.timer_id_) == -1) {
        std::print("failed to create interval timer for radio hal!!\n");
    }

    //init the store
    state_.fd_mmap_ = open("flash.bin", O_RDWR | O_CREAT, 0666);
    if(state_.fd_mmap_ != -1) {
        
        if(fs::file_size("flash.bin") != FAUX_FLASH_SIZE_BYTES) {
            if(ftruncate(state_.fd_mmap_, FAUX_FLASH_SIZE_BYTES) == -1) {
                std::print("Failed to truncate flash file\n");
            }
        }

        state_.flash_addr_ = (char*)mmap(nullptr, FAUX_FLASH_SIZE_BYTES, PROT_READ|PROT_WRITE, MAP_SHARED, state_.fd_mmap_, 0);
        if(state_.flash_addr_ == MAP_FAILED) {
            std::print("failed to create file backed memory mapping\n");
        }
    } else {
        std::print("Failed to open or create flash.bin\n");
    }

    if(std::filesystem::exists(TMP_LOCAL_PATH)) {
        if(state_.fd_temp_ = open(TMP_LOCAL_PATH, O_RDONLY | O_NONBLOCK); state_.fd_temp_ == -1) {
            std::println("failed to open temp device {}, cpu temp unavailable!", TMP_LOCAL_PATH);
        }
    }


}


void mcu_hal_exit() {

    state_.shutdown_ = true;

    state_.irq_thread_.join();

    munmap(state_.flash_addr_, FAUX_FLASH_SIZE_BYTES);

    close(state_.fd_mmap_);

    if(state_.gpio_dio1_) {
        gpiod_line_release(state_.gpio_dio1_);
    }

    if(state_.gpio_dio4_) {
        gpiod_line_release(state_.gpio_dio4_);
    }

    if(state_.gpio_reset_) {
        gpiod_line_release(state_.gpio_reset_);
    }

    if(state_.gpio_busy_) {
        gpiod_line_release(state_.gpio_busy_);
    }

    if(state_.gpio_cs_) {
        gpiod_line_release(state_.gpio_cs_);
    }

    if(state_.gpio_chip_) {
        gpiod_chip_close(state_.gpio_chip_);
    }


}

void mcu_hal_spi_init() {

    int mode = SPI_MODE_0;
    int bits = 8;
    int speed = 16000000;

    if(state_.fd_ = open("/dev/spidev0.0", O_RDWR); state_.fd_ > 0) {

        auto result = ioctl(state_.fd_, SPI_IOC_WR_MODE, &mode);
        if(result == -1) {
            std::print("failed to set spi mode\n");
        }

        result = ioctl(state_.fd_, SPI_IOC_RD_MODE32, &mode);
        if(result == -1) {
            std::print("failed to read spi mode\n");
        } 

        if(mode != SPI_MODE_0) {
            std::print("printf(device does not support spi mode\n");
        }

        result = ioctl(state_.fd_, SPI_IOC_WR_BITS_PER_WORD, &bits);
        if(result == -1) {
            std::print("failed to set bits per word\n");
        }

        result = ioctl(state_.fd_, SPI_IOC_RD_BITS_PER_WORD, &bits);
        if(result == -1) {
            std::print("failed to set bits per word\n");
        }

        result = ioctl(state_.fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
        if (result == -1){
            std::print("can't set max speed hz\n");
        }

        result = ioctl(state_.fd_, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
        if (result == -1) {
            std::print("can't get max speed hz\n");
        }
    } else {
        std::print("could not open device = /dev/spidev0.0\n");
    }
}

void mcu_hal_context_restore(const enum hal_flash_ctx ctx_type, uint32_t offset, uint8_t* buffer, const uint32_t size ) {
    //read flash and update cache
    char *p = nullptr;
    switch(ctx_type) {
        case hal_flash_ctx_modem:
        {
            p = state_.flash_addr_ + CTX_MODEM_START;
            break;
        }
        case hal_flash_ctx_key_modem:
        {
            p = state_.flash_addr_ + CTX_KEY_MODEM_START;         
            break;
        }
        case hal_flash_ctx_lora_stack:
        {
            p = state_.flash_addr_ + CTX_LORAWAN_STACK_START + offset;    
            break;
        }
        case hal_flash_ctx_sc:
        {
            p = state_.flash_addr_ + CTX_SECURE_ELEMENT_START + offset;         
            break;
        }
        case hal_flash_ctx_fuota:
        {
            p = state_.flash_addr_ + CTX_FUOTA_STACK_START + offset;          
            break;
        }
        case hal_flash_ctx_str_fwd:
        {
            p = state_.flash_addr_ + CTX_STORE_AND_FWD_START + offset;           
            break;
        }
        default:
            return;
            break;        
    };  

    memcpy(buffer, p, size);   
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
            uint32_t page = offset / FAUX_FLASH_SECTOR_SIZE;
            flash_offset = CTX_FUOTA_STACK_START + page;
            cache_offset = (uint8_t*)(&state_.ctx_cache_.ctx_fuota);                            
            break;
        }
        case hal_flash_ctx_str_fwd:
        {
            memcpy(&state_.ctx_cache_.ctx_str_fwd + offset, buffer, size);
            //compute the sector
            uint32_t page = offset / FAUX_FLASH_SECTOR_SIZE;
            flash_offset = CTX_STORE_AND_FWD_START + page;
            cache_offset = (uint8_t*)(&state_.ctx_cache_.ctx_str_fwd);                             
            break;
        }
        default:
            break;        
    };

    //update flash file
    memcpy(state_.flash_addr_ + flash_offset, cache_offset, size);
}

/*!
*  We don't have a notion of erasing the backing file, so lets fill it
*  with zeros.
*/
void mcu_hal_erase_flash_page(uint32_t flash_offset, uint32_t nb_pages) {
    uint32_t page = flash_offset / FAUX_FLASH_SECTOR_SIZE;
    bzero(state_.flash_addr_ + page, nb_pages*FAUX_FLASH_SECTOR_SIZE);   
}

uint32_t mcu_hal_get_time_in_ms() {
    auto time = time_point_cast<milliseconds>(steady_clock::now());
    return time.time_since_epoch().count();
}

void mcu_hal_sleep_ms(uint32_t ms) {
    std::this_thread::sleep_for(milliseconds(ms));
}
  
int mcu_hal_gpio_get(unsigned int gpio) {
    if((gpio >= GPIO_DIO1) && (gpio <= GPIO_CS)) {
        return gpiod_line_get_value(gpio_info[gpio].line_);
    }
    return -1;
}

int mcu_hal_gpio_set(unsigned int gpio, bool state) {
    if((gpio >= GPIO_DIO1) && (gpio <= GPIO_CS)) {
        return gpiod_line_set_value(gpio_info[gpio].line_, state ? 1 : 0);
    }
    return -1;
}

void mcu_hal_start_timer(uint32_t milliseconds, struct user_data_tm user_data) {
    if(!state_.timer_started_) {
        state_.timer_started_ = true;
        state_.user_data_ = user_data;
        state_.timer_ms_ = milliseconds;
    }     

    time_t secs = milliseconds / 1000;
    long int nsecs = (milliseconds % 1000) * 1000000;

    struct itimerspec ts { {0,0}, { secs, nsecs } };
    if(timer_settime(state_.timer_id_, 0, &ts, nullptr) == -1) {
        std::print("failed to start interval timer for radio hal\n");
    }
}

void mcu_hal_stop_timer() {
    if(state_.timer_started_) {
        state_.timer_started_ = false;
        struct itimerspec ts { {0,0}, {0,0} };
        if(timer_settime(state_.timer_id_, 0, &ts, nullptr) == -1) {
            std::print("failed to start interval timer for radio hal\n");
        }        
    }
}

/**
 *   Disable irqs used by the mode, timer/alarm and dio
 * 
 */ 
void mcu_hal_disable_irqs() {
    //We don't actually stop the timer we just prevent callbacks
    //while "irqs" are disabled
    state_.timer_enabled_ = false;

    set_uio_irq(false);
}

void mcu_hal_enable_irqs() {
    state_.timer_enabled_ = true;

    set_uio_irq(true);

    //was there a pending timer expiration while we had "irq" disabled
    if(state_.user_data_.cb && state_.timer_pending_) {
        state_.user_data_.cb(state_.user_data_.context);
        state_.timer_pending_ = false;
    }   
}

uint32_t mcu_hal_rng_get_random_in_range(const uint32_t val_1, const uint32_t val_2) {
    if( val_1 <= val_2 ) {
        return ( uint32_t )( ( rand() % ( val_2 - val_1 + 1 ) ) + val_1 );
    } else {
        return ( uint32_t )( ( rand() % ( val_1 - val_2 + 1 ) ) + val_2 );
    }    
}


uint16_t mcu_hal_adc_read(unsigned int channel) {
    return 0;
}

int8_t mcu_hal_read_temp() try {

    if(state_.fd_temp_ == -1) {
        return 0;
    }
    //read the temp value from hwmon for the pi
    std::string value(16, '0');     
    if(auto br = pread(state_.fd_temp_, &value[0], value.size(), 0); br != -1) {
        return stof(value)/1000.0;
    } else {
        return 0;
    }
      
}  catch(std::exception& ex) {
    return 0;
}  

float mcu_hal_read_batt_voltage() {
    return 0.0;
}

void mcu_hal_config_radio_irq(void ( *callback )( void* context ), void* context) {
    //disable interrupts in case we're changing handle
    set_uio_irq(false);
    state_.radio_cb_ = callback;
    state_.radio_cb_ctx_ = context;

    set_uio_irq(true);
}

void mcu_hal_clear_radio_irq(void) {
    //This is handled by pico sdk
}

int spi_transfer(const uint8_t* tx, uint8_t* rx, std::size_t len, uint16_t delay) {
    spi_ioc_transfer xfer = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = len,
    };

    gpiod_line_set_value(state_.gpio_cs_, 0);
    auto result = ioctl(state_.fd_, SPI_IOC_MESSAGE(1), &xfer);
    gpiod_line_set_value(state_.gpio_cs_, 1); 

   return result;
}

sx126x_hal_status_t sx126x_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length ) {


#if 0
    std::print("sx126x_hal_write: CMD [OP: {:#02x}]", command[0]);
    if(command_length > 1) {
        std::print(" ");
        for(int i = 1; i < command_length-1; i++) {
            std::print("{:#02x},", command[i]);
        }
        std::print("{:#02x}]", command[command_length-1]);          
    }

    if(data_length > 0) {
        std::print("\n\tDATA [");
        for(int i = 0; i < data_length-1; i++) {
            std::print("{:#02x},", data[i]);
        }
        std::print("{:#02x}]",data[data_length-1]);        
    }
    std::print("\n");
#endif

    uint8_t tx_buff[255];
    uint8_t rx_buff[255];
    memset(&tx_buff, 0, 255);
    memset(&rx_buff, 0, 255);
    memcpy(&tx_buff, command, command_length);
    memcpy(&tx_buff[command_length], data, data_length); 
    auto xfer_len = command_length + data_length;
    spi_transfer(tx_buff, rx_buff, xfer_len, 0);

    // 0x84 - SX126x_SET_SLEEP opcode. In sleep mode the radio dio is struck to 1 => do not test it
    if( command[0] != 0x84 ) {
        sx126x_hal_check_device_ready();
    } else {
        radio_mode = RADIO_SLEEP;
    }

    return SX126X_HAL_STATUS_OK;    
}

sx126x_hal_status_t sx126x_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length ) {
    sx126x_hal_check_device_ready(); 

    uint8_t tx_buff[255];
    uint8_t rx_buff[255];
    memset(&tx_buff, 0, 255);
    memset(&rx_buff, 0, 255);
    memcpy(&tx_buff, command, command_length);
    auto xfer_len = command_length + data_length;
    spi_transfer(tx_buff, rx_buff, xfer_len, 0);
    memcpy(data, &rx_buff[command_length], data_length);

    sx126x_hal_check_device_ready();

#if 0
    std::print("sx126x_hal_read: CMD [OP: {:#02x}]", command[0]);
    if(command_length > 1) {
        std::print(" ");
        for(int i = 1; i < command_length-1; i++) {
            std::print("{:#02x},", command[i]);
        }
        std::print("{:#02x}]", command[command_length-1]);          
    }

    if(data_length > 0) {
        std::print("\n\tDATA [");
        for(int i = 0; i < data_length-1; i++) {
            std::print("{:#02x},", data[i]);
        }
        std::print("{:#02x}]",data[data_length-1]);        
    }
    std::print("\n");
#endif

    return SX126X_HAL_STATUS_OK;  
}

sx126x_hal_status_t sx126x_hal_reset( const void* context ) {
    gpiod_line_set_value(state_.gpio_reset_, 0);
    usleep(10000);
    gpiod_line_set_value(state_.gpio_reset_, 1);
    usleep(10000);
    radio_mode = RADIO_AWAKE;
    
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup( const void* context ) {
    sx126x_hal_check_device_ready();
    
    return SX126X_HAL_STATUS_OK;
}


static void sx126x_hal_wait_on_busy( void )
{
    while(gpiod_line_get_value(state_.gpio_busy_) == true)
    {
    };
}

/*
*  Lower CS(NSS) to bring the device out of sleep mode
*/
static void sx126x_hal_check_device_ready( void )
{
    if( radio_mode != RADIO_SLEEP ) {
        sx126x_hal_wait_on_busy();
    } else {
        gpiod_line_set_value(state_.gpio_cs_, 0);
        sx126x_hal_wait_on_busy( );
        gpiod_line_set_value(state_.gpio_cs_, 1);
        radio_mode = RADIO_AWAKE;
    }
}