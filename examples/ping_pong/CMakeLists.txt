cmake_minimum_required(VERSION 3.20.0 FATAL_ERROR)

if(${PLATFORM_BOARD} STREQUAL PICO2 OR ${PLATFORM_BOARD} STREQUAL PICO  OR ${PLATFORM_BOARD} STREQUAL RP2040_LORA)
    include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
endif()

project(ping_pong LANGUAGES ASM C CXX)

add_executable(ping_pong ping_pong.cpp)

add_dependencies(ping_pong lora_basics_modem_lib)

if(${PLATFORM_BOARD} STREQUAL PICO2  OR ${PLATFORM_BOARD} STREQUAL PICO OR ${PLATFORM_BOARD} STREQUAL RP2040_LORA)
    pico_sdk_init()
    pico_enable_stdio_usb(ping_pong 1)
    pico_enable_stdio_uart(ping_pong 0)
    pico_add_extra_outputs(ping_pong)
    set(ADD_LIBS pico_stdlib hardware_spi hardware_i2c hardware_sync hardware_adc pico_rand)     
elseif(${PLATFORM_BOARD} STREQUAL LINUX)
    set(ignore "${PICO_BOARD}${PICO_PLATFORM}")
    set(ADD_LIBS "gpiod")     
else()
    message(FATAL_ERROR "You must define a PLATFORM_BOARD [PICO|PICO2|RP2040_LORA|LINUX]")
endif()

target_include_directories(ping_pong PRIVATE ${lora_basic_modem_ports_INCLUDE_DIRS})
target_link_libraries(ping_pong ${ADD_LIBS} ${lora_basic_modem_ports_LIBRARY_DIRS}/basic_modem.a ${lora_basic_modem_ports_LIBRARY_DIRS}/liblora_basics_modem_hal.a)     

install(TARGETS ping_pong DESTINATION bin)