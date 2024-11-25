cmake_minimum_required(VERSION 3.20.0 FATAL_ERROR)

if(${PLATFORM_BOARD} STREQUAL PICO2  OR ${PLATFORM_BOARD} STREQUAL PICO OR ${PLATFORM_BOARD} STREQUAL RP2040_LORA)
    include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
endif()

project(otaa_uplink LANGUAGES ASM C CXX)

add_executable(otaa_example otaa_example.cpp)

add_dependencies(otaa_example lora_basics_modem_lib)

if(${PLATFORM_BOARD} STREQUAL PICO2 OR ${PLATFORM_BOARD} STREQUAL PICO OR ${PLATFORM_BOARD} STREQUAL RP2040_LORA)
    pico_sdk_init()
    pico_enable_stdio_usb(otaa_example 1)
    pico_enable_stdio_uart(otaa_example 0)
    pico_add_extra_outputs(otaa_example)
    set(ADD_LIBS pico_stdlib hardware_spi hardware_i2c hardware_sync hardware_adc pico_rand) 
elseif(${PLATFORM_BOARD} STREQUAL LINUX)
    set(ignore "${PICO_BOARD}${PICO_PLATFORM}")
    set(ADD_LIBS gpiod)        
else()
    message(FATAL_ERROR "You must define a PLATFORM_BOARD [PICO|PICO2|RP2040_LORA|LINUX]")
endif()

target_sources(otaa_example PUBLIC ${HAL_SRCS})
target_include_directories(otaa_example PRIVATE  ${HAL_INCLUDES} ${MODEM_LIB_INCLUDES})
target_link_libraries(otaa_example ${ADD_LIBS} ${PLATFORM_LIBS} ${CMAKE_BINARY_DIR}/stage/basic_modem.a)     

install(TARGETS otaa_example DESTINATION bin)