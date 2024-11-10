cmake_minimum_required(VERSION 3.30.5 FATAL_ERROR)

#pico sdk likes to be setup in a specific order
if(${PLATFORM_BOARD} STREQUAL PICO2  OR ${PLATFORM_BOARD} STREQUAL PICO)
    include(/usr/share/pico-sdk/external/pico_sdk_import.cmake)
endif()

project(test_modem_port LANGUAGES ASM C CXX)

add_compile_options(-Wno-psabi)

add_executable(test_modem_port test_modem.cpp tests.cpp)

if(${PLATFORM_BOARD} STREQUAL PICO2 OR ${PLATFORM_BOARD} STREQUAL PICO)
    pico_sdk_init()
    pico_enable_stdio_usb(test_modem_port 1)
    pico_enable_stdio_uart(test_modem_port 0)
    pico_add_extra_outputs(test_modem_port)
elseif(${PLATFORM_BOARD} STREQUAL "ZERO_2W")
    message(STATUS "Building for ZERO 2W")
    set(ADD_LIBS "gpiod")
else()
    message(FATAL_ERROR "You must define a PLATFORM_BOARD [PICO|PICO2|ZERO_2W]")
endif()

target_sources(test_modem_port PUBLIC ${HAL_SRCS})
target_include_directories(test_modem_port PRIVATE  ${HAL_INCLUDES} ${MODEM_LIB_INCLUDES})
target_link_libraries(test_modem_port ${ADD_LIBS} ${PLATFORM_LIBS} ${pico_lora_basic_modem_lib_DIR}/build/basic_modem.a)     


