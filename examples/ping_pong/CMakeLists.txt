cmake_minimum_required(VERSION 3.30.5 FATAL_ERROR)

if(${PLATFORM_BOARD} STREQUAL PICO2  OR ${PLATFORM_BOARD} STREQUAL PICO)
    include(/usr/share/pico-sdk/external/pico_sdk_import.cmake)
endif()

project(ping_pong LANGUAGES ASM C CXX)

#disable abi warning about gcc 9.1 changes for std::print
add_compile_options(-Wno-psabi)

add_executable(ping_pong ping_pong.cpp)

if(${PLATFORM_BOARD} STREQUAL PICO2  OR ${PLATFORM_BOARD} STREQUAL PICO)
    pico_sdk_init()
    pico_enable_stdio_usb(ping_pong 1)
    pico_enable_stdio_uart(ping_pong 0)
    pico_add_extra_outputs(ping_pong)
elseif(${PLATFORM_BOARD} STREQUAL ZERO_2W)
    message(STATUS "Building for ZERO 2W")
    set(ADD_LIBS "gpiod")     
else()
    message(FATAL_ERROR "You must define a PLATFORM_BOARD [PICO|PICO2|ZERO_2W]")
endif()

target_sources(ping_pong PUBLIC ${HAL_SRCS})
target_include_directories(ping_pong PRIVATE ${HAL_INCLUDES} ${MODEM_LIB_INCLUDES})
target_link_libraries(ping_pong ${ADD_LIBS} ${PLATFORM_LIBS} ${pico_lora_basic_modem_lib_DIR}/build/basic_modem.a)     

