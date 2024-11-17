cmake_minimum_required(VERSION 3.30.5 FATAL_ERROR)

if(${PLATFORM_BOARD} STREQUAL PICO2  OR ${PLATFORM_BOARD} STREQUAL PICO)
    include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
endif()

project(ping_pong LANGUAGES ASM C CXX)

add_executable(ping_pong ping_pong.cpp)

add_dependencies(ping_pong lora_basics_modem_lib)

if(${PLATFORM_BOARD} STREQUAL PICO2  OR ${PLATFORM_BOARD} STREQUAL PICO)
    pico_sdk_init()
    pico_enable_stdio_usb(ping_pong 1)
    pico_enable_stdio_uart(ping_pong 0)
    pico_add_extra_outputs(ping_pong)
    set(ADD_LIBS pico_stdlib hardware_spi hardware_sync hardware_adc pico_rand)     
elseif(${PLATFORM_BOARD} STREQUAL LINUX)
    set(ignore "${PICO_BOARD}${PICO_PLATFORM}")
    set(ADD_LIBS "gpiod")     
else()
    message(FATAL_ERROR "You must define a PLATFORM_BOARD [PICO|PICO2|LINUX]")
endif()

target_sources(ping_pong PUBLIC ${HAL_SRCS})
target_include_directories(ping_pong PRIVATE ${HAL_INCLUDES} ${MODEM_LIB_INCLUDES})
target_link_libraries(ping_pong ${ADD_LIBS} ${PLATFORM_LIBS} ${CMAKE_BINARY_DIR}/stage/basic_modem.a)     

install(TARGETS ping_pong DESTINATION bin)