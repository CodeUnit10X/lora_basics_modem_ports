cmake_minimum_required(VERSION 3.30.5 FATAL_ERROR)

#pico sdk likes to be setup in a specific order
if(${PLATFORM_BOARD} STREQUAL PICO2  OR ${PLATFORM_BOARD} STREQUAL PICO)
    include(/usr/share/pico-sdk/external/pico_sdk_import.cmake)
endif()

project(test_modem_port LANGUAGES ASM C CXX)

add_executable(test_modem_port test_modem.cpp tests.cpp)

add_dependencies(test_modem_port lora_basics_modem_lib)


if(${PLATFORM_BOARD} STREQUAL PICO2 OR ${PLATFORM_BOARD} STREQUAL PICO)
    pico_sdk_init()
    pico_enable_stdio_usb(test_modem_port 1)
    pico_enable_stdio_uart(test_modem_port 0)
    pico_add_extra_outputs(test_modem_port)
elseif(${PLATFORM_BOARD} STREQUAL "LINUX")
    set(ignore "${PICO_BOARD}${PICO_PLATFORM}")
    set(ADD_LIBS "gpiod")
else()
    message(FATAL_ERROR "You must define a PLATFORM_BOARD [PICO|PICO2|LINUX]")
endif()

target_sources(test_modem_port PUBLIC ${HAL_SRCS})
target_include_directories(test_modem_port PRIVATE  ${HAL_INCLUDES} ${MODEM_LIB_INCLUDES})
target_link_libraries(test_modem_port ${ADD_LIBS} ${PLATFORM_LIBS} ${CMAKE_BINARY_DIR}/stage/basic_modem.a)     

