cmake_minimum_required(VERSION 3.20.0 FATAL_ERROR)

if(${PLATFORM_BOARD} STREQUAL PICO2  OR ${PLATFORM_BOARD} STREQUAL PICO OR ${PLATFORM_BOARD} STREQUAL RP2040_LORA)
    include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
endif()

project(otaa_uplink LANGUAGES ASM C CXX)

#cmake treats uf2 like an archive..
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/stage/bin)

add_executable(otaa_example otaa_example.cpp)
add_dependencies(otaa_example lora_basics_modem_lib)

if(${PLATFORM_BOARD} STREQUAL PICO2 OR ${PLATFORM_BOARD} STREQUAL PICO OR ${PLATFORM_BOARD} STREQUAL RP2040_LORA)
    pico_sdk_init()
    pico_enable_stdio_usb(otaa_example 1)
    pico_enable_stdio_uart(otaa_example 0)
    pico_add_extra_outputs(otaa_example)
    set(ADD_LIBS pico_stdlib) 
elseif(${PLATFORM_BOARD} STREQUAL LINUX)
    set(ignore "${PICO_BOARD}${PICO_PLATFORM}")
    set(ADD_LIBS gpiod)        
else()
    message(FATAL_ERROR "You must define a PLATFORM_BOARD [PICO|PICO2|RP2040_LORA|LINUX]")
endif()

target_include_directories(otaa_example PRIVATE ${lora_basic_modem_ports_INCLUDE_DIRS})
target_link_libraries(otaa_example ${lora_basic_modem_ports_LIBRARY_DIRS}/basic_modem.a ${lora_basic_modem_ports_LIBRARY_DIRS}/liblora_basics_modem_hal.a ${ADD_LIBS})     

add_custom_command(TARGET otaa_example POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:otaa_example> ${CMAKE_BINARY_DIR}/stage/bin)

#only applicable to linux/buildroot
install(TARGETS otaa_example DESTINATION bin)