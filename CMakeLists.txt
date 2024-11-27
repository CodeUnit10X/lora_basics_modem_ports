cmake_minimum_required(VERSION 3.22.0)

#handle pico sdk, it's a pain to use and requires includes to be in certain order
if(${PLATFORM_BOARD} STREQUAL PICO OR ${PLATFORM_BOARD} STREQUAL RP2040_LORA)
    set(PICO_PLATFORM rp2040)
    set(PICO_BOARD pico)
    include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)       
elseif(${PLATFORM_BOARD} STREQUAL PICO2)
    set(PICO_PLATFORM rp2350-arm-s)
    set(PICO_BOARD pico2)   
    include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
endif()

project(pico_lora_basic_modem CXX C ASM)

option(BUILD_EXAMPLES "Build example programs" OFF)

set(CMAKE_C_STANDARD 23)
set(CMAKE_CXX_STANDARD 23)

#set target radio and region
set(RADIO_TARGET sx1262)

#silence info message where compiler is telling us that the ABI has changed from gcc9.1
#this only applies if for some reason your linking code built with old compiler...
add_compile_options(-Wno-psabi)

#set the platform board
if(${PLATFORM_BOARD} STREQUAL PICO2 OR ${PLATFORM_BOARD} STREQUAL PICO OR ${PLATFORM_BOARD} STREQUAL RP2040_LORA)

    if(${PLATFORM_BOARD} STREQUAL RP2040_LORA)
       add_compile_definitions(RP2040_LORA)
    else()
       add_compile_definitions(PICO_LORA)
    endif()

    pico_sdk_init()

    set_property(GLOBAL PROPERTY MCU_FLAGS_PICO ${CMAKE_C_FLAGS})
    set_property(GLOBAL PROPERTY PICO_PLATFORM ${PICO_PLATFORM})
    set_property(GLOBAL PROPERTY PICO_BOARD ${PICO_BOARD})

    set(hal_sources hal/mcu_hal.c
                    modem_hal/smtc_modem_hal_pico.c
                    radio_hal/sx126x_pico_radio_hal.c
                    radio_hal/ral_sx126x_bsp.c)

    list(TRANSFORM hal_sources PREPEND "${CMAKE_SOURCE_DIR}/boards/pico/")

    set(hal_includes common 
                     pico/hal)

    list(TRANSFORM hal_includes PREPEND "${CMAKE_SOURCE_DIR}/boards/")

    set(hal_libraries pico_stdlib hardware_spi hardware_i2c hardware_sync hardware_adc pico_rand)

elseif(${PLATFORM_BOARD} STREQUAL LINUX)
    set(ignore "${PICO_BOARD}${PICO_PLATFORM}")

    set(hal_sources hal/mcu_hal.cpp
                    modem_hal/smtc_modem_hal_linux.c
                    radio_hal/ral_sx126x_bsp.c)

    list(TRANSFORM hal_sources PREPEND "${CMAKE_SOURCE_DIR}/boards/linux/")

    set(hal_includes ${CMAKE_SOURCE_DIR}/boards/common ${CMAKE_SOURCE_DIR}/boards/linux/hal)

else()
    message(FATAL_ERROR "You must define a PLATFORM_BOARD [PICO|PICO2|RP2040_LORA|LINUX]")
endif()

set_property(DIRECTORY PROPERTY EP_BASE ${CMAKE_BINARY_DIR}/subprojects)
set_property(GLOBAL PROPERTY RADIO_TARGET ${RADIO_TARGET})
set_property(GLOBAL PROPERTY RADIO_REGION ${RADIO_REGION})

set(STAGED_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/stage)

file(MAKE_DIRECTORY ${STAGED_INSTALL_PREFIX})
file(MAKE_DIRECTORY ${STAGED_INSTALL_PREFIX}/lib)
file(MAKE_DIRECTORY ${STAGED_INSTALL_PREFIX}/include)
file(MAKE_DIRECTORY ${STAGED_INSTALL_PREFIX}/bin)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/stage/lib)

include(FetchContent)

FetchContent_Declare(
  lora_basics_modem_lib
  GIT_REPOSITORY https://github.com/Lora-net/SWL2001.git
  GIT_TAG        v4.5.0
  SOURCE_DIR ${CMAKE_BINARY_DIR}/subprojects/Source
)

FetchContent_MakeAvailable(lora_basics_modem_lib)

add_custom_target(lora_basics_modem_lib COMMAND cd ${CMAKE_BINARY_DIR}/subprojects/Source/lbm_lib && 
                                                make VERBOSE=no basic_modem_${RADIO_TARGET} MCU_FLAGS=${CMAKE_C_FLAGS} REGION=${RADIO_REGION}  MODEM_TRACE=no MODEM_DEEP_TRACE=no CC=${CMAKE_C_COMPILER} &&
                                                cp ${CMAKE_BINARY_DIR}/subprojects/Source/lbm_lib/build/*.a ${STAGED_INSTALL_PREFIX}/lib &&
                                                cp ${CMAKE_BINARY_DIR}/subprojects/Source/lbm_lib/smtc_modem_api/*.h ${STAGED_INSTALL_PREFIX}/include &&
                                                cp ${CMAKE_BINARY_DIR}/subprojects/Source/lbm_lib/smtc_modem_hal/*.h ${STAGED_INSTALL_PREFIX}/include)

set(pico_lora_basic_modem_lib_DIR ${CMAKE_BINARY_DIR}/subprojects/Source/lbm_lib)

#lower level radio comms, see ping_pong you need stmc_modem_core
set(modem_lib_includes ${pico_lora_basic_modem_lib_DIR}/smtc_modem_core/radio_drivers/sx126x_driver/src
                       ${pico_lora_basic_modem_lib_DIR}/smtc_modem_core/smtc_ral/src/
                       ${pico_lora_basic_modem_lib_DIR}/smtc_modem_core/smtc_ralf/src/
                       ${pico_lora_basic_modem_lib_DIR}/smtc_modem_api
                       ${pico_lora_basic_modem_lib_DIR}/smtc_modem_hal                                             
                       ${STAGED_INSTALL_PREFIX}/include
                       ${CMAKE_SOURCE_DIR}/boards/common/)

#create the port hal library
add_library(lora_basics_modem_hal STATIC ${hal_sources})
target_include_directories(lora_basics_modem_hal PRIVATE  ${hal_includes} ${modem_lib_includes})
target_link_libraries(lora_basics_modem_hal ${hal_libraries})
add_dependencies(lora_basics_modem_hal lora_basics_modem_lib)

set(lora_basic_modem_ports_LIBRARY_DIRS ${CMAKE_BINARY_DIR}/stage/lib CACHE PATH "Path to modem libraries" FORCE)
set(lora_basic_modem_ports_INCLUDE_DIRS ${modem_lib_includes} CACHE PATH "Path to modem hal includes" FORCE)

if(BUILD_EXAMPLES)
    add_subdirectory(examples/lorawan/otaa_example)
    add_subdirectory(examples/ping_pong)

    if(${PLATFORM_BOARD} STREQUAL PICO2 OR ${PLATFORM_BOARD} STREQUAL PICO OR ${PLATFORM_BOARD} STREQUAL RP2040_LORA)
        #add_subdirectory(examples/lorawan/bme680_example)
    endif()
endif(BUILD_EXAMPLES)
