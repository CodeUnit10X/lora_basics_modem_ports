cmake_minimum_required(VERSION 3.22.0)

#handle pico sdk, its picky about order in which things happen
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

include(ExternalProject)

set(CMAKE_C_STANDARD 23)
set(CMAKE_CXX_STANDARD 23)

#set target radio and region
set(RADIO_TARGET sx1262)

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

set(STAGED_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/stage)

file(MAKE_DIRECTORY ${STAGED_INSTALL_PREFIX})


include(FetchContent)

FetchContent_Declare(
  lora_basics_modem_lib
  GIT_REPOSITORY https://github.com/Lora-net/SWL2001.git
  GIT_TAG        v4.5.0
  SOURCE_DIR ${CMAKE_BINARY_DIR}/subprojects/Source
)

FetchContent_MakeAvailable(lora_basics_modem_lib)


add_custom_target(lora_basics_modem_lib COMMAND cd ${CMAKE_BINARY_DIR}/subprojects/Source/lbm_lib && 
                                                make VERBOSE=no basic_modem_${RADIO_TARGET} MCU_FLAGS=${CMAKE_C_FLAGS} REGION=${RADIO_REGION} MODEM_TRACE=no MODEM_DEEP_TRACE=no CC=${CMAKE_C_COMPILER} &&
                                                cp ${CMAKE_BINARY_DIR}/subprojects/Source/lbm_lib/build/*.a ${STAGED_INSTALL_PREFIX} &&
                                                cp ${CMAKE_BINARY_DIR}/subprojects/Source/lbm_lib/smtc_modem_api/*.h ${STAGED_INSTALL_PREFIX})

set_property(DIRECTORY PROPERTY EP_BASE ${CMAKE_BINARY_DIR}/subprojects)
set_property(DIRECTORY PROPERTY STAGED_INSTALL_PREFIX ${STAGED_INSTALL_PREFIX})
set_property(GLOBAL PROPERTY RADIO_TARGET ${RADIO_TARGET})
set_property(GLOBAL PROPERTY RADIO_REGION ${RADIO_REGION})

set(pico_lora_basic_modem_lib_DIR ${CMAKE_BINARY_DIR}/subprojects/Source/lbm_lib CACHE PATH "Path to modem library" FORCE)
set(HAL_SRCS ${hal_sources} CACHE PATH "Path to modem library" FORCE)
set(HAL_INCLUDES ${hal_includes} CACHE PATH "Path to modem library" FORCE)

#common modem library includes and library paths
set(modem_lib_includes ${pico_lora_basic_modem_lib_DIR}/smtc_modem_core/radio_drivers/sx126x_driver/src
                       ${pico_lora_basic_modem_lib_DIR}/smtc_modem_core/smtc_ral/src/
                       ${pico_lora_basic_modem_lib_DIR}/smtc_modem_core/smtc_ralf/src/
                       ${pico_lora_basic_modem_lib_DIR}/smtc_modem_hal
                       ${pico_lora_basic_modem_lib_DIR}/smtc_modem_api)

set(MODEM_LIB_INCLUDES ${modem_lib_includes} CACHE PATH "Path to modem library" FORCE)


add_subdirectory(examples/lorawan/otaa_example)
add_subdirectory(examples/ping_pong)
#add_subdirectory(test)