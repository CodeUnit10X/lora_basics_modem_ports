cmake_minimum_required(VERSION 3.22.0)

#handle pico sdk, its picky about order in which things happen
if(${PLATFORM_BOARD} STREQUAL PICO)
    set(PICO_PLATFORM rp2040)
    set(PICO_BOARD pico)
    include(/usr/share/pico-sdk/external/pico_sdk_import.cmake)       
elseif(${PLATFORM_BOARD} STREQUAL PICO2)
    set(PICO_PLATFORM rp2350-arm-s)
    set(PICO_BOARD pico2)   
    include(/usr/share/pico-sdk/external/pico_sdk_import.cmake)
endif()

project(pico_lora_basic_modem CXX C ASM)

set(CMAKE_C_STANDARD 23)
set(CMAKE_CXX_STANDARD 23)

#set target radio and region
set(RADIO_TARGET sx1262)

#set(RADIO_REGION US_915)

#set the platform board
if(${PLATFORM_BOARD} STREQUAL PICO2 OR ${PLATFORM_BOARD} STREQUAL PICO)
    pico_sdk_init()
    set_property(GLOBAL PROPERTY MCU_FLAGS_PICO ${CMAKE_C_FLAGS})
    set_property(GLOBAL PROPERTY PICO_PLATFORM ${PICO_PLATFORM})
    set_property(GLOBAL PROPERTY PICO_BOARD ${PICO_BOARD})

    set(hal_sources hal/mcu_hal.c
                    modem_hal/smtc_modem_hal_pico.c
                    radio_hal/sx126x_pico_radio_hal.c
                    radio_hal/ral_sx126x_bsp.c)

    list(TRANSFORM hal_sources PREPEND "${CMAKE_SOURCE_DIR}/boards/pico/")

    #work around for external_project limitations
    string(REPLACE ";" "|" hal_sources "${hal_sources}")

    set(hal_includes common 
                     pico/hal)

    list(TRANSFORM hal_includes PREPEND "${CMAKE_SOURCE_DIR}/boards/")

    string(REPLACE ";" "|" hal_includes "${hal_includes}")

    set(platform_libs pico_stdlib hardware_spi hardware_sync hardware_adc pico_rand)

    #work around for external_project limitations
    string(REPLACE ";" "|" platform_libs "${platform_libs}")

    message(STATUS "hal_sources ${hal_sources}")
elseif(${PLATFORM_BOARD} STREQUAL RPI_LINUX)
    set(ignore "${PICO_BOARD}${PICO_PLATFORM}")

    set(hal_sources hal/mcu_hal.cpp
                    modem_hal/smtc_modem_hal_linux.c
                    radio_hal/ral_sx126x_bsp.c)

    list(TRANSFORM hal_sources PREPEND "${CMAKE_SOURCE_DIR}/boards/rpi/")

    #work around for external_project limitations
    string(REPLACE ";" "|" hal_sources "${hal_sources}")

    set(hal_includes ${CMAKE_SOURCE_DIR}/boards/common ${CMAKE_SOURCE_DIR}/boards/rpi/hal)

    string(REPLACE ";" "|" hal_includes "${hal_includes}")
else()

    message(FATAL_ERROR "You must define a PLATFORM_BOARD [PICO|PICO2|RPI_LINUX]")

endif()

#superbuild
set(STAGED_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/stage)
set_property(DIRECTORY PROPERTY EP_BASE ${CMAKE_BINARY_DIR}/subprojects)
set_property(DIRECTORY PROPERTY STAGED_INSTALL_PREFIX ${STAGED_INSTALL_PREFIX})
set_property(GLOBAL PROPERTY RADIO_TARGET ${RADIO_TARGET})
set_property(GLOBAL PROPERTY RADIO_REGION ${RADIO_REGION})

add_subdirectory(external/upstream)

set( pico_lora_basic_modem_lib_DIR ${CMAKE_BINARY_DIR}/subprojects/Source/pico_lora_basic_modem_lib/lbm_lib
CACHE PATH "Path to modem library" FORCE)

#common modem library includes and library paths
set(modem_lib_includes ${pico_lora_basic_modem_lib_DIR}/smtc_modem_core/radio_drivers/sx126x_driver/src
                       ${pico_lora_basic_modem_lib_DIR}/smtc_modem_core/smtc_ral/src/
                       ${pico_lora_basic_modem_lib_DIR}/smtc_modem_core/smtc_ralf/src/
                       ${pico_lora_basic_modem_lib_DIR}/smtc_modem_hal
                       ${pico_lora_basic_modem_lib_DIR}/smtc_modem_api)

string(REPLACE ";" "|" modem_lib_includes "${modem_lib_includes}")

message(STATUS "modem_lib_includes = ${modem_lib_includes}")

message(STATUS "CMAKE_C_COMPILER = ${CMAKE_C_COMPILER}")

include(ExternalProject)

ExternalProject_Add(${PROJECT_NAME}_test
                    DEPENDS
                    pico_lora_basic_modem_lib
                    SOURCE_DIR
                    ${CMAKE_CURRENT_LIST_DIR}/test
                    LIST_SEPARATOR |
                    CMAKE_ARGS 
                     -DCMAKE_C_STANDARD=${CMAKE_C_STANDARD}
                     -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}
                     -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
                     -DPLATFORM_BOARD=${PLATFORM_BOARD}
                     -DPICO_BOARD=${PICO_BOARD}
                     -DPICO_PLATFORM=${PICO_PLATFORM}
                     -DHAL_SRCS=${hal_sources}
                     -DHAL_INCLUDES=${hal_includes}
                     -DPLATFORM_LIBS=${platform_libs}
                     -DMODEM_LIB_INCLUDES=${modem_lib_includes}
                     -Dpico_lora_basic_modem_lib_DIR=${pico_lora_basic_modem_lib_DIR}
                    BUILD_ALWAYS
                     1
                    INSTALL_COMMAND 
                    ""
                    )                                     

ExternalProject_Add(${PROJECT_NAME}_otaa_example
                    DEPENDS
                    pico_lora_basic_modem_lib
                    SOURCE_DIR
                    ${CMAKE_CURRENT_LIST_DIR}/examples/lorawan/otaa_example
                    LIST_SEPARATOR |
                    CMAKE_ARGS 
                        -DCMAKE_C_STANDARD=${CMAKE_C_STANDARD}
                        -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}
                        -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
                        -DPLATFORM_BOARD=${PLATFORM_BOARD}
                        -DPICO_BOARD=${PICO_BOARD}
                        -DPICO_PLATFORM=${PICO_PLATFORM}
                        -DHAL_SRCS=${hal_sources}
                        -DHAL_INCLUDES=${hal_includes}
                        -DPLATFORM_LIBS=${platform_libs}
                        -DMODEM_LIB_INCLUDES=${modem_lib_includes}                                                
                        -Dpico_lora_basic_modem_lib_DIR=${pico_lora_basic_modem_lib_DIR}
                    BUILD_ALWAYS
                        1
                    INSTALL_COMMAND 
                    ""
                    )

ExternalProject_Add(${PROJECT_NAME}_ping_pong
                    DEPENDS
                    pico_lora_basic_modem_lib
                    SOURCE_DIR
                    ${CMAKE_CURRENT_LIST_DIR}/examples/ping_pong
                    LIST_SEPARATOR |    
                    CMAKE_ARGS 
                        -DCMAKE_C_STANDARD=${CMAKE_C_STANDARD}
                        -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}
                        -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
                        -DPLATFORM_BOARD=${PLATFORM_BOARD}
                        -DPICO_BOARD=${PICO_BOARD}
                        -DPICO_PLATFORM=${PICO_PLATFORM}
                        -DHAL_SRCS=${hal_sources}
                        -DHAL_INCLUDES=${hal_includes}
                        -DPLATFORM_LIBS=${platform_libs}
                        -DMODEM_LIB_INCLUDES=${modem_lib_includes}     
                        -Dpico_lora_basic_modem_lib_DIR=${pico_lora_basic_modem_lib_DIR}
                    BUILD_ALWAYS
                        1
                    INSTALL_COMMAND 
                    ""
                    )    