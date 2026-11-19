cmake_minimum_required(VERSION 3.13)

add_compile_definitions(PICO_XOSC_STARTUP_DELAY_MULTIPLIER=128)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(my_project LANGUAGES C CXX)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# ====================================================================
# 1. ZIEL ERSTELLEN (MUSS HIER STEHEN!)
# ====================================================================
add_executable(oXs)

# ====================================================================
# 2. PROTO_SELECT AUTOMATISCH AUS CONFIG.H AUSLESEN
# ====================================================================
file(STRINGS "${CMAKE_CURRENT_LIST_DIR}/src/config.h" CONFIG_LINE REGEX "^#define PROTO_SELECT")
string(REGEX MATCH "[0-9]+" PROTO_SELECT "${CONFIG_LINE}")

# Falls kein Wert in der config.h steht, nimm Modus 1 als Rückfallebene
if(NOT PROTO_SELECT)
    set(PROTO_SELECT 10)
endif()

# Den Namen der Ausgabedatei basierend auf der config.h festlegen
if(PROTO_SELECT EQUAL 1)
    set(OUTPUT_NAME "LTM_to_Jeti")
elseif(PROTO_SELECT EQUAL 2)
    set(OUTPUT_NAME "LTM_to_MPX")
elseif(PROTO_SELECT EQUAL 3)
    set(OUTPUT_NAME "LTM_to_HoTT")
elseif(PROTO_SELECT EQUAL 4)
    set(OUTPUT_NAME "FrSky_2_HoTT")
elseif(PROTO_SELECT EQUAL 5)
    set(OUTPUT_NAME "FrSky_2_CRSF")
elseif(PROTO_SELECT EQUAL 6)
    set(OUTPUT_NAME "FrSky_2_MPX")
elseif(PROTO_SELECT EQUAL 7)
    set(OUTPUT_NAME "FrSky_2_Jet")
elseif(PROTO_SELECT EQUAL 10)
    set(OUTPUT_NAME "RTH_CRSF")
endif()

# Weist dem Ziel den neuen dynamischen Namen zu
set_target_properties(oXs PROPERTIES OUTPUT_NAME ${OUTPUT_NAME})

# Zwingt CMake dazu, sich neu zu konfigurieren, wenn die config.h geändert wird
set_property(DIRECTORY APPEND PROPERTY 
    CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/src/config.h"
)

# ====================================================================
# 3. PIO HEADER GENERIERUNG & QUELLCODE
# ====================================================================
pico_generate_pio_header(oXs ${CMAKE_CURRENT_LIST_DIR}/src/uart_crsf_tx.pio )
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_gps_rx.pio )
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_gps_tx.pio )
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_hott_tx_rx.pio )
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_ibus_tx_rx.pio )
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_jeti_tx.pio )
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_mpx_tx_rx.pio )
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_sbus_tx.pio )
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_sport_tx_rx.pio )
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_srxl2_tx_rx.pio )
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_exbus_tx_rx.pio )
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_logger_tx.pio )
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_esc_rx.pio )
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_frsky_hub_tx.pio)
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_frsky_master.pio)

file(GLOB oXs_SRC CONFIGURE_DEPENDS "src/*.h" "src/*.cpp")
target_sources(oXs PRIVATE ${oXs_SRC})

target_link_libraries(oXs PRIVATE
        pico_stdlib
        pico_multicore
        hardware_pio
        hardware_i2c
        hardware_watchdog
        hardware_timer
        hardware_dma
        hardware_irq
        hardware_adc
        hardware_pwm
        hardware_spi
        )

#enable printf on usb
pico_enable_stdio_usb(oXs 1)
pico_enable_stdio_uart(oXs 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(oXs)

# ====================================================================
# 4. POST-BUILD KOPIERBEFEHLE
# ====================================================================
add_custom_command(TARGET oXs POST_BUILD
  COMMAND ${CMAKE_COMMAND}
  ARGS -E copy ${OUTPUT_NAME}.uf2 ${CMAKE_SOURCE_DIR}   
)

# ====================================================================
# 3. UF2-DATEI NACH DEM BUILD AUTOMATISCH IN DEN CONFIG-ORDNER KOPIEREN
# ====================================================================
add_custom_command(TARGET oXs POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "D:/Ardunio/Inav2Telemetrie/oXs_on_RP2040-main/oXs_on_RP2040-main - SER/Config"
    COMMAND ${CMAKE_COMMAND} -E copy 
        "${CMAKE_CURRENT_BINARY_DIR}/${OUTPUT_NAME}.uf2" 
        "D:/Ardunio/Inav2Telemetrie/oXs_on_RP2040-main/oXs_on_RP2040-main - SER/Config/${OUTPUT_NAME}.uf2"
    COMMENT "Kopiere fertige UF2-Datei in den Config-Ordner..."
)

add_custom_command(TARGET oXs POST_BUILD 
  COMMAND ${CMAKE_COMMAND} 
  ARGS -E copy ${OUTPUT_NAME}.uf2 E:
)

