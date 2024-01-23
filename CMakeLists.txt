# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/leon/Programs/pico/pico-sdk")

set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
include(pimoroni_pico_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(SDCardSource C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()


add_subdirectory(no-os-fatfs/src build) # add the FatFS library

# Add executable. Default name is the project name, version 0.1

add_executable(SDCardSource SDCardSource.cpp i2c.cpp comInterface.cpp)

pico_set_program_name(SDCardSource "SDCardSource")
pico_set_program_version(SDCardSource "0.2")

pico_enable_stdio_uart(SDCardSource 1)
pico_enable_stdio_usb(SDCardSource 0)

# Add the standard library to the build
target_link_libraries(SDCardSource
        pico_stdlib
        no-OS-FatFS-SD-SDIO-SPI-RPi-Pico)

# Add the standard include files to the build
target_include_directories(SDCardSource PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
  /home/leon/Programs/FreeThetics/Sources/SDCardSource/no-os-fatfs/src/..
  /home/leon/Programs/FreeThetics/Sources/SDCardSource/no-os-fatfs/src
  ${CMAKE_CURRENT_LIST_DIR}/no-os-fatfs/src/include
)

include(libraries/pico_unicorn/pico_unicorn)
# Add any user requested libraries
target_link_libraries(SDCardSource 
        hardware_spi
        hardware_i2c
        hardware_dma
        hardware_pio
        hardware_interp
        hardware_timer
        hardware_watchdog
        hardware_clocks
        pico_multicore
        pico_stdlib 
        )

pico_add_extra_outputs(SDCardSource)

