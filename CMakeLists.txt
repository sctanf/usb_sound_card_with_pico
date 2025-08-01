# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_BOARD pico CACHE STRING "Board type")
set(FAMILY rp2040 CACHE STRING "Board family")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(${PICO_SDK_PATH}/lib/tinyusb/hw/bsp/family_support.cmake)

project(usb_sound_with_pico)

# Initialise the Raspberry Pi Pico SDK
# pico_sdk_init()
family_initialize_project(usb_sound_with_pico ${CMAKE_CURRENT_LIST_DIR})

# Add executable. Default name is the project name, version 0.1
add_executable(usb_sound_with_pico)

pico_set_program_name(usb_sound_with_pico usb_sound_with_pico)
pico_set_program_version(usb_sound_with_pico "0.1")

pico_enable_stdio_uart(usb_sound_with_pico 0)
pico_enable_stdio_usb(usb_sound_with_pico 0)

pico_generate_pio_header(usb_sound_with_pico ${CMAKE_CURRENT_LIST_DIR}/pio/audio_i2s_32_out.pio)
pico_generate_pio_header(usb_sound_with_pico ${CMAKE_CURRENT_LIST_DIR}/pio/pulse_out.pio)

# Example source
target_sources(usb_sound_with_pico PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/device_main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/usb_descriptors.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/streaming.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/support.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/job_queue.cpp
)

# Add the standard include files to the build
target_include_directories(usb_sound_with_pico PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/src
)

# Add the standard library to the build
target_link_libraries(usb_sound_with_pico
        PRIVATE pico_stdlib
        PRIVATE pico_multicore)
# Add any user requested libraries
target_link_libraries(usb_sound_with_pico
        PRIVATE hardware_dma
        PRIVATE hardware_pio
        PRIVATE hardware_pwm
        PRIVATE hardware_interp)

# pico_add_extra_outputs(usbdac)
family_configure_device_example(usb_sound_with_pico noos)

target_compile_options(usb_sound_with_pico PUBLIC
  -Wno-unused-variable 
  -Wno-unused-but-set-variable
  -Wno-unused-function
  -Wno-unused-parameter
  -Wno-suggest-attribute=format
  -Wno-type-limits
  -Wno-conversion
  -Wno-sign-compare
  -Wno-shadow
  -Wno-cast-qual
  -Wno-cast-align
  -Wno-redundant-decls
  )
