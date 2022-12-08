cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(Pico-Flash-Utility LANGUAGES C CXX ASM)

# set (CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")
set (C_STANDARD 11)
set (CXX_STANDARD 17)
set (PICO_BOARD pico)

pico_sdk_init()


add_executable(Pico-Flash-Utility Pico-Flash-Utility.c)


# Send Pico's output to USB instead of UART (for debug purposes).
pico_enable_stdio_uart(Pico-Flash-Utility 1)
pico_enable_stdio_usb(Pico-Flash-Utility  1)

# The firmware version for the Pico W is too large to fit into RAM memory space...
# Regular Pico version does fit though, so build a Pico version (that will also run on the Pico W)...
pico_set_binary_type(Pico-Flash-Utility blocked_ram)  # this instruction, for some reason, will put garbage all over flash memory space... do not use

# Create map/bin/hex file etc.
pico_add_extra_outputs(Pico-Flash-Utility)


# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(Pico-Flash-Utility hardware_adc hardware_flash hardware_sync pico_stdlib pico_unique_id pico_multicore)
# Has been removed from libraries on the line above:  pico_cyw43_arch_none 

# add url via pico_set_program_url
# example_auto_set_url(Pico-Clock-Green)
