

cmake_minimum_required(VERSION 3.12)
include(pico_sdk_import.cmake)

project(picovga C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )

add_definitions(-DPICO=1)
add_definitions(-DUSE_MEMORY_POINTER=1)

pico_sdk_init()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(scanvideo)

add_executable(picovga
    main.c
)

# Link libraries
target_link_libraries(picovga
    pico_stdlib
    hardware_timer
    hardware_vreg
    pico_multicore
    hardware_pio
    hardware_dma
    hardware_irq
    scanvideo
)

pico_set_program_name(picovga "picovga")
pico_set_program_version(picovga "0.0.2")


# generate PIO headers
pico_generate_pio_header(picovga ${CMAKE_CURRENT_LIST_DIR}/input.pio)

# Set DEBUG and N_SD_CARDS compile definitions:
target_compile_definitions(picovga PUBLIC DEBUG=1)


# enable usb output, disable uart output
pico_enable_stdio_usb(picovga 1)
pico_enable_stdio_uart(picovga 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(picovga)
#pico_set_binary_type(picovga no_flash)