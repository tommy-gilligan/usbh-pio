cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(coverage_poc)
pico_sdk_init()

add_executable(coverage_poc)

target_sources(coverage_poc PRIVATE main.c)

target_link_libraries(coverage_poc PRIVATE
        pico_stdlib
        )

pico_enable_stdio_usb(coverage_poc 1)
pico_enable_stdio_uart(coverage_poc 0)
pico_add_extra_outputs(coverage_poc)
