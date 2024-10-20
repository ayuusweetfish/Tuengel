cmake_minimum_required(VERSION 3.13)

include(pico-sdk/pico_sdk_init.cmake)
project(cases-subt)
pico_sdk_init()

add_executable(cases-subt)
target_compile_definitions(cases-subt PRIVATE PICO_FLASH_SIZE_BYTES=16777216)
target_include_directories(cases-subt PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)

pico_generate_pio_header(cases-subt ${CMAKE_CURRENT_LIST_DIR}/src/uart.pio)
target_sources(cases-subt PRIVATE src/main.c)
target_link_libraries(cases-subt
  pico_stdio
  pico_stdio_usb
  pico_stdlib
  hardware_dma
  hardware_pio
)
pico_add_extra_outputs(cases-subt)
