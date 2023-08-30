cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zephyr_rjtag)

add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../../librjcore" librjcore)

target_sources(app
    PRIVATE
    src/main.c
    src/platform_impl.c
    src/platform_impl.h
    src/serial_queue.c
    src/serial_queue.h
)

target_link_libraries(app
    PUBLIC
    librjcore
)

#target_compile_definitions(librjcore
#    PUBLIC
#    RJCORE_UART_IDLE_TIMEOUT=10000
#)