cmake_minimum_required(VERSION 3.20)

# Have to explicitly enable assembly, so that the startup assembly file is included
project(rjtag
    LANGUAGES C CXX ASM
)

if(NOT CMAKE_CROSSCOMPILING)
    message(FATAL_ERROR "No cross compilation detected.  Did you include the toolchain file?")
endif()

# This is a workaround so that flags.cmake can find LPC845_flash.ld, without needing us to modify the file
set(ProjDirPath "${CMAKE_CURRENT_LIST_DIR}/toolchain/thirdparty" CACHE INTERNAL "Path to linker script")
include("${ProjDirPath}/flags.cmake")

# Use this instead of target_compile_features(rjtag PRIVATE cxx_std_20) because the version
# of CMake (3.22) that I'm using doesn't recognise gcc 12.2.1 and produces an error if I try
# to set the target compiler feature on it.
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 20)

add_library(lpc845_platform STATIC
    "${LPC845_SDK_DIR}/devices/LPC845/gcc/startup_LPC845.S"
    "${LPC845_SDK_DIR}/devices/LPC845/system_LPC845.c"
)
target_include_directories(lpc845_platform PUBLIC
    "${LPC845_SDK_DIR}/CMSIS/Core/Include"
    "${LPC845_SDK_DIR}/devices/LPC845"
)

add_compile_definitions(
    RJCORE_UART_IDLE_TIMEOUT=1000
)

add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../../librjcore" librjcore)

add_executable(rjtag
    gpio.cpp
    init.cpp
    main.cpp
    systick.cpp
    uart.cpp
)
target_link_libraries(rjtag
    lpc845_platform
    librjcore
)
set_target_properties(rjtag PROPERTIES SUFFIX ".elf")
