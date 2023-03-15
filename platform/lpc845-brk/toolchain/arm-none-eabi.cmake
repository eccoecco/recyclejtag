if(NOT DEFINED ENV{LPC845_SDK_DIR})
    message(FATAL_ERROR "Need to set LPC845_SDK_DIR in the environment")
endif()

set(LPC845_SDK_DIR "$ENV{LPC845_SDK_DIR}" CACHE INTERNAL "Path to MCUXpresso SDK for LPC845")

include("${LPC845_SDK_DIR}/tools/cmake_toolchain_files/armgcc.cmake")