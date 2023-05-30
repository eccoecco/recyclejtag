if(NOT DEFINED ENV{LPC845_SDK_DIR})
    message(FATAL_ERROR "Need to set LPC845_SDK_DIR in the environment")
endif()

cmake_path(SET LPC845_SDK_DIR "$ENV{LPC845_SDK_DIR}")

include("${LPC845_SDK_DIR}/tools/cmake_toolchain_files/armgcc.cmake")