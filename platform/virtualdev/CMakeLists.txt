cmake_minimum_required(VERSION 3.14)

project(rjcore)

add_subdirectory(../../librjcore librjcore)

add_executable(rjtag
    main.cpp
)
target_link_libraries(rjtag
    librjcore
)