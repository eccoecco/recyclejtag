cmake_minimum_required(VERSION 3.14)

project(rjcore)

add_subdirectory(../../librjcore librjcore)

add_executable(rjcore
    main.cpp
)
target_link_libraries(rjcore
    librjcore
)