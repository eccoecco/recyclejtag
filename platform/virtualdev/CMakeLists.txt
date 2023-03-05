cmake_minimum_required(VERSION 3.14)

project(rjcore)

add_subdirectory(../../librjcore librjcore)

add_executable(rjtag
    main.cpp
    runcore.cpp
)
target_link_libraries(rjtag
    librjcore
)
target_compile_features(rjtag PRIVATE cxx_std_20)

