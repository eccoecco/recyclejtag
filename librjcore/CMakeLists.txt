# Keep this project minimal, so that other project types can just include a single file
add_library(librjcore STATIC
    src/rjcore.c
)
target_include_directories(librjcore PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Make all derivative projects inherit the warning flags, as a shortcut
if(MSVC)
    # Unlikely, but... just in case
    target_compile_options(librjcore PUBLIC /W4 /WX)
else()
    target_compile_options(librjcore PUBLIC -Wall -Wextra -Wpedantic -Werror)
endif()