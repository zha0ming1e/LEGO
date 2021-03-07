cmake_minimum_required(VERSION 2.8)

add_subdirectory(base)
add_subdirectory(slam)

# examples
if(LEGO_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif(LEGO_BUILD_EXAMPLES)
