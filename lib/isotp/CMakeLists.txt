project(isotp)

include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# libraries
# library: isotp
add_library(isotp SHARED
        src/isotp.cpp
)
