cmake_minimum_required(VERSION 3.0.2)

project(nturt_can_parser)

find_package(catkin REQUIRED)

include_directories(inc)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
        src/can_parser.cpp
)

add_executable(can_parser_test
        test/can_parser_test.cpp
)

target_link_libraries(can_parser_test
${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
