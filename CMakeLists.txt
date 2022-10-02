cmake_minimum_required(VERSION 3.0.2)
project(nturt_can_parser)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
        can_msgs
        nturt_ros_interface
        roscpp
)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS can_msgs nturt_ros_interface roscpp
)

include_directories(
        include
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIRS}
)

# libraries
# libraries: nturt_can_parse
add_library(${PROJECT_NAME}
        src/can_parser.cpp
        src/yaml_loader.cpp
        src/can_handler.cpp
)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        yaml-cpp
)

# application
# executable: nturt_can_parse_node
add_executable(${PROJECT_NAME}_node
        app/can_parser_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
)

# tests
# executable: can_handler_test
add_executable(nturt_can_handler_test
        test/can_handler_test.cpp
)

target_link_libraries(nturt_can_handler_test
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
)

# executable: can_parser_test
add_executable(can_parser_test
       test/can_parser_test.cpp
)

target_link_libraries(can_parser_test
       ${PROJECT_NAME}
)

# executable: yaml_loader_test
add_executable(yaml_loader_test
        test/yaml_loader_test.cpp
)

target_link_libraries(yaml_loader_test 
        ${PROJECT_NAME}
)

add_executable(ly
        test/ly.cpp
)
target_link_libraries(ly
       ${PROJECT_NAME}
)
