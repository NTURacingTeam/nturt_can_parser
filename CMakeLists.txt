cmake_minimum_required(VERSION 3.0.2)
project(nturt_can_parser)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
        can_msgs
        message_generation
        roscpp
)

# custom interfaces
# message
add_message_files(FILES
        SendCanData.msg
)

# service
add_service_files(FILES
        GetCanData.srv
)

generate_messages(DEPENDENCIES
        std_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS can_msgs message_generation roscpp
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
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
# executable: yaml_loader_test
add_executable(yaml_loader_test
        test/yaml_loader_test.cpp
)

target_link_libraries(yaml_loader_test
        ${PROJECT_NAME}
)

# executable: can_parser_test
add_executable(can_parser_test
       test/can_parser_test.cpp
)

target_link_libraries(can_parser_test
       ${PROJECT_NAME}
)
