cmake_minimum_required(VERSION 3.0.2)

project(nturt_can_parser)

find_package(catkin REQUIRED COMPONENTS
        can_msgs
        roscpp
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS can_msgs roscpp
)

include_directories(
        include
        #test
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/can_parser.cpp
        src/yaml_loader.cpp
        #test/test.cpp
)

add_executable(${PROJECT_NAME}_node
        app/can_parser_node.cpp
)

add_executable(yaml_loader_test
        test/yaml_loader_test.cpp
)

add_executable(can_parser_main
        src/can_parser.cpp
)
#add_executable(can_parser_test
#        test/can_parser_test.cpp
#)
# add_executable(tttt
#         test/test.cpp
# )

target_link_libraries(${PROJECT_NAME}
        yaml-cpp
)

target_link_libraries(${PROJECT_NAME}
        yaml-cpp
)
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
)

target_link_libraries(yaml_loader_test
        ${PROJECT_NAME}
)

target_link_libraries(can_parser_main
        ${PROJECT_NAME}
)
#target_link_libraries(can_parser_test
#        ${PROJECT_NAME}
#)
#target_link_libraries(tttt
#        ${PROJECT_NAME}
#)
link_directories(${PROJECT_SOURCE_DIR/lib/yaml-cpp})
