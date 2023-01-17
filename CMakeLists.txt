cmake_minimum_required(VERSION 3.8)
project(nturt_can_parser)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(can_msgs REQUIRED)
find_package(nturt_realtime_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros2_socketcan REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
        include
)

# libraries
# library: ros2_socketcan_node
add_library(ros2_socketcan_node SHARED
        src/socket_can_receiver_node.cpp
        src/socket_can_sender_node.cpp
)
ament_target_dependencies(ros2_socketcan_node
        can_msgs
        rclcpp
        rclcpp_components
        ros2_socketcan
)
# register to node components
# node component: SocketCanSenderNode
rclcpp_components_register_nodes(ros2_socketcan_node "drivers::socketcan::SocketCanSenderNode")
set(node_plugins "${node_plugins}drivers::socketcan::SocketCanSenderNode;$<TARGET_FILE:ros2_socketcan_node>\n")
# node component: SocketCanReceiverNode
rclcpp_components_register_nodes(ros2_socketcan_node "drivers::socketcan::SocketCanReceiverNode")
set(node_plugins "${node_plugins}drivers::socketcan::SocketCanReceiverNode;$<TARGET_FILE:ros2_socketcan_node>\n")

# applications
# executable: ros2_socket_can_node
add_executable(ros2_socket_can_node
        app/ros2_socket_can_node.cpp
)
target_link_libraries(ros2_socket_can_node
        ros2_socketcan_node
)
ament_target_dependencies(ros2_socket_can_node
        nturt_realtime_utils
        rclcpp
)

# install headers
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
)

# install node components
install(TARGETS ros2_socketcan_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# install applications, test executables
install(TARGETS ros2_socket_can_node
        DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        # the following line skips the linter which checks for copyrights
        # comment the line when a copyright and license is added to all source files
        set(ament_cmake_copyright_FOUND TRUE)
        # the following line skips cpplint (only works in a git repo)
        # comment the line when this package is in a git repo and when
        # a copyright and license is added to all source files
        set(ament_cmake_cpplint_FOUND TRUE)
        ament_lint_auto_find_test_dependencies()
endif()

# export package dependencies
ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_export_dependencies(
        ament_cmake
        can_msgs
        nturt_realtime_utils
        rclcpp
        ros2_socketcan
)

ament_package()

# catkin_package(
#         INCLUDE_DIRS include
#         LIBRARIES ${PROJECT_NAME}
#         CATKIN_DEPENDS can_msgs nturt_ros_interface roscpp
# )

# include_directories(
#         include
#         lib/isotp/include
#         ${Boost_INCLUDE_DIRS}
#         ${catkin_INCLUDE_DIRS}
#         ${YAML_CPP_INCLUDE_DIRS}
# )

# # libraries
# # library: nturt_can_parser
# add_library(${PROJECT_NAME}
#         src/can_parser.cpp
#         src/yaml_loader.cpp
#         src/can_handler.cpp
# )

# target_link_libraries(${PROJECT_NAME}
#         ${catkin_LIBRARIES}
#         ${Boost_LIBRARIES}
#         yaml-cpp
# )

# # build this package after "nturt_ros_interface" has already finished generating
# add_dependencies(${PROJECT_NAME}
#         nturt_ros_interface_generate_messages_cpp)

# # applications
# # executable: nturt_can_parser_node
# add_executable(${PROJECT_NAME}_node
#         app/can_parser_node.cpp
# )

# target_link_libraries(${PROJECT_NAME}_node
#         ${catkin_LIBRARIES}
#         ${PROJECT_NAME}
# )

# # tests
# # executable: can_handler_test
# add_executable(nturt_can_handler_test
#         test/can_handler_test.cpp
# )

# target_link_libraries(nturt_can_handler_test
#         ${catkin_LIBRARIES}
#         ${PROJECT_NAME}
# )

# # executable: can_parser_test
# add_executable(can_parser_test
#         test/can_parser_test.cpp
# )

# target_link_libraries(can_parser_test
#         ${PROJECT_NAME}
# )

# # executable: fake_can_signal_generator
# add_executable(fake_can_signal_generator
#         test/fake_can_signal_generator.cpp
# )

# target_link_libraries(fake_can_signal_generator
#         ${PROJECT_NAME}
# )

# # executable: isotp_receive_test
# add_executable(isotp_receive_test
#         test/isotp_receive_test.cpp
# )

# target_link_libraries(isotp_receive_test
#         ${catkin_LIBRARIES}
#         isotp
# )

# # executable: isotp_send_test
# add_executable(isotp_send_test
#         test/isotp_send_test.cpp
# )

# target_link_libraries(isotp_send_test
#         ${catkin_LIBRARIES}
#         isotp
# )

# # executable: yaml_loader_test
# add_executable(yaml_loader_test
#         test/yaml_loader_test.cpp
# )

# target_link_libraries(yaml_loader_test 
#         ${PROJECT_NAME}
# )
