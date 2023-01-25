cmake_minimum_required(VERSION 3.8)
project(nturt_can_parser)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(can_msgs REQUIRED)
find_package(nturt_realtime_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros2_socketcan REQUIRED)
find_package(yaml-cpp REQUIRED)

# install python module for this package
ament_python_install_package(${PROJECT_NAME})

include_directories(
        include
)

# libraries
# library: socket_can
add_library(socket_can SHARED
        src/socket_can_receiver.cpp
        src/socket_can_sender.cpp
)
ament_target_dependencies(socket_can
        can_msgs
        rclcpp
        rclcpp_components
        ros2_socketcan
)
# register to node components
# node component: SocketCanSender
rclcpp_components_register_nodes(socket_can "SocketCanSender")
set(node_plugins "${node_plugins}SocketCanSender;$<TARGET_FILE:socket_can>\n")
# node component: SocketCanReceiver
rclcpp_components_register_nodes(socket_can "SocketCanReceiver")
set(node_plugins "${node_plugins}SocketCanReceiver;$<TARGET_FILE:socket_can>\n")

# applications
# executable: socket_can_receiver_node
add_executable(socket_can_receiver_node
        app/socket_can_receiver_node.cpp
)
target_link_libraries(socket_can_receiver_node
        socket_can
)
ament_target_dependencies(socket_can_receiver_node
        nturt_realtime_utils
        rclcpp
)
# executable: socket_can_sender_node
add_executable(socket_can_sender_node
        app/socket_can_sender_node.cpp
)
target_link_libraries(socket_can_sender_node
        socket_can
)
ament_target_dependencies(socket_can_sender_node
        nturt_realtime_utils
        rclcpp
)

# install headers
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
)

# install libraries
install(TARGETS socket_can
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

# install applications, test executables
install(TARGETS socket_can_receiver_node socket_can_sender_node
        DESTINATION lib/${PROJECT_NAME}
)

# install bash, python scripts
install(DIRECTORY scripts/
        DESTINATION lib/${PROJECT_NAME}
)

# install launch scripts
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
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
        ament_cmake_python
        can_msgs
        nturt_realtime_utils
        rclcpp
        rclpy
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
