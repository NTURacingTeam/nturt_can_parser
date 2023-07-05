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

# executable
# application: socket_can_receiver_node
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
# application: socket_can_sender_node
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

# test: monitor_can_node
add_executable(monitor_can_node
        test/monitor_can_node.cpp
)
ament_target_dependencies(monitor_can_node
        can_msgs
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
install(TARGETS socket_can_receiver_node socket_can_sender_node monitor_can_node
        DESTINATION lib/${PROJECT_NAME}
)

# install bash, python scripts
install(DIRECTORY scripts/
        DESTINATION lib/${PROJECT_NAME}
)

# install launch scripts
install(DIRECTORY launch/
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
        rclcpp_components
        rclpy
        ros2_socketcan
)

ament_package()
