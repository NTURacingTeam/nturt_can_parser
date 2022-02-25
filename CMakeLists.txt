cmake_minimum_required(VERSION 3.0.2)

project(nturt_can_parser)

find_package(catkin REQUIRED)

include_directories(inc)

catkin_package(
   INCLUDE_DIRS inc
   LIBRARIES ${PROJECT_NAME}
)

add_executable(parser_test src/NTURT_CAN_Parser.cpp src/main.cpp)
add_library(${PROJECT_NAME} src/NTURT_CAN_Parser.cpp)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

