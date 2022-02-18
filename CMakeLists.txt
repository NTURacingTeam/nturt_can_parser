# This CMakeLists aims at being a package that can be used by several
# other ROS nodes.
# 
# Config.cmake Guide:
# Ref: https://gitlab.kitware.com/cmake/community/-/wikis/doc/tutorials/How-to-create-a-ProjectConfig.cmake-file
# Some notes on useful ENV VAR
# Ref: https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/Useful-Variables
# A chinese note that lists common ENV VAR and functions for CMake
# Ref: https://www.cnblogs.com/xianghang123/p/3556425.html
cmake_minimum_required(VERSION 3.5)

# [OPTIONAL]
# Read version from the package.xml file. 
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/package.xml package_xml_str)
if(NOT package_xml_str MATCHES "<version>([0-9]+.[0-9]+.[0-9]+)</version>")
  message(FATAL_ERROR "Could not parse project version from package.xml. Aborting.")
  return()
endif()

set(NTU_CAN_PARSER_VERSION
  ${CMAKE_MATCH_1})
message("Read version:  ${NTU_CAN_PARSER_VERSION}")

# This will set the project information.
# Ref: https://cmake.org/cmake/help/latest/command/project.html
project(NTU_CAN_PARSER VERSION ${NTU_CAN_PARSER_VERSION} LANGUAGES CXX)

# This will set -g for debugging
# This part is optional.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DLINUX=1 -D_UNIX -D_CONSOLE -g")

# [OPTIONAL]
# Default to C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

# [MUST]
# This will tell CMake what to compile.
# add_libbrary(target sourceA sourceB...)
# Ref: https://cmake.org/cmake/help/latest/command/add_library.html
add_library(NTU_CAN_PARSER 
            src/NTURT_CAN_Parser.cpp inc/NTURT_CAN_Parser.hpp)

add_executable(Parser_Test
            src/NTURT_CAN_Parser.cpp inc/NTURT_CAN_Parser.hpp
            src/main.cpp)

# [MUST]
# This will tell CMake the include path.
# Ref: https://cmake.org/cmake/help/latest/command/target_include_directories.html
target_include_directories(NTU_CAN_PARSER PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# [MUST]
# This will tell CMake where our stuffs will be installed.
# Also, the headers and binarys will be in different places.
# Some values have default value, so I am not sure which part
# must be set up manually.
# Ref: https://cmake.org/cmake/help/latest/command/install.html
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS NTURT_CAN_Parser
  EXPORT  NTURT_CAN_ParserTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS Parser_Test
  # IMPORTANT: Add the Parser_Test executable to the "export-set"
  EXPORT  NTURT_CAN_ParserTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" 
  COMPONENT bin)

# [MUST]
# Create the NTURT_CAN_ParserConfig.cmake so that other package
# can find it using find_package() in CMake.
# Also we didn't do the version config part.
# In other packages' CMakeLists:
# find_package(NTURT_CAN_Parser)
# Ref: https://gitlab.kitware.com/cmake/community/-/wikis/doc/tutorials/How-to-create-a-ProjectConfig.cmake-file

# First we export targets
export(EXPORT NTURT_CAN_ParserTargets
    FILE "${PROJECT_BINARY_DIR}/NTURT_CAN_ParserTargets.cmake")

# Then we create ***Config.cmake
set(CONF_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
configure_file(cmake/NTURT_CAN_ParserConfig.cmake.in
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/NTURT_CAN_ParserConfig.cmake" @ONLY)

# Finally we install the files to install directories.
install(
    FILES
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/NTURT_CAN_ParserConfig.cmake"
    DESTINATION "share/${PROJECT_NAME}/cmake" 
    COMPONENT dev
)
install(
    EXPORT NTURT_CAN_ParserTargets
    DESTINATION "share/${PROJECT_NAME}/cmake"
    FILE NTURT_CAN_ParserTargets.cmake
    COMPONENT dev
)
