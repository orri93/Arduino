cmake_minimum_required(VERSION 3.8.2)

set(gos_arduino_folder_additional_module_path
  "${CMAKE_SOURCE_DIR}/.cmake")

set (CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/.cmake")

include(Arduino-Toolchain)

project(ArduinoFolder VERSION 1.0
  DESCRIPTION "GOS Arduino folder"
  LANGUAGES C CXX ASM)
  
add_subdirectory(gos/ArduinoTesting)

message(STATUS "")
message(STATUS "BUILD SUMMARY")
message(STATUS "  CMAKE_SYSTEM_NAME    : ${CMAKE_SYSTEM_NAME}")
message(STATUS "  CMAKE_GENERATOR      : ${CMAKE_GENERATOR}")
message(STATUS "  CMAKE_MODULE_PATH    : ${CMAKE_MODULE_PATH}")
message(STATUS "  Compiler ID          : ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "  Build type           : ${CMAKE_BUILD_TYPE}")
message(STATUS "  Build tests          : ${FDS_PACKAGE_TESTS}")
message(STATUS "  Force Shared CRT     : ${gtest_force_shared_crt}")
#message(STATUS "  Boost version        : ${Boost_VERSION}")
#message(STATUS "  Boost include dir    : ${Boost_INCLUDE_DIRS}")
message(STATUS "")
