cmake_minimum_required(VERSION 3.8.2)

#set(gos_arduino_folder_additional_module_path
#  "${CMAKE_SOURCE_DIR}/.cmake")

#set (CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/.cmake")

#include(Arduino-Toolchain)

project(ArduinoFolder VERSION 1.0
  DESCRIPTION "GOS Arduino folder"
  LANGUAGES C CXX ASM)

set(gos_arduino_sketches_dir
  "${CMAKE_CURRENT_SOURCE_DIR}/sketches")

set(gos_arduino_libraries_dir
  "${CMAKE_CURRENT_SOURCE_DIR}/libraries")

set(gos_arduino_sketches_unit_testing_src
  "${CMAKE_CURRENT_SOURCE_DIR}/tests/maxsensors.cpp")

set(gos_arduino_unit_testing_dir
  "${CMAKE_CURRENT_SOURCE_DIR}/gos/ArduinoUnitTesting")

set(gos_arduino_unit_testing_sublibraries_dir
  "${CMAKE_CURRENT_SOURCE_DIR}/gos/ArduinoUnitTesting/sublibraries")

set(gos_arduino_sketches_unit_testing_mock_include 
  "${gos_arduino_unit_testing_dir}/include/avr"
  "${gos_arduino_unit_testing_dir}/include/mock"
  "${gos_arduino_unit_testing_dir}/extern/arduinomock"
# "${gos_arduino_unit_testing_dir}/extern/arduinomock/include/arduino-mock"
  "${gos_arduino_unit_testing_dir}/extern/googletest/googlemock/include"
  "${gos_arduino_unit_testing_dir}/extern/googletest/googletest/include")

set(gos_arduino_sketches_unit_testing_include
  ${gos_arduino_sketches_dir}
  ${gos_arduino_sketches_unit_testing_mock_include}
  "${gos_arduino_unit_testing_dir}/include")

set(gos_arduino_sketches_unit_testing_target_link_libraries
  gtest
  gmock
  gmock_main
  libmock
  libavrlibc
  libgosutils
  libarduinomock)

set(gtest_force_shared_crt ON CACHE BOOL "Prefer to use Shared CRT for static")

option(GOS_ARDUINO_SKETCHES_PACKAGE_TESTS "Build tests" ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_PLATFORM_INDEPENDENT_CODE ON)

#add_subdirectory(gos/ArduinoTesting)
add_subdirectory(gos/ArduinoTools)
add_subdirectory(gos/ArduinoUnitTesting)

if(GOS_ARDUINO_SKETCHES_PACKAGE_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

message(STATUS "")
message(STATUS "Arduino Folder BUILD SUMMARY")
message(STATUS "  CMAKE_SYSTEM_NAME    : ${CMAKE_SYSTEM_NAME}")
message(STATUS "  CMAKE_GENERATOR      : ${CMAKE_GENERATOR}")
message(STATUS "  CMAKE_MODULE_PATH    : ${CMAKE_MODULE_PATH}")
message(STATUS "  Compiler ID          : ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "  Build type           : ${CMAKE_BUILD_TYPE}")
message(STATUS "  Build tests          : ${GOS_ARDUINO_SKETCHES_PACKAGE_TESTS}")
message(STATUS "  Force Shared CRT     : ${gtest_force_shared_crt}")
#message(STATUS "  Boost version        : ${Boost_VERSION}")
#message(STATUS "  Boost include dir    : ${Boost_INCLUDE_DIRS}")
message(STATUS "")
