cmake_minimum_required(VERSION 3.8.2)

project(ArduinoSketchesTestBoardTests VERSION 1.0
  DESCRIPTION "GOS Arduino Sketches Test Board Tests"
  LANGUAGES C CXX ASM)

add_executable(ExecuteArduinoSketchesTestBoardTests
  oledtest.cpp)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_PLATFORM_INDEPENDENT_CODE ON)

include_directories(
  ${gos_arduino_sketches_unit_testing_include}
  "${gos_arduino_libraries_dir}/GosTemplates/src"
  "${gos_arduino_unit_testing_sublibraries_dir}/arduinotick/src")

target_link_libraries(ExecuteArduinoSketchesTestBoardTests
  ${gos_arduino_sketches_unit_testing_target_link_libraries}
  libarduinotick)

add_test(NAME SketchesTestBoardTests COMMAND ExecuteArduinoSketchesTestBoardTests)
