cmake_minimum_required(VERSION 3.8.2)

project(ArduinoSketchesMaxSensorsTests VERSION 1.0
  DESCRIPTION "GOS Arduino Sketches Max Sensors Tests"
  LANGUAGES C CXX ASM)

add_executable(ExecutArduinoSketchesMaxSensorsTests
  maxsensors.cpp)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_PLATFORM_INDEPENDENT_CODE ON)

include_directories(
  ${gos_arduino_sketches_unit_testing_include}
  "${gos_arduino_sketches_dir}/maxsensors"
# "${gos_arduino_libraries_dir}/GosMax31865/src"
  "${gos_arduino_unit_testing_sublibraries_dir}/arduinomax31865/src"
  "${gos_arduino_unit_testing_sublibraries_dir}/arduinosensor/src"
  "${gos_arduino_unit_testing_sublibraries_dir}/arduinotick/src")

target_link_libraries(ExecutArduinoSketchesMaxSensorsTests
  ${gos_arduino_sketches_unit_testing_target_link_libraries}
  libarduinotick
  libarduinosensor
  libarduinomax31865)

add_test(NAME sketches_tests COMMAND ExecutArduinoSketchesMaxSensorsTests)
