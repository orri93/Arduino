cmake_minimum_required(VERSION 3.8.2)

project(ArduinoSketchesTestBoardMeltingPointTests VERSION 1.0
  DESCRIPTION "GOS Arduino Sketches Test Board Meltingpoint Tests"
  LANGUAGES C CXX ASM)

set(astbmpte ExecuteArduinoSketchesTestBoardMeltingpointTests)

add_executable(${astbmpte}
  meltingpoint.cpp)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_PLATFORM_INDEPENDENT_CODE ON)

include_directories(
  ${gos_arduino_sketches_unit_testing_include}
  "${gos_arduino_libraries_dir}/GosTemplates/src"
  "${gos_arduino_unit_testing_sublibraries_dir}/arduinotick/src"
  "${gos_arduino_unit_testing_sublibraries_dir}/arduinosensor/src"
  "${gos_arduino_unit_testing_sublibraries_dir}/arduinomax6675/src")

target_link_libraries(${astbmpte}
  ${gos_arduino_sketches_unit_testing_target_link_libraries}
  libarduinotick)

add_test(NAME SketchesTestBoardTests COMMAND ${astbmpte})
