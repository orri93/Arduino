cmake_minimum_required(VERSION 3.8.2)

project(ArduinoSketchesTests VERSION 1.0
  DESCRIPTION "GOS Arduino Sketches Tests"
  LANGUAGES C CXX ASM)

add_compile_definitions(GOS_ARDUINO_TEMPLATE_LIBRARY_CHECK_SIZE)
add_compile_definitions(GOS_ARDUINO_UNIT_TESTING)

add_subdirectory(maxsensors)
add_subdirectory(testboard)
