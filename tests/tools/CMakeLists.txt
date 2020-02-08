cmake_minimum_required(VERSION 3.8.2)

project(ArduinoTestTool VERSION 1.0.0.0
  DESCRIPTION "GOS Arduino test tools"
  LANGUAGES C CXX ASM)

add_subdirectory(libgosarduinotesttools)
add_subdirectory(modbus)
