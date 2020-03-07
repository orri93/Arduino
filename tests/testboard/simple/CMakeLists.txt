cmake_minimum_required(VERSION 3.8.2)

project(ArduinoSketchesTestboardTests VERSION 1.0
  DESCRIPTION "GOS Arduino Test Board Sketches Tests"
  LANGUAGES C CXX ASM)

option(GOS_ARDUINO_BUILD_MODBUS "Build Modbus sketches" ON)
option(GOS_ARDUINO_BUILD_MELTINGPOINT "Build Meltingpoint sketches" ON)
option(GOS_ARDUINO_BUILD_OLED_TEST "Build the OLED test sketches" ON)

if (GOS_ARDUINO_BUILD_MODBUS)
  add_subdirectory(modbus)
endif ()
if (GOS_ARDUINO_BUILD_MELTINGPOINT)
  add_subdirectory(meltingpoint)
endif ()
if (GOS_ARDUINO_BUILD_OLED_TEST)
  add_subdirectory(oledtest)
endif ()

add_subdirectory(rs485/rs485test)
