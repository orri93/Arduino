option(GOS_BUILD_ARDUINO_SKETCHES_TEST_BOARD_SIMPLE_MODBUS
  "Build simple Modbus test board sketches" ON)

if(GOS_BUILD_ARDUINO_SKETCHES_TEST_BOARD_SIMPLE_MODBUS)
  add_subdirectory(modbus)
endif()
