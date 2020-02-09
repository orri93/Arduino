option(GOS_BUILD_ARDUINO_SKETCHES_TEST_BOARD_SIMPLE
  "Build simple test board sketches" ON)

if(GOS_BUILD_ARDUINO_SKETCHES_TEST_BOARD_SIMPLE)
  add_subdirectory(simple)
endif()