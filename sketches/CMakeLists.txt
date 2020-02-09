option(GOS_BUILD_ARDUINO_SKETCHES_TEST_BOARD "Build test board sketches" ON)

if(GOS_BUILD_ARDUINO_SKETCHES_TEST_BOARD)
  add_subdirectory(testboard)
endif()