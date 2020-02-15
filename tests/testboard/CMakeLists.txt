option(GOS_ARDUINO_TESTS_BUILD_TESTBOARD_SIMPLE
  "Build the simple test board tests" OFF)

if (GOS_ARDUINO_TESTS_BUILD_TESTBOARD_SIMPLE)
  add_subdirectory(simple)
endif ()
