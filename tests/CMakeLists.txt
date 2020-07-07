option(GOS_ARDUINO_BUILD_MAX_SENSORS "Build the max sensors sketches" ON)

add_compile_definitions(GOS_ARDUINO_TEMPLATE_LIBRARY_CHECK_SIZE)
add_compile_definitions(GOS_ARDUINO_UNIT_TESTING)

if (GOS_ARDUINO_BUILD_MAX_SENSORS)
  add_subdirectory(maxsensors)
endif ()

if (GOS_ARDUINO_TEST_TOOLS)
  add_subdirectory(tools)
endif ()

if (GOS_ARDUINO_TOOLS)
  add_subdirectory(libpidtuning)
endif ()

add_subdirectory(testboard)
