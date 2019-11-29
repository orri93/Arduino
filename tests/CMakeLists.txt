cmake_minimum_required(VERSION 3.8.2)

option(GOS_ARDUINO_BUILD_MAX_SENSORS "Build the max sensors sketches" ON)

add_compile_definitions(GOS_ARDUINO_TEMPLATE_LIBRARY_CHECK_SIZE)
add_compile_definitions(GOS_ARDUINO_UNIT_TESTING)

if (GOS_ARDUINO_BUILD_MAX_SENSORS)
  add_subdirectory(maxsensors)
endif ()
add_subdirectory(testboard)
