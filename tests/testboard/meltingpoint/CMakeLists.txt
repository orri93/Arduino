cmake_minimum_required(VERSION 3.8.2)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_PLATFORM_INDEPENDENT_CODE ON)

set(astbmpte_target ExecuteArduinoSketchesTestBoardMeltingpointTests)

list(APPEND astbmpte_include_directories
  ${gos_arduino_sketches_unit_testing_include}
  ${gos_arduino_unit_testing_sublibraries_gatl_dir}
  "${gos_arduino_unit_testing_sublibraries_dir}/arduinotick/src"
  "${gos_arduino_unit_testing_sublibraries_dir}/arduinosensor/src"
  "${gos_arduino_unit_testing_sublibraries_dir}/arduinomax6675/src")
list(APPEND astbmpte_libraries
  ${gos_arduino_sketches_unit_testing_target_link_libraries}
  libarduinomax6675
  libarduinosensor
  libarduinotick)

add_executable(${astbmpte_target} meltingpoint.cpp)

target_include_directories(${astbmpte_target} PRIVATE
  ${astbmpte_include_directories})

target_link_libraries(${astbmpte_target}
  ${astbmpte_libraries})

add_test(NAME SketchesTestBoardTests COMMAND ${astbmpte_target})
