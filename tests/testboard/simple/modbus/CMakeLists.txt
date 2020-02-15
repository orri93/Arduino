cmake_minimum_required(VERSION 3.8.2)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_PLATFORM_INDEPENDENT_CODE ON)

set(astbmbte_target ExecuteArduinoSketchesTestBoardModbusTests)

list(APPEND astbmbte_include_directories
  ${gos_arduino_sketches_unit_testing_include}
  ${gos_arduino_unit_testing_sublibraries_gatl_dir}
  "${gos_arduino_unit_testing_sublibraries_dir}/arduinotick/src"
  "${gos_arduino_unit_testing_sublibraries_dir}/arduinosensor/src"
  "${gos_arduino_unit_testing_sublibraries_dir}/arduinomax6675/src")
list(APPEND astbmbte_libraries
  ${gos_arduino_sketches_unit_testing_target_link_libraries}
  libarduinomax6675
  libarduinosensor
  libarduinotick)

add_executable(${astbmbte_target} modbus.cpp)

target_compile_definitions(${astbmbte_target} PRIVATE
  GOS_ARDUINO_TEMPLATE_LIBRARY_BINDING_TESTING)

target_include_directories(${astbmbte_target} PRIVATE
  ${astbmbte_include_directories})

target_link_libraries(${astbmbte_target}
  ${astbmbte_libraries})

add_test(NAME SketchesTestBoardTests COMMAND ${astbmbte_target})
