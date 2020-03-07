cmake_minimum_required(VERSION 3.8.2)

project(ArduinoSketchesTestBoardRs485Tests VERSION 1.0
  DESCRIPTION "GOS Arduino Sketches Test Board RS-485 Tests"
  LANGUAGES C CXX ASM)

set(arduino_sketches_test_board_rs_485_tests_target
  arduino-sketches-test_board-rs-485-tests)

set(arduino_sketches_test_board_rs_485_tests_test
  "arduino-sketches-test_board-rs-485-test")

list(APPEND arduino_sketches_test_board_rs_485_tests_source
  rs485test.cpp)

add_executable(${arduino_sketches_test_board_rs_485_tests_target}
  ${arduino_sketches_test_board_rs_485_tests_source})

set_target_properties(${arduino_sketches_test_board_rs_485_tests_target}
  PROPERTIES
  CXX_STANDARD 11
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS OFF
  PLATFORM_INDEPENDENT_CODE YES)

target_include_directories(${arduino_sketches_test_board_rs_485_tests_target}
  PUBLIC
  ${gos_arduino_sketches_unit_testing_include}
  "${gos_arduino_libraries_dir}/GosTemplates/src"
  "${gos_arduino_unit_testing_sublibraries_dir}/arduinotick/src")

target_link_libraries(${arduino_sketches_test_board_rs_485_tests_target}
  ${gos_arduino_sketches_unit_testing_target_link_libraries})

add_test(NAME ${arduino_sketches_test_board_rs_485_tests_test} COMMAND
  ${arduino_sketches_test_board_rs_485_tests_target})
