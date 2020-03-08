list(APPEND test_board_simple_modbus_source
  "modbus.ino"
  "variable.cpp"
  "binding.cpp"
  "display.cpp"
  "eeprom.cpp"
  "format.cpp"
  "modbus.cpp"
  "value.cpp")

list(APPEND test_board_simple_modbus_include
  ${gos_arduino_sketches_include}
  ${CMAKE_CURRENT_SOURCE_DIR})

if (NOT test_board_simple_modbus_target)
  set(test_board_simple_modbus_target libtestboardsimplemodbus)
endif ()

add_library(${test_board_simple_modbus_target}
  ${test_board_simple_modbus_source})

#target_compile_definitions(${test_board_simple_modbus_target} PUBLIC
#  USE_ARDUINO_MODBUS_SLAVE)

target_include_directories(${test_board_simple_modbus_target} PUBLIC
  ${test_board_simple_modbus_include})

install(TARGETS ${test_board_simple_modbus_target}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
