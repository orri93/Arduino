list(APPEND gos_arduino_tool_modbus_include
  ${libgosarduinotools_include})

list(APPEND gos_arduino_tool_modbus_libraries
  ${libgosarduinotools_libraries})

add_subdirectory(libgosarduinotools)
add_subdirectory(modbus)
add_subdirectory(pidhmi)
