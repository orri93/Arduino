set(gos_test_tool_modbus_slave_target slave)

list(APPEND gos_test_tool_modbus_slave_source slave.cpp)

add_executable(${gos_test_tool_modbus_slave_target}
  ${gos_test_tool_modbus_slave_source})

if (libgosarduinotesttools_include)
  target_include_directories(${gos_test_tool_modbus_slave_target} PUBLIC
    ${libgosarduinotesttools_include})
endif ()

if (libgosarduinotesttools_libraries)
  target_link_libraries(${gos_test_tool_modbus_slave_target}
    ${libgosarduinotesttools_libraries})
endif ()

install(TARGETS ${gos_test_tool_modbus_slave_target}
  LIBRARY DESTINATION bin/tests/tools
  ARCHIVE DESTINATION bin/tests/tools)
