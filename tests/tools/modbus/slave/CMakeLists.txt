cmake_minimum_required(VERSION 3.15.0)

set(gos_test_tool_modbus_slave_target slave)

list(APPEND gos_test_tool_modbus_slave_source slave.cpp)

if(MSVC)
  set(GOS_FILEDESCRIPTION "GOS Modbus test tool slave")
  set(GOS_INTERNALNAME "${gos_test_tool_modbus_slave_target}")
  set(GOS_ORIGINALFILENAME "${gos_test_tool_modbus_slave_target}.exe")
  set(GOS_PRODUCTNAME "${gos_test_tool_modbus_slave_target}")
  configure_file("version.rc.in" "version.rc" @ONLY)
  list(APPEND gos_test_tool_modbus_slave_source "${CMAKE_CURRENT_BINARY_DIR}/version.rc")
endif()

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
