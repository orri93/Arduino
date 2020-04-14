cmake_minimum_required(VERSION 3.8.2)

set(gos_arduino_tool_modbus_master_target master)

list(APPEND gos_arduino_tool_modbus_master_source master.cpp)
list(APPEND gos_arduino_tool_modbus_master_include
  ${extern_libmodbus_include})
list(APPEND gos_arduino_tool_modbus_master_libraries
  ${extern_libmodbus}
  ws2_32)

if (gos_arduino_tool_modbus_definitions)
  list(APPEND gos_arduino_tool_modbus_master_definitions
    ${gos_arduino_tool_modbus_definitions})
endif ()

if (gos_arduino_tool_modbus_include)
  list(APPEND gos_arduino_tool_modbus_master_include
    ${gos_arduino_tool_modbus_include})
endif ()

if (gos_arduino_tool_modbus_libraries)
  list(APPEND gos_arduino_tool_modbus_master_libraries
    ${gos_arduino_tool_modbus_libraries})
endif ()

if (gos_arduino_pid_master_library_target)
  list(APPEND gos_arduino_tool_modbus_master_libraries
    ${gos_arduino_pid_master_library_target})
endif ()

add_executable(${gos_arduino_tool_modbus_master_target}
  ${gos_arduino_tool_modbus_master_source})

if (gos_arduino_tool_modbus_master_definitions)
  target_compile_definitions(${gos_arduino_tool_modbus_master_target} PUBLIC
    ${gos_arduino_tool_modbus_master_definitions})
endif ()

if (gos_arduino_tool_modbus_master_include)
  target_include_directories(${gos_arduino_tool_modbus_master_target} PUBLIC
    ${gos_arduino_tool_modbus_master_include})
endif ()

if (gos_arduino_tool_modbus_master_libraries)
  target_link_libraries(${gos_arduino_tool_modbus_master_target}
    ${gos_arduino_tool_modbus_master_libraries})
endif ()

install(TARGETS ${gos_arduino_tool_modbus_master_target}
  LIBRARY DESTINATION bin/tools
  ARCHIVE DESTINATION bin/tools)
