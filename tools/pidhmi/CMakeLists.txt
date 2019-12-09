set(gos_arduino_tools_pid_hmi_target pidhmi)

list(APPEND gos_arduino_tools_pid_hmi_source
  orchestration.cpp
  configuration.cpp
  pidparsource.cpp
  orchestration.h
  configuration.h
  pidparsource.h
  datasource.cpp
  resources.qrc
  datasource.h
  master.cpp
  main.cpp)

list(APPEND gos_arduino_tools_pid_hmi_include
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${extern_libmodbus_include})

list(APPEND gos_arduino_tools_pid_hmi_libraries
  ${extern_libmodbus}
  Qt5::QuickControls2
  Qt5::Charts
  Qt5::Quick  
  Qt5::Core
  Qt5::Qml
  ws2_32)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC OFF)

find_package(Qt5 COMPONENTS REQUIRED
  QuickControls2
  Charts
  Quick
  Core
  Qml)

add_executable(${gos_arduino_tools_pid_hmi_target}
  ${gos_arduino_tools_pid_hmi_source})

target_include_directories(${gos_arduino_tools_pid_hmi_target} PRIVATE
  ${gos_arduino_tools_pid_hmi_include})

target_link_libraries(${gos_arduino_tools_pid_hmi_target}
  ${gos_arduino_tools_pid_hmi_libraries})

add_custom_command(TARGET ${gos_arduino_tools_pid_hmi_target} POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_CURRENT_SOURCE_DIR}/configuration.ini
    $<TARGET_FILE_DIR:${gos_arduino_tools_pid_hmi_target}>)

install(TARGETS ${gos_arduino_tools_pid_hmi_target}
  LIBRARY DESTINATION bin/tools
  ARCHIVE DESTINATION bin/tools)
