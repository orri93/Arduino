set(gos_arduino_tools_prototype_qmloscilloscopechart_target
  qmloscilloscopechart)

list(APPEND gos_arduino_tools_prototype_qmloscilloscopechart_source
  datasource.cpp
  resources.qrc
  datasource.h
  main.cpp)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC OFF)

find_package(Qt5 COMPONENTS REQUIRED Charts Quick Core Qml)

list(APPEND gos_arduino_tools_prototype_qmloscilloscopechart_libraries
  Qt5::Charts
  Qt5::Quick  
  Qt5::Core
  Qt5::Qml
  ws2_32)

add_executable(${gos_arduino_tools_prototype_qmloscilloscopechart_target}
  ${gos_arduino_tools_prototype_qmloscilloscopechart_source})

target_link_libraries(${gos_arduino_tools_prototype_qmloscilloscopechart_target}
  ${gos_arduino_tools_prototype_qmloscilloscopechart_libraries})

install(TARGETS ${gos_arduino_tools_prototype_qmloscilloscopechart_target}
  LIBRARY DESTINATION bin/tools/prototype
  ARCHIVE DESTINATION bin/tools/prototype)
