set(gos_arduino_tools_prototype_qmloscilloscope_target qmloscilloscopel)

list(APPEND gos_arduino_tools_prototype_qmloscilloscope_source
  qmloscilloscope.qrc
  datasource.cpp
  datasource.h
  main.cpp)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC OFF)

find_package(Qt5 COMPONENTS REQUIRED Quick Core Qml DataVisualization)

list(APPEND gos_arduino_tools_prototype_qmloscilloscope_libraries
  Qt5::DataVisualization
  Qt5::Quick
  Qt5::Core
  ws2_32)

add_executable(${gos_arduino_tools_prototype_qmloscilloscope_target}
  ${gos_arduino_tools_prototype_qmloscilloscope_source})

target_link_libraries(${gos_arduino_tools_prototype_qmloscilloscope_target}
  ${gos_arduino_tools_prototype_qmloscilloscope_libraries})

install(TARGETS ${gos_arduino_tools_prototype_qmloscilloscope_target}
  LIBRARY DESTINATION bin/tools/prototype
  ARCHIVE DESTINATION bin/tools/prototype)
