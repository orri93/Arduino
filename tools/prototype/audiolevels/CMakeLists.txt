set(gos_arduino_tools_prototype_audiolevels_target audiolevels)

list(APPEND gos_arduino_tools_prototype_audiolevels_source
  audiolevelsiodevice.cpp
  audiolevelsiodevice.h
  audiolevels.cpp
  audiolevels.h
  main.cpp)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC OFF)

find_package(Qt5 COMPONENTS REQUIRED Core Multimedia DataVisualization)

list(APPEND gos_arduino_tools_prototype_audiolevels_libraries
  Qt5::DataVisualization
  Qt5::Multimedia
  Qt5::Core
  ws2_32)

add_executable(${gos_arduino_tools_prototype_audiolevels_target}
  ${gos_arduino_tools_prototype_audiolevels_source})

target_link_libraries(${gos_arduino_tools_prototype_audiolevels_target}
  ${gos_arduino_tools_prototype_audiolevels_libraries})

install(TARGETS ${gos_arduino_tools_prototype_audiolevels_target}
  LIBRARY DESTINATION bin/tools/prototype
  ARCHIVE DESTINATION bin/tools/prototype)
