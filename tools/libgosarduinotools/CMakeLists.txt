list(APPEND libgosarduinotools_source
  "exception.cpp"
  "options.cpp"
  "setting.cpp"
  "default.cpp"
  "serial.cpp"
  "types.cpp"
  "text.cpp")

add_library(${libgosarduinotools_target} ${libgosarduinotools_source})

message(STATUS "libgosarduinotools_public_header: ${libgosarduinotools_public_header}")

if(CLANG_TIDY_EXE)
  set_target_properties(${libgosarduinotools_target} PROPERTIES
    CXX_CLANG_TIDY "${CMAKE_CXX_CLANG_TIDY}")
endif()

if (libgosarduinotools_include)
  target_include_directories(${libgosarduinotools_target} PUBLIC
    ${libgosarduinotools_include})
endif ()

if (libgosarduinotools_public_header)
  set_target_properties(${libgosarduinotools_target} PROPERTIES PUBLIC_HEADER
    "${libgosarduinotools_public_header}")
endif ()

install(TARGETS ${libgosarduinotools_target}
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION include/gos/arduino/tools
  ARCHIVE DESTINATION lib)