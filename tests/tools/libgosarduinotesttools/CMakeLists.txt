list(APPEND libgosarduinotesttools_source
  "arduinoslave.cpp"
  "exception.cpp"
  "arduino.cpp"
  "options.cpp"
  "setting.cpp"
  "default.cpp"
  "version.cpp"
  "serial.cpp"
  "stream.cpp"
  "print.cpp"
  "types.cpp"
  "text.cpp"
  "gam.cpp")

add_library(${libgosarduinotesttools_target} ${libgosarduinotesttools_source})

message(STATUS "libgosarduinotesttools_public_header: ${libgosarduinotesttools_public_header}")

if(CLANG_TIDY_EXE)
  set_target_properties(${libgosarduinotesttools_target} PROPERTIES
    CXX_CLANG_TIDY "${CMAKE_CXX_CLANG_TIDY}")
endif()

if (libgosarduinotesttools_include)
  target_include_directories(${libgosarduinotesttools_target} PUBLIC
    ${libgosarduinotesttools_include})
endif ()

if (libgosarduinotesttools_public_header)
  set_target_properties(${libgosarduinotesttools_target}
    PROPERTIES PUBLIC_HEADER
    "${libgosarduinotesttools_public_header}")
endif ()

install(TARGETS ${libgosarduinotesttools_target}
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION include/gos/arduino/test/tools
  ARCHIVE DESTINATION lib)
