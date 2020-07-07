list(APPEND gos_arduino_pid_tuning_library_source
  "blackbox.cpp")

add_library(${gos_arduino_pid_tuning_library_target}
  ${gos_arduino_pid_tuning_library_source})

if(CLANG_TIDY_EXE)
  set_target_properties(${gos_arduino_pid_tuning_library_target} PROPERTIES
    CXX_CLANG_TIDY "${CMAKE_CXX_CLANG_TIDY}")
endif()

if (libgosarduinotools_include)
  list(APPEND gos_arduino_pid_tuning_library_include
    ${libgosarduinotools_include})
endif ()

target_include_directories(${gos_arduino_pid_tuning_library_target} PUBLIC
  ${gos_arduino_pid_tuning_library_include})

if (gos_arduino_pid_tuning_library_public_header)
  set_target_properties(${gos_arduino_pid_tuning_library_target} PROPERTIES
    PUBLIC_HEADER "${gos_arduino_pid_tuning_library_public_header}")
endif ()

install(TARGETS ${gos_arduino_pid_tuning_library_target}
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION include/gos/arduino/tools/pid/tuning
  ARCHIVE DESTINATION lib)
