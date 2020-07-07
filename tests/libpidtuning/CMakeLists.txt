set(gos_test_tool_pid_tuning_target pid-tuning-test)

list(APPEND gos_test_tool_pid_tuning_source blackbox.cpp)

add_executable(${gos_test_tool_pid_tuning_target}
  ${gos_test_tool_pid_tuning_source})

if (libgosarduinotools_include)
  list(APPEND gos_test_tool_pid_tuning_include
    ${libgosarduinotools_include})
endif ()

list(APPEND gos_test_tool_pid_tuning_include
  "${gos_arduino_unit_testing_googlemock_include_dir}"
  "${gos_arduino_unit_testing_googletest_include_dir}")

target_include_directories(${gos_test_tool_pid_tuning_target} PUBLIC
  ${gos_test_tool_pid_tuning_include})

target_link_libraries(${gos_test_tool_pid_tuning_target}
  ${gos_arduino_pid_tuning_library_target}
  ${libgosarduinotools_target}
  gtest
  gmock
  gmock_main)

add_test(NAME gos_test_tool_pid_tuning_test
  COMMAND ${gos_test_tool_pid_tuning_target})
