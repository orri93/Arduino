cmake_minimum_required(VERSION 3.8.2)

project(ArduinoFolder VERSION 1.0.0.0
  DESCRIPTION "GOS Arduino folder"
  LANGUAGES C CXX ASM)

option(GOS_ARDUINO_BUILD_ARDUINO_TOOLS
  "Build Arduino tools inside the gos folder" ON)
option(GOS_ARDUINO_LIB_MODBUS "Build modbus library" ON)
option(GOS_ARDUINO_SKETCHES_PACKAGE_TESTS "Build tests" ON)
option(GOS_ARDUINO_TOOLS "Build tools" ON)
option(GOS_ARDUINO_TEST_TOOLS "Build test tools" ON)
option(GOS_ARDUINO_BUILD_DOC "Build Documentation" OFF)
option(GOS_BUILD_ARDUINO_SKETCHES "Build Arduino sketches for testing" ON)
option(GOS_PID_TOOLKIT "Build PID Toolkit" ON)

set(gos_arduino_include_dir
  "${CMAKE_CURRENT_SOURCE_DIR}/include")

set(gos_arduino_sketches_dir
  "${CMAKE_CURRENT_SOURCE_DIR}/sketches")

#set(gos_arduino_libraries_dir
#  "${CMAKE_CURRENT_SOURCE_DIR}/libraries")

set(gos_arduino_test_tools_version_configure_file
  "${CMAKE_CURRENT_SOURCE_DIR}/include/gos/arduino/test/tools/version.h.in")
configure_file(
  "${gos_arduino_test_tools_version_configure_file}"
  "include/gos/arduino/test/tools/version.h"
  @ONLY)
set(gos_arduino_test_tools_version_include_file
  "${CMAKE_CURRENT_BINARY_DIR}/include/gos/arduino/test/tools/version.h")
set(gos_arduino_binary_include
  "${CMAKE_CURRENT_BINARY_DIR}/include")

list(APPEND gos_arduino_sketches_unit_testing_src
  "${CMAKE_CURRENT_SOURCE_DIR}/tests/maxsensors.cpp")

set(gos_arduino_unit_testing_dir
  "${CMAKE_CURRENT_SOURCE_DIR}/gos/ArduinoUnitTesting")

set(gos_arduino_unit_testing_sublibraries_dir
  "${CMAKE_CURRENT_SOURCE_DIR}/gos/ArduinoUnitTesting/sublibraries")
set(gos_arduino_unit_testing_sublibraries_gatl_dir
  "${gos_arduino_unit_testing_sublibraries_dir}/arduinotemplates/src")

list(APPEND gos_arduino_sketches_unit_testing_mock_include 
  "${gos_arduino_unit_testing_dir}/include"
  "${gos_arduino_unit_testing_dir}/include/mock"
  "${gos_arduino_unit_testing_dir}/extern/arduinomock"
  "${gos_arduino_unit_testing_dir}/extern/googletest/googlemock/include"
  "${gos_arduino_unit_testing_dir}/extern/googletest/googletest/include")

list(APPEND gos_arduino_sketches_unit_testing_include
  ${gos_arduino_sketches_dir}
  ${gos_arduino_sketches_unit_testing_mock_include})

add_subdirectory(gos/ArduinoUnitTesting)

list(APPEND gos_arduino_sketches_unit_testing_target_link_libraries
  gtest
  gmock
  gmock_main
  libmock
  libavrlibc
  libgosutils
  libarduinomock
  libarduinoextern)

set(gtest_force_shared_crt ON CACHE BOOL "Prefer to use Shared CRT for static")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_PLATFORM_INDEPENDENT_CODE ON)

if (GOS_ARDUINO_LIB_MODBUS)
  add_subdirectory(extern/libmodbus)
  list(APPEND extern_libmodbus_include
    ${CMAKE_CURRENT_BINARY_DIR}/extern/libmodbus/src
    ${CMAKE_CURRENT_SOURCE_DIR}/extern/libmodbus/src)
  set(extern_libmodbus libmodbus)
endif ()

if (GOS_ARDUINO_TOOLS)
  set(gos_build_dependency_boost ON)
  set(libgosarduinotools_target libgosarduinotools)
  list(APPEND gos_arduino_boost_package
    program_options
    log_setup
    regex
    log)
  list(APPEND libgosarduinotools_libraries
    ${libgosarduinotools_target}
    Boost::program_options
    Boost::filesystem
    Boost::date_time
    Boost::log_setup
    Boost::regex
    Boost::log)
  list(APPEND libgosarduinotools_include
     ${gos_arduino_include_dir})
  list(APPEND libgosarduinotools_public_header
    "${CMAKE_CURRENT_SOURCE_DIR}/include/gos/arduino/tools/exception.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/gos/arduino/tools/options.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/gos/arduino/tools/setting.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/gos/arduino/tools/default.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/gos/arduino/tools/types.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/gos/arduino/tools/text.h")
endif ()

if (GOS_PID_TOOLKIT)
  set(GOS_PID_TOOLKIT_INSTRUCT_GOOGLE_TEST_PATH 
    "${gos_arduino_unit_testing_dir}/extern/googletest")
  add_subdirectory(extern/pidtoolkit)
endif ()

if (GOS_ARDUINO_TEST_TOOLS)
  set(gos_build_dependency_boost ON)
  set(libgosarduinotesttools_target libgosarduinotesttools)
  set(arduinomodbusslave_lib libarduinomodbusslave)
  list(APPEND gos_arduino_boost_package
    program_options
    log_setup
    regex
    log)
  list(APPEND libgosarduinotesttools_libraries
    ${libgosarduinotesttools_target}
    ${arduinomodbusslave_lib}
    Boost::program_options
    Boost::filesystem
    Boost::date_time
    Boost::log_setup
    Boost::regex
    Boost::log)
  list(APPEND arduinomodbusslave_include
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/gos/arduino/test/tools/arduino")
  list(APPEND libgosarduinotesttools_include
    "${CMAKE_CURRENT_SOURCE_DIR}/gos/ArduinoUnitTesting/extern/libraries/ArduinoModbusSlave/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/gos/ArduinoUnitTesting/sublibraries/arduinotemplates/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/gos/arduino/test/tools/arduino"
    "${gos_arduino_binary_include}"
    "${gos_arduino_include_dir}")
  list(APPEND libgosarduinotesttools_public_header
    "${gos_arduino_test_tools_version_include_file}"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/gos/arduino/test/tools/exception.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/gos/arduino/test/tools/options.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/gos/arduino/test/tools/setting.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/gos/arduino/test/tools/default.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/gos/arduino/test/tools/types.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/gos/arduino/test/tools/text.h")
endif ()

add_subdirectory(gos/ArduinoUnitTesting/extern/libraries/ArduinoModbusSlave)

if (gos_build_dependency_boost)
  set(Boost_USE_MULTITHREADED        ON)
  set(Boost_USE_STATIC_RUNTIME      OFF)
  if (BUILD_SHARED_LIBS)
    set(Boost_USE_STATIC_LIBS       OFF)    
  else ()
    set(Boost_USE_STATIC_LIBS        ON)
  endif ()

  list(APPEND gos_arduino_boost_package
    system
    date_time
    filesystem)

  list(REMOVE_DUPLICATES gos_arduino_boost_package)

  find_package(Boost 1.71 COMPONENTS REQUIRED
    ${gos_arduino_boost_package})

  if (GOS_ARDUINO_TOOLS)
    list(APPEND libgosarduinotools_include
      ${Boost_INCLUDE_DIRS})
  endif ()
  if (GOS_ARDUINO_TEST_TOOLS)
    list(APPEND libgosarduinotesttools_include
      ${Boost_INCLUDE_DIRS})
  endif ()
endif ()

if (GOS_ARDUINO_TOOLS)
  add_subdirectory(tools)
endif ()

#add_subdirectory(gos/ArduinoTesting)
if (GOS_ARDUINO_BUILD_ARDUINO_TOOLS)
  add_subdirectory(gos/ArduinoTools)
endif ()

if(GOS_ARDUINO_SKETCHES_PACKAGE_TESTS OR GOS_ARDUINO_TEST_TOOLS)
  if(GOS_ARDUINO_SKETCHES_PACKAGE_TESTS)
    enable_testing()
  endif()
  add_subdirectory(tests)
endif()

if(GOS_BUILD_ARDUINO_SKETCHES)
  list(APPEND gos_arduino_sketches_include
    "${gos_arduino_sketches_unit_testing_mock_include}"
    "${CMAKE_CURRENT_SOURCE_DIR}/gos/ArduinoUnitTesting/sublibraries/arduinotemplates/src")
  add_subdirectory(sketches)
endif()

if (GOS_ARDUINO_BUILD_DOC)
  find_package(Doxygen)
  if(DOXYGEN_FOUND)
     add_subdirectory(docs)
   else ()
    message(WARNING
      "Doxygen needs to be installed to generate the documentation.")
  endif ()
endif ()

message(STATUS "")
message(STATUS "Arduino Folder BUILD SUMMARY")
message(STATUS "  CMAKE_SYSTEM_NAME    : ${CMAKE_SYSTEM_NAME}")
message(STATUS "  CMAKE_GENERATOR      : ${CMAKE_GENERATOR}")
message(STATUS "  CMAKE_MODULE_PATH    : ${CMAKE_MODULE_PATH}")
message(STATUS "  Compiler ID          : ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "  Build type           : ${CMAKE_BUILD_TYPE}")
message(STATUS "  Build tests          : ${GOS_ARDUINO_SKETCHES_PACKAGE_TESTS}")
message(STATUS "  Force Shared CRT     : ${gtest_force_shared_crt}")
#message(STATUS "  Boost version        : ${Boost_VERSION}")
#message(STATUS "  Boost include dir    : ${Boost_INCLUDE_DIRS}")
message(STATUS "")
