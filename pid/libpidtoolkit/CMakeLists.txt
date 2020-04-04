list(APPEND libpidtoolkit_source
  "exception.cpp"
  "version.cpp")

add_library(${libpidtoolkit_target} ${libpidtoolkit_source})

list(APPEND libpidtoolkit_include
  ${Boost_INCLUDE_DIRS})

if(CLANG_TIDY_EXE)
  set_target_properties(${libpidtoolkit_target} PROPERTIES
    CXX_CLANG_TIDY "${CMAKE_CXX_CLANG_TIDY}")
endif()

target_include_directories(${libpidtoolkit_target} PUBLIC
  ${libpidtoolkit_include})

if (libpidtoolkit_public_header)
  set_target_properties(${libpidtoolkit_target} PROPERTIES PUBLIC_HEADER
    "${libpidtoolkit_public_header}")
endif ()

install(TARGETS ${libpidtoolkit_target}
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION include/gos/pid/toolkit
  ARCHIVE DESTINATION lib)
