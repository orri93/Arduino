configure_file(Doxyfile Doxyfile @ONLY)

message(STATUS "Doxygen build started")

add_custom_target(Doxygen ALL
  COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#    COMMAND "Generating API documentation with Doxygen"
#    VERBATIM
)
