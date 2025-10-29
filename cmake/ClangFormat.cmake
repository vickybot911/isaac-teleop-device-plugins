# Reusable clang-format targets

find_program(CLANG_FORMAT_EXECUTABLE NAMES clang-format-14)

# Target to install developer dependencies (clang-format) via tools/setup-deps.sh
add_custom_target(setup_deps
  COMMAND ${CMAKE_COMMAND} -E env PROJECT_ROOT=${CMAKE_SOURCE_DIR} bash ${CMAKE_SOURCE_DIR}/tools/setup-deps.sh
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  USES_TERMINAL
)

if(CLANG_FORMAT_EXECUTABLE)
  message(STATUS "Found clang-format-14: ${CLANG_FORMAT_EXECUTABLE}")

  add_custom_target(format
    COMMAND ${CMAKE_COMMAND} -E echo "Formatting C/C++ sources with clang-format-14"
    COMMAND bash -lc "\"${CMAKE_SOURCE_DIR}/tools/run-format.sh\" \"${CLANG_FORMAT_EXECUTABLE}\""
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    USES_TERMINAL
  )

  add_custom_target(check-format
    COMMAND ${CMAKE_COMMAND} -E echo "Checking clang-format-14 compliance"
    COMMAND bash -lc "\"${CMAKE_SOURCE_DIR}/tools/check-format.sh\" \"${CLANG_FORMAT_EXECUTABLE}\""
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    USES_TERMINAL
  )
else()
  message(STATUS "clang-format-14 not found. Use 'cmake --build . --target setup_deps' to install.")

  add_custom_target(format
    COMMAND ${CMAKE_COMMAND} -E echo "clang-format-14 not found. Run: cmake --build . --target setup_deps"
  )

  add_custom_target(check-format
    COMMAND ${CMAKE_COMMAND} -E echo "clang-format-14 not found. Run: cmake --build . --target setup_deps"
  )
endif()


