if(NOT BUILD_HAPLY_PLUGIN)
  return()
endif()

# Haply plugin uses a WebSocket connection to the Haply SDK service at runtime.
# No external SDK libraries are required at build time - only nlohmann/json
# (bundled) and standard POSIX sockets.

add_library(isaac_haply_plugin SHARED
  HaplyHandTrackingPlugin.cpp
  HaplyHandTrackingPlugin.h
)

target_include_directories(isaac_haply_plugin
  PRIVATE
    $<BUILD_INTERFACE:${ISAAC_HANDTRACKING_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_link_libraries(isaac_haply_plugin
  PRIVATE
    isaac::handtracking_capi
    Threads::Threads
)

# Require pthreads
find_package(Threads REQUIRED)

# Tighten warnings sensibly for modern gcc/clang
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_compile_options(isaac_haply_plugin PRIVATE -Wall -Wextra -Wpedantic)
endif()

set_target_properties(isaac_haply_plugin PROPERTIES
  OUTPUT_NAME IsaacSimHaplyHandTracking
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}"
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES
  BUILD_WITH_INSTALL_RPATH NO
  SKIP_BUILD_RPATH NO
  BUILD_RPATH "$ORIGIN;$ORIGIN/../lib"
)

add_library(isaac::haply_plugin ALIAS isaac_haply_plugin)


# Unit tests
if(BUILD_TESTING)
  add_executable(HaplyPluginTest
    tests/HaplyPluginTest.cpp
  )
  target_include_directories(HaplyPluginTest PRIVATE
    ${ISAAC_HANDTRACKING_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party
  )
  target_link_libraries(HaplyPluginTest PRIVATE isaac::haply_plugin Threads::Threads)
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(HaplyPluginTest PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  enable_testing()
  add_test(NAME HaplyPluginTest COMMAND HaplyPluginTest)
endif()


# CLI tool to print Haply tracker output
add_executable(HaplyHandTrackerPrinter
  HaplyHandTrackerPrinter.cpp
)

target_include_directories(HaplyHandTrackerPrinter
  PRIVATE
    ${ISAAC_HANDTRACKING_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(HaplyHandTrackerPrinter
  PRIVATE
    isaac::handtracking_capi
    isaac::haply_plugin
    Threads::Threads
)

# Ensure the plugin's object code is pulled in even if no symbols are
# referenced directly from the executable (avoid --as-needed dropping it)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_link_options(HaplyHandTrackerPrinter PRIVATE -Wl,--no-as-needed)
  target_compile_options(HaplyHandTrackerPrinter PRIVATE -Wall -Wextra -Wpedantic)
endif()

set_target_properties(HaplyHandTrackerPrinter PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES
  BUILD_WITH_INSTALL_RPATH NO
  SKIP_BUILD_RPATH NO
  BUILD_RPATH "$ORIGIN;$ORIGIN/../lib"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
