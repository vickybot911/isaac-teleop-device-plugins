add_library(isaac_handtracking_capi SHARED
  IsaacSimHandTrackerCAPIEntry.cpp
)

target_include_directories(isaac_handtracking_capi
  PUBLIC
    $<BUILD_INTERFACE:${ISAAC_HANDTRACKING_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_definitions(isaac_handtracking_capi
  PRIVATE
    ISAACSIM_HANDTRACKER_CAPI_EXPORTS=1
)

# Tighten warnings sensibly for modern gcc/clang
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_compile_options(isaac_handtracking_capi PRIVATE -Wall -Wextra -Wpedantic)
endif()

set_target_properties(isaac_handtracking_capi PROPERTIES
  OUTPUT_NAME IsaacSimHandTrackerCAPI
  SOVERSION ${PROJECT_VERSION_MAJOR}
  VERSION ${PROJECT_VERSION}
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES
  BUILD_WITH_INSTALL_RPATH NO
  BUILD_RPATH "$ORIGIN;$ORIGIN/../lib"
)

# Provide an export header alias for convenience if needed by downstream
add_library(isaac::handtracking_capi ALIAS isaac_handtracking_capi)


