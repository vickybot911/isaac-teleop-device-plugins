cmake_minimum_required(VERSION 3.20)

project(omni_isaac_handtracking_plugin
  VERSION 0.1.0
  LANGUAGES C CXX
)

# Install/layout helpers
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Place all build outputs in a clean bin/lib layout
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# Linux-only build
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "This project currently supports Linux only.")
endif()

# Modern C++
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Prefer position independent code for shared objects
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(BUILD_MANUS_PLUGIN "Build the Manus plugin under vendor/manus/" ON)
option(COPY_MANUS_RUNTIME_IN_BUILD "Copy Manus SDK runtime into build lib directory" OFF)
option(BUILD_HAPLY_PLUGIN "Build the Haply plugin under vendor/haply/" ON)

# RPATH policy: use $ORIGIN in build tree and install tree; don't bake absolute paths
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)
set(CMAKE_SKIP_BUILD_RPATH OFF)
set(CMAKE_INSTALL_RPATH "$ORIGIN;$ORIGIN/../lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)

# Resolve Manus SDK root. Prefer environment variable if provided, otherwise default to bundled vendor/manus
if(NOT DEFINED MANUS_SDK_ROOT AND DEFINED ENV{MANUS_SDK_ROOT})
  set(MANUS_SDK_ROOT "$ENV{MANUS_SDK_ROOT}" CACHE PATH "Path to Manus SDK root containing include/ and lib/" FORCE)
else()
  set(MANUS_SDK_ROOT "${CMAKE_SOURCE_DIR}/vendor/manus/ManusSDK" CACHE PATH "Path to Manus SDK root containing include/ and lib/")
endif()

set(MANUS_SDK_INCLUDE_DIR "${MANUS_SDK_ROOT}/include" CACHE PATH "Manus SDK include path" FORCE)

# Auto-detect best library directory (supports lib or lib64)
set(_MANUS_LIBDIR_CANDIDATES
  "${MANUS_SDK_ROOT}/lib"
  "${MANUS_SDK_ROOT}/lib64"
)
set(_MANUS_LIBDIR "")
foreach(_cand IN LISTS _MANUS_LIBDIR_CANDIDATES)
  if(EXISTS "${_cand}")
    set(_MANUS_LIBDIR "${_cand}")
    break()
  endif()
endforeach()
if(NOT _MANUS_LIBDIR)
  set(_MANUS_LIBDIR "${MANUS_SDK_ROOT}/lib")
endif()
set(MANUS_SDK_LIBRARY_DIR "${_MANUS_LIBDIR}" CACHE PATH "Manus SDK library path" FORCE)

# Make include dir available to subprojects
set(ISAAC_HANDTRACKING_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/include" CACHE INTERNAL "Isaac handtracking public include directory")

add_subdirectory(src)

if(BUILD_MANUS_PLUGIN)
  add_subdirectory(vendor/manus)
endif()

if(BUILD_HAPLY_PLUGIN)
  add_subdirectory(vendor/haply)
endif()


# Developer utilities (formatting, setup)
include(cmake/ClangFormat.cmake OPTIONAL)


#
# Install & CMake package config
#

# Headers
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/isaac-handtracking-plugin
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Targets (export in a single package)
set(_EXPORT_SET omni_isaac_handtracking_pluginTargets)

install(TARGETS isaac_handtracking_capi
  EXPORT ${_EXPORT_SET}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

if(TARGET isaac_manus_plugin)
  install(TARGETS isaac_manus_plugin
    EXPORT ${_EXPORT_SET}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )
endif()

if(TARGET ManusHandTrackerPrinter)
  install(TARGETS ManusHandTrackerPrinter
    EXPORT ${_EXPORT_SET}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
endif()

if(TARGET isaac_haply_plugin)
  install(TARGETS isaac_haply_plugin
    EXPORT ${_EXPORT_SET}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )
endif()

if(TARGET HaplyHandTrackerPrinter)
  install(TARGETS HaplyHandTrackerPrinter
    EXPORT ${_EXPORT_SET}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
endif()

set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/omni_isaac_handtracking_plugin)

install(EXPORT ${_EXPORT_SET}
  FILE omni_isaac_handtracking_pluginTargets.cmake
  NAMESPACE omni_isaac_handtracking_plugin::
  DESTINATION ${INSTALL_CONFIGDIR}
)

set(PACKAGE_INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/omni_isaac_handtracking_pluginConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/omni_isaac_handtracking_pluginConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
  PATH_VARS PACKAGE_INCLUDE_INSTALL_DIR
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/omni_isaac_handtracking_pluginConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/omni_isaac_handtracking_pluginConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/omni_isaac_handtracking_pluginConfigVersion.cmake
  DESTINATION ${INSTALL_CONFIGDIR}
)

