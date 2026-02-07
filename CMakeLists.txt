# top dir cmake project for libhydrasdr + hydrasdr-tools

cmake_minimum_required(VERSION 3.10)

########################################################################
# Dependency Versions (used by FetchContent for MSVC builds)
########################################################################
set(LIBUSB_VERSION "1.0.29")
set(PTHREADS4W_VERSION "3.0.0")

# MSVC builds require CMake 3.18+ for:
# - FetchContent support
# - Native 7z archive extraction via file(ARCHIVE_EXTRACT)
if(MSVC)
  if(CMAKE_VERSION VERSION_LESS "3.18")
    message(FATAL_ERROR "MSVC builds require CMake 3.18+ for FetchContent and native 7z extraction. "
                        "Current version: ${CMAKE_VERSION}")
  endif()
endif()

# Disable vcpkg integration if accidentally inherited from environment
# This project manages its own dependencies and doesn't need vcpkg
set(VCPKG_MANIFEST_MODE OFF)
set(VCPKG_APPLOCAL_DEPS OFF)
if(DEFINED ENV{VCPKG_ROOT})
  message(STATUS "Note: VCPKG_ROOT is set but this project doesn't use vcpkg")
endif()

project(hydrasdr-host LANGUAGES C HOMEPAGE_URL https://hydrasdr.com)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

########################################################################
# Set consistent install prefix for Windows (both MSVC and MinGW)
########################################################################
if(WIN32 AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files/${PROJECT_NAME}" CACHE PATH
    "Install path prefix" FORCE)
  message(STATUS "Install prefix set to: ${CMAKE_INSTALL_PREFIX}")
endif()

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Build type not specified: defaulting to Release.")
  set(CMAKE_BUILD_TYPE Release)
endif()

#provide missing strtoull() for VC11
if(MSVC11)
  message("provide missing strtoull() for VC11")
  add_definitions(-Dstrtoull=_strtoui64)
endif(MSVC11)

if(MSVC)
  message("define _TIMESPEC_DEFINED and _CRT_SECURE_NO_WARNINGS() for MSVC")
  add_compile_definitions(_TIMESPEC_DEFINED _CRT_SECURE_NO_WARNINGS)
endif(MSVC)

########################################################################
# FetchContent Dependencies for MSVC builds
########################################################################
if(MSVC)
  include(FetchContent)

  # ====================================================================
  # libusb - Use pre-provided paths or download pre-built Windows binaries
  # ====================================================================
  # If LIBUSB_INCLUDE_DIR and LIBUSB_LIBRARIES are already provided (e.g., from CI),
  # use those instead of downloading via FetchContent.
  if(LIBUSB_INCLUDE_DIR AND LIBUSB_LIBRARIES AND EXISTS "${LIBUSB_LIBRARIES}")
    message(STATUS "Using pre-provided libusb:")
    message(STATUS "  Include: ${LIBUSB_INCLUDE_DIR}")
    message(STATUS "  Library: ${LIBUSB_LIBRARIES}")

    set(LIBUSB_LIBRARY "${LIBUSB_LIBRARIES}" CACHE FILEPATH "libusb import library" FORCE)
    if(NOT LIBUSB_DLL)
      get_filename_component(_libusb_lib_dir "${LIBUSB_LIBRARIES}" DIRECTORY)
      # Check same directory as .lib first, then sibling bin/ directory
      if(EXISTS "${_libusb_lib_dir}/libusb-1.0.dll")
        set(LIBUSB_DLL "${_libusb_lib_dir}/libusb-1.0.dll" CACHE FILEPATH "libusb DLL" FORCE)
      else()
        get_filename_component(_libusb_parent_dir "${_libusb_lib_dir}" DIRECTORY)
        set(LIBUSB_DLL "${_libusb_parent_dir}/bin/libusb-1.0.dll" CACHE FILEPATH "libusb DLL" FORCE)
      endif()
    endif()

    add_library(LIBUSB::LIBUSB SHARED IMPORTED GLOBAL)
    set_target_properties(LIBUSB::LIBUSB PROPERTIES
      IMPORTED_LOCATION "${LIBUSB_DLL}"
      IMPORTED_IMPLIB "${LIBUSB_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${LIBUSB_INCLUDE_DIR}"
    )
    set(LIBUSB_FOUND TRUE CACHE BOOL "libusb found" FORCE)
    set(LIBUSB_USE_DLL TRUE CACHE BOOL "Using libusb DLL" FORCE)
  else()
    # Download libusb pre-built binaries
    set(LIBUSB_ARCHIVE_URL "https://github.com/libusb/libusb/releases/download/v${LIBUSB_VERSION}/libusb-${LIBUSB_VERSION}.7z")
    set(LIBUSB_DOWNLOAD_DIR "${CMAKE_BINARY_DIR}/_deps/libusb-${LIBUSB_VERSION}")
    set(LIBUSB_ARCHIVE_FILE "${LIBUSB_DOWNLOAD_DIR}/libusb-${LIBUSB_VERSION}.7z")

    if(NOT EXISTS "${LIBUSB_DOWNLOAD_DIR}/include/libusb-1.0/libusb.h")
      message(STATUS "Downloading libusb ${LIBUSB_VERSION} pre-built binaries...")
      file(MAKE_DIRECTORY "${LIBUSB_DOWNLOAD_DIR}")

      if(NOT EXISTS "${LIBUSB_ARCHIVE_FILE}")
        file(DOWNLOAD
          "${LIBUSB_ARCHIVE_URL}"
          "${LIBUSB_ARCHIVE_FILE}"
          SHOW_PROGRESS
          STATUS DOWNLOAD_STATUS
        )
        list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
        if(NOT STATUS_CODE EQUAL 0)
          message(FATAL_ERROR "Failed to download libusb: ${DOWNLOAD_STATUS}")
        endif()
      endif()

      message(STATUS "Extracting libusb ${LIBUSB_VERSION}...")
      file(ARCHIVE_EXTRACT
        INPUT "${LIBUSB_ARCHIVE_FILE}"
        DESTINATION "${LIBUSB_DOWNLOAD_DIR}"
      )
    endif()

    if(MSVC_VERSION GREATER_EQUAL 1930)
      set(LIBUSB_VS_DIR "VS2022")
    elseif(MSVC_VERSION GREATER_EQUAL 1920)
      set(LIBUSB_VS_DIR "VS2019")
    else()
      set(LIBUSB_VS_DIR "VS2015")
    endif()

    set(LIBUSB_INCLUDE_DIR "${LIBUSB_DOWNLOAD_DIR}/include" CACHE PATH "libusb include directory" FORCE)
    set(LIBUSB_LIBRARY "${LIBUSB_DOWNLOAD_DIR}/${LIBUSB_VS_DIR}/MS64/dll/libusb-1.0.lib" CACHE FILEPATH "libusb import library" FORCE)
    set(LIBUSB_DLL "${LIBUSB_DOWNLOAD_DIR}/${LIBUSB_VS_DIR}/MS64/dll/libusb-1.0.dll" CACHE FILEPATH "libusb DLL" FORCE)

    if(NOT EXISTS "${LIBUSB_INCLUDE_DIR}/libusb.h")
      message(FATAL_ERROR "libusb header not found at ${LIBUSB_INCLUDE_DIR}/libusb.h")
    endif()
    if(NOT EXISTS "${LIBUSB_LIBRARY}")
      message(FATAL_ERROR "libusb import library not found at ${LIBUSB_LIBRARY}")
    endif()
    if(NOT EXISTS "${LIBUSB_DLL}")
      message(FATAL_ERROR "libusb DLL not found at ${LIBUSB_DLL}")
    endif()

    add_library(LIBUSB::LIBUSB SHARED IMPORTED GLOBAL)
    set_target_properties(LIBUSB::LIBUSB PROPERTIES
      IMPORTED_LOCATION "${LIBUSB_DLL}"
      IMPORTED_IMPLIB "${LIBUSB_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${LIBUSB_INCLUDE_DIR}"
    )

    set(LIBUSB_FOUND TRUE CACHE BOOL "libusb found" FORCE)
    set(LIBUSB_USE_DLL TRUE CACHE BOOL "Using libusb DLL" FORCE)

    message(STATUS "libusb ${LIBUSB_VERSION} (${LIBUSB_VS_DIR}):")
    message(STATUS "  Include: ${LIBUSB_INCLUDE_DIR}")
    message(STATUS "  Library: ${LIBUSB_LIBRARY}")
    message(STATUS "  DLL:     ${LIBUSB_DLL}")
  endif()

  # ====================================================================
  # pthreads4w - Build from source for static linking
  # ====================================================================
  # pthreads4w is the official continuation of pthreads-win32
  # Building from source ensures proper static library support

  FetchContent_Declare(
    pthreads4w
    GIT_REPOSITORY "https://github.com/jwinarske/pthreads4w.git"
    GIT_TAG "Version-3-0-0-release"
    GIT_SHALLOW TRUE
  )

  message(STATUS "Downloading pthreads4w ${PTHREADS4W_VERSION}...")
  FetchContent_MakeAvailable(pthreads4w)

  # pthreads4w has its own CMakeLists.txt that creates pthreadVC3 and pthreadVC3-static targets
  # We'll use the static target
  if(TARGET pthreadVC3-static)
    # Configure the static target for our needs
    target_compile_definitions(pthreadVC3-static PUBLIC PTW32_STATIC_LIB)

    # Create an alias for easier reference
    add_library(pthreads4w_static ALIAS pthreadVC3-static)

    # Set paths for downstream use
    set(THREADS_PTHREADS_INCLUDE_DIR "${pthreads4w_SOURCE_DIR}" CACHE PATH "pthreads include directory" FORCE)
    set(THREADS_PTHREADS_WIN32_LIBRARY pthreadVC3-static CACHE STRING "pthreads library target" FORCE)

    message(STATUS "pthreads4w source: ${pthreads4w_SOURCE_DIR}")
    message(STATUS "pthreads4w using pthreadVC3-static target")
  elseif(EXISTS "${pthreads4w_SOURCE_DIR}/pthread.c")
    # Fallback: Build manually if CMake targets not available
    add_library(pthreads4w_static STATIC
      "${pthreads4w_SOURCE_DIR}/pthread.c"
    )
    target_include_directories(pthreads4w_static PUBLIC "${pthreads4w_SOURCE_DIR}")
    target_compile_definitions(pthreads4w_static PUBLIC
      PTW32_STATIC_LIB
      HAVE_CONFIG_H
    )
    target_compile_definitions(pthreads4w_static PRIVATE
      PTW32_BUILD
      _TIMESPEC_DEFINED
    )
    # Use static runtime
    target_compile_options(pthreads4w_static PRIVATE
      $<$<CONFIG:Debug>:/MTd>
      $<$<CONFIG:Release>:/MT>
    )

    set(THREADS_PTHREADS_INCLUDE_DIR "${pthreads4w_SOURCE_DIR}" CACHE PATH "pthreads include directory" FORCE)
    set(THREADS_PTHREADS_WIN32_LIBRARY pthreads4w_static CACHE STRING "pthreads library target" FORCE)

    message(STATUS "pthreads4w built manually from source")
  else()
    message(FATAL_ERROR "pthreads4w: No suitable build method found")
  endif()

endif() # MSVC

########################################################################
# Code Coverage Support (GCC/Clang only)
########################################################################
option(ENABLE_COVERAGE "Enable code coverage with gcov" OFF)

if(ENABLE_COVERAGE AND NOT MSVC)
  message(STATUS "Code coverage enabled - adding coverage flags to all targets")
  set(COVERAGE_COMPILE_FLAGS "-fprofile-arcs -ftest-coverage")
  set(COVERAGE_LINK_FLAGS "-lgcov --coverage")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COVERAGE_COMPILE_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${COVERAGE_LINK_FLAGS}")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${COVERAGE_LINK_FLAGS}")
endif()

add_subdirectory(libhydrasdr)
add_subdirectory(hydrasdr-tools)

########################################################################
# CTest Integration (for running tests)
########################################################################
# Enable CTest at top level so 'ctest' command works from build directory
include(CTest)
enable_testing()

########################################################################
# Create uninstall target
########################################################################

configure_file(
  ${PROJECT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
@ONLY)

add_custom_target(uninstall
  COMMENT "Provide uninstall target"
  COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
)

########################################################################
# Copy Files for Windows builds
########################################################################

if(WIN32 AND MINGW)
  message("copy libusb-1.0.dll for MinGW")

  # Find the directory where the compiler is (e.g. .../mingw64/bin)
  get_filename_component(_MINGW_BIN ${CMAKE_C_COMPILER} DIRECTORY)

  # Find the DLL in that directory
  find_file(LIBUSB_1_0_DLL libusb-1.0.dll
    HINTS ${_MINGW_BIN}
    NO_DEFAULT_PATH
  )

  if(LIBUSB_1_0_DLL)
    message(STATUS "Found libusb-1.0.dll at ${LIBUSB_1_0_DLL}")
    configure_file(
      ${LIBUSB_1_0_DLL}
      ${CMAKE_CURRENT_BINARY_DIR}/hydrasdr-tools/src/libusb-1.0.dll
      COPYONLY)
  else()
    message(WARNING "libusb-1.0.dll not found in ${_MINGW_BIN}, skipping copy.")
  endif()
endif()

# For MSVC builds: libusb DLL copy is handled by POST_BUILD commands in hydrasdr-tools/src/CMakeLists.txt
# This ensures DLLs are copied at BUILD time (not just configure time) and works reliably on first build
if(MSVC AND LIBUSB_USE_DLL)
  message(STATUS "libusb DLL will be copied at build time via POST_BUILD commands")
  # Also install libusb DLL and import library for 'cmake --install' target
  if(LIBUSB_DLL AND EXISTS "${LIBUSB_DLL}")
    install(FILES "${LIBUSB_DLL}" DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime)
  endif()
  if(LIBUSB_LIBRARY AND EXISTS "${LIBUSB_LIBRARY}")
    install(FILES "${LIBUSB_LIBRARY}" DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development)
  endif()
endif()

########################################################################
# FetchContent Dependencies Summary
########################################################################

message(STATUS "")
message(STATUS "========== Dependency Configuration ==========")
if(MSVC)
  message(STATUS "Build Type:           MSVC (Visual Studio)")
  message(STATUS "")
  message(STATUS "FetchContent Dependencies:")
  message(STATUS "  libusb:")
  message(STATUS "    Version:          ${LIBUSB_VERSION}")
  message(STATUS "    Include:          ${LIBUSB_INCLUDE_DIR}")
  message(STATUS "    Library:          ${LIBUSB_LIBRARY}")
  message(STATUS "    DLL:              ${LIBUSB_DLL}")
  message(STATUS "    Linking:          Dynamic (DLL)")
  message(STATUS "")
  message(STATUS "  pthreads4w:")
  message(STATUS "    Version:          ${PTHREADS4W_VERSION}")
  message(STATUS "    Source:           ${pthreads4w_SOURCE_DIR}")
  message(STATUS "    Linking:          Static (no DLL dependency)")
  message(STATUS "")
elseif(MINGW)
  message(STATUS "Build Type:           MinGW")
  message(STATUS "  libusb:             System (via pkg-config or manual path)")
  message(STATUS "  pthreads:           Static linking (-Wl,-Bstatic -lwinpthread)")
else()
  message(STATUS "Build Type:           Unix/Linux")
  message(STATUS "  libusb:             System (via pkg-config)")
  message(STATUS "  pthreads:           System (POSIX threads)")
endif()
message(STATUS "===============================================")
message(STATUS "")
