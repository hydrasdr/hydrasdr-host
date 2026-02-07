# Copyright 2012 Jared Boone
# Copyright 2013-2026 Benjamin Vernoux
#
# This file is part of HydraSDR (based on HackRF project).
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

cmake_minimum_required(VERSION 3.10)
project(hydrasdr-tools LANGUAGES C VERSION 1.0)
include(GNUInstallDirs)
set(MAJOR_VERSION 1)
set(MINOR_VERSION 0)
set(PACKAGE hydrasdr-tools)
set(VERSION_STRING ${MAJOR_VERSION}.${MINOR_VERSION})
set(VERSION ${VERSION_STRING})
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../cmake/modules)
add_definitions(-D_FILE_OFFSET_BITS=64)

if(MSVC)
  include_directories(getopt)
  add_definitions(/D _CRT_SECURE_NO_WARNINGS)
  # Enable static runtime for both debug and release
  foreach(flag_var
      CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE
      CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO)
    if(${flag_var} MATCHES "/MD")
      string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
    endif()
  endforeach()
else()
  add_definitions(-Wall)
endif()

if(libhydrasdr_SOURCE_DIR)
  if(TARGET hydrasdr)
    add_library(HydraSDR::hydrasdr ALIAS hydrasdr)
  else()
    add_library(HydraSDR::hydrasdr ALIAS hydrasdr_static)
  endif()
elseif(LIBHYDRASDR_INCLUDE_DIR AND LIBHYDRASDR_LIBRARIES
    AND NOT TARGET HydraSDR::hydrasdr)
  add_library(HydraSDR::hydrasdr INTERFACE IMPORTED)
  set_target_properties(
    HydraSDR::hydrasdr
    PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${LIBHYDRASDR_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "${LIBHYDRASDR_LIBRARIES}")
else()
  message(STATUS "finding library for hydrasdr-tools")
  find_package(HydraSDR REQUIRED)
endif()

add_subdirectory(src)

# Build tests if BUILD_TESTS option is enabled (default: ON) and tests directory exists
option(BUILD_TESTS "Build test executables and enable CTest" ON)
if(BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests")
  # Enable CTest before adding tests subdirectory so tests are registered
  include(CTest)
  enable_testing()
  add_subdirectory(tests)
endif()

########################################################################
# Create uninstall target
########################################################################

if(NOT hydrasdr-host_SOURCE_DIR)
configure_file(
  ${PROJECT_SOURCE_DIR}/../cmake/cmake_uninstall.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
@ONLY)

add_custom_target(uninstall
  COMMENT "Provide hydrasdr-tools uninstall target"
  COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
)
endif()

########################################################################
# Install udev rules
########################################################################
option(INSTALL_UDEV_RULES "Install udev rules for HydraSDR" OFF)
if (INSTALL_UDEV_RULES)
  install (
    FILES 51-hydrasdr.rules
    DESTINATION "/etc/udev/rules.d"
    COMPONENT "udev"
  )
else (INSTALL_UDEV_RULES)
  message (STATUS "Udev rules not being installed.")
  message (STATUS "Install them with -DINSTALL_UDEV_RULES=ON")
endif (INSTALL_UDEV_RULES)

########################################################################
# Install mingw64 runtime DLLs (so 'cmake --build . --target install' includes them)
########################################################################
# You can override it on the configure command line:
#   -DMINGW_MSYS64_BIN_DIR="C:/msys64/mingw64/bin"
set(MINGW_MSYS64_BIN_DIR "C:/msys64/mingw64/bin" CACHE PATH "MSYS2 MinGW64 bin directory (used to find runtime DLLs on MinGW builds)")

if(WIN32 AND MINGW)
  # Common runtime DLLs used by the built tools
  set(_mingw_dlls
    libusb-1.0.dll
    libwinpthread-1.dll
  )

  foreach(_dll IN LISTS _mingw_dlls)
    # prefer forward slashes for portability
    set(_dll_path "${MINGW_MSYS64_BIN_DIR}/${_dll}")
    if(EXISTS "${_dll_path}")
      install(FILES "${_dll_path}" DESTINATION ${CMAKE_INSTALL_BINDIR})
    else()
      message(WARNING "Mingw DLL ${_dll} not found at ${_dll_path}; it will not be installed.")
    endif()
  endforeach()
endif()
