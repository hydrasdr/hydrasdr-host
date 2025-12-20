# Copyright 2012 Jared Boone
# Copyright 2013-2025 Benjamin Vernoux
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
project(libhydrasdr LANGUAGES C)
include(GNUInstallDirs)
include(GenerateExportHeader)

file(READ ${CMAKE_CURRENT_SOURCE_DIR}/src/hydrasdr.h HYDRASDR_H_CONTENTS)

STRING(REGEX MATCH "\#define HYDRASDR_VER_MAJOR[ \t]+([0-9]+)"
  HYDRASDR_VER_MJ ${HYDRASDR_H_CONTENTS})
STRING(REGEX MATCH "([0-9]+)" HYDRASDR_VER_MJ ${HYDRASDR_VER_MJ})

STRING(REGEX MATCH "\#define HYDRASDR_VER_MINOR[ \t]+([0-9]+)"
  HYDRASDR_VER_MI ${HYDRASDR_H_CONTENTS})
STRING(REGEX MATCH "([0-9]+)" HYDRASDR_VER_MI ${HYDRASDR_VER_MI})

STRING(REGEX MATCH "\#define HYDRASDR_VER_REVISION[ \t]+([0-9]+)"
  HYDRASDR_VER_RE ${HYDRASDR_H_CONTENTS})
STRING(REGEX MATCH "([0-9]+)" HYDRASDR_VER_RE ${HYDRASDR_VER_RE})

set(HYDRASDR_VER_MAJOR ${HYDRASDR_VER_MJ})
set(HYDRASDR_VER_MINOR ${HYDRASDR_VER_MI})
set(HYDRASDR_VER_REVISION ${HYDRASDR_VER_RE})

set(VERSION_STRING ${HYDRASDR_VER_MAJOR}.${HYDRASDR_VER_MINOR})
set(VERSION ${VERSION_STRING})
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../cmake/modules)

if(APPLE)
  set(GR_LIBRARY_DIR "lib")
  if(NOT CMAKE_INSTALL_NAME_DIR)
    set(CMAKE_INSTALL_NAME_DIR
      ${CMAKE_INSTALL_PREFIX}/${GR_LIBRARY_DIR} CACHE
      PATH "Library Install Name Destination Directory" FORCE)
  endif(NOT CMAKE_INSTALL_NAME_DIR)
  if(NOT CMAKE_INSTALL_RPATH)
    set(CMAKE_INSTALL_RPATH
      ${CMAKE_INSTALL_PREFIX}/${GR_LIBRARY_DIR} CACHE
      PATH "Library Install RPath" FORCE)
  endif(NOT CMAKE_INSTALL_RPATH)
  if(NOT CMAKE_BUILD_WITH_INSTALL_RPATH)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH ON CACHE
      BOOL "Do Build Using Library Install RPath" FORCE)
  endif(NOT CMAKE_BUILD_WITH_INSTALL_RPATH)
endif(APPLE)

if(MSVC)
  set(THREADS_USE_PTHREADS_WIN32 true)
else()
  add_definitions(-Wall)

  include(TestBigEndian)
  TEST_BIG_ENDIAN(BIGENDIAN)
  if(${BIGENDIAN})
    add_definitions(-DHYDRASDR_BIG_ENDIAN)
  endif(${BIGENDIAN})
endif()

find_package(LIBUSB REQUIRED)
find_package(Threads REQUIRED)

add_subdirectory(src)

########################################################################
# Set library versioning for proper symlink creation by ldconfig
# This ensures ldconfig creates:
#   libhydrasdr.so -> libhydrasdr.so.X
#   libhydrasdr.so.X -> libhydrasdr.so.X.Y.Z
#   libhydrasdr.so.X.Y.Z (actual library)
########################################################################
set_target_properties(hydrasdr PROPERTIES
    VERSION ${HYDRASDR_VER_MAJOR}.${HYDRASDR_VER_MINOR}.${HYDRASDR_VER_REVISION}
    SOVERSION ${HYDRASDR_VER_MAJOR}
)

########################################################################
# Create Cmake Config-file package interface
########################################################################
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/HydraSDR/HydraSDRConfigVersion.cmake"
  VERSION ${VERSION_STRING}
  COMPATIBILITY AnyNewerVersion
)

export(EXPORT HydraSDRTargets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/HydraSDR/HydraSDRTargets.cmake"
  NAMESPACE HydraSDR::
)
configure_file(HydraSDRConfig.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/HydraSDR/HydraSDRConfig.cmake"
  COPYONLY
)

set(CONFIG_PACKAGE_LOCATION ${CMAKE_INSTALL_LIBDIR}/cmake/HydraSDR)
install(EXPORT HydraSDRTargets
  FILE
    HydraSDRTargets.cmake
  NAMESPACE
    HydraSDR::
  DESTINATION
    ${CONFIG_PACKAGE_LOCATION}
)
install(
  FILES
    HydraSDRConfig.cmake
    ../cmake/modules/FindLIBUSB.cmake
    "${CMAKE_CURRENT_BINARY_DIR}/HydraSDR/HydraSDRConfigVersion.cmake"
  DESTINATION
    ${CONFIG_PACKAGE_LOCATION}
  COMPONENT
    Devel
)

########################################################################
# Create Pkg Config File
########################################################################
FOREACH(inc ${LIBUSB_INCLUDE_DIR})
  LIST(APPEND HYDRASDR_PC_CFLAGS "-I${inc}")
ENDFOREACH(inc)

# use space-separation format for the pc file
STRING(REPLACE ";" " " HYDRASDR_PC_CFLAGS "${HYDRASDR_PC_CFLAGS}")
STRING(REPLACE ";" " " HYDRASDR_PC_LIBS "${HYDRASDR_PC_LIBS}")

# unset these vars to avoid hard-coded paths to cross environment
IF(CMAKE_CROSSCOMPILING)
  UNSET(HYDRASDR_PC_CFLAGS)
  UNSET(HYDRASDR_PC_LIBS)
ENDIF(CMAKE_CROSSCOMPILING)

set(prefix ${CMAKE_INSTALL_PREFIX})
set(exec_prefix \${prefix})
set(libdir \${exec_prefix}/lib${LIB_SUFFIX})
set(includedir \${prefix}/include)

CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/libhydrasdr.pc.in
  ${CMAKE_CURRENT_BINARY_DIR}/libhydrasdr.pc
@ONLY)

INSTALL(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/libhydrasdr.pc
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
)

########################################################################
# Create uninstall target
########################################################################
if(NOT hydrasdr_all_SOURCE_DIR)
configure_file(
  ${PROJECT_SOURCE_DIR}/../cmake/cmake_uninstall.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
@ONLY)

add_custom_target(uninstall
  COMMENT "Provide libhydrasdr uninstall target"
  COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
)
endif()
