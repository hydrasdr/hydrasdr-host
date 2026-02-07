# Copyright (c) 2012, Jared Boone <jared@sharebrained.com>
# Copyright (c) 2013, Michael Ossmann <mike@ossmann.com>
# Copyright (c) 2013, Youssef Touil <youssef@airspy.com>
# Copyright (c) 2013-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
#
# This file is part of HydraSDR (based on AirSpy & HackRF project).
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following
# disclaimer. Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided with
# the distribution. Neither the name of HydraSDR nor the
# names of its contributors may be used to endorse or promote products
# derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

option(ENABLE_STATIC_LIB "Build and Install libhydrasdr.a static library" ON)
option(ENABLE_SHARED_LIB "Build and Install libhydrasdr.so shared library" ON)
option(
  DISABLE_USB_DEVICE_DISCOVERY
  "Prevent libusb from trying to enumerate devices. Useful on non-root android"
  ANDROID)

# Targets
set(_C_SOURCES_
  ${CMAKE_CURRENT_SOURCE_DIR}/hydrasdr.c
  ${CMAKE_CURRENT_SOURCE_DIR}/hydrasdr_shared.c
  ${CMAKE_CURRENT_SOURCE_DIR}/hydrasdr_rfone.c
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter.c
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_lut.c
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_float.c
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_int16.c
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_float_opt.c
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_int16_opt.c
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_float_opt33.c
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_int16_opt33.c
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_float_opt65.c
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_int16_opt65.c
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_float_opt83.c
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_int16_opt83.c
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_decimator.c
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_float_opt_dec.c
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_int16_opt_dec.c
  CACHE INTERNAL "List of C sources")
set(_C_HEADERS_
  ${CMAKE_CURRENT_SOURCE_DIR}/hydrasdr.h
  ${CMAKE_CURRENT_SOURCE_DIR}/hydrasdr_commands.h
  ${CMAKE_CURRENT_SOURCE_DIR}/hydrasdr_internal.h
  ${CMAKE_CURRENT_SOURCE_DIR}/hydrasdr_shared.h
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter.h
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_lut.h
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_float.h
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_int16.h
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_float_opt.h
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_int16_opt.h
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_float_opt33.h
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_int16_opt33.h
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_float_opt65.h
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_int16_opt65.h
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_float_opt83.h
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_int16_opt83.h
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_decimator.h
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_float_opt_dec.h
  ${CMAKE_CURRENT_SOURCE_DIR}/iqconverter_int16_opt_dec.h
  ${CMAKE_CURRENT_SOURCE_DIR}/filters.h
  ${CMAKE_CURRENT_SOURCE_DIR}/filters_opt.h
  ${CMAKE_CURRENT_SOURCE_DIR}/compat_opt.h
  ${CMAKE_CURRENT_SOURCE_DIR}/spsc_queue.h
  ${CMAKE_CURRENT_SOURCE_DIR}/buffer_pool.h
  CACHE INTERNAL "List of C headers")

# For cygwin just force UNIX OFF and WIN32 ON
if( ${CYGWIN} )
  SET(UNIX OFF)
  SET(WIN32 ON)
endif( ${CYGWIN} )

if(${WIN32})
  if(NOT MINGW)
    list(APPEND _C_SOURCES_ win32/hydrasdr.rc)
  endif()
endif()

if(MINGW)
  # This gets us DLL resource information when compiling on MinGW.
  if(NOT CMAKE_RC_COMPILER)
    set(CMAKE_RC_COMPILER windres.exe)
  endif()

  add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/hydrasdrrc.obj
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/hydrasdr.h
    COMMENT "Resource compiler used to generate hydrasdr.obj"
    COMMAND ${CMAKE_RC_COMPILER}
    -D GCC_WINDRES
    -I ${CMAKE_CURRENT_SOURCE_DIR}
    -I ${CMAKE_CURRENT_BINARY_DIR}
    -o ${CMAKE_CURRENT_BINARY_DIR}/hydrasdrrc.obj
    -i ${CMAKE_CURRENT_SOURCE_DIR}/win32/hydrasdr.rc)
  set(HYDRASDR_DLL_OBJS ${CMAKE_CURRENT_BINARY_DIR}/hydrasdrrc.obj)
  SET_SOURCE_FILES_PROPERTIES(
    ${HYDRASDR_DLL_OBJS}
    PROPERTIES
    EXTERNAL_OBJECT true
    GENERATED true)
endif(MINGW)

# Settings common for shared and static libraries
function(libhydrasdr_common_settings libtarget)
  set_target_properties(${libtarget} PROPERTIES CLEAN_DIRECT_OUTPUT 1)

  target_include_directories(
    ${libtarget} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/libhydrasdr>)

  # ============================================================================
  # PORTABLE OPTIMIZATION FLAGS
  # These flags improve performance while maintaining portability across:
  # - x86/x86-64: GCC, Clang, MinGW, MSVC
  # - ARM32/ARM64: GCC, Clang (RPi3/4/5, Apple Silicon)
  # ============================================================================

  if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang" OR MINGW)
    # Base optimization (Release only)
    target_compile_options(${libtarget} PRIVATE
      $<$<CONFIG:Release>:-O3>
    )

    # Vectorization and loop optimizations (Release only)
    target_compile_options(${libtarget} PRIVATE
      $<$<CONFIG:Release>:-ftree-vectorize>      # Enable auto-vectorization
      $<$<CONFIG:Release>:-funroll-loops>        # Unroll loops for better pipelining
      $<$<CONFIG:Release>:-fomit-frame-pointer>  # Free up a register
    )

    # Safe floating-point optimizations (Release only)
    # These are safe for DSP/SDR: no NaN/Inf handling needed, no errno
    target_compile_options(${libtarget} PRIVATE
      $<$<CONFIG:Release>:-fno-math-errno>       # Don't set errno for math funcs
      $<$<CONFIG:Release>:-fno-trapping-math>    # Assume no FP traps
      $<$<CONFIG:Release>:-fno-signed-zeros>     # Ignore signed zero (-0.0)
    )

    # GCC-specific optimizations
    if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
      target_compile_options(${libtarget} PRIVATE
        $<$<CONFIG:Release>:-ftree-loop-distribution>  # Better cache usage
        $<$<CONFIG:Release>:-ftree-loop-vectorize>     # Loop vectorization
      )
      # GCC 11+ has improved vectorizer
      if(CMAKE_C_COMPILER_VERSION VERSION_GREATER_EQUAL "11.0")
        target_compile_options(${libtarget} PRIVATE
          $<$<CONFIG:Release>:-fvect-cost-model=dynamic>  # Better vector decisions
        )
      endif()
    endif()

    # Clang-specific optimizations
    if(CMAKE_C_COMPILER_ID STREQUAL "Clang")
      target_compile_options(${libtarget} PRIVATE
        $<$<CONFIG:Release>:-fvectorize>           # Enable vectorization
        $<$<CONFIG:Release>:-fslp-vectorize>       # Enable SLP vectorization
      )
    endif()

    # ARM-specific optimizations (RPi3/4/5, Apple Silicon)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
      # ARM64: NEON is mandatory, optimize for native CPU
      message(STATUS "ARM64 detected: enabling native CPU optimizations")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7|armv6|arm")
      # ARM32 (RPi3 in 32-bit mode): Enable NEON/VFP if available
      # Android NDK only supports softfp, skip hard float ABI
      if(NOT ANDROID)
        target_compile_options(${libtarget} PRIVATE
          $<$<CONFIG:Release>:-mfpu=neon-vfpv4>      # Enable NEON
          $<$<CONFIG:Release>:-mfloat-abi=hard>      # Use hardware FP
        )
      endif()
      message(STATUS "ARM32 detected: enabling NEON/VFPv4 optimizations")
    endif()

  endif()

  # MSVC optimizations (Visual Studio 2019/2022+)
  if(MSVC)
    target_compile_options(${libtarget} PRIVATE
      # Fast floating-point (safe for DSP)
      $<$<CONFIG:Release>:/fp:fast>
      # Enable intrinsic functions
      $<$<CONFIG:Release>:/Oi>
      # Favor fast code over small code
      $<$<CONFIG:Release>:/Ot>
      # Whole program optimization
      $<$<CONFIG:Release>:/GL>
    )
    # Enable AVX for x64 builds (baseline for modern CPUs)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
      target_compile_options(${libtarget} PRIVATE
        $<$<CONFIG:Release>:/arch:AVX>
      )
      message(STATUS "MSVC x64: enabling AVX baseline")
    endif()
    # Linker optimization for whole program optimization
    target_link_options(${libtarget} PRIVATE
      $<$<CONFIG:Release>:/LTCG>
    )
  endif()

  # Dependencies
  target_link_libraries(${libtarget} PRIVATE LIBUSB::LIBUSB)

  if(MINGW)
    # For MinGW, we want to link system libs and pthreads statically
    # so the DLL doesn't depend on libwinpthread-1.dll or libgcc_s_seh-1.dll.
    target_link_options(${libtarget} PRIVATE -static-libgcc -static-libstdc++)

    # Link winpthread statically, then switch back to dynamic for other libs (like libusb)
    target_link_libraries(${libtarget} PRIVATE -Wl,-Bstatic -lwinpthread -Wl,-Bdynamic)
  elseif(MSVC AND TARGET pthreads4w_static)
    # MSVC with FetchContent: pthreads handled in WIN32 section below
  elseif(TARGET Threads::Threads)
    # Standard behavior for Linux (POSIX threads)
    target_link_libraries(${libtarget} PRIVATE Threads::Threads)
  endif()

  if( ${UNIX} )
    install(TARGETS ${libtarget} EXPORT HydraSDRTargets
      LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
      COMPONENT sharedlibs)
  endif( ${UNIX} )
  if( ${WIN32} )
    if(MSVC AND TARGET pthreads4w_static)
      # Use FetchContent-built pthreads4w static library (no DLL dependency)
      target_link_libraries(${libtarget} PRIVATE pthreads4w_static)
      target_compile_definitions(${libtarget} PRIVATE PTW32_STATIC_LIB)
    elseif( THREADS_PTHREADS_WIN32_LIBRARY AND NOT MINGW )
      # Fallback: use external pthread lib definition if NOT MinGW (handled above)
      target_link_libraries(${libtarget} PRIVATE ${THREADS_PTHREADS_WIN32_LIBRARY} )
    endif()
    if( THREADS_PTHREADS_INCLUDE_DIR )
      target_include_directories(${libtarget} PRIVATE ${THREADS_PTHREADS_INCLUDE_DIR})
    endif()
    target_include_directories(${libtarget} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/win32 )
    if(MSVC AND TARGET pthreads4w_static)
      # MSVC FetchContent builds: install without export (self-contained)
      install(TARGETS ${libtarget}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        COMPONENT sharedlibs)
    elseif(MINGW)
      # MinGW builds: install DLL to bin, import lib (.dll.a) to lib
      install(TARGETS ${libtarget} EXPORT HydraSDRTargets
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        COMPONENT sharedlibs)
    else()
      install(TARGETS ${libtarget} EXPORT HydraSDRTargets
        DESTINATION bin
        COMPONENT sharedlibs)
    endif()
  endif( ${WIN32} )

  # Enable static runtime for both debug and release
  if(MSVC)
    target_compile_options(${libtarget} PRIVATE
      $<$<CONFIG:Debug>:/MTd>
      $<$<CONFIG:Release>:/MT>
    )
  endif()
endfunction()

# Dynamic library
if(ENABLE_SHARED_LIB)
  add_library(hydrasdr SHARED ${_C_SOURCES_} ${HYDRASDR_DLL_OBJS})
  set_target_properties(hydrasdr PROPERTIES VERSION
    ${HYDRASDR_VER_MAJOR}.${HYDRASDR_VER_MINOR}.${HYDRASDR_VER_REVISION}
  )
  set_target_properties(hydrasdr PROPERTIES SOVERSION 0)
  libhydrasdr_common_settings(hydrasdr)
  target_compile_definitions(hydrasdr PRIVATE ADD_EXPORTS)
  generate_export_header(hydrasdr)
  if( ${WIN32} )
    set_target_properties(hydrasdr PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY_RELEASE ../../hydrasdr-tools/src)
    # Also copy hydrasdr.dll to config-specific subdirectory where executables are built
    if(MSVC)
      set(TOOLS_OUTPUT_DIR "${CMAKE_BINARY_DIR}/hydrasdr-tools/src/$<CONFIG>")

      # Copy hydrasdr.dll (create directory first!)
      add_custom_command(TARGET hydrasdr POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "${TOOLS_OUTPUT_DIR}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "$<TARGET_FILE:hydrasdr>"
          "${TOOLS_OUTPUT_DIR}/"
        COMMENT "Copying hydrasdr.dll to $<CONFIG> directory"
      )

      # Copy libusb DLL to output directory (always, on every hydrasdr build)
      # LIBUSB_DLL is set as CACHE variable in root CMakeLists.txt
      add_custom_command(TARGET hydrasdr POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${LIBUSB_DLL}"
          "${TOOLS_OUTPUT_DIR}/"
        COMMENT "Copying libusb-1.0.dll to $<CONFIG> directory"
      )
    endif()
  endif( ${WIN32} )
endif()

# Static library
if(ENABLE_STATIC_LIB)
  add_library(hydrasdr_static STATIC ${_C_SOURCES_} ${HYDRASDR_DLL_OBJS})
  libhydrasdr_common_settings(hydrasdr_static)
  target_compile_definitions(hydrasdr_static PRIVATE ADD_EXPORTS)
  if(MSVC)
    set_target_properties(hydrasdr_static PROPERTIES
      OUTPUT_NAME "hydrasdr_static")
  else()
    set_target_properties(hydrasdr_static PROPERTIES
      OUTPUT_NAME "hydrasdr")
  endif()
endif()

install(FILES ${_C_HEADERS_}
  # ${CMAKE_CURRENT_BINARY_DIR}/hydrasdr_export.h
  DESTINATION include/${PROJECT_NAME}
  COMPONENT headers
)

# Verbose compiler information
message(STATUS "")
message(STATUS "========== libhydrasdr Compiler Configuration ==========")
message(STATUS "C Compiler ID:        ${CMAKE_C_COMPILER_ID}")
message(STATUS "C Compiler Version:   ${CMAKE_C_COMPILER_VERSION}")
message(STATUS "C Compiler:           ${CMAKE_C_COMPILER}")
message(STATUS "CXX Compiler ID:      ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "CXX Compiler Version: ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "CXX Compiler:         ${CMAKE_CXX_COMPILER}")
message(STATUS "Build Type:           ${CMAKE_BUILD_TYPE}")
message(STATUS "")
message(STATUS "C Flags:              ${CMAKE_C_FLAGS}")
message(STATUS "C Flags (Debug):      ${CMAKE_C_FLAGS_DEBUG}")
message(STATUS "C Flags (Release):    ${CMAKE_C_FLAGS_RELEASE}")
message(STATUS "C Flags (RelWithDeb): ${CMAKE_C_FLAGS_RELWITHDEBINFO}")
message(STATUS "C Flags (MinSizeRel): ${CMAKE_C_FLAGS_MINSIZEREL}")
message(STATUS "")
message(STATUS "CXX Flags:            ${CMAKE_CXX_FLAGS}")
message(STATUS "CXX Flags (Debug):    ${CMAKE_CXX_FLAGS_DEBUG}")
message(STATUS "CXX Flags (Release):  ${CMAKE_CXX_FLAGS_RELEASE}")
message(STATUS "")
message(STATUS "Linker Flags:         ${CMAKE_EXE_LINKER_FLAGS}")
message(STATUS "Shared Linker Flags:  ${CMAKE_SHARED_LINKER_FLAGS}")
message(STATUS "Static Linker Flags:  ${CMAKE_STATIC_LINKER_FLAGS}")
message(STATUS "")
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang" OR MINGW)
  message(STATUS "Custom Optimization:  -O3 + vectorization + FP optimizations")
endif()
if(MSVC)
  message(STATUS "MSVC Optimization:    /fp:fast /Oi /Ot /GL /LTCG")
endif()
if(MINGW)
  message(STATUS "MinGW Link Options:   -static-libgcc -static-libstdc++")
  message(STATUS "MinGW Static Libs:    -Wl,-Bstatic -lwinpthread -Wl,-Bdynamic")
endif()
message(STATUS "")
message(STATUS "System:               ${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_VERSION}")
message(STATUS "Processor:            ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "=========================================================")
message(STATUS "")
