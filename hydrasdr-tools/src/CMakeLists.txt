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

# Standard tools use shared library if available, otherwise static
if(TARGET HydraSDR::hydrasdr)
  list(APPEND TOOLS_LINK_LIBS HydraSDR::hydrasdr)
else()
  list(APPEND TOOLS_LINK_LIBS hydrasdr_static)
endif()

# DDC test tools must use static library (iqconverter is internal API)
# Use direct target name for in-tree builds
list(APPEND DDC_TOOLS_LINK_LIBS hydrasdr_static)

if(MSVC)
  add_library(libgetopt_static STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../getopt/getopt.c)
  list(APPEND TOOLS_LINK_LIBS libgetopt_static)
endif()

add_executable(hydrasdr_gpio hydrasdr_gpio.c)
target_link_libraries(hydrasdr_gpio ${TOOLS_LINK_LIBS})
install(TARGETS hydrasdr_gpio RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(hydrasdr_gpiodir hydrasdr_gpiodir.c)
target_link_libraries(hydrasdr_gpiodir ${TOOLS_LINK_LIBS})
install(TARGETS hydrasdr_gpiodir RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(hydrasdr_lib_version hydrasdr_lib_version.c)
target_link_libraries(hydrasdr_lib_version ${TOOLS_LINK_LIBS})
install(TARGETS hydrasdr_lib_version RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# Clock generator tool (modern HAL-based API)
add_executable(hydrasdr_clockgen hydrasdr_clockgen.c)
target_link_libraries(hydrasdr_clockgen ${TOOLS_LINK_LIBS})
install(TARGETS hydrasdr_clockgen RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# Legacy SI5351C tool (deprecated, kept for backward compatibility)
add_executable(hydrasdr_si5351c hydrasdr_si5351c.c)
target_link_libraries(hydrasdr_si5351c ${TOOLS_LINK_LIBS})
install(TARGETS hydrasdr_si5351c RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# Legacy R82x tuner tool (deprecated, RFOne only - use hydrasdr_rf_frontend instead)
add_executable(hydrasdr_r82x hydrasdr_r82x.c)
target_link_libraries(hydrasdr_r82x ${TOOLS_LINK_LIBS})
install(TARGETS hydrasdr_r82x RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# RF frontend register access tool (modern HAL-based API)
add_executable(hydrasdr_rf_frontend hydrasdr_rf_frontend.c)
target_link_libraries(hydrasdr_rf_frontend ${TOOLS_LINK_LIBS})
install(TARGETS hydrasdr_rf_frontend RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(hydrasdr_spiflash hydrasdr_spiflash.c)
target_link_libraries(hydrasdr_spiflash ${TOOLS_LINK_LIBS})
install(TARGETS hydrasdr_spiflash RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(hydrasdr_calibrate hydrasdr_calibrate.c)
target_link_libraries(hydrasdr_calibrate ${TOOLS_LINK_LIBS})
install(TARGETS hydrasdr_calibrate RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(hydrasdr_info hydrasdr_info.c)
target_link_libraries(hydrasdr_info ${TOOLS_LINK_LIBS})
install(TARGETS hydrasdr_info RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# Note: DLL copying is handled by libhydrasdr's POST_BUILD on the hydrasdr target
# This ensures DLLs are copied whenever the library is rebuilt

add_executable(hydrasdr_rx hydrasdr_rx.c)
target_link_libraries(hydrasdr_rx ${TOOLS_LINK_LIBS})
install(TARGETS hydrasdr_rx RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# Async RX tool - modern cross-platform streaming example
add_executable(hydrasdr_async_rx hydrasdr_async_rx.c)
target_link_libraries(hydrasdr_async_rx ${TOOLS_LINK_LIBS})
install(TARGETS hydrasdr_async_rx RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(hydrasdr_reset hydrasdr_reset.c)
target_link_libraries(hydrasdr_reset ${TOOLS_LINK_LIBS})
install(TARGETS hydrasdr_reset RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(hydrasdr_set_rf_port hydrasdr_set_rf_port.c)
target_link_libraries(hydrasdr_set_rf_port ${TOOLS_LINK_LIBS})
install(TARGETS hydrasdr_set_rf_port RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(hydrasdr_list_devices hydrasdr_list_devices.c)
target_link_libraries(hydrasdr_list_devices ${TOOLS_LINK_LIBS})
install(TARGETS hydrasdr_list_devices RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# DDC benchmark tool - uses internal iqconverter API (requires static linking)
add_executable(hydrasdr_ddc_benchmark hydrasdr_ddc_benchmark.c cpu_features.c)
target_include_directories(hydrasdr_ddc_benchmark PRIVATE ${CMAKE_SOURCE_DIR}/libhydrasdr/src ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(hydrasdr_ddc_benchmark PRIVATE
  ADD_EXPORTS
  BUILD_OPT_FLAGS="${CMAKE_C_FLAGS_RELEASE}"
)
target_link_libraries(hydrasdr_ddc_benchmark ${DDC_TOOLS_LINK_LIBS})
install(TARGETS hydrasdr_ddc_benchmark RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# API benchmark tool - comprehensive API & USB latency testing
# Uses internal iqconverter API (requires static linking)
add_executable(hydrasdr_api_benchmark hydrasdr_api_benchmark.c cpu_features.c)
target_include_directories(hydrasdr_api_benchmark PRIVATE ${CMAKE_SOURCE_DIR}/libhydrasdr/src ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(hydrasdr_api_benchmark PRIVATE ADD_EXPORTS)
target_link_libraries(hydrasdr_api_benchmark ${DDC_TOOLS_LINK_LIBS})
if(NOT MSVC)
  target_link_libraries(hydrasdr_api_benchmark m)
endif()
install(TARGETS hydrasdr_api_benchmark RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

#######################################################################
# Copy DLLs to tools output directory
#######################################################################
if(WIN32)
  if(TARGET hydrasdr)
    if(MSVC)
      # MSVC: Copy to config-specific Release directory
      set(_TOOLS_RELEASE_DIR "${CMAKE_CURRENT_BINARY_DIR}/Release")
      add_custom_target(copy_dlls ALL
        COMMAND ${CMAKE_COMMAND} -E make_directory "${_TOOLS_RELEASE_DIR}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${CMAKE_CURRENT_BINARY_DIR}/hydrasdr.dll"
          "${_TOOLS_RELEASE_DIR}/"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${LIBUSB_DLL}"
          "${_TOOLS_RELEASE_DIR}/"
        COMMENT "Copying hydrasdr.dll and libusb-1.0.dll to Release directory"
      )
      add_dependencies(copy_dlls hydrasdr hydrasdr_info)
    elseif(MINGW)
      # MinGW: Copy to tools directory
      if(LIBUSB_1_0_DLL AND EXISTS "${LIBUSB_1_0_DLL}")
        add_custom_target(copy_dlls ALL
          COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:hydrasdr>"
            "$<TARGET_FILE_DIR:hydrasdr_info>/"
          COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${LIBUSB_1_0_DLL}"
            "$<TARGET_FILE_DIR:hydrasdr_info>/"
          COMMENT "Copying hydrasdr.dll and libusb-1.0.dll to tools directory"
        )
        add_dependencies(copy_dlls hydrasdr hydrasdr_info)
      else()
        add_custom_target(copy_dlls ALL
          COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:hydrasdr>"
            "$<TARGET_FILE_DIR:hydrasdr_info>/"
          COMMENT "Copying hydrasdr.dll to tools directory"
        )
        add_dependencies(copy_dlls hydrasdr hydrasdr_info)
      endif()
    endif()
  endif()
endif()
