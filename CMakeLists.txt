###################################################
# Build Soapy SDR support module for Airspy Devices
###################################################

cmake_minimum_required(VERSION 3.5)

project(SoapyAirspyHF CXX)

# This module needs c++17
# Seems that SoapySDR 0.7 overrides this to C++11.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS OFF)

# Create a compile_commands.json file for use with clang tooling
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -pedantic
    -Wshadow
    -Wextra
    -Wpointer-arith
    -Wcast-qual
    -Wcast-align
    -Wconversion
    -Wstrict-aliasing=2
    # -faligned-new # allow alignas in C++11.
  )
endif()

find_package(SoapySDR "0.7.0" NO_MODULE REQUIRED)
if (NOT SoapySDR_FOUND)
    message(FATAL_ERROR "Soapy SDR development files not found...")
endif ()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(LibAIRSPYHF)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${LIBAIRSPYHF_INCLUDE_DIRS})

list(APPEND AIRSPYHF_LIBS ${LIBAIRSPYHF_LIBRARIES})

SOAPY_SDR_MODULE_UTIL(
  TARGET airspyhfSupport
  SOURCES
  SoapyAirspyHF.hpp
  Registration.cpp
  Settings.cpp
  Streaming.cpp
  RingBuffer.hpp
  LIBRARIES
  ${AIRSPYHF_LIBS}
)

########################################################################
# uninstall target
########################################################################
add_custom_target(uninstall
    "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)
