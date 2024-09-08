# ##############################################################################
# Build Soapy SDR support module for Airspy Devices
# ##############################################################################

# Copyright 2024 SM6WJM

cmake_minimum_required(VERSION 3.5)

project(SoapyAirspyHF CXX)
enable_testing()

# Export compile_commands.json by default.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# This module needs c++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS OFF)

# Try to find cppcheck
find_program(CPPCHECK_EXECUTABLE NAMES cppcheck)

# Try to find include-what-you-use
find_program(IWYU_EXECUTABLE NAMES include-what-you-use)

# Try to find clang-tidy
find_program(CLANG_TIDY_EXECUTABLE NAMES clang-tidy)

# Run clang-tidy if we found it.
if(CLANG_TIDY_EXECUTABLE)
  set(CMAKE_CXX_CLANG_TIDY ${CLANG_TIDY_EXECUTABLE})
else()
  message(WARNING "clang-tidy not found - skipping static analysis")
endif()

# If we have cppcheck, use it.
if(CPPCHECK_EXECUTABLE)
  set(CMAKE_CXX_CPPCHECK
      ${CPPCHECK_EXECUTABLE}
    "--suppressions-list=${CMAKE_SOURCE_DIR}/.cppcheck-suppressions.txt")
else()
  message(WARNING "cppcheck not found - skipping static analysis")
endif()

# Include what you use
if(IWYU_EXECUTABLE) # include-what-you-use
  set(CMAKE_CXX_INCLUDE_WHAT_YOU_USE ${IWYU_EXECUTABLE})
else()
  message(WARNING "include-what-you-use not found - skipping static analysis")
endif()

# select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
  message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE
    ${CMAKE_BUILD_TYPE}
    CACHE STRING "")

# Add strict compiler flags.
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
    -Wstrict-aliasing=2)
endif()

find_package(SoapySDR CONFIG)

if(NOT SoapySDR_FOUND)
  message(WARNING "SoapySDR development files not found - skipping support")
  return()
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

# Find libairspyhf with pkg_config https://github.com/airspy/airspyhf
find_package(PkgConfig REQUIRED)
pkg_check_modules(AIRSPYHF REQUIRED IMPORTED_TARGET libairspyhf)

# fmt is for string formatting
find_package(fmt REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${AIRSPYHF_INCLUDE_DIRS})

list(APPEND AIRSPYHF_LIBS ${LIBAIRSPYHF_LIBRARIES})

soapy_sdr_module_util(
  TARGET
  airspyhfSupport
  SOURCES
  src/SoapyAirspyHF.hpp
  src/Registration.cpp
  src/Settings.cpp
  src/Streaming.cpp
  src/RingBuffer.hpp
  LIBRARIES
  PkgConfig::AIRSPYHF
  fmt::fmt)
