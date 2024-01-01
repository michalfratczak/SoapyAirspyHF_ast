###################################################
# Build Soapy SDR support module for Airspy Devices
###################################################

cmake_minimum_required(VERSION 3.5)

project(SoapyAirspyHF CXX)
enable_testing()

# Export compile_commands.json by default.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# This module needs c++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS OFF)

#select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
   message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")

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
    -Wstrict-aliasing=2
  )
endif()

find_package(SoapySDR CONFIG)

if (NOT SoapySDR_FOUND)
    message(WARNING "SoapySDR development files not found - skipping support")
    return()
endif ()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

# Find libairspyhf with pkg_config
# https://github.com/airspy/airspyhf
find_package(PkgConfig REQUIRED)
pkg_check_modules(AIRSPYHF REQUIRED IMPORTED_TARGET libairspyhf)

find_package(fmt REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${AIRSPYHF_INCLUDE_DIRS})

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
  PkgConfig::AIRSPYHF
  fmt::fmt
)
