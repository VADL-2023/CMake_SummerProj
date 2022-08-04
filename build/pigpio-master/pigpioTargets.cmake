# Generated by CMake

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.5)
   message(FATAL_ERROR "CMake >= 2.6.0 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 2.6...3.17)
#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Protect against multiple inclusion, which would fail when already imported targets are added once more.
set(_targetsDefined)
set(_targetsNotDefined)
set(_expectedTargets)
foreach(_expectedTarget pigpio::pigpio pigpio::pigpiod_if pigpio::pigpiod_if2 pigpio::pig2vcd pigpio::pigpiod pigpio::pigs)
  list(APPEND _expectedTargets ${_expectedTarget})
  if(NOT TARGET ${_expectedTarget})
    list(APPEND _targetsNotDefined ${_expectedTarget})
  endif()
  if(TARGET ${_expectedTarget})
    list(APPEND _targetsDefined ${_expectedTarget})
  endif()
endforeach()
if("${_targetsDefined}" STREQUAL "${_expectedTargets}")
  unset(_targetsDefined)
  unset(_targetsNotDefined)
  unset(_expectedTargets)
  set(CMAKE_IMPORT_FILE_VERSION)
  cmake_policy(POP)
  return()
endif()
if(NOT "${_targetsDefined}" STREQUAL "")
  message(FATAL_ERROR "Some (but not all) targets in this export set were already defined.\nTargets Defined: ${_targetsDefined}\nTargets not yet defined: ${_targetsNotDefined}\n")
endif()
unset(_targetsDefined)
unset(_targetsNotDefined)
unset(_expectedTargets)


# Create imported target pigpio::pigpio
add_library(pigpio::pigpio SHARED IMPORTED)

# Create imported target pigpio::pigpiod_if
add_library(pigpio::pigpiod_if SHARED IMPORTED)

# Create imported target pigpio::pigpiod_if2
add_library(pigpio::pigpiod_if2 SHARED IMPORTED)

# Create imported target pigpio::pig2vcd
add_executable(pigpio::pig2vcd IMPORTED)

# Create imported target pigpio::pigpiod
add_executable(pigpio::pigpiod IMPORTED)

# Create imported target pigpio::pigs
add_executable(pigpio::pigs IMPORTED)

# Import target "pigpio::pigpio" for configuration ""
set_property(TARGET pigpio::pigpio APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(pigpio::pigpio PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "/home/pi/CMake_SummerProj/build/pigpio-master/libpigpio.so"
  IMPORTED_SONAME_NOCONFIG "libpigpio.so"
  )

# Import target "pigpio::pigpiod_if" for configuration ""
set_property(TARGET pigpio::pigpiod_if APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(pigpio::pigpiod_if PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "/home/pi/CMake_SummerProj/build/pigpio-master/libpigpiod_if.so"
  IMPORTED_SONAME_NOCONFIG "libpigpiod_if.so"
  )

# Import target "pigpio::pigpiod_if2" for configuration ""
set_property(TARGET pigpio::pigpiod_if2 APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(pigpio::pigpiod_if2 PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "/home/pi/CMake_SummerProj/build/pigpio-master/libpigpiod_if2.so"
  IMPORTED_SONAME_NOCONFIG "libpigpiod_if2.so"
  )

# Import target "pigpio::pig2vcd" for configuration ""
set_property(TARGET pigpio::pig2vcd APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(pigpio::pig2vcd PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "/home/pi/CMake_SummerProj/build/pigpio-master/pig2vcd"
  )

# Import target "pigpio::pigpiod" for configuration ""
set_property(TARGET pigpio::pigpiod APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(pigpio::pigpiod PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "/home/pi/CMake_SummerProj/build/pigpio-master/pigpiod"
  )

# Import target "pigpio::pigs" for configuration ""
set_property(TARGET pigpio::pigs APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(pigpio::pigs PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "/home/pi/CMake_SummerProj/build/pigpio-master/pigs"
  )

# This file does not depend on other imported targets which have
# been exported from the same project but in a separate export set.

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)
