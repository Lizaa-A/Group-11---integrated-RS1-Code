# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_planting_conditions_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED planting_conditions_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(planting_conditions_FOUND FALSE)
  elseif(NOT planting_conditions_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(planting_conditions_FOUND FALSE)
  endif()
  return()
endif()
set(_planting_conditions_CONFIG_INCLUDED TRUE)

# output package information
if(NOT planting_conditions_FIND_QUIETLY)
  message(STATUS "Found planting_conditions: 1.0.3 (${planting_conditions_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'planting_conditions' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${planting_conditions_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(planting_conditions_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${planting_conditions_DIR}/${_extra}")
endforeach()
