# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Sean Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)

project(stereoVisionLibraries)

find_package(YARP 3.0.0 COMPONENTS OS sig dev math rosmsg idl_tools REQUIRED)
find_package(ICUB REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH}
                              ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(OpenCV REQUIRED)

set(USE_SIFT_GPU FALSE CACHE BOOL "Select false to run without GPU, or true for GPU acceleration")
if(USE_SIFT_GPU)
  find_package(SIFTGPU REQUIRED)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES)
  if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
  endif()
endif()

add_subdirectory(src)
