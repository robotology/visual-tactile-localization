#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

cmake_minimum_required(VERSION 3.5)

project(object-tracking
        LANGUAGES CXX
        VERSION 0.0.0)

set(CMAKE_CXX_STANDARD 11)

# Superimpose Mesh
find_package(SuperimposeMesh 0.9.4.0 QUIET)
if(NOT SuperimposeMesh_FOUND)
    message(STATUS "Did not found required master release of SuperimposeMesh. Looking for devel version.")
    find_package(SuperimposeMesh 0.11.100 REQUIRED)
endif()
# Temporary fix avoiding issues with 'assimp' and 'glm'
set_target_properties(SI::SuperimposeMesh PROPERTIES INTERFACE_LINK_LIBRARIES "")

# Bayes Filters
find_package(BayesFilters 0.9.0 QUIET)
if(NOT BayesFilters_FOUND)
    message(STATUS "Did not found required master release of BayesFilters. Looking for devel version.")
    find_package(BayesFilters 0.9.100 REQUIRED)
endif()

# Eigen
find_package(Eigen3 QUIET CONFIG)
if(NOT EIGEN3_FOUND)
    # in case the cmake/FindEigen3.cmake fails
    find_package(Eigen3 REQUIRED)
endif()

# iCub contrib
find_package(ICUBcontrib REQUIRED)

# ICUB
find_package(ICUB REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# Default build type to Release
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
endif()

include(GNUInstallDirs)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake
                              ${ICUBCONTRIB_MODULE_PATH}
                              )
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

icubcontrib_set_default_prefix()
icubcontrib_add_uninstall_target()

add_subdirectory(src)



