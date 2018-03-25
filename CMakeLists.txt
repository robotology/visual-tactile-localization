##############################################################################
#                                                                            #
# Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        #
# All Rights Reserved.                                                       #
#                                                                            #
##############################################################################

#
# author: Giulia Vezzani <giulia.vezzani@iit.it>
# author: Nicola Piga <nicolapiga@gmail.com>
#

# require minimum cmake version
cmake_minimum_required(VERSION 2.8.9)

# C++11
set(CMAKE_CXX_STANDARD 11)

# produce the cmake var PROJECT_NAME
project(upf-localizer)

# find packages
find_package(YARP REQUIRED)
find_package(ICUB)
find_package(ICUBcontrib REQUIRED)
find_package(CGAL REQUIRED COMPONENTS)

# extend the current search path used by cmake to load
# yarp, icub and icubcontrib helpers
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

# include helpers
include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

# CGAL
include(${CGAL_USE_FILE})

# specify to install in $ICUBcontrib_DIR/bin
icubcontrib_set_default_prefix()

# include directories
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

# set headers and sources
set(headers
  headers/geometryCGAL.h
  headers/unscentedParticleFilter.h
  headers/localizer-module.h
  headers/filterData.h
  headers/PointCloud.h)
set(sources
  src/unscentedParticleFilter.cpp
  src/localizer-module.cpp
  src/filterData.cpp
  src/main.cpp)

# executable
add_executable(${PROJECT_NAME} ${headers} ${sources})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

# configuration file
set (confMustard ${PROJECT_SOURCE_DIR}/config/config.ini)
yarp_install(FILES ${confMustard} DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/simVisualTactileLocalization)

# mesh file for YCB Mustard
set (meshMustard ${PROJECT_SOURCE_DIR}/models/mustard.off)
yarp_install(FILES ${meshMustard} DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/simVisualTactileLocalization/models)

# mesh file for box
set (meshBox ${PROJECT_SOURCE_DIR}/models/box.off)
yarp_install(FILES ${meshBox} DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/simVisualTactileLocalization/models)

# output files folder with empty placeholder
set (outputDraft ${PROJECT_SOURCE_DIR}/output/empty)
yarp_install(FILES ${outputDraft} DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/simVisualTactileLocalization/output)

# add uninstall target
icubcontrib_add_uninstall_target()
