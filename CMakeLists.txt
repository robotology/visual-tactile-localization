# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Giulia Vezzani
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)

set(PROJECTNAME localizer)
project(${PROJECTNAME})

set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)

find_package(YARP)
find_package(ICUB)
find_package(ICUBcontrib)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
find_package(CGAL REQUIRED COMPONENTS)
find_package(Boost REQUIRED)

include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

set(headers headers/geometryCGAL.h  headers/unscentedParticleFilter.h headers/localizer.h )
set(sources src/unscentedParticleFilter.cpp src/main.cpp src/localizer.cpp) 
source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})

include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR} )
include(${CGAL_USE_FILE})
include(CGAL_CreateSingleSourceCGALProgram)

add_executable(${PROJECTNAME} ${headers} ${sources})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)
icubcontrib_add_uninstall_target()
