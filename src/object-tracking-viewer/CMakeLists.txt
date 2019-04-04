#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME object-tracking-viewer)

# Eigen
find_package(Eigen3 QUIET CONFIG)
if(NOT EIGEN3_FOUND)
    # in case the cmake/FindEigen3.cmake fails
    find_package(Eigen3 REQUIRED)
endif()

# YARP
find_package(YARP CONFIG REQUIRED
             COMPONENTS
             OS
             sig
             eigen
             )

# ICUB
find_package(ICUB REQUIRED)

# Superimpose Mesh
find_package(SuperimposeMesh 0.9.4.0 QUIET)
if(NOT SuperimposeMesh_FOUND)
    message(STATUS "Did not found required master release of SuperimposeMesh. Looking for devel version.")
    find_package(SuperimposeMesh 0.10.100.0 REQUIRED)
endif()

# VTK
find_package(VTK             REQUIRED)
include(${VTK_USE_FILE})

# Application source and header files
set(${EXE_TARGET_NAME}_HDR
    include/iCubArmModel.h
    include/Viewer.h
    include/VtkiCubHand.h
    include/VtkMesh.h
    )

set(${EXE_TARGET_NAME}_SRC
    src/iCubArmModel.cpp
    src/Viewer.cpp
    src/VtkiCubHand.cpp
    src/VtkMesh.cpp
    src/main.cpp
    )

# Application target calls
add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_HDR} ${${EXE_TARGET_NAME}_SRC})

target_include_directories(${EXE_TARGET_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")


if(NOT TARGET Eigen3)
    target_include_directories(${EXE_TARGET_NAME} PRIVATE ${EIGEN3_INCLUDE_DIR})
else()
    target_link_libraries(${EXE_TARGET_NAME} PRIVATE Eigen3::Eigen)
endif()

target_link_libraries(${EXE_TARGET_NAME} PRIVATE
                                         ${VTK_LIBRARIES}
                                         YARP::YARP_init
                                         YARP::YARP_OS
                                         YARP::YARP_sig
                                         YARP::YARP_eigen
                                         iCubGazeCtrlLibrary
                                         SuperimposeMesh::SuperimposeMesh
                                         ${ICUB_LIBRARIES}
                                         iCubFingersEncoders
                                         )

set(${EXE_TARGET_NAME}_CONF ${CMAKE_CURRENT_SOURCE_DIR}/conf/config.ini)

yarp_install(FILES ${${EXE_TARGET_NAME}_CONF} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking-viewer)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
