#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME object-tracking-viewer)

# YARP
find_package(YARP CONFIG REQUIRED
             COMPONENTS
             OS
             sig
             eigen
             )

# VTK
find_package(VTK             REQUIRED)
include(${VTK_USE_FILE})

# OpenCV
find_package(OpenCV REQUIRED)

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

set(${EXE_TARGET_NAME}_THRIFT_HDR thrift/viewer.thrift)

yarp_add_idl(${EXE_TARGET_NAME}_THRIFT_SRC ${${EXE_TARGET_NAME}_THRIFT_HDR})

# Application target calls
add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_HDR} ${${EXE_TARGET_NAME}_SRC} ${${EXE_TARGET_NAME}_THRIFT_SRC})

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
                                         CameraLibrary
                                         SI::SuperimposeMesh
                                         ${ICUB_LIBRARIES}
                                         ${OpenCV_LIBS}
                                         iCubFingersEncoders
                                         )

set(${EXE_TARGET_NAME}_CONF_ICUB ${CMAKE_CURRENT_SOURCE_DIR}/conf/icub/config.ini)
set(${EXE_TARGET_NAME}_CONF_R1 ${CMAKE_CURRENT_SOURCE_DIR}/conf/r1/config.ini)

yarp_install(FILES ${${EXE_TARGET_NAME}_CONF_ICUB} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking-viewer-icub)
yarp_install(FILES ${${EXE_TARGET_NAME}_CONF_R1}   DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking-viewer-r1)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
