#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME object-tracking-ground-truth)

# YARP
find_package(YARP CONFIG REQUIRED
             COMPONENTS
             OS
             sig
             eigen
             cv
             )

# Application source and header files
set(${EXE_TARGET_NAME}_HDR
    include/ArucoMeasurement.h
    include/ArucoTracker.h
    include/Correction.h
    include/KinematicModel.h
    )

set(${EXE_TARGET_NAME}_SRC
    src/main.cpp
    src/ArucoMeasurement.cpp
    src/ArucoTracker.cpp
    src/Correction.cpp
    src/KinematicModel.cpp
    src/main.cpp
    )

set(${EXE_TARGET_NAME}_THRIFT_HDR thrift/arucotracker.thrift)

# Application target calls
yarp_add_idl(${EXE_TARGET_NAME}_THRIFT_SRC ${${EXE_TARGET_NAME}_THRIFT_HDR})

add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_HDR} ${${EXE_TARGET_NAME}_SRC} ${${EXE_TARGET_NAME}_THRIFT_SRC})

target_include_directories(${EXE_TARGET_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")


if(NOT TARGET Eigen3)
    target_include_directories(${EXE_TARGET_NAME} PRIVATE ${EIGEN3_INCLUDE_DIR})
else()
    target_link_libraries(${EXE_TARGET_NAME} PRIVATE Eigen3::Eigen)
endif()

target_link_libraries(${EXE_TARGET_NAME} PRIVATE
                                         BayesFilters::BayesFilters
                                         CameraLibrary
                                         ${OpenCV_LIBS}
                                         ${VTK_LIBRARIES}
                                         YARP::YARP_init
                                         YARP::YARP_OS
                                         YARP::YARP_sig
                                         YARP::YARP_eigen
                                         YARP::YARP_cv
                                         )

set(${EXE_TARGET_NAME}_CONF ${CMAKE_CURRENT_SOURCE_DIR}/conf/config.ini
                            ${CMAKE_CURRENT_SOURCE_DIR}/conf/icub_camera_config.ini
                            ${CMAKE_CURRENT_SOURCE_DIR}/conf/realsense_camera_config.ini
                            ${CMAKE_CURRENT_SOURCE_DIR}/conf/marker_offsets.ini
                            )

yarp_install(FILES ${${EXE_TARGET_NAME}_CONF} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking-ground-truth)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
