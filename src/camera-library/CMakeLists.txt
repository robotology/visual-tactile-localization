#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

project("CameraLibrary")

set(LIBRARY_TARGET_NAME ${PROJECT_NAME})

# YARP
find_package(YARP 3.0.0 REQUIRED
             COMPONENTS
             OS
             cv
             eigen
             sig
             )

# ICUB
find_package(ICUB REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

set(${LIBRARY_TARGET_NAME}_HDR
    include/Camera.h
    include/CameraParameters.h
    include/iCubCamera.h
    include/RealsenseCamera.h
    include/YCBVideoCamera.h
    include/YCBVideoCameraNRT.h
    )

set(${LIBRARY_TARGET_NAME}_SRC
    src/Camera.cpp
    src/iCubCamera.cpp
    src/RealsenseCamera.cpp
    src/YCBVideoCamera.cpp
    src/YCBVideoCameraNRT.cpp
    )

add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_HDR} ${${LIBRARY_TARGET_NAME}_SRC})

target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC
                                                  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                                  )

target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC
                      YARP::YARP_OS
                      YARP::YARP_cv
                      YARP::YARP_eigen
                      YARP::YARP_sig
                      ${ICUB_LIBRARIES}
                      ${OpenCV_LIBS}
                      )
