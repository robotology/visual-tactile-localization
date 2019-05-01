#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME object-tracking-manipulation)

# YARP
find_package(YARP CONFIG REQUIRED
             COMPONENTS
             OS
             sig
             eigen
             )

find_package(ICUB REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})

set(${EXE_TARGET_NAME}_HDR
    include/ArmController.h
    include/LemniscateGenerator.h
    include/ObjectHelper.h
    include/SinusoidalTrajectory.h
    include/TrajectoryGenerator.h
    )

set(${EXE_TARGET_NAME}_SRC
    src/ArmController.cpp
    src/LemniscateGenerator.cpp
    src/ObjectHelper.cpp
    src/SinusoidalTrajectory.cpp
    src/main.cpp
    )

set(${EXE_TARGET_NAME}_THRIFT_HDR thrift/objecttrackingmanipulation.thrift)
yarp_add_idl(${EXE_TARGET_NAME}_THRIFT_SRC ${${EXE_TARGET_NAME}_THRIFT_HDR})

add_executable(${EXE_TARGET_NAME}
               ${${EXE_TARGET_NAME}_HDR}
               ${${EXE_TARGET_NAME}_SRC}
               ${${EXE_TARGET_NAME}_THRIFT_SRC}
               )

target_include_directories(${EXE_TARGET_NAME}
                           PRIVATE
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
                           ${ICUB_INCLUDE_DIRS}
                           )

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      YARP::YARP_OS
                      YARP::YARP_eigen
                      ${ICUB_LIBRARIES}
                      iKin
                      iCubGazeCtrlLibrary
                      )

if(NOT TARGET Eigen3)
    target_include_directories(${EXE_TARGET_NAME}
                               PRIVATE
                               ${EIGEN3_INCLUDE_DIR}
                               )
else()
    target_link_libraries(${EXE_TARGET_NAME}
                          PRIVATE
                          Eigen3::Eigen
                          )
endif()

set(${EXE_TARGET_NAME}_CONF
    ${CMAKE_CURRENT_SOURCE_DIR}/conf/manipulation_config.ini
    ${CMAKE_CURRENT_SOURCE_DIR}/conf/object_helper_config.ini
    ${CMAKE_CURRENT_SOURCE_DIR}/conf/hand_sequences.ini
    ${CMAKE_CURRENT_SOURCE_DIR}/conf/springy_calibration_left.ini
    ${CMAKE_CURRENT_SOURCE_DIR}/conf/springy_calibration_right.ini
    )

yarp_install(FILES ${${EXE_TARGET_NAME}_CONF}                           DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking-manipulation)
install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
