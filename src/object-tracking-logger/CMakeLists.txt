#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME object-tracking-logger)

# YARP
find_package(YARP CONFIG REQUIRED
             COMPONENTS
             OS
             sig
             eigen
             )

set(${EXE_TARGET_NAME}_HDR
    include/Logger.h
    )

set(${EXE_TARGET_NAME}_SRC
    src/Logger.cpp
    src/main.cpp
    )

set(${EXE_TARGET_NAME}_THRIFT_HDR thrift/logger.thrift)

yarp_add_idl(${EXE_TARGET_NAME}_THRIFT_SRC ${${EXE_TARGET_NAME}_THRIFT_HDR})

add_executable(${EXE_TARGET_NAME}
               ${${EXE_TARGET_NAME}_HDR}
               ${${EXE_TARGET_NAME}_SRC}
               ${${EXE_TARGET_NAME}_THRIFT_SRC}
               )

target_include_directories(${EXE_TARGET_NAME}
                           PRIVATE
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
                           )

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      BayesFilters::BayesFilters
                      YARP::YARP_init
                      YARP::YARP_OS
                      YARP::YARP_sig
                      YARP::YARP_eigen
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
    ${CMAKE_CURRENT_SOURCE_DIR}/conf/config.ini
    )

yarp_install(FILES ${${EXE_TARGET_NAME}_CONF} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking-logger)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
