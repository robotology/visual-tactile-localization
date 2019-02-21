set(EXE_TARGET_NAME object-tracking-ground-truth)

# Eigen
find_package(Eigen3 QUIET CONFIG)
if(NOT EIGEN3_FOUND)
    # in case the cmake/FindEigen3.cmake fails
    find_package(Eigen3 REQUIRED)
  endif()

# Bayes Filters
find_package(BayesFilters 0.9.0 QUIET)
if(NOT BayesFilters_FOUND)
    message(STATUS "Did not found required master release of BayesFilters. Looking for devel version.")
    find_package(BayesFilters 0.9.100 REQUIRED)
endif()

# YARP
find_package(YARP CONFIG REQUIRED
             COMPONENTS
             OS
             sig
             eigen
             )

# Application source and header files
set(${EXE_TARGET_NAME}_HDR
    include/ArucoMeasurement.h
    include/KinematicModel.h
    )

set(${EXE_TARGET_NAME}_SRC
    src/main.cpp
    src/ArucoMeasurement.cpp
    src/KinematicModel.cpp
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
                                         BayesFilters::BayesFilters
                                         )

set(${EXE_TARGET_NAME}_CONF ${CMAKE_CURRENT_SOURCE_DIR}/conf/config.ini
                            ${CMAKE_CURRENT_SOURCE_DIR}/conf/marker_offsets.ini
                            ${CMAKE_CURRENT_SOURCE_DIR}/conf/opencv_icub_intrinsics_left
                            ${CMAKE_CURRENT_SOURCE_DIR}/conf/opencv_icub_intrinsics_right
                            )

yarp_install(FILES ${${EXE_TARGET_NAME}_CONF} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking-ground-truth)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
