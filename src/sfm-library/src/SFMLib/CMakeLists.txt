# Copyright: 2010 RobotCub Consortium
# Author: Sean Ryan Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

option(USE_OPENMP_SFMLib "Use OpenMP for SFMLib" OFF)

project(SFMLib)

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 QUIET CONFIG)
if(NOT EIGEN3_FOUND)
    find_package(Eigen3 REQUIRED)
endif()

# OpenMP
if (USE_OPENMP_SFMLib)
  find_package(OpenMP REQUIRED)
endif()

set(LIBRARY_TARGET_NAME ${PROJECT_NAME})

set(${LIBRARY_TARGET_NAME}_HDR SFM.h)
set(${LIBRARY_TARGET_NAME}_SRC SFM.cpp)

add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_HDR} ${${LIBRARY_TARGET_NAME}_SRC})

if (USE_OPENMP_SFMLib)
    if(NOT TARGET OpenMP::OpenMP_CXX)
        find_package(Threads REQUIRED)
        add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
        set_property(TARGET OpenMP::OpenMP_CXX
                     PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
        # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
        set_property(TARGET OpenMP::OpenMP_CXX
                     PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)
    endif()
    target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX)
endif()

if(NOT TARGET Eigen3)
    target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include> $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                               PRIVATE ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME}      PUBLIC iCubGazeCtrlLibrary
                                               PRIVATE stereoVision iKin ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})
else()
    target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include> $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
    target_link_libraries(${PROJECT_NAME}      PUBLIC iCubGazeCtrlLibrary
                                               PRIVATE stereoVision iKin Eigen3::Eigen ${OpenCV_LIBRARIES} ${YARP_LIBRARIES} iCubGazeCtrlLibrary)
endif()
