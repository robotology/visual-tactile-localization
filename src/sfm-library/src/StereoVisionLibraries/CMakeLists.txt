# Copyright: (C) 2011 RobotCub Consortium
# Author: Sean Ryan Fanello, Giulia Pasquale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(stereoVision)

############## LIBELAS ##############

list(APPEND CMAKE_CXX_FLAGS "-msse3")

option(USE_OPENMP_libElas "Use OpenMP for libElas" OFF)

if (USE_OPENMP_libElas)
  find_package(OpenMP REQUIRED)
endif()

if(USE_OPENMP_libElas)
  set(folder_source_elas src/elas/elas_omp.cpp
                         src/elas/descriptor.cpp
                         src/elas/filter.cpp
                         src/elas/matrix.cpp
                         src/elas/triangle.cpp)
else()
  set(folder_source_elas src/elas/elas.cpp
                         src/elas/descriptor.cpp
                         src/elas/filter.cpp
                         src/elas/matrix.cpp
                         src/elas/triangle.cpp)
endif()

####################################

set(folder_source src/camera.cpp
                  src/stereoCamera.cpp
                  src/disparityThread.cpp
                  src/opticalFlowThread.cpp
                  src/sceneFlow.cpp
                  src/elasWrapper.cpp)

set(folder_header include/iCub/stereoVision/camera.h
                  include/iCub/stereoVision/stereoCamera.h
                  include/iCub/stereoVision/disparityThread.h
                  include/iCub/stereoVision/opticalFlowThread.h
                  include/iCub/stereoVision/sceneFlow.h
                  include/iCub/stereoVision/elasWrapper.h
                  include/iCub/stereoVision/elas/elas.h
                  include/iCub/stereoVision/elas/descriptor.h
                  include/iCub/stereoVision/elas/image.h
                  include/iCub/stereoVision/elas/filter.h
                  include/iCub/stereoVision/elas/timer.h
                  include/iCub/stereoVision/elas/matrix.h
                  include/iCub/stereoVision/elas/triangle.h)

if(USE_SIFT_GPU)
    list(APPEND folder_source src/utils.cpp)
    list(APPEND folder_header include/iCub/stereoVision/utils.h)
endif()

if(OpenCV_VERSION_MAJOR GREATER 2)
  add_definitions(-DOPENCV_GREATER_2)
endif()

source_group("Source Files" FILES ${folder_source})
source_group("ELAS Source Files" FILES ${folder_source_elas})
source_group("Header Files" FILES ${folder_header})

if(USE_SIFT_GPU)
    include_directories(${SIFTGPU_INCLUDE_DIRS})
    add_definitions(-DUSING_GPU)
endif()

add_library(${PROJECT_NAME} ${folder_source} ${folder_source_elas} ${folder_header})

target_include_directories(${PROJECT_NAME} PUBLIC  $<INSTALL_INTERFACE:include> $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/iCub/stereoVision/elas)

target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBRARIES} iKin ${YARP_LIBRARIES})

if(USE_SIFT_GPU)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${SIFTGPU_LIBRARIES})
endif()

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
