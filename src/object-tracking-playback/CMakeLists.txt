#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME object-tracking-playback)

# VTK
find_package(VTK             REQUIRED)
include(${VTK_USE_FILE})

# Eigen
find_package(Eigen3          QUIET CONFIG)
if(NOT EIGEN3_FOUND)
    find_package(Eigen3      REQUIRED)
endif()

# Application source and header files
set(${EXE_TARGET_NAME}_HDR
    include/Viewer.h
    )

set(${EXE_TARGET_NAME}_SRC
    src/Viewer.cpp
    src/main.cpp
    )

# Application target calls
add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_HDR} ${${EXE_TARGET_NAME}_SRC})

target_include_directories(${EXE_TARGET_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")


if(NOT TARGET Eigen3)
target_include_directories(${EXE_TARGET_NAME} PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(${EXE_TARGET_NAME} PRIVATE ${VTK_LIBRARIES}
                                                 )
else()
target_link_libraries(${EXE_TARGET_NAME} PRIVATE ${VTK_LIBRARIES}
                                                 Eigen3::Eigen
                                                 )
endif()

# Install
install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
