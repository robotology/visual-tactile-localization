#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME validation)

set(${EXE_TARGET_NAME}_HDR
    include/Metric.h
    include/MetricAdd.h
    include/MetricAddS.h
    include/ObjectSampler.h
    include/ObjectMeshSampler.h
    include/SingleVideoResult.h
    include/VCGTriMesh.h
    )

set(${EXE_TARGET_NAME}_SRC
    src/Metric.cpp
    src/MetricAdd.cpp
    src/MetricAddS.cpp
    src/ObjectSampler.cpp
    src/ObjectMeshSampler.cpp
    src/SingleVideoResult.cpp
    src/main.cpp
    )

add_executable(${EXE_TARGET_NAME}
               ${${EXE_TARGET_NAME}_HDR}
               ${${EXE_TARGET_NAME}_SRC}
               )

target_include_directories(${EXE_TARGET_NAME}
                           PRIVATE
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
                           )

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      )

if(NOT TARGET Eigen3)
    target_include_directories(${EXE_TARGET_NAME}
                               PRIVATE
                               ${EIGEN3_INCLUDE_DIR}
                               # Include Visualization and Computer Graphics Library (VCG)
                               # http://vcg.isti.cnr.it/vcglib/
                               # https://github.com/cnr-isti-vclab/vcglib
                               ${CMAKE_CURRENT_SOURCE_DIR}/include/vcglib
                               )
else()
    target_link_libraries(${EXE_TARGET_NAME}
                          PRIVATE
                          Eigen3::Eigen
                          )
endif()

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
