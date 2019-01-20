set(EXE_TARGET_NAME object-tracking)

# Bayes Filters
find_package(BayesFilters 0.9.0 QUIET)
if(NOT BayesFilters_FOUND)
    message(STATUS "Did not found required master release of BayesFilters. Looking for devel version.")
    find_package(BayesFilters 0.9.100 REQUIRED)
endif()

# Superimpose Mesh
find_package(SuperimposeMesh 0.9.4.0 QUIET)
if(NOT SuperimposeMesh_FOUND)
    message(STATUS "Did not found required master release of SuperimposeMesh. Looking for devel version.")
    find_package(SuperimposeMesh 0.10.100.0 REQUIRED)
endif()

# YARP
find_package(YARP CONFIG REQUIRED
             COMPONENTS
             OS
             )
# assimp
find_package(assimp REQUIRED)

# nanoflann
find_package(nanoflann)

# Eigen
find_package(Eigen3 QUIET CONFIG)
if(NOT EIGEN3_FOUND)
    # in case the cmake/FindEigen3.cmake fails
    find_package(Eigen3 REQUIRED)
endif()

# OpenCV
find_package(OpenCV REQUIRED)

set(${EXE_TARGET_NAME}_HDR
    include/Correction.h
    include/iCubPointCloud.h
    include/KinematicModel.h
    include/MeshImporter.h
    include/NanoflannPointCloudPrediction.h
    include/PointCloudModel.h
    include/PointCloudPrediction.h
    include/Random3DPose.h
    include/SimulatedPointCloud.h
    include/VCGTriMesh.h
    include/vcg_import_obj_w_stream.h
    )

set(${EXE_TARGET_NAME}_SRC
    src/Correction.cpp
    src/iCubPointCloud.cpp
    src/KinematicModel.cpp
    src/MeshImporter.cpp
    src/NanoflannPointCloudPrediction.cpp
    src/PointCloudModel.cpp
    src/Random3DPose.cpp
    src/SimulatedPointCloud.cpp
    src/main.cpp
    )

add_executable(${EXE_TARGET_NAME}
               ${${EXE_TARGET_NAME}_HDR}
               ${${EXE_TARGET_NAME}_SRC}
               )

target_include_directories(${EXE_TARGET_NAME}
                           PRIVATE
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
                           # Include Visualization and Computer Graphics Library (VCG)
                           # http://vcg.isti.cnr.it/vcglib/
                           # https://github.com/cnr-isti-vclab/vcglib
                           ${CMAKE_CURRENT_SOURCE_DIR}/include/vcglib
                           )

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      assimp
                      nanoflann::nanoflann
                      BayesFilters::BayesFilters
                      SuperimposeMesh::SuperimposeMesh
                      YARP::YARP_OS
                      # YARP::YARP_init
                      ${OpenCV_LIBS}
                      SFMLib
                      stereoVision
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

set(${EXE_TARGET_NAME}_CONF ${CMAKE_CURRENT_SOURCE_DIR}/conf/config.ini)

yarp_install(FILES ${${EXE_TARGET_NAME}_CONF} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
