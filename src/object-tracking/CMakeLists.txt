option(USE_OPENMP "Use OpenMP" OFF)

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


# Eigen
find_package(Eigen3 QUIET CONFIG)
if(NOT EIGEN3_FOUND)
    # in case the cmake/FindEigen3.cmake fails
    find_package(Eigen3 REQUIRED)
endif()

# YARP
find_package(YARP CONFIG REQUIRED
             COMPONENTS
             OS
             sig
             eigen
             )
# assimp
find_package(assimp REQUIRED)

# nanoflann
find_package(nanoflann QUIET)

# OpenCV
find_package(OpenCV REQUIRED)

# OpenMP
if (USE_OPENMP)
  find_package(OpenMP REQUIRED)
endif()

set(${EXE_TARGET_NAME}_HDR
    include/Correction.h
    include/DiscreteKinematicModel.h
    include/DiscretizedKinematicModel.h
    include/Filter.h
    include/GaussianFilter_.h
    include/iCubPointCloud.h
    include/MeshImporter.h
    include/NanoflannPointCloudPrediction.h
    include/PointCloudModel.h
    include/PointCloudPrediction.h
    include/Random3DPose.h
    include/SimulatedFilter.h
    include/SimulatedPointCloud.h
    include/VCGTriMesh.h
    include/vcg_import_obj_w_stream.h
    )

set(${EXE_TARGET_NAME}_SRC
    src/Correction.cpp
    src/DiscreteKinematicModel.cpp
    src/DiscretizedKinematicModel.cpp
    src/Filter.cpp
    src/GaussianFilter_.cpp
    src/iCubPointCloud.cpp
    src/MeshImporter.cpp
    src/NanoflannPointCloudPrediction.cpp
    src/PointCloudModel.cpp
    src/Random3DPose.cpp
    src/SimulatedFilter.cpp
    src/SimulatedPointCloud.cpp
    src/main.cpp
    )

set(${EXE_TARGET_NAME}_THRIFT_HDR thrift/objecttracking.thrift)

yarp_add_idl(${EXE_TARGET_NAME}_THRIFT_SRC ${${EXE_TARGET_NAME}_THRIFT_HDR})

add_executable(${EXE_TARGET_NAME}
               ${${EXE_TARGET_NAME}_HDR}
               ${${EXE_TARGET_NAME}_SRC}
               ${${EXE_TARGET_NAME}_THRIFT_SRC}
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
                      BayesFilters::BayesFilters
                      SuperimposeMesh::SuperimposeMesh
                      YARP::YARP_OS
                      YARP::YARP_eigen
                      # YARP::YARP_init
                      ${OpenCV_LIBS}
                      SFMLib
                      stereoVision
                      iCubGazeCtrlLibrary
                      )

if (USE_OPENMP)
    if(NOT TARGET OpenMP::OpenMP_CXX)
        find_package(Threads REQUIRED)
        add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
        set_property(TARGET OpenMP::OpenMP_CXX
                     PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
        # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
        set_property(TARGET OpenMP::OpenMP_CXX
                     PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)
    endif()
    target_link_libraries(${EXE_TARGET_NAME} PRIVATE OpenMP::OpenMP_CXX)
endif()

if (nanoflann_FOUND)
    message(STATUS "nanoflann found on the system. Using system library.")
    target_link_libraries(${EXE_TARGET_NAME} PRIVATE nanoflann::nanoflann)
else()
    message(STATUS "nanoflann not found on the system. Using header shipped with this package.")
    set(nanoflann_INCLUDE_DIR include/nanoflann/)
    target_include_directories(${EXE_TARGET_NAME}
                               PRIVATE
                               ${nanoflann_INCLUDE_DIR}
                               )
endif()

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
    ${CMAKE_CURRENT_SOURCE_DIR}/conf/config_simulation.ini
    ${CMAKE_CURRENT_SOURCE_DIR}/conf/sfm_config.ini
    )

set(${EXE_TARGET_NAME}_SHADER_VERT
        ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_frame.vert
        ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_model.vert
        ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_background.vert
)

set(${EXE_TARGET_NAME}_SHADER_FRAG
        ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_frame.frag
        ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_model.frag
        ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_background.frag
)

set(${EXE_TARGET_NAME}_YCB_MUSTARD     ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_mustard/nontextured.ply ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_mustard/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_POWER_DRILL ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_power_drill/nontextured.ply)

file(GLOB ${EXE_TARGET_NAME}_XML ${CMAKE_CURRENT_SOURCE_DIR}/app/scripts/*.xml)

yarp_install(FILES ${${EXE_TARGET_NAME}_CONF}            DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking)
yarp_install(FILES ${${EXE_TARGET_NAME}_SHADER_VERT}     DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/shader)
yarp_install(FILES ${${EXE_TARGET_NAME}_SHADER_FRAG}     DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/shader)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_MUSTARD}     DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_mustard)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_POWER_DRILL} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_power_drill)
yarp_install(FILES ${${EXE_TARGET_NAME}_XML}             DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
