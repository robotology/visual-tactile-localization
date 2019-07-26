#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

option(USE_OPENMP "Use OpenMP" OFF)
option(USE_SUPERQUADRICLIB "Use SuperquadricLib" OFF)
option(USE_PCL "Use PointCloudLibrary" OFF)

set(EXE_TARGET_NAME object-tracking)

# YARP
find_package(YARP CONFIG REQUIRED
             COMPONENTS
             OS
             sig
             eigen
             cv
             )

# assimp
find_package(assimp REQUIRED)

# nanoflann
find_package(nanoflann QUIET)

# OpenMP
if (USE_OPENMP)
  find_package(OpenMP REQUIRED)
endif()

# SuperquadricLib
if (USE_SUPERQUADRICLIB)
   find_package(SuperquadricLib REQUIRED)
   add_definitions(-DUSE_SUPERQUADRICLIB)
endif()

# OpenGL
set(OpenGL_GL_PREFERENCE "LEGACY")
find_package(OpenGL REQUIRED)

# PCL
if(USE_PCL)
  find_package(PCL REQUIRED)
  add_definitions(-DUSE_PCL)
endif()

# VTK
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

# mlpack
find_package(MLPACK REQUIRED)

# armadillo
find_package(Armadillo REQUIRED)

set(${EXE_TARGET_NAME}_HDR
    include/BoundingBoxSegmentation.h
    include/ContactDetection.h
    include/Correction.h
    include/DiscreteKinematicModel.h
    include/DiscretizedKinematicModel.h
    include/DiscretizedKinematicModelTDD.h
    include/Filter.h
    include/iCubArmModel.h
    include/iCubHandContactsModel.h
    include/iCubHandOcclusion.h
    include/iCubObjectMeasurements.h
    include/iCubSpringyFingersDetection.h
    include/InitParticles.h
    include/InitParticlesGroundTruth.h
    include/InHandObjectSegmentation.h
    include/LocalizeSuperquadricSampler.h
    include/MaskSegmentation.h
    include/MeshImporter.h
    include/MeshModel.h
    include/NanoflannPointCloudPrediction.h
    include/ObjectOcclusion.h
    include/ObjectSampler.h
    include/ObjectMeasurements.h
    include/ObjectMeshSampler.h
    include/ObjectPointCloudPrediction.h
    include/ObjectRenderer.h
    include/ParticlesCorrection.h
    include/PFilter.h
    include/PointCloudAdaptor.h
    include/PointCloudDumper.h
    include/PointCloudModel.h
    include/PointCloudPrediction.h
    include/PointCloudSegmentation.h
    include/ProximityLikelihood.h
    include/Random3DPose.h
    include/RateStabilizer.h
    include/SimulatedFilter.h
    include/SimulatedPointCloud.h
    include/SuperquadricSampler.h
    include/Validator2D.h
    include/VCGTriMesh.h
    include/kdTree.h
    include/kdTreeWithNormals.h
    include/vcg_import_obj_w_stream.h
    include/springyFingers.h
    include/iCub/perception/private/models.h
    include/iCub/perception/private/ports.h
    )

set(${EXE_TARGET_NAME}_SRC
    src/BoundingBoxSegmentation.cpp
    src/Correction.cpp
    src/DiscreteKinematicModel.cpp
    src/DiscretizedKinematicModel.cpp
    src/DiscretizedKinematicModelTDD.cpp
    src/Filter.cpp
    src/iCubArmModel.cpp
    src/iCubHandContactsModel.cpp
    src/iCubHandOcclusion.cpp
    src/iCubObjectMeasurements.cpp
    src/iCubSpringyFingersDetection.cpp
    src/InHandObjectSegmentation.cpp
    src/InitParticles.cpp
    src/InitParticlesGroundTruth.cpp
    src/LocalizeSuperquadricSampler.cpp
    src/MaskSegmentation.cpp
    src/MeshImporter.cpp
    src/NanoflannPointCloudPrediction.cpp
    src/ObjectOcclusion.cpp
    src/ObjectMeasurements.cpp
    src/ObjectMeshSampler.cpp
    src/ObjectPointCloudPrediction.cpp
    src/ObjectSampler.cpp
    src/ObjectRenderer.cpp
    src/ParticlesCorrection.cpp
    src/PFilter.cpp
    src/PointCloudModel.cpp
    src/PointCloudDumper.cpp
    src/PointCloudSegmentation.cpp
    src/ProximityLikelihood.cpp
    src/Random3DPose.cpp
    src/RateStabilizer.cpp
    src/SimulatedFilter.cpp
    src/SimulatedPointCloud.cpp
    src/SuperquadricSampler.cpp
    src/Validator2D.cpp
    src/main.cpp
    src/springyFingers.cpp
    )

set(${EXE_TARGET_NAME}_THRIFT_HDR
    thrift/objecttracking.thrift
    thrift/model.thrift
    thrift/localizesuperquadricsampler.thrift
    thrift/ratestabilizer.thrift
    thrift/pointclouddumper.thrift
    )

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
                           ${OPENGL_INCLUDE_DIRS}
                           ${ARMADILLO_INCLUDE_DIRS}
                           ${MLPACK_INCLUDE_DIRS}
                           )

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      assimp
                      BayesFilters::BayesFilters
                      SI::SuperimposeMesh
                      YARP::YARP_OS
                      YARP::YARP_eigen
                      YARP::YARP_cv
                      ${OpenCV_LIBS}
                      iCubGazeCtrlLibrary
                      ${OPENGL_LIBRARIES}
                      ${ICUB_LIBRARIES}
                      iCubFingersEncoders
                      CameraLibrary
                      ${VTK_LIBRARIES}
                      ${ARMADILLO_LIBRARIES}
                      ${MLPACK_LIBRARIES}
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

if (USE_SUPERQUADRICLIB)
   target_link_libraries(${EXE_TARGET_NAME} PRIVATE SuperquadricLib::SuperquadricLibModel)
endif()


if (USE_PCL)
  target_link_libraries(${EXE_TARGET_NAME} PRIVATE ${PCL_LIBRARIES})
  target_include_directories(${EXE_TARGET_NAME} PRIVATE ${PCL_INCLUDE_DIRS})
  target_compile_definitions(${EXE_TARGET_NAME} PRIVATE ${PCL_DEFINITIONS})
  target_compile_options(${EXE_TARGET_NAME} PRIVATE ${PCL_COMPILE_OPTIONS})

  target_compile_definitions(CameraLibrary PRIVATE ${PCL_DEFINITIONS})
  target_compile_options(CameraLibrary PRIVATE ${PCL_COMPILE_OPTIONS})
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

set(${EXE_TARGET_NAME}_SHADER_VERT
        ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_frame.vert
        ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_model.vert
        ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_background.vert
)

set(${EXE_TARGET_NAME}_SHADER_FRAG
        ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_frame.frag
        ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_model.frag
        ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_model_texture.frag
        ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_background.frag
)

set(${EXE_TARGET_NAME}_ICUB_RIGHT_HAND_MESHES
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ail0.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ail1.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ail2.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ail3.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ail3_cleaned.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_forearm.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_indexbase.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_lil0.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_lil1.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_lil2.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_lil3.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_lil3_cleaned.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_littlebase.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ml0.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ml1.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ml2.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ml3.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ml3_cleaned.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_palm.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ril0.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ril1.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ril2.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ril3.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ril3_cleaned.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ringbase.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_tl0.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_tl1.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_tl2.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_tl3.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_tl4.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_tl4_cleaned.obj
        )

file(GLOB ${EXE_TARGET_NAME}_YCB_MESHES ${CMAKE_CURRENT_SOURCE_DIR}/mesh/*.obj)

file(GLOB ${EXE_TARGET_NAME}_CONF ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

file(GLOB ${EXE_TARGET_NAME}_XML ${CMAKE_CURRENT_SOURCE_DIR}/app/scripts/*.xml)

yarp_install(FILES ${${EXE_TARGET_NAME}_CONF}                           DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking)
yarp_install(FILES ${${EXE_TARGET_NAME}_SHADER_VERT}                    DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/shader)
yarp_install(FILES ${${EXE_TARGET_NAME}_SHADER_FRAG}                    DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/shader)
yarp_install(FILES ${${EXE_TARGET_NAME}_ICUB_RIGHT_HAND_MESHES}         DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/icub_right_arm)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_MESHES}                     DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh)
yarp_install(FILES ${${EXE_TARGET_NAME}_XML}                            DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
