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
find_package(PCL 1.3 REQUIRED)

# VTK
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

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
    include/ObjectRenderer.h
    include/ParticlesCorrection.h
    include/PFilter.h
    include/PointCloudModel.h
    include/PointCloudPrediction.h
    include/PointCloudSegmentation.h
    include/ProximityLikelihood.h
    include/Random3DPose.h
    include/SimulatedFilter.h
    include/SimulatedPointCloud.h
    include/SuperquadricSampler.h
    include/VCGTriMesh.h
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
    src/LocalizeSuperquadricSampler.cpp
    src/MaskSegmentation.cpp
    src/MeshImporter.cpp
    src/NanoflannPointCloudPrediction.cpp
    src/ObjectOcclusion.cpp
    src/ObjectMeasurements.cpp
    src/ObjectMeshSampler.cpp
    src/ObjectSampler.cpp
    src/ObjectRenderer.cpp
    src/ParticlesCorrection.cpp
    src/PFilter.cpp
    src/PointCloudModel.cpp
    src/PointCloudSegmentation.cpp
    src/ProximityLikelihood.cpp
    src/Random3DPose.cpp
    src/SimulatedFilter.cpp
    src/SimulatedPointCloud.cpp
    src/SuperquadricSampler.cpp
    src/main.cpp
    src/springyFingers.cpp
    )

set(${EXE_TARGET_NAME}_THRIFT_HDR thrift/objecttracking.thrift thrift/modelinitialization.thrift)

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
                           ${PCL_INCLUDE_DIRS}
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
                      ${PCL_LIBRARIES}
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
    ${CMAKE_CURRENT_SOURCE_DIR}/conf/analogs_configuration.ini
    ${CMAKE_CURRENT_SOURCE_DIR}/conf/config_icub_320_240.ini
    ${CMAKE_CURRENT_SOURCE_DIR}/conf/config_mask_320_240.ini
    ${CMAKE_CURRENT_SOURCE_DIR}/conf/config_mask_640_480.ini
    ${CMAKE_CURRENT_SOURCE_DIR}/conf/config_realsense_320_240.ini
    ${CMAKE_CURRENT_SOURCE_DIR}/conf/icub_camera_config.ini
    ${CMAKE_CURRENT_SOURCE_DIR}/conf/realsense2_dev.ini
    ${CMAKE_CURRENT_SOURCE_DIR}/conf/realsense_camera_config.ini 
    ${CMAKE_CURRENT_SOURCE_DIR}/conf/sfm_config.ini
    ${CMAKE_CURRENT_SOURCE_DIR}/conf/springy_calibration_left.ini
    ${CMAKE_CURRENT_SOURCE_DIR}/conf/springy_calibration_right.ini
)

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

set(${EXE_TARGET_NAME}_YCB_ADJUSTABLE_WRENCH    ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_adjustable_wrench/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_adjustable_wrench/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_APPLE                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_apple/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_apple/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_BANANA               ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_banana/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_banana/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_BASEBALL             ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_baseball/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_baseball/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_BLEACH_CLEANSER      ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_bleach_cleanser/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_bleach_cleanser/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_BOWL                 ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_bowl/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_bowl/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_CHAIN                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_chain/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_chain/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_CHIPS_CAN            ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_chips_can/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_chips_can/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_CRACKER_BOX          ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_cracker_box/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_cracker_box/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_DICE                 ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_dice/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_dice/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_EXTRA_LARGE_CLAMP    ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_extra_large_clamp/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_extra_large_clamp/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_FLAT_SCREWDRIVER     ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_flat_screwdriver/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_flat_screwdriver/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_FOAM_BRICK           ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_foam_brick/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_foam_brick/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_FORK                 ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_fork/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_fork/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_GELATIN_BOX          ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_gelatin_box/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_gelatin_box/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_GOLF_BALL            ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_golf_ball/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_golf_ball/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_HAMMER               ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_hammer/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_hammer/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_KEY                  ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_key/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_key/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_KNIFE                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_knife/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_knife/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_LARGE_CLAMP          ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_large_clamp/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_large_clamp/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_LARGE_MARKER         ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_large_marker/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_large_marker/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_LEMON                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_lemon/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_lemon/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_MASTER_CHEF_CAN      ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_master_chef_can/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_master_chef_can/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_MEDIUM_CLAMP         ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_medium_clamp/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_medium_clamp/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_MINI_SOCCER_BALL     ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_mini_soccer_ball/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_mini_soccer_ball/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_MUG                  ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_mug/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_mug/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_MUSTARD_BOTTLE       ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_mustard_bottle/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_mustard_bottle/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_ORANGE               ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_orange/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_orange/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_PADLOCK              ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_padlock/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_padlock/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_PEACH                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_peach/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_peach/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_PEAR                 ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_pear/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_pear/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_PHILLIPS_SCREWDRIVER ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_phillips_screwdriver/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_phillips_screwdriver/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_PITCHER_BASE         ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_pitcher_base/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_pitcher_base/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_PLATE                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_plate/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_plate/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_PLUM                 ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_plum/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_plum/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_POTTED_MEAT_CAN      ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_potted_meat_can/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_potted_meat_can/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_POWER_DRILL          ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_power_drill/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_power_drill/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_PUDDING_BOX          ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_pudding_box/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_pudding_box/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_RACQUETBALL          ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_racquetball/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_racquetball/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_RUBIKS_CUBE          ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_rubiks_cube/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_rubiks_cube/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_SCISSORS             ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_scissors/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_scissors/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_SKILLET              ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_skillet/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_skillet/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_SMALL_CLAMP          ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_small_clamp/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_small_clamp/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_SMALL_MARKER         ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_small_marker/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_small_marker/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_SOFTBALL             ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_softball/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_softball/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_SPATULA              ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_spatula/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_spatula/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_SPONGE               ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_sponge/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_sponge/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_SPOON                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_spoon/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_spoon/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_STRAWBERRY           ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_strawberry/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_strawberry/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_SUGAR_BOX            ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_sugar_box/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_sugar_box/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_TENNIS_BALL          ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_tennis_ball/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_tennis_ball/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_TIMER                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_timer/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_timer/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_TOMATO_SOUP_CAN      ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_tomato_soup_can/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_tomato_soup_can/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_TUNA_FISH_CAN        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_tuna_fish_can/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_tuna_fish_can/nontextured.obj)
set(${EXE_TARGET_NAME}_YCB_WINDEX_BOTTLE        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_windex_bottle/nontextured.ply
                                                ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ycb_windex_bottle/nontextured.obj)

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

file(GLOB ${EXE_TARGET_NAME}_XML ${CMAKE_CURRENT_SOURCE_DIR}/app/scripts/*.xml)

yarp_install(FILES ${${EXE_TARGET_NAME}_CONF}                           DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking)
yarp_install(FILES ${${EXE_TARGET_NAME}_SHADER_VERT}                    DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/shader)
yarp_install(FILES ${${EXE_TARGET_NAME}_SHADER_FRAG}                    DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/shader)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_ADJUSTABLE_WRENCH}          DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_adjustable_wrench)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_APPLE}                      DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_apple)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_BANANA}                     DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_banana)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_BASEBALL}                   DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_baseball)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_BLEACH_CLEANSER}            DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_bleach_cleanser)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_BOWL}                       DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_bowl)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_CHAIN}                      DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_chain)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_CHIPS_CAN}                  DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_chips_can)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_CRACKER_BOX}                DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_cracker_box)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_DICE}                       DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_dice)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_EXTRA_LARGE_CLAMP}          DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_extra_large_clamp)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_FLAT_SCREWDRIVER}           DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_flat_screwdriver)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_FOAM_BRICK}                 DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_foam_brick)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_FORK}                       DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_fork)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_GELATIN_BOX}                DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_gelatin_box)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_GOLF_BALL}                  DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_golf_ball)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_HAMMER}                     DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_hammer)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_KEY}                        DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_key)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_KNIFE}                      DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_knife)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_LARGE_CLAMP}                DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_large_clamp)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_LARGE_MARKER}               DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_large_marker)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_LEMON}                      DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_lemon)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_MASTER_CHEF_CAN}            DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_master_chef_can)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_MEDIUM_CLAMP}               DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_medium_clamp)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_MINI_SOCCER_BALL}           DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_mini_soccer_ball)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_MUG}                        DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_mug)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_MUSTARD_BOTTLE}             DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_mustard_bottle)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_ORANGE}                     DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_orange)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_PADLOCK}                    DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_padlock)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_PEACH}                      DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_peach)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_PEAR}                       DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_pear)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_PHILLIPS_SCREWDRIVER}       DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_phillips_screwdriver)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_PITCHER_BASE}               DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_pitcher_base)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_PLATE}                      DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_plate)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_PLUM}                       DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_plum)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_POTTED_MEAT_CAN}            DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_potted_meat_can)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_POWER_DRILL}                DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_power_drill)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_PUDDING_BOX}                DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_pudding_box)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_RACQUETBALL}                DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_racquetball)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_RUBIKS_CUBE}                DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_rubiks_cube)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_SCISSORS}                   DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_scissors)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_SKILLET}                    DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_skillet)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_SMALL_CLAMP}                DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_small_clamp)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_SMALL_MARKER}               DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_small_marker)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_SOFTBALL}                   DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_softball)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_SPATULA}                    DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_spatula)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_SPONGE}                     DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_sponge)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_SPOON}                      DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_spoon)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_STRAWBERRY}                 DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_strawberry)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_SUGAR_BOX}                  DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_sugar_box)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_TENNIS_BALL}                DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_tennis_ball)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_TIMER}                      DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_timer)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_TOMATO_SOUP_CAN}            DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_tomato_soup_can)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_TUNA_FISH_CAN}              DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_tuna_fish_can)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_WINDEX_BOTTLE}              DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/ycb_windex_bottle)
yarp_install(FILES ${${EXE_TARGET_NAME}_ICUB_RIGHT_HAND_MESHES}         DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/mesh/icub_right_arm)
yarp_install(FILES ${${EXE_TARGET_NAME}_XML}                            DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
