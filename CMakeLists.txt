##############################################################################
#                                                                            #
# Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        #
# All Rights Reserved.                                                       #
#                                                                            #
##############################################################################

#
# author: Giulia Vezzani <giulia.vezzani@iit.it>
# author: Nicola Piga <nicolapiga@gmail.com>
#

# require minimum cmake version
cmake_minimum_required(VERSION 2.8.9)

# C++11
set(CMAKE_CXX_STANDARD 11)

# produce the cmake var PROJECT_NAME
project(upf-localizer)

# find packages
find_package(YARP REQUIRED COMPONENTS gsl)
find_package(ICUB)
find_package(ICUBcontrib REQUIRED)
find_package(CGAL REQUIRED COMPONENTS)
find_package(VTK REQUIRED)
find_package(Eigen3 REQUIRED)

# extend the current search path used by cmake to load
# yarp, icub and icubcontrib helpers
# list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

# include helpers
# include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

# CGAL
include(${CGAL_USE_FILE})

# VTK
include(${VTK_USE_FILE})

# specify to install in $ICUBcontrib_DIR/bin
icubcontrib_set_default_prefix()

# include directories
include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR})

# set headers and sources
set(headers
  headers/geometryCGAL.h
  headers/unscentedParticleFilter.h
  headers/localizer-module.h
  headers/FilterCommand.h
  headers/PointCloud.h
  headers/fwd_kin_ext.h
  headers/eigenmvn.h)
set(sources
  src/unscentedParticleFilter.cpp
  src/localizer-module.cpp
  src/FilterCommand.cpp
  src/main.cpp
  src/fwd_kin_ext.cpp)

# executable
add_executable(${PROJECT_NAME} ${headers} ${sources})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
if(VTK_LIBRARIES)
  target_link_libraries(${PROJECT_NAME} ${VTK_LIBRARIES})
else()
  target_link_libraries(${PROJECT_NAME} vtkHybrid vtkWidgets)
endif()
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

# configuration file
set (configSim ${PROJECT_SOURCE_DIR}/config/simulation/localizer_config.ini)
yarp_install(FILES ${configSim} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/simVisualTactileLocalization)

set (configSetup ${PROJECT_SOURCE_DIR}/config/setup/localizer_config.ini)
yarp_install(FILES ${configSetup} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/visualTactileLocalization)

# mesh file for YCB Mustard
set (meshMustard ${PROJECT_SOURCE_DIR}/models/mustard.off)
yarp_install(FILES ${meshMustard} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/simVisualTactileLocalization/models)
yarp_install(FILES ${meshMustard} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/visualTactileLocalization/models)

# mesh file for box
set (meshBox ${PROJECT_SOURCE_DIR}/models/box.off)
yarp_install(FILES ${meshBox} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/simVisualTactileLocalization/models)
yarp_install(FILES ${meshBox} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/visualTactileLocalization/models)

# mesh file for book
set (meshBook ${PROJECT_SOURCE_DIR}/models/book.off)
yarp_install(FILES ${meshBook} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/simVisualTactileLocalization/models)
yarp_install(FILES ${meshBook} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/visualTactileLocalization/models)

# mesh file for rice
set (meshRice ${PROJECT_SOURCE_DIR}/models/rice.off)
yarp_install(FILES ${meshBook} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/simVisualTactileLocalization/models)
yarp_install(FILES ${meshRice} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/visualTactileLocalization/models)

# calibration files for springy fingers
set (springyFingersCalib ${PROJECT_SOURCE_DIR}/config/springy_finger_calib)
yarp_install(FILES ${springyFingersCalib}/grasp_model_left.ini DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/simVisualTactileLocalization/springy_fingers_calib)
yarp_install(FILES ${springyFingersCalib}/grasp_model_right.ini DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/simVisualTactileLocalization/springy_fingers_calib)
yarp_install(FILES ${springyFingersCalib}/grasp_model_left.ini DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/visualTactileLocalization/springy_fingers_calib)
yarp_install(FILES ${springyFingersCalib}/grasp_model_right.ini DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/visualTactileLocalization/springy_fingers_calib)

# output files folder with empty placeholder
# set (outputDraft ${PROJECT_SOURCE_DIR}/output/empty)
# yarp_install(FILES ${outputDraft} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/simVisualTactileLocalization/output)

# add uninstall target
icubcontrib_add_uninstall_target()
