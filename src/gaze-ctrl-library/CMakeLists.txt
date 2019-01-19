project("iCubGazeCtrlLibrary")

set(LIBRARY_TARGET_NAME ${PROJECT_NAME})

find_package(YARP 3.0.0 COMPONENTS OS math REQUIRED)
find_package(ICUB REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})

set(${LIBRARY_TARGET_NAME}_HDR include/GazeController.h)
set(${LIBRARY_TARGET_NAME}_SRC src/GazeController.cpp)

add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_HDR} ${${LIBRARY_TARGET_NAME}_SRC})

target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC
                                                  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                                  )

target_link_libraries(${LIBRARY_TARGET_NAME} PRIVATE
                                             YARP::YARP_OS
                                             YARP::YARP_math
                                             ${ICUB_LIBRARIES}
                                             )
