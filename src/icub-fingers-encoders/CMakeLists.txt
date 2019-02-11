project("iCubFingersEncoders")

set(LIBRARY_TARGET_NAME ${PROJECT_NAME})

find_package(YARP 3.0.0 COMPONENTS
                        OS
                        dev
                        math
                        REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})

set(${LIBRARY_TARGET_NAME}_HDR include/iCubFingersEncoders.h)
set(${LIBRARY_TARGET_NAME}_SRC src/iCubFingersEncoders.cpp)

add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_HDR} ${${LIBRARY_TARGET_NAME}_SRC})

target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC
                                                  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                                  )

target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC
                                             YARP::YARP_dev
                                             YARP::YARP_math
                                             YARP::YARP_OS
                                             )
