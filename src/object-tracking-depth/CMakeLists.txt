set(EXE_TARGET_NAME object-tracking-depth)

# YARP
find_package(YARP CONFIG REQUIRED
             COMPONENTS
             OS
             )
set(${EXE_TARGET_NAME}_SRC
    src/depth.cpp
    )

# Application target calls
add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC})

target_link_libraries(${EXE_TARGET_NAME} PRIVATE
                                         YARP::YARP_init
                                         YARP::YARP_OS
                                         SFMLib
                                         )

set(${EXE_TARGET_NAME}_CONF ${CMAKE_CURRENT_SOURCE_DIR}/conf/config.ini)

yarp_install(FILES ${${EXE_TARGET_NAME}_CONF} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking-depth)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
