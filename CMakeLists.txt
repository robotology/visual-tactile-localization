cmake_minimum_required(VERSION 3.5)

project(object-tracking
        LANGUAGES CXX
        VERSION 0.0.0)

set(CMAKE_CXX_STANDARD 11)

# iCub contrib
find_package(ICUBcontrib REQUIRED)

# Default build type to Release
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
endif()

include(GNUInstallDirs)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake
                              ${ICUBCONTRIB_MODULE_PATH}
                              )
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

icubcontrib_set_default_prefix()
icubcontrib_add_uninstall_target()

add_subdirectory(src)



