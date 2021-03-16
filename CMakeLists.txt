cmake_minimum_required(VERSION 3.16)

# CMakeLists.txt layout from    https://github.com/foonathan/memory/blob/master/CMakeLists.txt
#                               https://github.com/nlohmann/json/blob/develop/CMakeLists.txt

##
## PROJECT
##
set(COBRA_VERSION 1.0.0)
project(cobra VERSION ${COBRA_VERSION} LANGUAGES CXX)

##
## OPTIONS
##
option(COBRA_TESTING                "Build the unit tests and setup for code coverage."     OFF)
option(COBRA_WARNINGS_AS_ERRORS     "Build with Werror"                                     ON)

##
## CONFIGURATION
##
include(GNUInstallDirs)

set(COBRA_TARGET_NAME                   ${PROJECT_NAME})
set(COBRA_CONFIG_INSTALL_DIR            "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}" CACHE INTERNAL "")
set(COBRA_INCLUDE_INSTALL_DIR           "${CMAKE_INSTALL_INCLUDEDIR}")
set(COBRA_TARGETS_EXPORT_NAME           "${PROJECT_NAME}Targets")
set(COBRA_CMAKE_CONFIG_TEMPLATE         "cmake/config.cmake.in")
set(COBRA_CMAKE_CONFIG_DIR              "${CMAKE_CURRENT_BINARY_DIR}")
set(COBRA_CMAKE_VERSION_CONFIG_FILE     "${COBRA_CMAKE_CONFIG_DIR}/${PROJECT_NAME}ConfigVersion.cmake")
set(COBRA_CMAKE_PROJECT_CONFIG_FILE     "${COBRA_CMAKE_CONFIG_DIR}/${PROJECT_NAME}Config.cmake")
set(COBRA_CMAKE_PROJECT_TARGETS_FILE    "${COBRA_CMAKE_CONFIG_DIR}/${PROJECT_NAME}Targets.cmake")
set(COBRA_INCLUDE_BUILD_DIR             "${PROJECT_SOURCE_DIR}/include/")

set(CMAKE_CXX_STANDARD 17)

set(WARNING_FLAGS "-Wall -Wextra -Wpedantic -Wuninitialized")
set(SANITIZERS_FLAGS "-fno-omit-frame-pointer ")
set(OPT_FLAGS "-O3 -march=native -ffat-lto-objects -flto -fwhole-program ")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${WARNING_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${SANITIZERS_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${OPT_FLAGS}")

if(COBRA_WARNINGS_AS_ERRORS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
endif()

if(COBRA_TESTING)
    set(CMAKE_BUILD_TYPE Debug)
else()
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE Release)
    endif()
endif()



##
## SUBDIRECTORIES
##
add_subdirectory(src)


##
## INSTALL
##
install(EXPORT cobraTargets
        DESTINATION ${COBRA_CONFIG_INSTALL_DIR}
        FILE cobra-config.cmake
        CONFIGURATIONS Release)

##
## UNINSTALL TARGET
## `sudo make uninstall` from source directory
##
if(NOT TARGET uninstall)
    configure_file(
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
            "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake"
            IMMEDIATE @ONLY)

    add_custom_target(uninstall
            COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake)
endif()
