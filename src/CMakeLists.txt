##
## LIBRARY HEADERS
##
set(HEADER_PATH "${PROJECT_SOURCE_DIR}/include/cobra")
set(HEADER
    ${HEADER_PATH}/mm.hpp
    ${HEADER_PATH}/NonCopyable.hpp
    ${HEADER_PATH}/Instance.hpp
    ${HEADER_PATH}/AbstractInstanceParser.hpp
    ${HEADER_PATH}/Solution.hpp
    ${HEADER_PATH}/FixedSizeValueStack.hpp
    ${HEADER_PATH}/LocalSearch.hpp
    ${HEADER_PATH}/MoveGenerators.hpp
    ${HEADER_PATH}/Flat2DVector.hpp
    ${HEADER_PATH}/PrettyPrinter.hpp
    ${HEADER_PATH}/Welford.hpp
    ${HEADER_PATH}/BitMatrix.hpp
    ${HEADER_PATH}/LRUCache.hpp
    ${HEADER_PATH}/SimulatedAnnealing.hpp
    ${HEADER_PATH}/VertexSet.hpp
    )

##
## LIBRARY SOURCES
##
set(SOURCE
        macro/macro.hpp
        solution/Solution.cpp
        solution/SolutionConstruction.cpp
        movegen/MoveGenerators.cpp
        movegen/AbstractMoveGeneratorsView.cpp
        movegen/MoveGeneratorsHeap.cpp
        solution/SolutionUtilities.cpp)

##
## TARGET
##
add_library(${COBRA_TARGET_NAME} ${HEADER} ${SOURCE})
add_library(${PROJECT_NAME}::${COBRA_TARGET_NAME} ALIAS ${COBRA_TARGET_NAME})

target_compile_features(cobra PUBLIC cxx_std_17)            # All users of this library will need at least C++17

target_include_directories(
        ${COBRA_TARGET_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${COBRA_INCLUDE_BUILD_DIR}>
        $<INSTALL_INTERFACE:include>
)

set_target_properties(${COBRA_TARGET_NAME} PROPERTIES
        OUTPUT_NAME "${COBRA_TARGET_NAME}-${COBRA_VERSION}"
        POSITION_INDEPENDENT_CODE ON)

install(TARGETS ${COBRA_TARGET_NAME} EXPORT cobraTargets)

##
## LIBRARY VERSION
##
include(CMakePackageConfigHelpers)
set(VERSION_FILE "${CMAKE_CURRENT_BINARY_DIR}/cmake/${COBRA_TARGET_NAME}-config-version.cmake")
write_basic_package_version_file(${VERSION_FILE}
        VERSION ${COBRA_VERSION}
        COMPATIBILITY AnyNewerVersion)

##
## INSTALL
## includes and version file
##
install(FILES ${HEADER}                                   DESTINATION ${COBRA_INCLUDE_INSTALL_DIR}/cobra)
install(FILES ${VERSION_FILE}                             DESTINATION ${COBRA_CONFIG_INSTALL_DIR})
