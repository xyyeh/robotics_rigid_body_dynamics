# based on:
# https://pabloariasal.github.io/2018/02/19/its-time-to-do-cmake-right/

# mimumum cmake version that support targets
cmake_minimum_required(VERSION 3.2)

# include helper functions for printing variables
# usage: cmake_print_variables(CMAKE_BUILD_TYPE)
include(CMakePrintHelpers)

# git submodule cmake_tools for subsequent use
if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_tools/.git)
    execute_process(
        COMMAND git submodule update --init cmake/cmake_tools
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        )
endif()

# project configuration
# 1.x.x - update if incompatible API changes are made
# x.1.x - update if changes are backward compatible
# x.x.1 - update if backward compatible bug fixes are made
project(robotics_rigid_body_dynamics VERSION 1.0.0)

# third party options
option(BUILD_THIRD_PARTY "Build third party dependencies" OFF)

# project default settings
# debug/release builds
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_tools/DefaultBuildType.cmake)
DefaultBuildType()
# static/shared library builds are defined using arguments
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

# setup dependencies
if(${BUILD_THIRD_PARTY})
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_tools/BuildThirdParty.cmake)

    find_package(SpaceVecAlg HINTS ${CMAKE_INSTALL_PREFIX})
    if(NOT ${SpaceVecAlg_FOUND})
        BuildThirdParty(${CMAKE_CURRENT_SOURCE_DIR}/third_party/build_SpaceVecAlg.sh)
    else()
        message(STATUS "Found SpaceVecAlg at ${SpaceVecAlg_DIR}")
    endif()
    
    find_package(RBDyn HINTS ${CMAKE_INSTALL_PREFIX})
    if(NOT ${RBDyn_FOUND})
        BuildThirdParty(${CMAKE_CURRENT_SOURCE_DIR}/third_party/build_RBDyn.sh)
    else()
        message(STATUS "Found RBDyn at ${RBDyn_DIR}")
    endif()

    find_package(spdlog 1.6.0 HINTS ${CMAKE_INSTALL_PREFIX})
    if(NOT ${spdlog_FOUND})
        BuildThirdParty(${CMAKE_CURRENT_SOURCE_DIR}/third_party/build_spdlog.sh)
    else()
        message(STATUS "Found spdlog at ${spdlog_DIR}")
    endif()
endif()

# setup library version with branch and commit hash
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_tools/GitVersion.cmake)
GitVersion()
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.h.in"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/robotics/rigid_body_dynamics/config.h"
)

# set location where static (archive) and shared libraries are built
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)

# set library sources
file(GLOB SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc)

# create library with default build type
add_library(${PROJECT_NAME} ${SRC_FILES})

# create an alias of the library using robotics namespace
add_library(robotics::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# set location of include headers
# public includes are used for build and are installed
# private includes are used only to build and will not be installed
target_include_directories(
    ${PROJECT_NAME}
    # Public if includes are used for build and exposed
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<INSTALL_INTERFACE:include> #<prefix>/include/${PROJECT_NAME}
    # Private if includes are used for build and not exposed  
    PRIVATE
)

# set compile features to require c++14
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
# make warnings in to errors for this project only
target_compile_options(${PROJECT_NAME} PRIVATE -Werror)
# specify required features such as cxx_*
# target_compile_features(${PROJECT_NAME} PRIVATE cxx_lambdas)
set(CMAKE_VERBOSE_MAKEFILE ON)

# setup internal dependencies
# public dependencies will be used in headers and their interface is offerred to clients
# private dependencies will be used in source and their interface is not offerred to clients
if (${BUILD_THIRD_PARTY})
    # submodule third party dependencies
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_tools/GitSubmoduleUpdateInit.cmake)

    # add dependency to the build
    # if (NOT TARGET <robotics_x>)
    #     GitSubmoduleUpdateInit(third_party/robotics_x)
    #     add_subdirectory(third_party/robotics_x)
    # endif()
else()
    # otherwise, find installed packages
    # find_package(robotics_x REQUIRED)
endif()

# setup external dependencies
find_package(SpaceVecAlg REQUIRED HINTS ${CMAKE_INSTALL_PREFIX})
find_package(RBDyn REQUIRED HINTS ${CMAKE_INSTALL_PREFIX})
find_package(spdlog REQUIRED HINTS ${CMAKE_INSTALL_PREFIX})

# link dependencies
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        RBDyn::RBDyn
        SpaceVecAlg::SpaceVecAlg
        spdlog::spdlog
    PRIVATE
        RBDyn::Parsers
)

# install library
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_tools/InstallLibrary.cmake)
InstallLibrary()

# test
# add more tests using add_test(). BUILD_TESTING is automatically set ON by CTest
include( CTest )
if (${BUILD_TESTING})
    # create executable from source
    file(GLOB TEST_FILES ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cc)
    add_executable(${PROJECT_NAME}_test ${TEST_FILES})

    # find all packages except ${PROJECT_NAME} since it's not created yet
    find_package(GTest REQUIRED)

    # set location of headers, use private since we are not installing
    target_include_directories(
        ${PROJECT_NAME}_test
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${GTEST_INCLUDE_DIRS}
    )

    # link libraries
    target_link_libraries(
        ${PROJECT_NAME}_test
        robotics::${PROJECT_NAME}
        ${GTEST_BOTH_LIBRARIES}
        -pthread
    )

    # make test will run ${PROJECT_NAME}_test executable
    add_test(NAME ${PROJECT_NAME}_test COMMAND ${PROJECT_NAME}_test)
endif()