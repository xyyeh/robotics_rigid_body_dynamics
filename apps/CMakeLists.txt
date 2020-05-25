# mimumum cmake version that support targets
cmake_minimum_required(VERSION 3.2)

# include helper functions for printing variables
# usage: cmake_print_variables(CMAKE_BUILD_TYPE)
include(CMakePrintHelpers)

# project configuration
project(example_app)

# project options

# project default settings
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/cmake_tools/DefaultBuildType.cmake)
DefaultBuildType()

# setup executable
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../bin")

# create executable from source
file(GLOB SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)
add_executable(${PROJECT_NAME} ${SRC_FILES})

# setup dependencies
find_package(robotics_rigid_body_dynamics REQUIRED)
target_link_libraries(${PROJECT_NAME}
  robotics::robotics_rigid_body_dynamics
)

# set cmake compile features
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)
target_compile_options(${PROJECT_NAME} PRIVATE -Werror)
set(CMAKE_VERBOSE_MAKEFILE ON)