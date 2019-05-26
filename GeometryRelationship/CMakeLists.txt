cmake_minimum_required(VERSION 3.0)
project(GeometryRelationship CXX C)

set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

### install lib, only do that if *.c or *.cpp exist
# since C srcs exist, use *.c* as pattern, not *.cpp
file(GLOB ex_srcs "${PROJECT_SOURCE_DIR}/src/GeometryRelationship.cpp")

# Eigen structure
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# PCL structure
find_package(PCL 1.8 REQUIRED COMPONENTS visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# export cmake setting to the source code
include_directories("${PROJECT_SOURCE_DIR}/include")

add_executable(${PROJECT_NAME} ${ex_srcs})
# linking with dependences

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen  ${PCL_LIBRARIES})