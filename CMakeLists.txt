cmake_minimum_required(VERSION 3.15)
include(FetchContent)

project(Sim6DOF VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions("-Wall -Wextra")
file(GLOB_RECURSE src ./src/*.cc)

add_executable(6dof ${src})

configure_file(${PROJECT_SOURCE_DIR}/src/Sim6DOFInfo.h.in ${PROJECT_SOURCE_DIR}/src/Sim6DOFInfo.h)
target_include_directories(6dof PUBLIC
                           ${PROJECT_BINARY_DIR}
                           )
# CAELUS_FDM Headers
include_directories(PUBLIC "/usr/local/include/caelus_fdm/")
# Eigen Headers
include_directories(PUBLIC "/usr/local/Cellar/eigen/3.3.9/include/eigen3/")
# MAVLink headers
target_include_directories(6dof PUBLIC ${CMAKE_SOURCE_DIR}/mavlink/standard/)

add_subdirectory(dependencies)
target_link_libraries(6dof PUBLIC caelus_fdm)
