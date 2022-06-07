cmake_minimum_required(VERSION 3.10.2)
project(divar3v)

set(DRAKE_BUILD_DIR "/opt/drake")
list(APPEND CMAKE_PREFIX_PATH ${DRAKE_BUILD_DIR})
list(APPEND CMAKE_PREFIX_PATH "${DRAKE_BUILD_DIR}/lib/cmake")
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")

find_package(drake CONFIG REQUIRED)
# find_package(GFlags MODULE REQUIRED COMPONENTS shared)

set(SPDLOG_INCLUDE_DIR /opt/drake/include/spdlog)
message(STATUS "SPDLOG_INCLUDE_DIR variable defaulted in CMakeList: " ${SPDLOG_INCLUDE_DIR})

configure_file(./src/urdfs/iCubGenova04_drake_obj.urdf "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)
configure_file(./src/urdfs/lcl.urdf "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)

add_executable(divar3v src/divar3v.cc)

target_link_libraries(divar3v
	drake::drake
	gflags
)
