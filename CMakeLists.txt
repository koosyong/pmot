cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

## Includes
set( sources
    src/main.cpp
    src/pctracking.cpp
    src/pctracking_weight.cpp
    src/particlepose.cpp
    src/csfeature.cpp

    src/pcobject.cpp
    src/pcobjectcontainer.cpp
    src/track.hpp
    src/trackcontainer.hpp
    src/pctrackcontainer.cpp

    src/imft.hpp
    src/gaussian.cpp
    src/gaussiantrackcontainer.cpp
 )

set( headers
    src/pctracking.h
    src/pctracking_weight.h
    src/particlepose.h
    src/csfeature.h
    src/csgpu.h

    src/pcobject.h
    src/pcobjectcontainer.h
    src/track.h
    src/trackcontainer.h
    src/pctrackcontainer.h
    src/imft.h
    src/gaussian.h
    src/gaussiantrackcontainer.h
)

## CUDA
FIND_PACKAGE(CUDA REQUIRED)
INCLUDE(FindCUDA)
CUDA_ADD_LIBRARY (csgpulib src/csgpu.cu src/csgpu.h)
CUDA_ADD_EXECUTABLE(pmot src/csgpu.cu)

## VXL
include_directories(
    /usr/local/include/vxl/vcl
    /usr/local/include/vxl/core
    /usr/local/include/vxl/core/vnl/algo
    /usr/local/include/vxl/v3p/netlib
)
FIND_PACKAGE(VXL)

## PCL 1.8
find_package(PCL 1.8 REQUIRED)
include_directories(BEFORE ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
if (NOT PCL_FOUND)
    MESSAGE(FATAL_ERROR "PCL not found.\n")
endif (NOT PCL_FOUND)

## Qt
find_package(Qt4)
set(QT_USE_QTOPENGL TRUE)
include(${QT_USE_FILE})

## ROS build
rosbuild_add_executable(pmot ${sources} ${headers})
target_link_libraries(pmot vnl_algo vnl vcl QtGui ${PCL_LIBRARIES})
SET_TARGET_PROPERTIES(pmot PROPERTIES LINK_FLAGS -L${PCL_LIBRARY_DIRS})
