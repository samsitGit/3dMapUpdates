cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(3dMapUpdates)

set(VCPKG_LIBRARY_LINKAGE static)
set(ZLIB_ROOT "C:/Users/irfan/vcpkg/packages/zlib_x64-windows")
find_package(ZLIB)
find_package(freetype 2.13.2 REQUIRED)
find_package(glew 2.2.0 REQUIRED)
find_package(PCL 1.14 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (3dMapUpdates 3dMapUpdates.cpp "Trace.cpp" "Trace.h" "Basemap.cpp" "Basemap.h" "utils.h" "utils.cpp" "TraceProcessor.cpp" "TraceProcessor.h" "FrameProcessor.h" "FrameProcessor.cpp" "Cluster.cpp" "Cluster.h" "ctpl.h")
target_link_libraries (3dMapUpdates ${PCL_LIBRARIES})
