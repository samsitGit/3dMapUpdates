cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(3dMapUpdates)

set(VCPKG_LIBRARY_LINKAGE static)
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (3dMapUpdates 3dMapUpdates.cpp "Trace.cpp" "Trace.h" "Basemap.cpp" "Basemap.h")
target_link_libraries (3dMapUpdates ${PCL_LIBRARIES})
