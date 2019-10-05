cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloud_viewer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
# Add Packages
find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(glib)
include_directories(.)

add_subdirectory(glib)
aux_source_directory(. MAIN_SRC)
# add_executable (SegFSR ${MAIN_SRC})
add_executable (SegFSR main.cpp BoundingBox.cpp zBuffer.cpp SegFSR.cpp graph.cpp)

# link library
target_link_libraries (SegFSR ${PCL_LIBRARIES})
target_link_libraries(SegFSR GLIB)
target_link_libraries(SegFSR ${OpenCV_LIBS})
