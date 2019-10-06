cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloud_viewer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
# Add Packages
find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)

# OpenMP
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(glib)
include_directories(.)

add_subdirectory(glib)
aux_source_directory(. MAIN_SRC)
# add_executable (SegFSR ${MAIN_SRC})
add_executable (SegFSR main.cpp BoundingBox.cpp SegFSR.cpp FloodFill.cpp)

# link library
target_link_libraries (SegFSR ${PCL_LIBRARIES})
target_link_libraries(SegFSR GLIB)
target_link_libraries(SegFSR ${OpenCV_LIBS})
