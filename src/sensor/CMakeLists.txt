cmake_minimum_required(VERSION 2.8)
project(sensor)

find_package(PCL 1.2 REQUIRED)
set(SENSOR_INCLUDE_DIRS "include")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(srcs
src/PlaneModel.cpp 
src/PlaneModelSegmentation.cpp 
src/utility.cpp 
)

add_library(sensor ${srcs})
target_link_libraries(sensor ${PCL_LIBRARIES})