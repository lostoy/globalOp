cmake_minimum_required(VERSION 2.8)
project(GlobalOp)
set(BOOST_LIBRARYDIR C:/local/boost_1_55_0/lib64-msvc-11.0)
set(BOOST_INCLUDEDIR C:/local/boost_1_55_0)

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_STATIC ON)
set(Boost_USE_MULTITHREAD )
set(Boost_NO_BOOST_CMAKE ON)

find_package(Boost COMPONENTS filesystem system REQUIRED)
find_package(PCL 1.2 REQUIRED COMPONENTS common io io_ply)

include_directories(${Boost_INCLUDE_DIRS} "C:/Program Files (x86)/Eigen/include")


set(
srcs_op src/mat2graph_main.cpp
)  

set(
srcs_view src/graph2mat_main.cpp)

set(
srcs_transform src/transform_main.cpp)

add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})

add_executable(Mat2Graph ${srcs_op} )
add_executable(Graph2Mat ${srcs_view})
add_executable(TransformOp ${srcs_transform})

target_link_libraries(Mat2Graph ${Boost_LIBRARIES} )
target_link_libraries(Graph2Mat ${Boost_LIBRARIES})

target_link_libraries(TransformOp ${Boost_LIBRARIES} ${PCL_LIBRARIES})


