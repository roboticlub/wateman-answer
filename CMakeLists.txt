cmake_minimum_required(VERSION 2.8)

project(test)
set(CMAKE_CXX_FLAGS -std=c++11)
set(CMAKE_BUILD_TYPE RELEASE)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} )

add_executable(answer1 "1.cpp")
add_executable(answer2 "2.cpp")
target_link_libraries(answer1 ${PCL_LIBRARIES} )
