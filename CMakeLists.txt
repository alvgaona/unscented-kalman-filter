cmake_minimum_required(VERSION 3.11.3)

set(CMAKE_CXX_STANDARD 14)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS} -pthread")

project(unscented_kalman_filter)

find_package(PCL 1.7 REQUIRED)

file(GLOB SOURCES src/*.cpp)

add_executable (ukf ${SOURCES})
add_definitions(${PCL_DEFINITIONS})
target_include_directories(ukf PRIVATE
    include
    ${PCL_INCLUDE_DIRS})
target_link_directories(ukf PRIVATE
    lib
    ${PCL_LIBRARY_DIRS})
target_link_libraries (ukf PRIVATE ${PCL_LIBRARIES})

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
