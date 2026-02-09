cmake_minimum_required(VERSION 3.10)
project(playback)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable(environment
  src/environment.cpp
  src/render/render.cpp
  src/processPointClouds.cpp
)

target_link_libraries(environment ${PCL_LIBRARIES})



