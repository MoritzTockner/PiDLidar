cmake_minimum_required(VERSION 2.8)
project(PiDLidar)
#add_definitions(-std=c++11) # Use C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -I/usr/include/python3.5 ")
include_directories(include)
include_directories(src)

option(BUILD_CPP_EXAMPLE "Build the c++ example project" OFF)

FILE(GLOB SDK_SRC 
  "src/*.cpp"
  "src/*.c"
  "src/*.h"
  "src/impl/unix/*.cpp"
  "src/impl/unix/*.h"
)

if(NOT BUILD_CPP_EXAMPLE)
  add_subdirectory(pybind11)
  pybind11_add_module(PiDLidar src/wrapYDLidar.cpp)
else(NOT BUILD_CPP_EXAMPLE)
  add_subdirectory(samples)
endif(NOT BUILD_CPP_EXAMPLE)

add_library(ydlidar_driver STATIC ${SDK_SRC})

if(BUILD_CPP_EXAMPLE)
  target_link_libraries(ydlidar_driver rt pthread)
else(BUILD_CPP_EXAMPLE)
  target_link_libraries(PiDLidar ydlidar_driver rt pthread)
endif(BUILD_CPP_EXAMPLE)
