cmake_minimum_required(VERSION 2.8)
project(PiDLidar)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(PythonLibs REQUIRED)
link_directories(${PYTHON_LIBRARIES})
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(include)
include_directories(src)

option(BUILD_CPP_EXAMPLE "Build the c++ example project" OFF)

IF (WIN32)
FILE(GLOB SDK_SRC
  "src/*.cpp"
  "src/*.c"
  "src/*.h"
  "src/impl/windows/*.cpp"
  "src/impl/windows/*.h"
)
ELSE()
FILE(GLOB SDK_SRC
  "src/*.cpp"
  "src/*.c"
  "src/*.h"
  "src/impl/unix/*.cpp"
  "src/impl/unix/*.h"
)
ENDIF()

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
  IF (WIN32)
  target_link_libraries(PiDLidar ydlidar_driver setupapi Winmm ${PYTHON_LIBRARIES})
  ELSE()
  target_link_libraries(PiDLidar ydlidar_driver rt pthread ${PYTHON_LIBRARIES})
  ENDIF()
endif(BUILD_CPP_EXAMPLE)
