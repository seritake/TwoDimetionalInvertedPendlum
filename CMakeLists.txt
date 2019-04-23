cmake_minimum_required(VERSION 3.5)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(Eigen 3 QUIET)
if(NOT Eigen_FOUND)
  include(./cmake/FindEigen3.cmake)
  set(Eigen_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR})
endif()
include_directories(${Eigen_INCLUDE_DIR})

add_subdirectory(src)
add_executable(Main main.cpp)

find_package(OpenCV REQUIRED)

# インクルードパスを指定
include_directories(${OpenCV_INCLUDE_DIRS})

# リンクするライブラリ指定
target_link_libraries(Main ${OpenCV_LIBS})
include_directories(include)
#include_directories(/usr/local/include/eigen3)
find_package (Threads)
target_link_libraries(Main Robot ${CMAKE_THREAD_LIBS_INIT})