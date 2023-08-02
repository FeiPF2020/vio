cmake_minimum_required(VERSION 3.20)
project(chapter01)

include_directories("usr/include/eigen3")

set(CMAKE_CXX_STANDARD 14)

add_executable(chapter01 main.cpp)
