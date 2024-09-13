cmake_minimum_required(VERSION 3.15)
project(CEVRP)

set(CMAKE_CXX_STANDARD 17)

add_executable(CEVRP main.cpp
        include/case.hpp
        src/case.cpp)

# Add include directories, so that the compiler can find the header files
target_include_directories(CEVRP PRIVATE include external/include)