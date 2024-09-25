cmake_minimum_required(VERSION 3.15)
project(CEVRP)

set(GOOGLETEST_VERSION 1.11.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3") # -O3 optimization argument


set(DEPENDENCIES
        include/case.hpp
        src/case.cpp
        include/individual.hpp
        src/individual.cpp
        include/utils.hpp
        src/utils.cpp
        include/stats_interface.hpp
        src/stats_interface.cpp
        include/ma.hpp
        src/ma.cpp
        include/heuristic_interface.hpp
)


add_executable(Run main.cpp ${DEPENDENCIES})

# Add include directories, so that the compiler can find the header files
target_include_directories(Run PRIVATE include external/include)

add_subdirectory(external/googletest) # Add GoogleTest as a subdirectory
add_executable(Tests tests/test_main.cpp
        ${DEPENDENCIES}
        tests/case_test.cpp
        tests/utils_test.cpp
        tests/stats_interface_test.cpp
        tests/ma_test.cpp)
target_include_directories(Tests PRIVATE include external/include) # Include directories for the tests
target_link_libraries(Tests gtest gtest_main) # Link GoogleTest for tests
