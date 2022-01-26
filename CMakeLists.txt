cmake_minimum_required(VERSION 3.16)
project(SimpleGraphsProject VERSION 1.0.0)

enable_testing()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_DEBUG_INIT "-g -Wall -Wextra -pedantic -O0")
set(CMAKE_CXX_FLAGS_RELEASE_INIT "-O3")

add_subdirectory(CustomClass)

add_library(SimpleGraphs INTERFACE)

add_executable(main main.cpp)
add_executable(runTests tests.cpp)

add_test(
    NAME runTests
    COMMAND runTests
)

target_link_libraries(runTests PRIVATE SimpleGraphs)
target_link_libraries(runTests PRIVATE CustomClass)