cmake_minimum_required(VERSION 3.16)

project(simpleGraphs VERSION 1.0.0 LANGUAGES CXX)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "In-source builds not allowed. Please make a build directory.")
endif()

if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to Debug")
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_CXX_STANDARD          17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)                   # NOTE: does nothing if generator is set to Visual Studio
set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE ON)

# optional targets
option(ENABLE_DOXYGEN "Create doxygen documentation target" OFF)
option(ENABLE_TESTING "Build the test executables" ON)
option(ENABLE_COVERAGE_REPORT "Create code coverage report for tests" ON)   # requires lcov for GNU and llvm-cov for [Apple]Clang compilers

if(ENABLE_COVERAGE_REPORT AND NOT ENABLE_TESTING)
    message(FATAL_ERROR "Code coverage only available when building tests")
endif()

# compilers detection and flags
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "8")
        message(FATAL_ERROR "\nGNU v8 or later is required")
    endif()

    set(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wextra -pedantic -O0")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "6.0")
        message(FATAL_ERROR "\nClang v6.0 or later is required")
    endif()

    set(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wextra -pedantic -O0")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "10.0")
        message(FATAL_ERROR "\nAppleClang v10.0 or later is required")
    endif()

    set(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wextra -pedantic -O0")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    if(NOT MSVC_VERSION GREATER_EQUAL 1915)
        message(FATAL_ERROR "\nMSVC++ v19.15 or later is required")
    endif()

    if(ENABLE_COVERAGE_REPORT)
        message(FATAL_ERROR "\nCoverage report generation only supported on GNU and Clang compilers")
    endif()

    set(CMAKE_CXX_FLAGS_DEBUG_INIT "/Wall")
    set(CMAKE_CXX_FLAGS_RELEASE_INIT "/Ox")
else()
    message(FATAL_ERROR "\n\
        Currently supported compilers:\n\
        - GNU v8 or above\n\
        - Clang v6.0 or above\n\
        - AppleClang v10.0 or above\n\
        - MSVC v19.15 or above\n")
endif()

add_subdirectory(include)

if(ENABLE_DOXYGEN)
    include(docs)
endif()

if(ENABLE_TESTING)
    if(ENABLE_COVERAGE_REPORT)
        # To generate html report, build the ccov-all target
        set(CODE_COVERAGE ON)
        include(code-coverage)
        add_code_coverage_all_targets()
    endif()

    include(CTest)
    message(STATUS "Building tests")
    add_subdirectory(tests)
endif()