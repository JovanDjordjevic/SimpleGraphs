add_library(
    CustomClass
    customClass.cpp
    customClass.hpp
)

target_include_directories(CustomClass PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")