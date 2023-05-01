set(CUSTOM_CLASS_SOURCES "customClass.cpp")
set(LIBRARY_HEADERS "customClass.hpp")
set(LIBRARY_INCLUDES  "./")

add_library(customClass STATIC ${CUSTOM_CLASS_SOURCES})
target_include_directories(customClass PUBLIC ${LIBRARY_INCLUDES})