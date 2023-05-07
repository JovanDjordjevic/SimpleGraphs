set(CUSTOM_CLASS_SOURCES "customClass.cpp")
set(CUSTOM_CLASS_HEADERS "customClass.hpp")
set(CUSTOM_CLASS_INCLUDES  "./")

add_library(customClass STATIC ${CUSTOM_CLASS_SOURCES})
target_include_directories(customClass PUBLIC ${CUSTOM_CLASS_INCLUDES})