set(LIBRARY_HEADERS "simpleGraphs.hpp")
set(LIBRARY_INCLUDES  "./")

add_library(simpleGraphs INTERFACE)

target_include_directories(simpleGraphs INTERFACE ${LIBRARY_INCLUDES})