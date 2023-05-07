set(SIMPLE_GRAPHS_HEADERS "simpleGraphs.hpp")
set(SIMPLE_GRAPHS_INCLUDES  "./")

add_library(simpleGraphs INTERFACE)

target_include_directories(simpleGraphs INTERFACE ${SIMPLE_GRAPHS_INCLUDES})