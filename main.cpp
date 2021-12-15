#include <iostream>
#include <string>

#include "customClass.hpp"
#include "SimpleGraphs.hpp"

#include <unordered_set>

int globalAlloc = 0;

void* operator new(size_t size)
{
    // std::cout << "Heap allocation" << std::endl;      // i dont't know why but this line causes a crash
    ++globalAlloc;
    return malloc(size);
}


int main () {

    const char* fileName1 = "testInputs/int_int_artic.txt";
    GraphClasses::Graph<int, int> g1;

    const char* fileName2 = "testInputs/int_int.txt";
    GraphClasses::Graph<int, int> g2;

    // const char* fileName = "testInputs/string_double.txt";
    // GraphClasses::Graph<std::string, double> g;

    // const char* fileName = "testInputs/DIMACS_instances/USA-road-d.NY.txt";
    // GraphClasses::Graph<unsigned, unsigned> g;

    // const char* fileName = "testInputs/custom_float.txt";
    // GraphClasses::Graph<CustomClass, double> g;      // FIXME: using float instead of double gives compilation error when trying to use dijsktra algorithm
                                                     // so far other algorithms work fine with float, gives warning for possible loss of data during conversion
                                                     // compilation fails with MSVC but not with clang

    g1.configureDirections(GraphClasses::GraphType::Directed);
    g1.configureWeights(GraphClasses::GraphWeights::Weighted);

    g1.readFromTxt(fileName1);

    std::cout << "Node count: " << g1.getNodeCount() << " Edge count: " << g1.getEdgeCount() << std::endl;
    std::cout << g1 << std::endl;
    std::cout << "--------" << std::endl;

    g2.configureDirections(GraphClasses::GraphType::Directed);
    g2.configureWeights(GraphClasses::GraphWeights::Weighted);
    
    g2.readFromTxt(fileName2);

    std::cout << "Node count: " << g2.getNodeCount() << " Edge count: " << g2.getEdgeCount() << std::endl;
    std::cout << g2 << std::endl;
    std::cout << "--------" << std::endl;


    //auto ret = GraphUtility::mergeGraphs(g1, g2);
    auto ret = GraphUtility::intersectGraphs(g1, g2);
    std::cout << "Node count: " << ret.getNodeCount() << " Edge count: " << ret.getEdgeCount() << std::endl;
    std::cout << ret << std::endl;

    return 0;
}