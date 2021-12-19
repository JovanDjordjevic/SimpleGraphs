#include <iostream>
#include <string>

#include "customClass.hpp"
#include "SimpleGraphs.hpp"


int globalAlloc = 0;

void* operator new(size_t size)
{
    // std::cout << "Heap allocation" << std::endl;      // i dont't know why but this line causes a crash
    ++globalAlloc;
    return malloc(size);
}


int main () {

    // const char* fileName = "testInputs/int_int_artic.txt";
    // GraphClasses::Graph<int, int> g;

    const char* fileName = "testInputs/string_double.txt";
    GraphClasses::Graph<std::string, double> g;

    // const char* fileName = "testInputs/custom_float.txt";
    // GraphClasses::Graph<CustomClass, double> g;      // FIXME: using float instead of double gives compilation error when trying to use dijsktra algorithm
                                                     // so far other algorithms work fine with float, gives warning for possible loss of data during conversion
                                                     // compilation fails with MSVC but not with clang

    g.configureDirections(GraphClasses::GraphType::Undirected);
    g.configureWeights(GraphClasses::GraphWeights::Weighted);

    g.readFromTxt(fileName);

    std::cout << "Node count: " << g.getNodeCount() << " Edge count: " << g.getEdgeCount() << std::endl;
    std::cout << g << std::endl;
    //g.writeToTxt("test_otput.txt");

    auto ret1 = GraphAlgorithms::mcstPrim(g);
    auto ret2 = GraphAlgorithms::mcstKruskal(g);

    return 0;
}