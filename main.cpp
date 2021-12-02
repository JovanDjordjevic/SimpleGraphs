#include <iostream>
#include <string>

#include "SimpleGraphs.hpp"

int main () {

    // const char* fileName = "testInputs/int_int.txt";
    // GraphClasses::Graph<int, int> g;

    // const char* fileName = "testInputs/string_double.txt";
    // GraphClasses::Graph<std::string, double> g;

    const char* fileName = "testInputs/DIMACS_instances/USA-road-d.NY.txt";
    GraphClasses::Graph<unsigned, unsigned> g;

    g.configureDirections(GraphClasses::GraphType::Directed);
    g.configureWeights(GraphClasses::GraphWeights::Weighted);

    //g.readFromTxt(fileName);
    g.readFromDimacs(fileName);

    std::cout << "Node count: " << g.getNodeCount() << " Edge count: " << g.getEdgeCount() << std::endl;
    //std::cout << g << std::endl;
    g.writeToTxt("test_otput.txt");

    return 0;
}