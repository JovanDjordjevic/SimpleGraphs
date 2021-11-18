#include <iostream>
#include <string>

#include "SimpleGraphs.hpp"

int main () {

    // const char* fileName = "testInputs/int_int.txt";
    // GraphClasses::Graph<int, int> g;

    const char* fileName = "testInputs/string_double.txt";
    GraphClasses::Graph<std::string, double> g;
    
    g.configureDirections(GraphClasses::GraphType::Directed);
    g.configureWeights(GraphClasses::GraphWeights::Weighted);

    g.readFromTxt(fileName);

    std::cout << g << std::endl;

    g.writeToTxt("tmp.txt");

    // std::cout << "-----------------------------" << std::endl;
    // GraphClasses::Graph<std::string, double> g2;
    // // GraphClasses::Graph<const char*, double> g2;   // FIXME: duplicated inserting with const char*
    // g2.configureDirections(GraphClasses::GraphType::Undirected);
    // g2.configureWeights(GraphClasses::GraphWeights::Unweighted);
    // g2.addEdge("node1", "node2");
    // g2.addEdge("node2", "node3");
    // g2.addEdge("node1", "node3");
    // // g2.addEdge("node5", "node6", 9);
    // g2.addNode("node4");
    // std::cout << g2 << std::endl;

    return 0;
}