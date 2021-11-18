#include <iostream>
#include <string>

#include "SimpleGraphs.hpp"

int main () {
    GraphClasses::Graph<int, int> g1;
    // g.configureDirections(GraphClasses::GraphType::Unset);
    // g.configureWeights(GraphClasses::GraphWeights::Unset);
    // std::cout << g.isConfigured() << std::endl;

    g1.configureDirections(GraphClasses::GraphType::Directed);
    g1.configureWeights(GraphClasses::GraphWeights::Weighted);
    //std::cout << g.isConfigured() << std::endl;

    g1.addEdge(2, 5, 1);
    g1.addEdge(2, 6, 2);
    g1.addEdge(3, 5, 4);
    // g1.addEdge(7, 8);
    g1.addNode(5);
    std::cout << g1 << std::endl;

    std::cout << "-----------------------------" << std::endl;
    GraphClasses::Graph<std::string, double> g2;
    // GraphClasses::Graph<const char*, double> g2;   // FIXME: duplicated inserting with const char*
    g2.configureDirections(GraphClasses::GraphType::Undirected);
    g2.configureWeights(GraphClasses::GraphWeights::Unweighted);
    g2.addEdge("node1", "node2");
    g2.addEdge("node2", "node3");
    g2.addEdge("node1", "node3");
    // g2.addEdge("node5", "node6", 9);
    g2.addNode("node4");
    std::cout << g2 << std::endl;



    return 0;
}