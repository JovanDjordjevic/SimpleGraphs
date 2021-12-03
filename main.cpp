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

    std::cout << "Node count: " << g.getNodeCount() << " Edge count: " << g.getEdgeCount() << std::endl;
    std::cout << g << std::endl;

    // int startNode = 1;
    // int endNode = 8;
    std::string startNode = "node2";
    std::string endNode = "node6"; 

    // auto ret = GraphAlgorithms::dijkstra(g, startNode, endNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    // auto ret = GraphAlgorithms::bellmanFord(g, startNode, GraphAlgorithms::AlgorithmBehavior::PrintAndReturn);
    auto ret = GraphAlgorithms::floydWarshall(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);

    return 0;
}