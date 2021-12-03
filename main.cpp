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

    // const char* fileName = "testInputs/int_int.txt";
    // GraphClasses::Graph<int, int> g;

    // const char* fileName = "testInputs/string_double.txt";
    // GraphClasses::Graph<std::string, double> g;

    // const char* fileName = "testInputs/DIMACS_instances/USA-road-d.NY.txt";
    // GraphClasses::Graph<unsigned, unsigned> g;

    const char* fileName = "testInputs/custom_float.txt";
    GraphClasses::Graph<CustomClass, float> g;

    g.configureDirections(GraphClasses::GraphType::Directed);
    g.configureWeights(GraphClasses::GraphWeights::Weighted);

    g.readFromTxt(fileName);
    //g.readFromDimacs(fileName);

    std::cout << "Node count: " << g.getNodeCount() << " Edge count: " << g.getEdgeCount() << std::endl;
    std::cout << g << std::endl;
    //g.writeToTxt("test_otput.txt");

    // int start = 1;
    // std::string start = "node1";
    CustomClass start = {1, 2, 3};
    // GraphAlgorithms::dfs(g, start);
    GraphAlgorithms::bfs(g, start);

    //std::cout << globalAlloc << std::endl;
    // int startNode = 1;
    // int endNode = 8;
    // std::string startNode = "node2";
    // std::string endNode = "node6"; 

    // auto ret = GraphAlgorithms::dijkstra(g, startNode, endNode, GraphAlgorithms::AlgorithmBehavior::PrintAndReturn);
    // auto ret = GraphAlgorithms::bellmanFord(g, startNode, GraphAlgorithms::AlgorithmBehavior::PrintAndReturn);
    // auto ret = GraphAlgorithms::floydWarshall(g, GraphAlgorithms::AlgorithmBehavior::PrintAndReturn);

    return 0;
}