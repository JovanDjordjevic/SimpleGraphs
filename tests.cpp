#include <iostream>
#include <string>
#include "CustomClass/customClass.hpp"
#include "SimpleGraphs.hpp"
#include <cassert>
#include <iomanip>
#include <cmath>

// apart from these tests, all functions were tested arbitrarily and seem to be working
// this DOES NOT guarantee correctness

template<typename DataType, typename WeightType>
void test_dfs(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, unsigned dfsTreeSize) {
    auto ret = GraphAlgorithms::dfs(g, startNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == dfsTreeSize);
}

template<typename DataType, typename WeightType>
void test_bfs(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, unsigned bfsTreeSize) {
    auto ret = GraphAlgorithms::bfs(g, startNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == bfsTreeSize);
}

template<typename DataType, typename WeightType>
void test_dijkstra(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, DataType endNode, unsigned edgesOnPath) {
    auto ret = GraphAlgorithms::dijkstra(g, startNode, endNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert((ret.size() - 1) == edgesOnPath);
}

template<typename DataType, typename WeightType>
void test_bellmanFord(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, DataType someEndNode, unsigned edgesOnPathToEndNode) {
    auto ret = GraphAlgorithms::bellmanFord(g, startNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert((ret[someEndNode].size() - 1) == edgesOnPathToEndNode);
}

template<typename DataType, typename WeightType>
void test_floydWarshall(GraphClasses::Graph<DataType, WeightType> &g, DataType someStartNode, DataType someEndNode, WeightType distance) {
    auto ret = GraphAlgorithms::floydWarshall(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(internal::equals(ret[someStartNode][someEndNode], distance));
}

template<typename DataType, typename WeightType>
void test_findArticulationPoints_without_start(GraphClasses::Graph<DataType, WeightType> &g, unsigned numOfArticulationPoints) {
    assert(g.getGraphType() == GraphClasses::GraphType::Undirected);
    auto ret = GraphAlgorithms::findArticulationPoints(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == numOfArticulationPoints);
}

template<typename DataType, typename WeightType>
void test_findArticulationPoints_with_start(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, unsigned numOfArticulationPoints) {
    auto ret = GraphAlgorithms::findArticulationPoints(g, startNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == numOfArticulationPoints);
}

template<typename DataType, typename WeightType>
void test_findBridges_without_start(GraphClasses::Graph<DataType, WeightType> &g, unsigned numOfBridges) {
    assert(g.getGraphType() == GraphClasses::GraphType::Undirected);
    auto ret = GraphAlgorithms::findBridges(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == numOfBridges);
}

template<typename DataType, typename WeightType>
void test_findBridges_with_start(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, unsigned numOfBridges) {
    auto ret = GraphAlgorithms::findBridges(g, startNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == numOfBridges);
}

template<typename DataType, typename WeightType>
void test_topsortKhan(GraphClasses::Graph<DataType, WeightType> &g, DataType firstNode, DataType lastNode) {
    auto ret = GraphAlgorithms::topsortKhan(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret[0] == firstNode);
    assert(ret[g.getNodeCount() - 1] == lastNode);
}

template<typename DataType, typename WeightType>
void test_mcstPrimTotalCostOnly(GraphClasses::Graph<DataType, WeightType> &g, WeightType totalCost) {
    auto ret = GraphAlgorithms::mcstPrimTotalCostOnly(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(internal::equals(ret, totalCost));
}

template<typename DataType, typename WeightType>
void test_mcstPrim(GraphClasses::Graph<DataType, WeightType> &g, unsigned edgeCount) {
    auto ret = GraphAlgorithms::mcstPrim(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == edgeCount);
}

template<typename DataType, typename WeightType>
void test_mcstKruskal(GraphClasses::Graph<DataType, WeightType> &g, unsigned edgeCount) {
    auto ret = GraphAlgorithms::mcstKruskal(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == edgeCount);
}

template<typename DataType, typename WeightType>
void test_findStronglyConnectedComponentsTarjan_without_start(GraphClasses::Graph<DataType, WeightType> &g, unsigned numOfComponents) {
    assert(g.getGraphType() == GraphClasses::GraphType::Undirected);
    auto ret = GraphAlgorithms::findStronglyConnectedComponentsTarjan(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == numOfComponents);
}

template<typename DataType, typename WeightType>
void test_findStronglyConnectedComponentsTarjan_with_start(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, unsigned numOfComponents) {
    auto ret = GraphAlgorithms::findStronglyConnectedComponentsTarjan(g, startNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == numOfComponents);
}

template<typename DataType, typename WeightType>
void test_findWeaklyConnectedComponents(GraphClasses::Graph<DataType, WeightType> &g, unsigned numOfComponents) {
    assert(g.getGraphType() == GraphClasses::GraphType::Undirected);
    auto ret = GraphAlgorithms::findWeaklyConnectedComponents(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == numOfComponents);
}

template<typename DataType, typename WeightType>
void test_mergeGraphs(GraphClasses::Graph<DataType, WeightType> &g1, GraphClasses::Graph<DataType, WeightType> &g2) {
    assert(g1.getGraphType() == g2.getGraphType());
    assert(g1.getGraphWeights() == g2.getGraphWeights());
    GraphClasses::Graph<DataType, WeightType> ret = GraphUtility::mergeGraphs(g1, g2);
    // std::cout << "Node count: " << ret.getNodeCount() << " Edge count: " << ret.getEdgeCount() << std::endl;
    // std::cout << ret << std::endl;
    // testing that 2 same graphs do not produce extra nodes or edges after merging
    assert(ret.getNodeCount() == g1.getNodeCount());
    assert(ret.getEdgeCount() == g1.getEdgeCount());
    assert(ret.getNodeCount() == g2.getNodeCount());
    assert(ret.getEdgeCount() == g2.getEdgeCount());
}

template<typename DataType, typename WeightType>
void test_intersectGraphs(GraphClasses::Graph<DataType, WeightType> &g1, GraphClasses::Graph<DataType, WeightType> &g2) {
    assert(g1.getGraphType() == g2.getGraphType());
    assert(g1.getGraphWeights() == g2.getGraphWeights());
    GraphClasses::Graph<DataType, WeightType> ret = GraphUtility::intersectGraphs(g1, g2);
    //std::cout << "Node count: " << ret.getNodeCount() << " Edge count: " << ret.getEdgeCount() << std::endl;
    //std::cout << ret << std::endl;
    // testing that intersection of 2 same graphs is the exact same starting graph
    assert(ret.getNodeCount() == g1.getNodeCount());
    assert(ret.getEdgeCount() == g1.getEdgeCount());
    assert(ret.getNodeCount() == g2.getNodeCount());
    assert(ret.getEdgeCount() == g2.getEdgeCount());
}

template<typename DataType, typename WeightType>
void test_getSubgraphFromNodes(GraphClasses::Graph<DataType, WeightType> &g, std::unordered_set<DataType>& nodes, unsigned expectedNumOfNodes, unsigned expectedNumOfEdges) {
    GraphClasses::Graph<DataType, WeightType> ret = GraphUtility::getSubgraphFromNodes(g, nodes);
    // std::cout << "Node count: " << ret.getNodeCount() << " Edge count: " << ret.getEdgeCount() << std::endl;
    // std::cout << ret << std::endl;
    // testing that intersection of 2 same graphs is the exact same starting graph
    assert(ret.getNodeCount() == expectedNumOfNodes);
    assert(ret.getEdgeCount() == expectedNumOfEdges);
}

void test_internal_operators() {
    std::cout << "=============== test_internal_operators ===============" << std::endl;

    // for integral types (not much testing is needed as these just call the built in operators for all non floating types)
    std::cout << "\tTesting for integral types     ";

    assert(internal::equals(1u, 1u) == (1u == 1u));
    assert(internal::equals(-1, -1) == (-1 == -1));
    assert(!internal::equals(1u, 2u) == (1u != 2u));
    assert(!internal::equals(-1, -2) == (-1 != -2));

    assert(internal::lessThan(1u, 2u) == (1u < 2u)); 
    assert(internal::lessThan(1, 2) == (1 < 2)); 
    assert(internal::lessThan(-1, -2) == (-1 < -2)); 
    assert((internal::lessThan(1, 2) || internal::equals(1, 2)) == (1 <= 2)); 

    assert(internal::greaterThan(1u, 2u) == (1u > 2u)); 
    assert(internal::greaterThan(1, 2) == (1 > 2)); 
    assert(internal::greaterThan(-1, -2) == (-1 > -2)); 
    assert((internal::greaterThan(1, 2) || internal::equals(1, 2)) == (1 >= 2)); 

    std::cout << "SUCCESS" << std::endl;
    
    // for flaoting point types (testing for float and double)
    std::cout << "\t\tEpsilon for float set to: " << internal::FLOATING_POINT_EPSIOLON<float> << std::endl;
    std::cout << "\t\tEpsilon for double set to: " << internal::FLOATING_POINT_EPSIOLON<double> << std::endl;
    std::cout << "\tTesting for floating point types     ";

    assert(internal::equals(0.f, 0.f) == (0.f == 0.f));
    assert(internal::equals(0., 0.) == (0. == 0.));
    assert(internal::equals(0.000000123f, 0.000000234f)); 
    assert(internal::equals(0.000000123, 0.000000234)); // should be equal
    assert(internal::equals(0.000002, 0.0000019999999) && !(0.000002 == 0.0000019999999));  // different from the standard ==

    assert(!internal::lessThan(0.000000123, 0.000000234)); // false because they are about an epsilon apart
    assert(!internal::lessThan(0.000000123f, 0.000000234f)); 
    assert(internal::lessThan(0.000000222, 0.00000333)); 
    assert(internal::lessThan(0.000000222f, 0.00000333f)); 
    assert(internal::lessThan(0.000000222, 0.00000333) || internal::equals(0.000000222, 0.00000333) == (0.000000222 <= 0.00000333)); 
    assert(internal::lessThan(0.000000222f, 0.00000333f) || internal::equals(0.000000222f, 0.00000333f) == (0.000000222f <= 0.00000333f)); 

    assert(!internal::greaterThan(0.0000008251234, 0.0000008)); // false because they are about an epsilon apart
    assert(!internal::greaterThan(0.0000008251234f, 0.0000008f));
    assert(internal::greaterThan(0.0008251234, 0.000799999));
    assert(internal::greaterThan(0.0008251234f, 0.000799999f));
    assert(internal::greaterThan(0.0008251234, 0.000799999) || internal::equals(0.0008251234, 0.000799999) == (0.0008251234 >= 0.000799999));
    assert(internal::greaterThan(0.0008251234f, 0.000799999f) || internal::equals(0.0008251234f, 0.000799999f) == (0.0008251234f >= 0.000799999f));

    std::cout << "SUCCESS" << std::endl;
    std::cout << "=======================================================" << std::endl << std::endl;
}

void test_graph_class_member_functions() {
    std::cout << "=============== test_graph_class_member_functions ===============" << std::endl;
    
    GraphClasses::Graph<unsigned, unsigned> g1;
    g1.configureDirections(GraphClasses::GraphType::Undirected);
    g1.configureWeights(GraphClasses::GraphWeights::Weighted);
    const char* fileName1 = "testInputs/int_int.txt";
    g1.readFromTxt(fileName1);
    // std::cout << "Node count: " << g1.getNodeCount() << " Edge count: " << g1.getEdgeCount() << " Density: " << g1.getDensity() << std::endl;
    // std::cout << g1 << std::endl;

    std::cout << "\tTesting member functions     ";

    assert(g1.isConfigured());
    assert(g1.getGraphType() == GraphClasses::GraphType::Undirected);
    assert(g1.getGraphWeights() == GraphClasses::GraphWeights::Weighted);

    assert(g1.getNodeCount() == 8);
    assert(g1.getEdgeCount() == 26);
    assert(internal::equals(g1.getDensity(), 0.928571));

    assert(g1.getNodeSet().size() == g1.getNodeCount());

    assert(g1.getDegreeOfNode(3) == 4);
    auto degrees = g1.getDegreesOfNodes();
    assert(degrees[5] == 4 && degrees[7] == 2);

    g1.addNode(10);
    assert(g1.getNodeCount() == 9);
    assert(g1.getEdgeCount() == 26);

    g1.addEdge(10, 6, 2);
    g1.addEdge(10, 2, 15);
    assert(g1.getNodeCount() == 9);
    assert(g1.getEdgeCount() == 30);

    g1.deleteEdge(1, 2);
    assert(g1.getNodeCount() == 9);
    assert(g1.getEdgeCount() == 28);

    g1.deleteNode(3);
    assert(g1.getNodeCount() == 8);
    assert(g1.getEdgeCount() == 20);
    
    g1.addEdge(5, GraphClasses::Edge<unsigned, unsigned>(11, 15));
    assert(g1.getNodeCount() == 9);
    assert(g1.getEdgeCount() == 21);

    g1.addEdge(1, 44, 2);
    assert(g1.getNodeCount() == 10);
    assert(g1.getEdgeCount() == 23);

    g1.clearGraph();
    assert(g1.getNodeCount() == 0);
    assert(g1.getEdgeCount() == 0);

    // test member functions specific to directed graphs
    GraphClasses::Graph<CustomClass, float> g2;
    g2.configureDirections(GraphClasses::GraphType::Directed);
    g2.configureWeights(GraphClasses::GraphWeights::Weighted);
    const char* fileName2 = "testInputs/custom_float.txt";
    g2.readFromTxt(fileName2);
    // std::cout << "Node count: " << g2.getNodeCount() << " Edge count: " << g2.getEdgeCount() << " Density: " << g2.getDensity() << std::endl;
    // std::cout << g2 << std::endl;
    
    assert(g2.getInDegreeOfNode(CustomClass(5, 2, 6)) == 1);
    auto inDegrees = g2.getInDegreesOfNodes();
    assert(inDegrees[CustomClass(1, 7, 3)] == 1 && inDegrees[CustomClass(1, 2, 3)] == 1);
    
    assert(g2.getOutDegreeOfNode(CustomClass(5, 2, 6)) == 2);
    auto outDegrees = g2.getOutDegreesOfNodes();
    assert(outDegrees[CustomClass(2, 2, 2)] == 0 && outDegrees[CustomClass(1, 2, 3)] == 1);

    std::cout << "SUCCESS" << std::endl;
    std::cout << "=================================================================" << std::endl << std::endl;
}

void test_string_double_undirected_weighted() {
    std::cout << "=============== test_string_double_undirected_weighted ===============" << std::endl;

    GraphClasses::Graph<std::string, double> g1;
    g1.configureDirections(GraphClasses::GraphType::Undirected);
    g1.configureWeights(GraphClasses::GraphWeights::Weighted);
    const char* fileName1 = "testInputs/string_double.txt";
    g1.readFromTxt(fileName1);
    // std::cout << "Node count: " << g1.getNodeCount() << " Edge count: " << g1.getEdgeCount() << " Density: " << g1.getDensity() << std::endl;
    // std::cout << g1 << std::endl;

    // ---- alg testing ----
    std::cout << "\tTesting algorithms     ";

    std::string startNode = "node1";
    test_dfs(g1, startNode, g1.getNodeCount());
    test_bfs(g1, startNode, g1.getNodeCount());
    std::string endNode = "node6";
    test_dijkstra(g1, startNode, endNode, 4);
    test_bellmanFord(g1, startNode, endNode, 4);
    test_floydWarshall(g1, startNode, endNode, 296.65);
    test_findArticulationPoints_without_start(g1, 2);
    test_findArticulationPoints_with_start(g1, startNode, 2);    //should be same as without start for undirected
    test_findBridges_without_start(g1, 2);
    test_findBridges_with_start(g1, startNode, 2);
    // topsort makes no sense for undirected graphs and is not tested here
    test_mcstPrimTotalCostOnly(g1, 6199.467744);
    test_mcstPrim(g1, g1.getNodeCount() - 1);
    test_mcstKruskal(g1, g1.getNodeCount() - 1);
    test_findStronglyConnectedComponentsTarjan_without_start(g1, 1);
    test_findStronglyConnectedComponentsTarjan_with_start(g1, startNode, 1); //should be same as without start for undirected
    test_findWeaklyConnectedComponents(g1, 1);

    std::cout << "SUCCESS" << std::endl;

    // ---- utility testing ----
    std::cout << "\tTesting utility functions     ";

    GraphClasses::Graph<std::string, double> g2;
    g2.configureDirections(GraphClasses::GraphType::Undirected);
    g2.configureWeights(GraphClasses::GraphWeights::Weighted);
    const char* fileName2 = "testInputs/string_double.txt";
    g2.readFromTxt(fileName2);
    // std::cout << "Node count: " << g2.getNodeCount() << " Edge count: " << g2.getEdgeCount() << " Density: " << g2.getDensity() << std::endl;
    // std::cout << g2 << std::endl;

    test_mergeGraphs(g1, g2);
    test_intersectGraphs(g1, g2);
    std::unordered_set<std::string> someNodes{"node1", "node2", "node5", "node7"};
    test_getSubgraphFromNodes(g1, someNodes, 4, 6);

    std::cout << "SUCCESS" << std::endl;
    std::cout << "======================================================================" << std::endl << std::endl;
}

void test_int_int_undirected_unweighted() {
    std::cout << "=============== test_int_int_undirected_unweighted ===============" << std::endl;

    GraphClasses::Graph<int, int> g1;
    g1.configureDirections(GraphClasses::GraphType::Undirected);
    g1.configureWeights(GraphClasses::GraphWeights::Unweighted);
    const char* fileName1 = "testInputs/int_int_u_u.txt";
    g1.readFromTxt(fileName1);
    // std::cout << "Node count: " << g1.getNodeCount() << " Edge count: " << g1.getEdgeCount() << " Density: " << g1.getDensity() << std::endl;
    // std::cout << g1 << std::endl;

    // ---- alg testing ----
    std::cout << "\tTesting algorithms     ";

    int startNode = 1;
    test_dfs(g1, startNode, g1.getNodeCount());
    test_bfs(g1, startNode, g1.getNodeCount());
    int endNode = 5;
    test_dijkstra(g1, startNode, endNode, 2);
    test_bellmanFord(g1, startNode, 6, 2);
    test_floydWarshall(g1, 4, 5, 2);
    test_findArticulationPoints_without_start(g1, 0);
    test_findArticulationPoints_with_start(g1, startNode, 0);
    test_findBridges_without_start(g1, 0);
    test_findBridges_with_start(g1, 6, 0);
    // topsort makes no sense for undirected graphs and is not tested here
    test_mcstPrim(g1, 7);
    test_mcstPrimTotalCostOnly(g1, 7);
    test_mcstKruskal(g1, 7);
    test_findStronglyConnectedComponentsTarjan_without_start(g1, 1);
    test_findStronglyConnectedComponentsTarjan_with_start(g1, startNode, 1);
    test_findWeaklyConnectedComponents(g1, 1);

    std::cout << "SUCCESS" << std::endl;

    // ---- utility testing ----
    std::cout << "\tTesting utility functions     ";

    GraphClasses::Graph<int, int> g2;
    g2.configureDirections(GraphClasses::GraphType::Undirected);
    g2.configureWeights(GraphClasses::GraphWeights::Unweighted);
    const char* fileName2 = "testInputs/int_int_u_u.txt";
    g2.readFromTxt(fileName2);

    test_mergeGraphs(g1, g2);
    test_intersectGraphs(g1, g2);
    std::unordered_set<int> someNodes{2, 5, 3, 7};
    test_getSubgraphFromNodes(g1, someNodes, 4, 4);
    
    std::cout << "SUCCESS" << std::endl;
    std::cout << "==================================================================" << std::endl << std::endl;
}

void test_custom_float_directed_weighted() {
    std::cout << "=============== test_custom_float_directed_weighted ===============" << std::endl;

    GraphClasses::Graph<CustomClass, float> g1;
    g1.configureDirections(GraphClasses::GraphType::Directed);
    g1.configureWeights(GraphClasses::GraphWeights::Weighted);
    const char* fileName1 = "testInputs/custom_float.txt";
    g1.readFromTxt(fileName1);
    // std::cout << "Node count: " << g1.getNodeCount() << " Edge count: " << g1.getEdgeCount() << " Density: " << g1.getDensity() << std::endl;
    // std::cout << g1 << std::endl;

    // ---- alg testing ----
    std::cout << "\tTesting algorithms     ";

    CustomClass startNode = CustomClass(1, 2, 3);
    test_dfs(g1, startNode, g1.getNodeCount());
    test_bfs(g1, startNode, g1.getNodeCount());
    CustomClass endNode = CustomClass(2, 2, 2);
    test_dijkstra(g1, startNode, endNode, 3);
    test_bellmanFord(g1, startNode, endNode, 3);
    test_floydWarshall(g1, startNode, endNode, 13.7f);
    // articulation points without start not supported for directed graphs
    test_findArticulationPoints_with_start(g1, startNode, 2);
    // bridges without start not supported for directed graphs
    test_findBridges_with_start(g1, startNode, 2);
        // removing edge before topsort testing because this example graph is not acyclic
        g1.deleteEdge(CustomClass(5,2,6), startNode);
    test_topsortKhan(g1, startNode, endNode);
        g1.addEdge(CustomClass(5,2,6), startNode, 124.5f);
    // MCST algorithms not supported for directed graphs
    // tarjan alg without start not supported for directed graphs
    test_findStronglyConnectedComponentsTarjan_with_start(g1, startNode, 3);
    // weakly connected components currently not supported for directed graphs

    std::cout << "SUCCESS" << std::endl;

    // ---- utility testing ----
    std::cout << "\tTesting utility functions     ";

    GraphClasses::Graph<CustomClass, float> g2;
    g2.configureDirections(GraphClasses::GraphType::Directed);
    g2.configureWeights(GraphClasses::GraphWeights::Weighted);
    const char* fileName2 = "testInputs/custom_float.txt";
    g2.readFromTxt(fileName2);

    test_mergeGraphs(g1, g2);
    test_intersectGraphs(g1, g2);
    std::unordered_set<CustomClass> someNodes{startNode, CustomClass(4, 5, 6), endNode};
    test_getSubgraphFromNodes(g1, someNodes, 3, 1);

    std::cout << "SUCCESS" << std::endl;
    std::cout << "===================================================================" << std::endl << std::endl;
}

void test_char_ull_directed_unweighted() {
    std::cout << "=============== test_char_ull_directed_unweighted ===============" << std::endl;

    GraphClasses::Graph<char, unsigned long long> g1;
    g1.configureDirections(GraphClasses::GraphType::Directed);
    g1.configureWeights(GraphClasses::GraphWeights::Unweighted);
    const char* fileName1 = "testInputs/char_ull_d_u.txt";
    g1.readFromTxt(fileName1);
    // std::cout << "Node count: " << g1.getNodeCount() << " Edge count: " << g1.getEdgeCount() << " Density: " << g1.getDensity() << std::endl;
    // std::cout << g1 << std::endl;

    // ---- alg testing ----
    std::cout << "\tTesting algorithms     ";
    char startNode = 'a';
    test_dfs(g1, startNode, g1.getNodeCount());
    test_bfs(g1, startNode, g1.getNodeCount());
    char endNode = 'l';
    test_dijkstra(g1, startNode, endNode, 4);
    test_bellmanFord(g1, startNode, 'o', 5);
    test_floydWarshall(g1, 'h', 'j', static_cast<unsigned long long>(4));
    // articulation points without start not supported for directed graphs
    test_findArticulationPoints_with_start(g1, startNode, 3);
    // bridges without start not supported for directed graphs
    test_findBridges_with_start(g1, startNode, 3);
    // removing some edges before topsort testing because this example graph is not acyclic
        g1.deleteEdge('p', 'a');
        g1.deleteEdge('o', 'p');
        g1.deleteEdge('h', 'b');
        g1.deleteEdge('m', 'h');
        g1.deleteEdge('j', 'i');
        g1.deleteEdge('l', 'h');
    test_topsortKhan(g1, 'a', 'o');
        g1.addEdge('p', 'a');
        g1.addEdge('o', 'p');
        g1.addEdge('h', 'b');
        g1.addEdge('m', 'h');
        g1.addEdge('j', 'i');
        g1.addEdge('l', 'h');
    // MCST algorithms not supported for directed graphs
    // tarjan alg without start not supported for directed graphs
    test_findStronglyConnectedComponentsTarjan_with_start(g1, startNode, 3);
    // weakly connected components currently not supported for directed graphs

    std::cout << "SUCCESS" << std::endl;

    // ---- utility testing ----
    std::cout << "\tTesting utility functions     ";

    GraphClasses::Graph<char, unsigned long long> g2;
    g2.configureDirections(GraphClasses::GraphType::Directed);
    g2.configureWeights(GraphClasses::GraphWeights::Unweighted);
    const char* fileName2 = "testInputs/char_ull_d_u.txt";
    g2.readFromTxt(fileName2);

    test_mergeGraphs(g1, g2);
    test_intersectGraphs(g1, g2);
    std::unordered_set<char> someNodes{'a', 'c', 'd', 'e', 'i', 'j'};
    test_getSubgraphFromNodes(g1, someNodes, 6, 6);

    std::cout << "SUCCESS" << std::endl;
    std::cout << "=================================================================" << std::endl << std::endl;
}

int main() {
    test_internal_operators();

    test_graph_class_member_functions();

    test_string_double_undirected_weighted();

    test_int_int_undirected_unweighted();

    test_custom_float_directed_weighted();

    test_char_ull_directed_unweighted();

    return 0;
}