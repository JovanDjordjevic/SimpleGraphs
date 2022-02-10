#include <iostream>
#include <string>
#include "CustomClass/customClass.hpp"
#include "SimpleGraphs.hpp"
#include <cassert>
#include <iomanip>

auto EPS = std::numeric_limits<double>::epsilon();

// NOTE: currently tests written only for <string, double> graphs that are undirected and weighed. Other tests will be added in the future
// prior to this, all functions were tested arbitrarily and seem to be working

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

    assert(ret.size() == edgesOnPath);
}

template<typename DataType, typename WeightType>
void test_bellmanFord(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, unsigned result) {
    auto ret = GraphAlgorithms::bellmanFord(g, startNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    
    assert(ret.size() == result);
    //assert(ret["node6"].size() == 5);
}

template<typename DataType, typename WeightType>
void test_floydWarshall(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, DataType endNode, WeightType distance) {
    auto ret = GraphAlgorithms::floydWarshall(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    
    // NOTE: not pretty, only testing up to 5 decimal places precission
    WeightType retDist = ret[startNode][endNode];
    // std::cout << retDist << " " << distance << " " << EPS << std::endl;
    retDist = static_cast<double>(static_cast<int>(retDist * 100000)) / 100000;
    distance  = static_cast<double>(static_cast<int>(distance * 100000)) / 100000;
    // std::cout << retDist << " " << distance << " " << EPS << std::endl;

    assert(std::abs(retDist - distance) < EPS);
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
    
    // NOTE: not pretty, only testing up to 5 decimal places precission
    //std::cout << ret << " " << totalCost << " " << EPS << std::endl;
    ret  = static_cast<double>(static_cast<int>(ret * 100000)) / 100000;
    totalCost  = static_cast<double>(static_cast<int>(totalCost * 100000)) / 100000;
    //std::cout << ret << " " << totalCost << " " << EPS << std::endl;

    assert(std::abs(ret - totalCost) < EPS);
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
void test_getSubgraphFromNodes(GraphClasses::Graph<DataType, WeightType> &g, std::unordered_set<DataType>& nodes, 
                               unsigned expectedNumOfNodes, unsigned expectedNumOfEdges ) {
    GraphClasses::Graph<DataType, WeightType> ret = GraphUtility::getSubgraphFromNodes(g, nodes);
    // std::cout << "Node count: " << ret.getNodeCount() << " Edge count: " << ret.getEdgeCount() << std::endl;
    // std::cout << ret << std::endl;

    // testing that intersection of 2 same graphs is the exact same starting graph
    assert(ret.getNodeCount() == expectedNumOfNodes);
    assert(ret.getEdgeCount() == expectedNumOfEdges);
}

void string_double_graphs_tests() {
    //std::cout << std::setprecision(10);

    // ---- alg testing ----
    GraphClasses::Graph<std::string, double> g1;
    g1.configureDirections(GraphClasses::GraphType::Undirected);
    g1.configureWeights(GraphClasses::GraphWeights::Weighted);

    const char* fileName1 = "../testInputs/string_double.txt";
    g1.readFromTxt(fileName1);

    std::cout << "Node count: " << g1.getNodeCount() << " Edge count: " << g1.getEdgeCount() << std::endl;
    std::cout << g1 << std::endl;

    std::string startNode = "node1";
    test_dfs(g1, startNode, g1.getNodeCount());
    test_bfs(g1, startNode, g1.getNodeCount());

    std::string endNode = "node6";
    test_dijkstra(g1, startNode, endNode, 5);
    test_bellmanFord(g1, startNode, g1.getNodeCount());
    test_floydWarshall(g1, startNode, endNode, 296.65);
    test_findArticulationPoints_without_start(g1, 2);
    test_findArticulationPoints_with_start(g1, startNode, 2);    //should be same as without start for undirected
    test_findBridges_without_start(g1, 2);
    test_findBridges_with_start(g1, startNode, 2);
        // topsort makes no sense for undirected graphs
        // endNode = "node9";
        // g.deleteEdge("node2", "node1");
        // g.deleteEdge("node4", "node2");
        // test_topsortKhan(g1, startNode, endNode);
    test_mcstPrimTotalCostOnly(g1, 6199.467744);
    test_mcstPrim(g1, g1.getNodeCount() - 1);
    test_mcstKruskal(g1, g1.getNodeCount() - 1);
    test_findStronglyConnectedComponentsTarjan_without_start(g1, 1);
    test_findStronglyConnectedComponentsTarjan_with_start(g1, startNode, 1); //should be same as without start for undirected
    test_findWeaklyConnectedComponents(g1, 1);

    // ---- utility testing ----

    GraphClasses::Graph<std::string, double> g2;
    g2.configureDirections(GraphClasses::GraphType::Undirected);
    g2.configureWeights(GraphClasses::GraphWeights::Weighted);

    const char* fileName2 = "../testInputs/string_double.txt";
    g2.readFromTxt(fileName2);

    std::cout << "Node count: " << g2.getNodeCount() << " Edge count: " << g2.getEdgeCount() << std::endl;
    std::cout << g2 << std::endl;
    test_mergeGraphs(g1, g2);
    test_intersectGraphs(g1, g2);
    std::unordered_set<std::string> someNodes{"node1", "node2", "node5", "node7"};
    test_getSubgraphFromNodes(g1, someNodes, 4, 6);
}

int main() {

    string_double_graphs_tests();

    return 0;
}