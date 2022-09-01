#include <iostream>
#include <string>
#include "CustomClass/customClass.hpp"
#include "SimpleGraphs.hpp"
#include <cassert>
#include <iomanip>
#include <cmath>

// apart from these tests, all functions were tested arbitrarily and seem to be working
// this DOES NOT guarantee correctness
// At this time usage of pointer and reference types as NodeType is not supported

// NOTE: even though the library supports float as WeightType, usage of double is recommended becasue of calculation precission

template<typename NodeType, typename WeightType>
void test_depthFirstTraverse(GraphClasses::Graph<NodeType, WeightType> &g, NodeType startNode, unsigned dfsTreeSize) {
    auto ret = GraphAlgorithms::depthFirstTraverse(g, startNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == dfsTreeSize);
}

template<typename NodeType, typename WeightType>
void test_depthFirstSearch(GraphClasses::Graph<NodeType, WeightType> &g, NodeType startNode, NodeType nodeToFind, bool shouldFind) {
    auto [ifFound, traverseOrder] = GraphAlgorithms::depthFirstSearch(g, startNode, nodeToFind, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ifFound == shouldFind);
}

template<typename NodeType, typename WeightType>
void test_breadthFirstTraverse(GraphClasses::Graph<NodeType, WeightType> &g, NodeType startNode, unsigned bfsTreeSize) {
    auto ret = GraphAlgorithms::breadthFirstTraverse(g, startNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == bfsTreeSize);
}

template<typename NodeType, typename WeightType>
void test_breadthFirstSearch(GraphClasses::Graph<NodeType, WeightType> &g, NodeType startNode, NodeType nodeToFind, bool shouldFind) {
    auto [ifFound, traverseOrder] = GraphAlgorithms::breadthFirstSearch(g, startNode, nodeToFind, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ifFound == shouldFind);
}

template<typename NodeType, typename WeightType>
void test_dijkstraShortestPath(GraphClasses::Graph<NodeType, WeightType> &g, NodeType startNode, NodeType endNode, unsigned edgesOnPath, WeightType pathDistance) {
    auto [path, dist] = GraphAlgorithms::dijkstraShortestPath(g, startNode, endNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert((path.size() - 1) == edgesOnPath);
    assert(internal::equals(pathDistance, dist));
}

template<typename NodeType, typename WeightType>
void test_dijkstraAllShortestPathsFromStart(GraphClasses::Graph<NodeType, WeightType> &g, NodeType startNode, NodeType someEndNode, unsigned edgesOnPathToEndNode, WeightType pathDistance) {
    auto ret = GraphAlgorithms::dijkstraAllShortestPathsFromStart(g, startNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert((ret[someEndNode].first.size() - 1) == edgesOnPathToEndNode);
    assert(internal::equals(ret[someEndNode].second, pathDistance));
}

template<typename NodeType, typename WeightType>
void test_bellmanFordShortestPaths(GraphClasses::Graph<NodeType, WeightType> &g, NodeType startNode, NodeType someEndNode, unsigned edgesOnPathToEndNode, WeightType pathDistance) {
    auto ret = GraphAlgorithms::bellmanFordShortestPaths(g, startNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert((ret[someEndNode].first.size() - 1) == edgesOnPathToEndNode);
    assert(internal::equals(ret[someEndNode].second, pathDistance));
}

template<typename NodeType, typename WeightType>
void test_shortestPathFasterAlgorithm(GraphClasses::Graph<NodeType, WeightType> &g, NodeType startNode, NodeType someEndNode, unsigned edgesOnPathToEndNode, WeightType pathDistance) {
    auto ret = GraphAlgorithms::shortestPathFasterAlgorithm(g, startNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert((ret[someEndNode].first.size() - 1) == edgesOnPathToEndNode);
    assert(internal::equals(ret[someEndNode].second, pathDistance));
}

template<typename NodeType, typename WeightType>
void test_floydWarshallAllShortestPaths(GraphClasses::Graph<NodeType, WeightType> &g, NodeType someStartNode, NodeType someEndNode, WeightType distance) {
    auto ret = GraphAlgorithms::floydWarshallAllShortestPaths(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(internal::equals(ret[someStartNode][someEndNode], distance));
}

template<typename NodeType, typename WeightType>
void test_johnsonAllShortestsPaths(GraphClasses::Graph<NodeType, WeightType> &g, NodeType someStartNode, NodeType someEndNode,  unsigned edgesOnPathToEndNode, WeightType pathDistance) {
    auto ret = GraphAlgorithms::johnsonAllShortestsPaths(g, {}, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert((ret[someStartNode][someEndNode].first.size() - 1) == edgesOnPathToEndNode);
    assert(internal::equals(ret[someStartNode][someEndNode].second, pathDistance));
}

template<typename NodeType, typename WeightType>
void test_johnson_bellmanford_equivalence(GraphClasses::Graph<NodeType, WeightType> &g) {
    // std::cout << std::endl;
    int wrongDistanceCounter = 0;
    int wrongPathCounter = 0;
    auto johnsonReturn = GraphAlgorithms::johnsonAllShortestsPaths(g, {}, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    for (auto& [startNode, endNodeAndPathData] : johnsonReturn) {

        auto bellmanReturn = GraphAlgorithms::bellmanFordShortestPaths(g, startNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);

        for (auto& [endNode, pathVectAndWeight] : endNodeAndPathData) {
            auto& pathVect = pathVectAndWeight.first;
            auto& pathWeight = pathVectAndWeight.second;

            if (!internal::equals(pathWeight, bellmanReturn[endNode].second)) {
                // std::cout << std::setprecision(10);
                // std::cout << "[" << startNode << "] to [" << endNode << "] : JOHNSON : " << std::setw(15) << weight  
                //                                                     << " \tBELLMAN : " << std::setw(15) << bellmanReturn[endNode].second << std::endl
                ++wrongDistanceCounter;

                assert(bellmanReturn[endNode].first.size() == pathVect.size());

                if (bellmanReturn[endNode].first.size() !=0 && pathVect.size() != 0) {
                    for (size_t i = 0; i < pathVect.size(); ++i) {
                        if (!internal::equals((bellmanReturn[endNode].first)[i], pathVect[i])) {
                            // std::cout << "Paths from " << "[" << startNode << "] to [" << endNode << "] do not match" << std::endl;
                            ++wrongPathCounter;
                        }
                    }
                }    
            }        
        }   
    }
    // std::cout << "WRONG DISTANCE COUNT: " << wrongDistanceCounter << std::endl;
    // std::cout << "WRONG PATH COUNT: " << wrongPathCounter << std::endl;
    assert(wrongDistanceCounter == 0);
    assert(wrongPathCounter == 0);
}

template<typename NodeType, typename WeightType>
void test_all_path_algs_equivalence(GraphClasses::Graph<NodeType, WeightType> &g) {
    // std::cout << std::endl;
    int wrongDistanceCounter = 0;
    int wrongPathCounter = 0;

    auto floydReturn = GraphAlgorithms::floydWarshallAllShortestPaths(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);

    for (auto& [startNode, pathData] : floydReturn) {
        auto bellmanReturn = GraphAlgorithms::bellmanFordShortestPaths(g, startNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
        auto spfaReturn = GraphAlgorithms::shortestPathFasterAlgorithm(g, startNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
        auto dijkstraAllPathsReturn = GraphAlgorithms::dijkstraAllShortestPathsFromStart(g, startNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);

        for (auto& [endNode, weight] : pathData) {
            auto dijkstraReturn = GraphAlgorithms::dijkstraShortestPath(g, startNode, endNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);

            if (!internal::equals(weight, bellmanReturn[endNode].second) 
                || !internal::equals(bellmanReturn[endNode].second, dijkstraAllPathsReturn[endNode].second)
                || !internal::equals(dijkstraAllPathsReturn[endNode].second, dijkstraReturn.second) 
                || !internal::equals(dijkstraReturn.second, spfaReturn[endNode].second)
                || !internal::equals(spfaReturn[endNode].second, weight)
                ) {
                // std::cout << std::setprecision(10);
                // std::cout << "[" << startNode << "] to [" << endNode << "] : FLOYD : " << std::setw(15) << weight  
                //                                                     << " \tBELLMAN : " << std::setw(15) << bellmanReturn[endNode].second << std::endl
                //                                                     << " \tDIJSKTRA_ALL : " << std::setw(15) << dijkstraAllPathsReturn[endNode].second << std::endl
                //                                                     << " \tDIJKSTRA : " << std::setw(15) << dijkstraReturn.second << std::endl
                //                                                     << " \tSPFA : " << std::setw(15) << spfaReturn[endNode].second << std::endl;
                ++wrongDistanceCounter;

                assert(dijkstraReturn.first.size() == bellmanReturn[endNode].first.size() 
                        && bellmanReturn[endNode].first.size() == dijkstraAllPathsReturn[endNode].first.size()
                        && dijkstraAllPathsReturn[endNode].first.size() == spfaReturn[endNode].first.size());

                if (dijkstraReturn.first.size() !=0 
                        && bellmanReturn[endNode].first.size() !=0 
                        && dijkstraAllPathsReturn[endNode].first.size() != 0
                        && spfaReturn[endNode].first.size() != 0
                        ) {
                    for (size_t i = 0; i < (dijkstraReturn.first).size(); ++i) {
                        if (!internal::equals((bellmanReturn[endNode].first)[i], (dijkstraReturn.first)[i])
                            || !internal::equals((dijkstraReturn.first)[i], (dijkstraAllPathsReturn[endNode].first)[i])
                            || !internal::equals((dijkstraAllPathsReturn[endNode].first)[i], (spfaReturn[endNode].first)[i])
                            ) {
                            // std::cout << "Paths from " << "[" << startNode << "] to [" << endNode << "] do not match" << std::endl;
                            ++wrongPathCounter;
                        }
                    }
                }    
            }        
        }   
    }
    // std::cout << "WRONG DISTANCE COUNT: " << wrongDistanceCounter << std::endl;
    // std::cout << "WRONG PATH COUNT: " << wrongPathCounter << std::endl;
    assert(wrongDistanceCounter == 0);
    assert(wrongPathCounter == 0);
}

template<typename NodeType, typename WeightType>
void test_findArticulationPoints(GraphClasses::Graph<NodeType, WeightType> &g, unsigned numOfArticulationPoints) {
    auto ret = GraphAlgorithms::findArticulationPoints(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == numOfArticulationPoints);
}

template<typename NodeType, typename WeightType>
void test_findBridges(GraphClasses::Graph<NodeType, WeightType> &g, unsigned numOfBridges) {
    auto ret = GraphAlgorithms::findBridges(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == numOfBridges);
}

template<typename NodeType, typename WeightType>
void test_topsortKhan(GraphClasses::Graph<NodeType, WeightType> &g, NodeType firstNode, NodeType lastNode) {
    auto ret = GraphAlgorithms::topsortKhan(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret[0] == firstNode);
    assert(ret[g.getNodeCount() - 1] == lastNode);
}

template<typename NodeType, typename WeightType>
void test_mcstPrimTotalCostOnly(GraphClasses::Graph<NodeType, WeightType> &g, WeightType totalCost) {
    auto ret = GraphAlgorithms::mcstPrimTotalCostOnly(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(internal::equals(ret, totalCost));
}

template<typename NodeType, typename WeightType>
void test_mcstPrim(GraphClasses::Graph<NodeType, WeightType> &g, unsigned edgeCount) {
    auto ret = GraphAlgorithms::mcstPrim(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == edgeCount);
}

template<typename NodeType, typename WeightType>
void test_mcstKruskal(GraphClasses::Graph<NodeType, WeightType> &g, unsigned edgeCount) {
    auto ret = GraphAlgorithms::mcstKruskal(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == edgeCount);
}

template<typename NodeType, typename WeightType>
void test_findStronglyConnectedComponentsTarjan(GraphClasses::Graph<NodeType, WeightType> &g, unsigned numOfComponents) {
    auto ret = GraphAlgorithms::findStronglyConnectedComponentsTarjan(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == numOfComponents);
}

template<typename NodeType, typename WeightType>
void test_findWeaklyConnectedComponents(GraphClasses::Graph<NodeType, WeightType> &g, unsigned numOfComponents) {
    auto ret = GraphAlgorithms::findWeaklyConnectedComponents(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == numOfComponents);
}

template<typename NodeType, typename WeightType>
void test_edmondsKarpMaximumFlow(GraphClasses::Graph<NodeType, WeightType> &g, NodeType source, NodeType sink, WeightType expectedMaxFlow) {
    auto [maxFlow, residualGraph] = GraphAlgorithms::edmondsKarpMaximumFlow(g, source, sink, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(internal::equals(maxFlow, expectedMaxFlow));
}

template<typename NodeType, typename WeightType>
void test_findIsolatedNodes(GraphClasses::Graph<NodeType, WeightType> &g, unsigned numOfIsolatedNodes) {
    auto ret = GraphAlgorithms::findIsolatedNodes(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == numOfIsolatedNodes);
}

template<typename NodeType, typename WeightType>
void test_johnsonAllCycles(GraphClasses::Graph<NodeType, WeightType> &g, unsigned numOfCycles) {
    auto ret = GraphAlgorithms::johnsonAllCycles(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == numOfCycles);
}

template<typename NodeType, typename WeightType>
void test_findAllCycles(GraphClasses::Graph<NodeType, WeightType> &g, unsigned numOfCycles) {
    auto ret = GraphAlgorithms::findAllCycles(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == numOfCycles);
}

template<typename NodeType, typename WeightType>
void test_mergeGraphs(GraphClasses::Graph<NodeType, WeightType> &g1, GraphClasses::Graph<NodeType, WeightType> &g2) {
    assert(g1.getGraphType() == g2.getGraphType());
    assert(g1.getGraphWeights() == g2.getGraphWeights());
    GraphClasses::Graph<NodeType, WeightType> ret = GraphUtility::mergeGraphs(g1, g2);
    // std::cout << "Node count: " << ret.getNodeCount() << " Edge count: " << ret.getEdgeCount() << std::endl;
    // std::cout << ret << std::endl;
    // testing that 2 same graphs do not produce extra nodes or edges after merging
    assert(ret.getNodeCount() == g1.getNodeCount());
    assert(ret.getEdgeCount() == g1.getEdgeCount());
    assert(ret.getNodeCount() == g2.getNodeCount());
    assert(ret.getEdgeCount() == g2.getEdgeCount());
}

template<typename NodeType, typename WeightType>
void test_intersectGraphs(GraphClasses::Graph<NodeType, WeightType> &g1, GraphClasses::Graph<NodeType, WeightType> &g2) {
    assert(g1.getGraphType() == g2.getGraphType());
    assert(g1.getGraphWeights() == g2.getGraphWeights());
    GraphClasses::Graph<NodeType, WeightType> ret = GraphUtility::intersectGraphs(g1, g2);
    //std::cout << "Node count: " << ret.getNodeCount() << " Edge count: " << ret.getEdgeCount() << std::endl;
    //std::cout << ret << std::endl;
    // testing that intersection of 2 same graphs is the exact same starting graph
    assert(ret.getNodeCount() == g1.getNodeCount());
    assert(ret.getEdgeCount() == g1.getEdgeCount());
    assert(ret.getNodeCount() == g2.getNodeCount());
    assert(ret.getEdgeCount() == g2.getEdgeCount());
}

template<typename NodeType, typename WeightType>
void test_getSubgraphFromNodes(GraphClasses::Graph<NodeType, WeightType> &g, std::unordered_set<NodeType>& nodes, unsigned expectedNumOfNodes, unsigned expectedNumOfEdges) {
    GraphClasses::Graph<NodeType, WeightType> ret = GraphUtility::getSubgraphFromNodes(g, nodes);
    // std::cout << "Node count: " << ret.getNodeCount() << " Edge count: " << ret.getEdgeCount() << std::endl;
    // std::cout << ret << std::endl;
    // testing that intersection of 2 same graphs is the exact same starting graph
    assert(ret.getNodeCount() == expectedNumOfNodes);
    assert(ret.getEdgeCount() == expectedNumOfEdges);
}

template<typename NodeType, typename WeightType>
void test_transposeOfGraph(GraphClasses::Graph<NodeType, WeightType> &g) {
    GraphClasses::Graph<NodeType, WeightType> ret = GraphUtility::transposeOfGraph(g);
    // std::cout << "Node count: " << ret.getNodeCount() << " Edge count: " << ret.getEdgeCount() << std::endl;
    // std::cout << ret << std::endl;
    // transpose should ahve same number of nodes and edges as the original
    assert(ret.getNodeCount() == g.getNodeCount());
    assert(ret.getEdgeCount() == g.getEdgeCount());
}

template<typename NodeType>
void test_constructCompleteGraphFromNodes_without_default_weight(std::unordered_set<NodeType>& nodes, GraphClasses::GraphType graphType) {
    GraphClasses::Graph<NodeType> ret = GraphUtility::constructCompleteGraphFromNodes(nodes, graphType);
    assert(ret.getNodeCount() == nodes.size());
    assert(ret.getEdgeCount() == (nodes.size() * (nodes.size() - 1)));
}

template<typename NodeType, typename WeightType>
void test_constructCompleteGraphFromNodes_with_default_weight(std::unordered_set<NodeType>& nodes, GraphClasses::GraphType graphType, GraphClasses::GraphWeights graphWeights, std::optional<WeightType> defaultWeight) {
    GraphClasses::Graph<NodeType, WeightType> ret = GraphUtility::constructCompleteGraphFromNodes(nodes, graphType, graphWeights, defaultWeight);
    assert(ret.getNodeCount() == nodes.size());
    assert(ret.getEdgeCount() == (nodes.size() * (nodes.size() - 1)));
}

template<typename NodeType, typename WeightType>
void test_complementOfGraph(GraphClasses::Graph<NodeType, WeightType> &g) {
    GraphClasses::Graph<NodeType, WeightType> g_compl = GraphUtility::complementOfGraph(g);
    
    auto orgNodeCount = g.getNodeCount();
    auto orgEdgeCount = g.getEdgeCount();

    assert(g_compl.getNodeCount() == orgNodeCount);
    assert(g_compl.getEdgeCount() == (orgNodeCount * (orgNodeCount - 1)) - orgEdgeCount);

    GraphClasses::Graph<NodeType, WeightType> g_compl_compl = GraphUtility::complementOfGraph(g_compl);
    assert(g_compl_compl.getNodeCount() == orgNodeCount);
    assert(g_compl_compl.getEdgeCount() == orgEdgeCount);
}

template<typename NodeType, typename WeightType>
void test_transitiveClosureOfGraph(GraphClasses::Graph<NodeType, WeightType> &g, unsigned expectedNumOfNodes, unsigned expectedNumOfEdges ) {
    GraphClasses::Graph<NodeType, WeightType> closure = GraphUtility::transitiveClosureOfGraph(g);
    assert(closure.getNodeCount() == expectedNumOfNodes);
    assert(closure.getEdgeCount() == expectedNumOfEdges);
}

template<typename NodeType, typename WeightType>
void test_transitiveReductionOfGraph(GraphClasses::Graph<NodeType, WeightType> &g) {
    GraphClasses::Graph<NodeType, WeightType> reduction = GraphUtility::transitiveReductionOfGraph(g);
    GraphClasses::Graph<NodeType, WeightType> closureOfReduction = GraphUtility::transitiveReductionOfGraph(reduction);
    GraphClasses::Graph<NodeType, WeightType> closureOfOriginal = GraphUtility::transitiveReductionOfGraph(g);
    test_mergeGraphs(closureOfOriginal, closureOfReduction);
    test_intersectGraphs(closureOfOriginal, closureOfReduction);
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
    g1.addEdge(6, 10, 2);
    g1.addEdge(10, 2, 15);
    g1.addEdge(2, 10, 15);
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
    g1.addEdge(44, 1, 2);
    assert(g1.getNodeCount() == 10);
    assert(g1.getEdgeCount() == 23);

    g1.clearGraph();
    assert(g1.getNodeCount() == 0);
    assert(g1.getEdgeCount() == 0);

    // testing that export/import does not change the graph for all graph configurations
    // testing 2 times for each configuration
    g1.configureDirections(GraphClasses::GraphType::Undirected);
    g1.configureWeights(GraphClasses::GraphWeights::Weighted);
    g1.readFromTxt(fileName1);
    auto orgNodeCount = g1.getNodeCount();
    auto orgEdgeCount = g1.getEdgeCount();
    g1.exportToTxt("test.txt");
    g1.clearGraph();
    g1.readFromTxt("test.txt");
    assert(g1.getNodeCount() == orgNodeCount && g1.getEdgeCount() == orgEdgeCount);
    g1.clearGraph();
    g1.readFromTxt("test.txt");
    assert(g1.getNodeCount() == orgNodeCount && g1.getEdgeCount() == orgEdgeCount);

    g1.configureDirections(GraphClasses::GraphType::Undirected);
    g1.configureWeights(GraphClasses::GraphWeights::Unweighted);
    g1.readFromTxt(fileName1);
    orgNodeCount = g1.getNodeCount();
    orgEdgeCount = g1.getEdgeCount();
    g1.exportToTxt("test.txt");
    g1.clearGraph();
    g1.readFromTxt("test.txt");
    assert(g1.getNodeCount() == orgNodeCount && g1.getEdgeCount() == orgEdgeCount);
    g1.clearGraph();
    g1.readFromTxt("test.txt");
    assert(g1.getNodeCount() == orgNodeCount && g1.getEdgeCount() == orgEdgeCount);

    g1.configureDirections(GraphClasses::GraphType::Directed);
    g1.configureWeights(GraphClasses::GraphWeights::Weighted);
    g1.readFromTxt(fileName1);
    orgNodeCount = g1.getNodeCount();
    orgEdgeCount = g1.getEdgeCount();
    g1.exportToTxt("test.txt");
    g1.clearGraph();
    g1.readFromTxt("test.txt");
    assert(g1.getNodeCount() == orgNodeCount && g1.getEdgeCount() == orgEdgeCount);
    g1.clearGraph();
    g1.readFromTxt("test.txt");
    assert(g1.getNodeCount() == orgNodeCount && g1.getEdgeCount() == orgEdgeCount);

    g1.configureDirections(GraphClasses::GraphType::Directed);
    g1.configureWeights(GraphClasses::GraphWeights::Unweighted);
    g1.readFromTxt(fileName1);
    orgNodeCount = g1.getNodeCount();
    orgEdgeCount = g1.getEdgeCount();
    g1.exportToTxt("test.txt");
    g1.clearGraph();
    g1.readFromTxt("test.txt");
    assert(g1.getNodeCount() == orgNodeCount && g1.getEdgeCount() == orgEdgeCount);
    g1.clearGraph();
    g1.readFromTxt("test.txt");
    assert(g1.getNodeCount() == orgNodeCount && g1.getEdgeCount() == orgEdgeCount);

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
    assert(internal::equals(g1.getEccentricityOfNode(startNode), static_cast<double>(6125.78774)));
    auto [radius, diameter, center] = g1.getRadiusDiameterAndCenter();
    assert(internal::equals(radius, static_cast<double>(5991.55124)) && internal::equals(diameter, static_cast<double>(6125.78774)) && center.size() == 1);
    auto [circumference, girth] = g1.getCircumferenceAndGirth();
    assert(internal::equals(circumference, static_cast<double>(430.886504)) && internal::equals(girth, static_cast<double>(430.886504)));

    test_depthFirstTraverse(g1, startNode, g1.getNodeCount());
    test_depthFirstSearch(g1, startNode, std::string{"node5"}, true);
    test_depthFirstSearch(g1, startNode, std::string{"node222"}, false);
    test_breadthFirstTraverse(g1, startNode, g1.getNodeCount());
    test_breadthFirstSearch(g1, startNode, std::string{"node5"}, true);
    test_breadthFirstSearch(g1, startNode, std::string{"node222"}, false);
    std::string endNode = "node6";
    test_dijkstraShortestPath(g1, startNode, endNode, 4, static_cast<double>(134.236504));
    test_dijkstraAllShortestPathsFromStart(g1, startNode, endNode, 4, static_cast<double>(134.236504));
    test_bellmanFordShortestPaths(g1, startNode, endNode, 4, static_cast<double>(134.236504));
    test_shortestPathFasterAlgorithm(g1, startNode, endNode, 4, static_cast<double>(134.236504));
    test_floydWarshallAllShortestPaths(g1, startNode, endNode, static_cast<double>(134.236504));
    test_johnsonAllShortestsPaths(g1, startNode, endNode, 4, static_cast<double>(134.236504));
    test_johnson_bellmanford_equivalence(g1);
    test_all_path_algs_equivalence(g1);
    test_findArticulationPoints(g1, 2);
    test_findBridges(g1, 2);
    // topsort makes no sense for undirected graphs and is not tested here
    test_mcstPrimTotalCostOnly(g1, static_cast<double>(6199.467744));
    test_mcstPrim(g1, g1.getNodeCount() - 1);
    test_mcstKruskal(g1, g1.getNodeCount() - 1);
    test_findStronglyConnectedComponentsTarjan(g1, 1);
    test_findWeaklyConnectedComponents(g1, 1);
    test_edmondsKarpMaximumFlow(g1, startNode, endNode, static_cast<double>(51.550004));
    test_findIsolatedNodes(g1, 0);
    // johnson algorithm for all cycles not tested for undirected graph
    test_findAllCycles(g1, 1);

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
    // transposing makes no sense for undirected graphs and is not tested here
    test_constructCompleteGraphFromNodes_with_default_weight<std::string, double>(someNodes, g1.getGraphType(), g1.getGraphWeights(), static_cast<double>(1.0));
    // complement of weighted graps is not supported and is not tested here
    auto g1NodeCount = g1.getNodeCount();
    test_transitiveClosureOfGraph(g1, g1NodeCount, (g1NodeCount * (g1NodeCount - 1) + g1NodeCount));
    test_transitiveReductionOfGraph(g1);

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
    assert(internal::equals(g1.getEccentricityOfNode(startNode), 3));
    auto [radius, diameter, center] = g1.getRadiusDiameterAndCenter();
    assert(internal::equals(radius, 2) && internal::equals(diameter, 3) && center.size() == 4);
    auto [circumference, girth] = g1.getCircumferenceAndGirth();
    assert(internal::equals(circumference, 8) && internal::equals(girth, 3));

    test_depthFirstTraverse(g1, startNode, g1.getNodeCount());
    test_depthFirstSearch(g1, startNode, 6, true);
    test_depthFirstSearch(g1, startNode, 222, false);
    test_breadthFirstTraverse(g1, startNode, g1.getNodeCount());
    test_breadthFirstSearch(g1, startNode, 6, true);
    test_breadthFirstSearch(g1, startNode, 222, false);
    int endNode = 5;
    test_dijkstraShortestPath(g1, startNode, endNode, 2, 2);
    test_dijkstraAllShortestPathsFromStart(g1, startNode, 6, 2, 2);
    test_bellmanFordShortestPaths(g1, startNode, 6, 2, 2);
    test_shortestPathFasterAlgorithm(g1, startNode, 6, 2, 2);
    test_floydWarshallAllShortestPaths(g1, 4, 5, 2);
    // johnsonAllShortest paths not supported for unweigted graphs and is not tested here
    test_all_path_algs_equivalence(g1);
    test_findArticulationPoints(g1, 0);
    test_findBridges(g1, 0);
    // topsort makes no sense for undirected graphs and is not tested here
    test_mcstPrim(g1, 7);
    test_mcstPrimTotalCostOnly(g1, 7);
    test_mcstKruskal(g1, 7);
    test_findStronglyConnectedComponentsTarjan(g1, 1);
    test_findWeaklyConnectedComponents(g1, 1);
    test_edmondsKarpMaximumFlow(g1, 1, 8, 3);
    test_findIsolatedNodes(g1, 0);
    // johnson algorithm for all cycles not tested for undirected graph
    test_findAllCycles(g1, 38);

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
    // transposing makes no sense for undirected graphs and is not tested here
    test_constructCompleteGraphFromNodes_without_default_weight(someNodes, g1.getGraphType());
    test_complementOfGraph(g1);
    auto g1NodeCount = g1.getNodeCount();
    test_transitiveClosureOfGraph(g1, g1NodeCount, (g1NodeCount * (g1NodeCount - 1) + g1NodeCount));
    test_transitiveReductionOfGraph(g1);
    
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
    assert(internal::equals(g1.getEccentricityOfNode(startNode), 13.69999981f));
    auto [radius, diameter, center] = g1.getRadiusDiameterAndCenter();
    assert(internal::equals(radius, 13.69999981f) && internal::equals(diameter, GraphClasses::MAX_WEIGHT<float>) && center.size() == 1);
    auto [circumference, girth] = g1.getCircumferenceAndGirth();
    assert(internal::equals(circumference, static_cast<float>(137.1999969)) && internal::equals(girth, static_cast<float>(137.1999969)));

    test_depthFirstTraverse(g1, startNode, g1.getNodeCount());
    test_depthFirstSearch(g1, startNode, CustomClass(1, 7, 3), true);
    test_depthFirstSearch(g1, startNode, CustomClass(9, 9, 9), false);
    test_breadthFirstTraverse(g1, startNode, g1.getNodeCount());
    test_breadthFirstSearch(g1, startNode, CustomClass(1, 7, 3), true);
    test_breadthFirstSearch(g1, startNode, CustomClass(9, 9, 9), false);
    CustomClass endNode = CustomClass(2, 2, 2);
    test_dijkstraShortestPath(g1, startNode, endNode, 3, 13.7f);
    test_dijkstraAllShortestPathsFromStart(g1, startNode, endNode, 3, 13.7f);
    test_bellmanFordShortestPaths(g1, startNode, endNode, 3, 13.7f);
    test_shortestPathFasterAlgorithm(g1, startNode, endNode, 3, 13.7f);
    test_floydWarshallAllShortestPaths(g1, startNode, endNode, 13.7f);
    test_johnsonAllShortestsPaths(g1, startNode, endNode, 3, 13.7f);
    test_johnson_bellmanford_equivalence(g1);
    test_all_path_algs_equivalence(g1);
    test_findArticulationPoints(g1, 2);
    test_findBridges(g1, 2);
        // removing edge before topsort testing because this example graph is not acyclic
        g1.deleteEdge(CustomClass(5, 2, 6), startNode);
    test_topsortKhan(g1, startNode, endNode);
        g1.addEdge(CustomClass(5, 2, 6), startNode, 124.5f);
    // MCST algorithms not supported for directed graphs
    test_findStronglyConnectedComponentsTarjan(g1, 3);
    test_findWeaklyConnectedComponents(g1, 1);
    test_edmondsKarpMaximumFlow(g1, startNode, endNode, static_cast<float>(0.3000000119));
    test_findIsolatedNodes(g1, 0);
    test_johnsonAllCycles(g1, 1);
    // dfs based algorithm for all cycles not tested for directed graph

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
    test_transposeOfGraph(g1);
    test_constructCompleteGraphFromNodes_with_default_weight<CustomClass, float>(someNodes, g1.getGraphType(), g1.getGraphWeights(), 1.0f);
    // complement of weighted graps is not supported and is not tested here
    test_transitiveClosureOfGraph(g1, g1.getNodeCount(), 17);
    test_transitiveReductionOfGraph(g1);

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
    assert(internal::equals(g1.getEccentricityOfNode(startNode), static_cast<unsigned long long>(5)));
    auto [radius, diameter, center] = g1.getRadiusDiameterAndCenter();
    assert(internal::equals(radius, static_cast<unsigned long long>(5)) && internal::equals(diameter, GraphClasses::MAX_WEIGHT<unsigned long long>) && center.size() == 3);
    auto [circumference, girth] = g1.getCircumferenceAndGirth();
    assert(internal::equals(circumference, static_cast<unsigned long long>(9)) && internal::equals(girth, static_cast<unsigned long long>(2)));

    test_depthFirstTraverse(g1, startNode, g1.getNodeCount());
    test_depthFirstSearch(g1, startNode, 'm', true);
    test_depthFirstSearch(g1, startNode, 'x', false);
    test_breadthFirstTraverse(g1, startNode, g1.getNodeCount());
    test_breadthFirstSearch(g1, startNode, 'm', true);
    test_breadthFirstSearch(g1, startNode, 'x', false);
    char endNode = 'l';
    test_dijkstraShortestPath(g1, startNode, endNode, 4, static_cast<unsigned long long>(4));
    test_dijkstraAllShortestPathsFromStart(g1, startNode, 'o', 5, static_cast<unsigned long long>(5));
    test_bellmanFordShortestPaths(g1, startNode, 'o', 5, static_cast<unsigned long long>(5));
    test_shortestPathFasterAlgorithm(g1, startNode, 'o', 5, static_cast<unsigned long long>(5));
    test_floydWarshallAllShortestPaths(g1, 'h', 'j', static_cast<unsigned long long>(4));
    // johnsonAllShortest paths not supported for unweigted graphs and is not tested here
    test_all_path_algs_equivalence(g1);
    test_findArticulationPoints(g1, 3);
    test_findBridges(g1, 3);
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
    test_findStronglyConnectedComponentsTarjan(g1, 3);
    test_findWeaklyConnectedComponents(g1, 1);
    test_edmondsKarpMaximumFlow(g1, startNode, 'd', static_cast<unsigned long long>(2));
    test_findIsolatedNodes(g1, 0);
    test_johnsonAllCycles(g1, 11);
    // dfs based algorithm for all cycles not tested for directed graph

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
    test_transposeOfGraph(g1);
    test_constructCompleteGraphFromNodes_without_default_weight(someNodes, g1.getGraphType());
    test_complementOfGraph(g1);
    test_transitiveClosureOfGraph(g1, g1.getNodeCount(), 226);
    test_transitiveReductionOfGraph(g1);

    std::cout << "SUCCESS" << std::endl;
    std::cout << "=================================================================" << std::endl << std::endl;
}

// for quicker callgrind testing
void string_double() {
    GraphClasses::Graph<std::string, double> g;
    g.configureDirections(GraphClasses::GraphType::Undirected);
    g.configureWeights(GraphClasses::GraphWeights::Weighted);
    const char* fileName = "testInputs/string_double.txt";
    g.readFromTxt(fileName);

    // std::unordered_set<std::string> someNodes{"node1", "node2", "node5", "node7"};
    std::string startNode = "node1";
    std::string endNode = "node6";

    auto ret = GraphAlgorithms::edmondsKarpMaximumFlow(g, startNode, endNode, GraphAlgorithms::AlgorithmBehavior::PrintAndReturn);
}

void int_int() {
    GraphClasses::Graph<int, int> g;
    g.configureDirections(GraphClasses::GraphType::Undirected);
    g.configureWeights(GraphClasses::GraphWeights::Unweighted);
    const char* fileName = "testInputs/int_int_u_u.txt";
    g.readFromTxt(fileName);

    // std::unordered_set<int> someNodes{2, 5, 3, 7};
    int startNode = 1;
    int endNode = 8;

    auto ret = GraphAlgorithms::edmondsKarpMaximumFlow(g, startNode, endNode, GraphAlgorithms::AlgorithmBehavior::PrintAndReturn);
}

void custom_float() {
    GraphClasses::Graph<CustomClass, float> g;
    g.configureDirections(GraphClasses::GraphType::Directed);
    g.configureWeights(GraphClasses::GraphWeights::Weighted);
    const char* fileName = "testInputs/custom_float.txt";
    g.readFromTxt(fileName);

    CustomClass startNode = CustomClass(1, 2, 3);
    CustomClass endNode = CustomClass(2, 2, 2);

    auto ret = GraphAlgorithms::edmondsKarpMaximumFlow(g, startNode, endNode, GraphAlgorithms::AlgorithmBehavior::PrintAndReturn);
}

void char_ull() {
    GraphClasses::Graph<char, unsigned long long> g;
    g.configureDirections(GraphClasses::GraphType::Directed);
    g.configureWeights(GraphClasses::GraphWeights::Unweighted);
    const char* fileName = "testInputs/char_ull_d_u.txt";
    g.readFromTxt(fileName);

    char startNode = 'a';
    char endNode = 'd';

    auto ret = GraphAlgorithms::edmondsKarpMaximumFlow(g, startNode, endNode, GraphAlgorithms::AlgorithmBehavior::PrintAndReturn);
}

int main() {
    test_internal_operators();

    test_graph_class_member_functions();

    test_string_double_undirected_weighted();

    test_int_int_undirected_unweighted();

    test_custom_float_directed_weighted();

    test_char_ull_directed_unweighted();

    // for quicker callgrind testing

    // string_double();
    // int_int();
    // custom_float();
    // char_ull();

    return 0;
}