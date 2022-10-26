#include <cassert>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

#include "CustomClass/customClass.hpp"
#include "SimpleGraphs.hpp"

// apart from these tests, all functions were tested arbitrarily and seem to be working
// this DOES NOT guarantee correctness
// At this time usage of pointer and reference types as NodeType is not supported

// NOTE: even though the library supports float as WeightType, usage of double is recommended becasue of calculation precission

// ----- traversal algorithms tests -----

template<typename NodeType, typename WeightType>
void test_breadthFirstSearch(GraphClasses::Graph<NodeType, WeightType> &g, NodeType startNode, NodeType nodeToFind, bool shouldFind) {
    auto [ifFound, traverseOrder] = GraphAlgorithms::breadthFirstSearch(g, startNode, nodeToFind, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ifFound == shouldFind);
}

template<typename NodeType, typename WeightType>
void test_breadthFirstTraverse(GraphClasses::Graph<NodeType, WeightType> &g, NodeType startNode, size_t bfsTreeSize) {
    auto ret = GraphAlgorithms::breadthFirstTraverse(g, startNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == bfsTreeSize);
}

template<typename NodeType, typename WeightType>
void test_depthFirstSearch(GraphClasses::Graph<NodeType, WeightType> &g, NodeType startNode, NodeType nodeToFind, bool shouldFind) {
    auto [ifFound, traverseOrder] = GraphAlgorithms::depthFirstSearch(g, startNode, nodeToFind, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ifFound == shouldFind);
}

template<typename NodeType, typename WeightType>
void test_depthFirstTraverse(GraphClasses::Graph<NodeType, WeightType> &g, NodeType startNode, size_t dfsTreeSize) {
    auto ret = GraphAlgorithms::depthFirstTraverse(g, startNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == dfsTreeSize);
}

// ----- path finding algorithms tests -----

template<typename NodeType, typename WeightType>
void test_bellmanFordShortestPaths(GraphClasses::Graph<NodeType, WeightType> &g, NodeType startNode, NodeType someEndNode, size_t edgesOnPathToEndNode, WeightType pathDistance) {
    auto ret = GraphAlgorithms::bellmanFordShortestPaths(g, startNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert((ret[someEndNode].first.size() - 1) == edgesOnPathToEndNode);
    assert(internal::equals(ret[someEndNode].second, pathDistance));
}

template<typename NodeType, typename WeightType>
void test_dijkstraAllShortestPathsFromStart(GraphClasses::Graph<NodeType, WeightType> &g, NodeType startNode, NodeType someEndNode, size_t edgesOnPathToEndNode, WeightType pathDistance) {
    auto ret = GraphAlgorithms::dijkstraAllShortestPathsFromStart(g, startNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert((ret[someEndNode].first.size() - 1) == edgesOnPathToEndNode);
    assert(internal::equals(ret[someEndNode].second, pathDistance));
}

template<typename NodeType, typename WeightType>
void test_dijkstraShortestPath(GraphClasses::Graph<NodeType, WeightType> &g, NodeType startNode, NodeType endNode, size_t edgesOnPath, WeightType pathDistance) {
    auto [path, dist] = GraphAlgorithms::dijkstraShortestPath(g, startNode, endNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert((path.size() - 1) == edgesOnPath);
    assert(internal::equals(pathDistance, dist));
}

template<typename NodeType, typename WeightType>
void test_floydWarshallAllShortestPaths(GraphClasses::Graph<NodeType, WeightType> &g, NodeType someStartNode, NodeType someEndNode, WeightType distance) {
    auto ret = GraphAlgorithms::floydWarshallAllShortestPaths(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(internal::equals(ret[someStartNode][someEndNode], distance));
}

template<typename NodeType, typename WeightType>
void test_johnsonAllShortestsPaths(GraphClasses::Graph<NodeType, WeightType> &g, NodeType someStartNode, NodeType someEndNode,  size_t edgesOnPathToEndNode, WeightType pathDistance) {
    auto ret = GraphAlgorithms::johnsonAllShortestsPaths(g, {}, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert((ret[someStartNode][someEndNode].first.size() - 1) == edgesOnPathToEndNode);
    assert(internal::equals(ret[someStartNode][someEndNode].second, pathDistance));
}

template<typename NodeType, typename WeightType>
void test_shortestPathFasterAlgorithm(GraphClasses::Graph<NodeType, WeightType> &g, NodeType startNode, NodeType someEndNode, size_t edgesOnPathToEndNode, WeightType pathDistance) {
    auto ret = GraphAlgorithms::shortestPathFasterAlgorithm(g, startNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert((ret[someEndNode].first.size() - 1) == edgesOnPathToEndNode);
    assert(internal::equals(ret[someEndNode].second, pathDistance));
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

// for all algorithms except johnson as that one is not supported for all graphs
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

// ----- articulation point and bridge algorithms tests -----

template<typename NodeType, typename WeightType>
void test_findArticulationPoints(GraphClasses::Graph<NodeType, WeightType> &g, size_t numOfArticulationPoints) {
    auto ret = GraphAlgorithms::findArticulationPoints(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == numOfArticulationPoints);
}

template<typename NodeType, typename WeightType>
void test_findBridges(GraphClasses::Graph<NodeType, WeightType> &g, size_t numOfBridges) {
    auto ret = GraphAlgorithms::findBridges(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == numOfBridges);
}

// ----- topological sorting algorithms -----

// expensive
template<typename NodeType, typename WeightType>
void test_allTopsorts(GraphClasses::Graph<NodeType, WeightType> &g, size_t numOfTopsorts) {
    auto ret = GraphAlgorithms::allTopsorts(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == numOfTopsorts);
}

// expensive
template<typename NodeType, typename WeightType>
void test_dfs_and_khan_topsort(GraphClasses::Graph<NodeType, WeightType> &g) {
    auto retDFS = GraphAlgorithms::dfsTopsort(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    auto retKhan = GraphAlgorithms::khanTopsort(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);

    auto retAll = GraphAlgorithms::allTopsorts(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);

    if (retDFS.size() == 0 && retKhan.size() == 0 && retAll.size() == 0) {
        // cycle found, no topsort exists
        return;
    } 

    bool khanValid = false;
    bool dfsValid = false;

    for (auto& topsort : retAll) {
        if (!khanValid) {
            khanValid = (retKhan == topsort);
        }

        if (!dfsValid) {
            dfsValid = (retDFS == topsort);
        }

        if (khanValid && dfsValid) {
            break;
        }
    }

    assert(khanValid);
    assert(dfsValid);
}

// ----- spanning tree algorithms -----

template<typename NodeType, typename WeightType>
void test_boruvkaMinimumSpanningTree(GraphClasses::Graph<NodeType, WeightType> &g, WeightType expectedTotalCost) {
    auto [totalCost, spanningTree] = GraphAlgorithms::boruvkaMinimumSpanningTree(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(internal::equals(totalCost, expectedTotalCost));
}

template<typename NodeType, typename WeightType>
void test_commonMinimumCostSpannigTree(GraphClasses::Graph<NodeType, WeightType> &g1, GraphClasses::Graph<NodeType, WeightType> &g2, WeightType expectedTotalCost) {
    auto [totalCost, spanningTree] = GraphAlgorithms::commonMinimumCostSpannigTree(g1, g2, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(internal::equals(totalCost, expectedTotalCost));
}

template<typename NodeType, typename WeightType>
void test_kruskalMinimumSpanningTree(GraphClasses::Graph<NodeType, WeightType> &g, WeightType expectedTotalCost) {
    auto [totalCost, spanningTree] = GraphAlgorithms::kruskalMinimumSpanningTree(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(internal::equals(totalCost, expectedTotalCost));
}

template<typename NodeType, typename WeightType>
void test_primMinimumSpanningTree(GraphClasses::Graph<NodeType, WeightType> &g, WeightType expectedTotalCost) {
    auto [totalCost, spanningTree] = GraphAlgorithms::primMinimumSpanningTree(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(internal::equals(totalCost, expectedTotalCost));
}

template<typename NodeType, typename WeightType>
void test_reverseDeleteMinimumSpanningTree(GraphClasses::Graph<NodeType, WeightType> &g, WeightType expectedTotalCost) {
    auto [totalCost, spanningTree] = GraphAlgorithms::reverseDeleteMinimumSpanningTree(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(internal::equals(totalCost, expectedTotalCost));
}

template<typename NodeType, typename WeightType>
void test_all_mcst_algs_equivalence(GraphClasses::Graph<NodeType, WeightType> &g) {
    auto primRet = GraphAlgorithms::primMinimumSpanningTree(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    auto kruskalRet = GraphAlgorithms::kruskalMinimumSpanningTree(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    auto boruvkaRet = GraphAlgorithms::boruvkaMinimumSpanningTree(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    auto reverseDeleteRet = GraphAlgorithms::reverseDeleteMinimumSpanningTree(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    
    assert(internal::equals(primRet.first, kruskalRet.first));
    assert(internal::equals(kruskalRet.first, boruvkaRet.first));
    assert(internal::equals(boruvkaRet.first, reverseDeleteRet.first));
    assert(internal::equals(reverseDeleteRet.first, primRet.first));
}

// ----- component algorithms tests -----

template<typename NodeType, typename WeightType>
void test_findWeaklyConnectedComponents(GraphClasses::Graph<NodeType, WeightType> &g, size_t numOfComponents) {
    auto ret = GraphAlgorithms::findWeaklyConnectedComponents(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == numOfComponents);
}

template<typename NodeType, typename WeightType>
void test_kosarajuFindStronglyConnectedComponents(GraphClasses::Graph<NodeType, WeightType> &g, size_t numOfComponents) {
    auto ret = GraphAlgorithms::kosarajuFindStronglyConnectedComponents(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == numOfComponents);
}

template<typename NodeType, typename WeightType>
void test_tarjanFindBiconnectedComponents(GraphClasses::Graph<NodeType, WeightType> &g, size_t numOfComponents) {
    auto ret = GraphAlgorithms::tarjanFindBiconnectedComponents(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == numOfComponents);
}

template<typename NodeType, typename WeightType>
void test_tarjanFindStronglyConnectedComponents(GraphClasses::Graph<NodeType, WeightType> &g, size_t numOfComponents) {
    auto ret = GraphAlgorithms::tarjanFindStronglyConnectedComponents(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == numOfComponents);
}

// ----- cycle algorithms tests -----

template<typename NodeType, typename WeightType>
void test_findAllCycles(GraphClasses::Graph<NodeType, WeightType> &g, size_t numOfCycles) {
    auto ret = GraphAlgorithms::findAllCycles(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == numOfCycles);
}

template<typename NodeType, typename WeightType>
void test_johnsonAllCycles(GraphClasses::Graph<NodeType, WeightType> &g, size_t numOfCycles) {
    auto ret = GraphAlgorithms::johnsonAllCycles(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == numOfCycles);
}

// ----- flow algorithms tests -----

template<typename NodeType, typename WeightType>
void test_edmondsKarpMaximumFlow(GraphClasses::Graph<NodeType, WeightType> &g, NodeType source, NodeType sink, WeightType expectedMaxFlow) {
    auto [maxFlow, residualGraph] = GraphAlgorithms::edmondsKarpMaximumFlow(g, source, sink, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(internal::equals(maxFlow, expectedMaxFlow));
}

template<typename NodeType, typename WeightType>
void test_pushRelabelMaximumFlow(GraphClasses::Graph<NodeType, WeightType> &g, NodeType source, NodeType sink, WeightType expectedMaxFlow) {
    auto [maxFlow, flowGraph] = GraphAlgorithms::pushRelabelMaximumFlow(g, source, sink, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(internal::equals(maxFlow, expectedMaxFlow));
}

template<typename NodeType, typename WeightType>
void test_all_maxFlow_algs_equivalence(GraphClasses::Graph<NodeType, WeightType> &g) {
    int wrongFlowCounter = 0;

    auto neighborList = g.getNeighborList();

    for (auto& [startNode, startNodeNeighbors] : neighborList) {
        for (auto& [endNode, endNodeNeighbors] : startNodeNeighbors) {
            auto edmondsKarpReturn = GraphAlgorithms::edmondsKarpMaximumFlow(g, startNode, endNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
            auto pushRelabelReturn = GraphAlgorithms::pushRelabelMaximumFlow(g, startNode, endNode, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);

            if (!internal::equals(edmondsKarpReturn.first, pushRelabelReturn.first)) {
                // std::cout << std::setprecision(10);
                // std::cout << "[" << startNode << "] to [" << endNode << "] : EDMONDSKARP : " << std::setw(15) << edmondsKarpReturn.first  
                //                                                      << " \tPUSHRELABEL : " << std::setw(15) <<  pushRelabelReturn.first << std::endl;
                
                ++wrongFlowCounter;
            }
        }   
    }

    assert(wrongFlowCounter == 0);
}

// ----- eulerian path and cycle algorithms tests -----

template<typename NodeType, typename WeightType>
void test_hierholzerFindEulerianCycle(GraphClasses::Graph<NodeType, WeightType> &g, size_t numNodesInCycle) {
    auto ret = GraphAlgorithms::hierholzerFindEulerianCycle(g, {}, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == numNodesInCycle);
    if (g.getGraphDirections() == GraphClasses::GraphDirections::Directed) {
        assert(ret.size() == g.getEdgeCount() + 1);
    }
    else {
        assert(ret.size() == ((g.getEdgeCount() / 2) + 1));
    }
}

template<typename NodeType, typename WeightType>
void test_hierholzerFindEulerianPath(GraphClasses::Graph<NodeType, WeightType> &g, NodeType startNode, NodeType endNode) {
    auto ret = GraphAlgorithms::hierholzerFindEulerianPath(g, {}, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(internal::equals(*std::begin(ret), startNode));
    assert(internal::equals(ret.back(), endNode));   
}

// ----- other algorithms tests -----

template<typename NodeType, typename WeightType>
void test_countTriangles(GraphClasses::Graph<NodeType, WeightType> &g, size_t numOfTriangles) {
    auto ret = GraphAlgorithms::countTriangles(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret == numOfTriangles);
}

template<typename NodeType, typename WeightType>
void test_findIsolatedNodes(GraphClasses::Graph<NodeType, WeightType> &g, size_t numOfIsolatedNodes) {
    auto ret = GraphAlgorithms::findIsolatedNodes(g, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
    assert(ret.size() == numOfIsolatedNodes);
}

// ----- utility functions tests -----

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

template<typename NodeType>
void test_constructCompleteGraphFromNodes_without_default_weight(std::unordered_set<NodeType>& nodes, GraphClasses::GraphDirections graphDirections) {
    GraphClasses::Graph<NodeType> ret = GraphUtility::constructCompleteGraphFromNodes(nodes, graphDirections);
    assert(ret.getNodeCount() == nodes.size());
    assert(ret.getEdgeCount() == (nodes.size() * (nodes.size() - 1)));
}

template<typename NodeType, typename WeightType>
void test_constructCompleteGraphFromNodes_with_default_weight(std::unordered_set<NodeType>& nodes, GraphClasses::GraphDirections graphDirections, GraphClasses::GraphWeights graphWeights, std::optional<WeightType> defaultWeight) {
    GraphClasses::Graph<NodeType, WeightType> ret = GraphUtility::constructCompleteGraphFromNodes(nodes, graphDirections, graphWeights, defaultWeight);
    assert(ret.getNodeCount() == nodes.size());
    assert(ret.getEdgeCount() == (nodes.size() * (nodes.size() - 1)));
}

template<typename NodeType, typename WeightType>
void test_getSubgraphFromNodes(GraphClasses::Graph<NodeType, WeightType> &g, std::unordered_set<NodeType>& nodes, size_t expectedNumOfNodes, size_t expectedNumOfEdges) {
    GraphClasses::Graph<NodeType, WeightType> ret = GraphUtility::getSubgraphFromNodes(g, nodes);
    // testing that intersection of 2 same graphs is the exact same starting graph
    assert(ret.getNodeCount() == expectedNumOfNodes);
    assert(ret.getEdgeCount() == expectedNumOfEdges);
}

template<typename NodeType, typename WeightType>
void test_intersectGraphs(GraphClasses::Graph<NodeType, WeightType> &g1, GraphClasses::Graph<NodeType, WeightType> &g2) {
    assert(g1.getGraphDirections() == g2.getGraphDirections());
    assert(g1.getGraphWeights() == g2.getGraphWeights());
    GraphClasses::Graph<NodeType, WeightType> ret = GraphUtility::intersectGraphs(g1, g2);
    // testing that intersection of 2 same graphs is the exact same starting graph
    assert(ret.getNodeCount() == g1.getNodeCount());
    assert(ret.getEdgeCount() == g1.getEdgeCount());
    assert(ret.getNodeCount() == g2.getNodeCount());
    assert(ret.getEdgeCount() == g2.getEdgeCount());
}

template<typename NodeType, typename WeightType>
void test_mergeGraphs(GraphClasses::Graph<NodeType, WeightType> &g1, GraphClasses::Graph<NodeType, WeightType> &g2) {
    assert(g1.getGraphDirections() == g2.getGraphDirections());
    assert(g1.getGraphWeights() == g2.getGraphWeights());
    GraphClasses::Graph<NodeType, WeightType> ret = GraphUtility::mergeGraphs(g1, g2);
    // testing that 2 same graphs do not produce extra nodes or edges after merging
    assert(ret.getNodeCount() == g1.getNodeCount());
    assert(ret.getEdgeCount() == g1.getEdgeCount());
    assert(ret.getNodeCount() == g2.getNodeCount());
    assert(ret.getEdgeCount() == g2.getEdgeCount());
}

template<typename NodeType, typename WeightType>
void test_transitiveClosureOfGraph(GraphClasses::Graph<NodeType, WeightType> &g, size_t expectedNumOfNodes, size_t expectedNumOfEdges ) {
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

template<typename NodeType, typename WeightType>
void test_transposeOfGraph(GraphClasses::Graph<NodeType, WeightType> &g) {
    GraphClasses::Graph<NodeType, WeightType> ret = GraphUtility::transposeOfGraph(g);
    // transpose should ahve same number of nodes and edges as the original
    assert(ret.getNodeCount() == g.getNodeCount());
    assert(ret.getEdgeCount() == g.getEdgeCount());
}

// ----- main tests tests -----

void test_internal_operators() {
    std::cout << "==================== test_internal_operators ====================" << std::endl;

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
    std::cout << "=================================================================" << std::endl << std::endl;
}

void test_graph_class_member_functions() {
    std::cout << "==================== test_graph_class_member_functions ====================" << std::endl;
    
    GraphClasses::Graph<unsigned, unsigned> g1;
    g1.configureDirections(GraphClasses::GraphDirections::Undirected);
    g1.configureWeights(GraphClasses::GraphWeights::Weighted);
    const char* fileName1 = "testInputs/int_int.txt";
    g1.readFromTxt(fileName1);
    // std::cout << "Node count: " << g1.getNodeCount() << " Edge count: " << g1.getEdgeCount() << " Density: " << g1.getDensity() << std::endl;
    // std::cout << g1 << std::endl;

    std::cout << "\tTesting member functions     ";

    assert(g1.isConfigured());
    assert(g1.getGraphDirections() == GraphClasses::GraphDirections::Undirected);
    assert(g1.getGraphWeights() == GraphClasses::GraphWeights::Weighted);

    assert(g1.getNodeCount() == 8);
    assert(g1.getEdgeCount() == 26);

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
    g1.configureDirections(GraphClasses::GraphDirections::Undirected);
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

    g1.configureDirections(GraphClasses::GraphDirections::Undirected);
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

    g1.configureDirections(GraphClasses::GraphDirections::Directed);
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

    g1.configureDirections(GraphClasses::GraphDirections::Directed);
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
    g2.configureDirections(GraphClasses::GraphDirections::Directed);
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
    std::cout << "===========================================================================" << std::endl << std::endl;
}

void test_string_double_undirected_weighted() {
    std::cout << "==================== test_string_double_undirected_weighted ====================" << std::endl;

    GraphClasses::Graph<std::string, double> g1;
    g1.configureDirections(GraphClasses::GraphDirections::Undirected);
    g1.configureWeights(GraphClasses::GraphWeights::Weighted);
    const char* fileName1 = "testInputs/string_double.txt";
    g1.readFromTxt(fileName1);
    // std::cout << "Node count: " << g1.getNodeCount() << " Edge count: " << g1.getEdgeCount() << " Density: " << g1.getDensity() << std::endl;
    // std::cout << g1 << std::endl;

    std::string startNode = "node1";

    assert(internal::equals(g1.getEccentricityOfNode(startNode), static_cast<double>(6125.78774)));
    auto [radius, diameter, center] = g1.getRadiusDiameterAndCenter();
    assert(internal::equals(radius, static_cast<double>(5991.55124)) && internal::equals(diameter, static_cast<double>(6125.78774)) && center.size() == 1);
    auto [circumference, girth] = g1.getCircumferenceAndGirth();
    assert(internal::equals(circumference, static_cast<double>(430.886504)) && internal::equals(girth, static_cast<double>(430.886504)));
    auto [hasEulerianCycle, hasEulerianPath] = g1.hasEulerianCycleOrPath();
    assert(!hasEulerianCycle && !hasEulerianPath);

    // ---- alg testing ----
    std::cout << '\t' <<std::left << std::setw(50) << "Testing traversal algorithms";
    test_breadthFirstSearch(g1, startNode, std::string{"node5"}, true);
    test_breadthFirstSearch(g1, startNode, std::string{"node222"}, false);
    test_breadthFirstTraverse(g1, startNode, g1.getNodeCount());
    test_depthFirstSearch(g1, startNode, std::string{"node5"}, true);
    test_depthFirstSearch(g1, startNode, std::string{"node222"}, false);
    test_depthFirstTraverse(g1, startNode, g1.getNodeCount());
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::string endNode = "node6";

    std::cout << '\t' <<std::left << std::setw(50) << "Testing path finding algorithms";
    test_bellmanFordShortestPaths(g1, startNode, endNode, 4, static_cast<double>(134.236504));
    test_dijkstraAllShortestPathsFromStart(g1, startNode, endNode, 4, static_cast<double>(134.236504));
    test_dijkstraShortestPath(g1, startNode, endNode, 4, static_cast<double>(134.236504)); 
    test_floydWarshallAllShortestPaths(g1, startNode, endNode, static_cast<double>(134.236504));
    test_johnsonAllShortestsPaths(g1, startNode, endNode, 4, static_cast<double>(134.236504));
    test_shortestPathFasterAlgorithm(g1, startNode, endNode, 4, static_cast<double>(134.236504));
    test_johnson_bellmanford_equivalence(g1);
    test_all_path_algs_equivalence(g1);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' <<std::left << std::setw(50) << "Testing articulation point and bridge algorithms";
    test_findArticulationPoints(g1, 2);
    test_findBridges(g1, 2);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    // std::cout << '\t' <<std::left << std::setw(50) << "Testing topological sort algorithms";
    // topsort makes no sense for undirected graphs and is not tested here
    // std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' <<std::left << std::setw(50) << "Testing spanning tree algorithms";
    test_boruvkaMinimumSpanningTree(g1, static_cast<double>(6199.467744));
    test_commonMinimumCostSpannigTree(g1, g1, static_cast<double>(6199.467744));
    test_kruskalMinimumSpanningTree(g1, static_cast<double>(6199.467744));
    test_primMinimumSpanningTree(g1, static_cast<double>(6199.467744));
    test_reverseDeleteMinimumSpanningTree(g1, static_cast<double>(6199.467744));
    test_all_mcst_algs_equivalence(g1);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' << std::left << std::setw(50) << "Testing component algorithms";
    test_findWeaklyConnectedComponents(g1, 1);
    // kosaraju only supported for directed graphs
    test_tarjanFindBiconnectedComponents(g1, 3);
    test_tarjanFindStronglyConnectedComponents(g1, 1);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' <<std::left << std::setw(50) << "Testing cycle algorithms";
    test_findAllCycles(g1, 1);
    // johnson algorithm for all cycles not tested for undirected graph
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' <<std::left << std::setw(50) << "Testing flow algorithms";
    test_edmondsKarpMaximumFlow(g1, startNode, endNode, static_cast<double>(51.550004));
    test_pushRelabelMaximumFlow(g1, startNode, endNode, static_cast<double>(51.550004));
    test_all_maxFlow_algs_equivalence(g1);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' <<std::left << std::setw(50) << "Testing eulerian path and cycle algorithms";
    // changing the graph a bit so eulerian cycles/paths can exist
        g1.deleteNode("node9");
        g1.deleteNode("node7");
    test_hierholzerFindEulerianPath(g1, std::string("node1"), std::string("node4"));
        g1.addEdge("node1", "node4", 0); g1.addEdge("node4", "node1", 0);
    test_hierholzerFindEulerianCycle(g1, 10);
        g1.deleteEdge("node1", "node4");
        g1.addEdge("node3", "node9", static_cast<double>(22.13)); g1.addEdge("node9", "node3", static_cast<double>(22.13));
        g1.addEdge("node5", "node7", static_cast<double>(5991.55124)); g1.addEdge("node7", "node5", static_cast<double>(5991.55124));
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' <<std::left << std::setw(50) << "Testing other algorithms";
    test_countTriangles(g1, 0);
    test_findIsolatedNodes(g1, 0);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    GraphClasses::Graph<std::string, double> g2;
    g2.configureDirections(GraphClasses::GraphDirections::Undirected);
    g2.configureWeights(GraphClasses::GraphWeights::Weighted);
    const char* fileName2 = "testInputs/string_double.txt";
    g2.readFromTxt(fileName2);
    // std::cout << "Node count: " << g2.getNodeCount() << " Edge count: " << g2.getEdgeCount() << " Density: " << g2.getDensity() << std::endl;
    // std::cout << g2 << std::endl;

    // ---- utility testing ----
    std::cout << '\t' <<std::left << std::setw(50) << "Testing utility functions";
    // complement of weighted graps is not supported and is not tested here
    std::unordered_set<std::string> someNodes{"node1", "node2", "node5", "node7"};
    test_constructCompleteGraphFromNodes_with_default_weight<std::string, double>(someNodes, g1.getGraphDirections(), g1.getGraphWeights(), static_cast<double>(1.0));
    test_getSubgraphFromNodes(g1, someNodes, 4, 6);
    test_intersectGraphs(g1, g2);
    test_mergeGraphs(g1, g2);
    auto g1NodeCount = g1.getNodeCount();
    test_transitiveClosureOfGraph(g1, g1NodeCount, (g1NodeCount * (g1NodeCount - 1) + g1NodeCount));
    test_transitiveReductionOfGraph(g1);
    // transposing makes no sense for undirected graphs and is not tested here
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << "================================================================================" << std::endl << std::endl;
}

void test_int_int_undirected_unweighted() {
    std::cout << "==================== test_int_int_undirected_unweighted ====================" << std::endl;

    GraphClasses::Graph<int, int> g1;
    g1.configureDirections(GraphClasses::GraphDirections::Undirected);
    g1.configureWeights(GraphClasses::GraphWeights::Unweighted);
    const char* fileName1 = "testInputs/int_int_u_u.txt";
    g1.readFromTxt(fileName1);
    // std::cout << "Node count: " << g1.getNodeCount() << " Edge count: " << g1.getEdgeCount() << " Density: " << g1.getDensity() << std::endl;
    // std::cout << g1 << std::endl;

    int startNode = 1;

    assert(internal::equals(g1.getEccentricityOfNode(startNode), 3));
    auto [radius, diameter, center] = g1.getRadiusDiameterAndCenter();
    assert(internal::equals(radius, 2) && internal::equals(diameter, 3) && center.size() == 4);
    auto [circumference, girth] = g1.getCircumferenceAndGirth();
    assert(internal::equals(circumference, 8) && internal::equals(girth, 3));
    auto [hasEulerianCycle, hasEulerianPath] = g1.hasEulerianCycleOrPath();
    assert(!hasEulerianCycle && hasEulerianPath);

    // ---- alg testing ----
    std::cout << '\t' <<std::left << std::setw(50) << "Testing traversal algorithms";
    test_breadthFirstSearch(g1, startNode, 6, true);
    test_breadthFirstSearch(g1, startNode, 222, false);
    test_breadthFirstTraverse(g1, startNode, g1.getNodeCount());
    test_depthFirstSearch(g1, startNode, 6, true);
    test_depthFirstSearch(g1, startNode, 222, false);    
    test_depthFirstTraverse(g1, startNode, g1.getNodeCount());
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    int endNode = 5;

    std::cout << '\t' <<std::left << std::setw(50) << "Testing path finding algorithms";
    test_bellmanFordShortestPaths(g1, startNode, 6, 2, 2);
    test_dijkstraAllShortestPathsFromStart(g1, startNode, 6, 2, 2);
    test_dijkstraShortestPath(g1, startNode, endNode, 2, 2);
    test_floydWarshallAllShortestPaths(g1, 4, 5, 2);
    // johnsonAllShortest paths not supported for unweigted graphs and is not tested here
    test_shortestPathFasterAlgorithm(g1, startNode, 6, 2, 2);
    // johnson and bellmanford equivalence not tested here
    test_all_path_algs_equivalence(g1);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' <<std::left << std::setw(50) << "Testing articulation point and bridge algorithms";
    test_findArticulationPoints(g1, 0);
    test_findBridges(g1, 0);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    // std::cout << '\t' <<std::left << std::setw(50) << "Testing topological sort algorithms";
    // topsort makes no sense for undirected graphs and is not tested here
    // std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' <<std::left << std::setw(50) << "Testing spanning tree algorithms";
    test_boruvkaMinimumSpanningTree(g1, 7);
    test_commonMinimumCostSpannigTree(g1, g1, 7);
    test_kruskalMinimumSpanningTree(g1, 7);
    test_primMinimumSpanningTree(g1, 7);
    test_reverseDeleteMinimumSpanningTree(g1, 7);
    test_all_mcst_algs_equivalence(g1);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' << std::left << std::setw(50) << "Testing component algorithms";
    test_findWeaklyConnectedComponents(g1, 1);
    // kosaraju only supported for directed graphs
    test_tarjanFindBiconnectedComponents(g1, 1);
    test_tarjanFindStronglyConnectedComponents(g1, 1);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' <<std::left << std::setw(50) << "Testing cycle algorithms";
    test_findAllCycles(g1, 38);
    // johnson algorithm for all cycles not tested for undirected graph
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' <<std::left << std::setw(50) << "Testing flow algorithms";
    test_edmondsKarpMaximumFlow(g1, 1, 8, 3);
    test_pushRelabelMaximumFlow(g1, 1, 8, 3);
    test_all_maxFlow_algs_equivalence(g1);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' <<std::left << std::setw(50) << "Testing eulerian path and cycle algorithms";
    // changing the graph a bit so eulerian cycles/paths can exist
    test_hierholzerFindEulerianPath(g1, 1, 8);
        g1.addEdge(1, 8); g1.addEdge(8, 1);
    test_hierholzerFindEulerianCycle(g1, 15);
        g1.deleteEdge(1, 8);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' <<std::left << std::setw(50) << "Testing other algorithms";
    test_countTriangles(g1, 4);
    test_findIsolatedNodes(g1, 0);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    GraphClasses::Graph<int, int> g2;
    g2.configureDirections(GraphClasses::GraphDirections::Undirected);
    g2.configureWeights(GraphClasses::GraphWeights::Unweighted);
    const char* fileName2 = "testInputs/int_int_u_u.txt";
    g2.readFromTxt(fileName2);
    // std::cout << "Node count: " << g2.getNodeCount() << " Edge count: " << g2.getEdgeCount() << " Density: " << g2.getDensity() << std::endl;
    // std::cout << g2 << std::endl;

    // ---- utility testing ----
    std::cout << '\t' <<std::left << std::setw(50) << "Testing utility functions";
    test_complementOfGraph(g1);
    std::unordered_set<int> someNodes{2, 5, 3, 7};
    test_constructCompleteGraphFromNodes_without_default_weight(someNodes, g1.getGraphDirections());
    test_getSubgraphFromNodes(g1, someNodes, 4, 4);
    test_intersectGraphs(g1, g2);
    test_mergeGraphs(g1, g2);
    auto g1NodeCount = g1.getNodeCount();
    test_transitiveClosureOfGraph(g1, g1NodeCount, (g1NodeCount * (g1NodeCount - 1) + g1NodeCount));
    test_transitiveReductionOfGraph(g1);
    // transposing makes no sense for undirected graphs and is not tested here
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;
    
    std::cout << "============================================================================" << std::endl << std::endl;
}

void test_custom_float_directed_weighted() {
    std::cout << "==================== test_custom_float_directed_weighted ====================" << std::endl;

    GraphClasses::Graph<CustomClass, float> g1;
    g1.configureDirections(GraphClasses::GraphDirections::Directed);
    g1.configureWeights(GraphClasses::GraphWeights::Weighted);
    const char* fileName1 = "testInputs/custom_float.txt";
    g1.readFromTxt(fileName1);
    // std::cout << "Node count: " << g1.getNodeCount() << " Edge count: " << g1.getEdgeCount() << " Density: " << g1.getDensity() << std::endl;
    // std::cout << g1 << std::endl;

    CustomClass startNode = CustomClass(1, 2, 3);
    
    assert(internal::equals(g1.getEccentricityOfNode(startNode), 13.69999981f));
    auto [radius, diameter, center] = g1.getRadiusDiameterAndCenter();
    assert(internal::equals(radius, 13.69999981f) && internal::equals(diameter, GraphClasses::MAX_WEIGHT<float>) && center.size() == 1);
    auto [circumference, girth] = g1.getCircumferenceAndGirth();
    assert(internal::equals(circumference, static_cast<float>(137.1999969)) && internal::equals(girth, static_cast<float>(137.1999969)));
    auto [hasEulerianCycle, hasEulerianPath] = g1.hasEulerianCycleOrPath();
    assert(!hasEulerianCycle && !hasEulerianPath);

    // ---- alg testing ----
    std::cout << '\t' <<std::left << std::setw(50) << "Testing traversal algorithms";
    test_breadthFirstSearch(g1, startNode, CustomClass(1, 7, 3), true);
    test_breadthFirstSearch(g1, startNode, CustomClass(9, 9, 9), false);
    test_breadthFirstTraverse(g1, startNode, g1.getNodeCount());
    test_depthFirstSearch(g1, startNode, CustomClass(1, 7, 3), true);
    test_depthFirstSearch(g1, startNode, CustomClass(9, 9, 9), false);
    test_depthFirstTraverse(g1, startNode, g1.getNodeCount());
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    CustomClass endNode = CustomClass(2, 2, 2);

    std::cout << '\t' <<std::left << std::setw(50) << "Testing path finding algorithms";
    test_bellmanFordShortestPaths(g1, startNode, endNode, 3, 13.7f);
    test_dijkstraAllShortestPathsFromStart(g1, startNode, endNode, 3, 13.7f);
    test_dijkstraShortestPath(g1, startNode, endNode, 3, 13.7f);
    test_floydWarshallAllShortestPaths(g1, startNode, endNode, 13.7f);
    test_johnsonAllShortestsPaths(g1, startNode, endNode, 3, 13.7f);
    test_shortestPathFasterAlgorithm(g1, startNode, endNode, 3, 13.7f);
    test_johnson_bellmanford_equivalence(g1);
    test_all_path_algs_equivalence(g1);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' <<std::left << std::setw(50) << "Testing articulation point and bridge algorithms";
    test_findArticulationPoints(g1, 2);
    test_findBridges(g1, 2);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' <<std::left << std::setw(50) << "Testing topological sort algorithms";
    // removing edge before topsort testing because this example graph is not acyclic
        g1.deleteEdge(CustomClass(5, 2, 6), startNode);
    test_allTopsorts(g1, 3);
    test_dfs_and_khan_topsort(g1);
        g1.addEdge(CustomClass(5, 2, 6), startNode, 124.5f);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;
    
    // std::cout << '\t' <<std::left << std::setw(50) << "Testing spanning tree algorithms";
    // MCST algorithms not supported for directed graphs
    // std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;
    
    std::cout << '\t' << std::left << std::setw(50) << "Testing component algorithms";
    test_findWeaklyConnectedComponents(g1, 1);
    test_kosarajuFindStronglyConnectedComponents(g1, 3);
    // tarjan biconnected only supported for undirected graphs
    test_tarjanFindStronglyConnectedComponents(g1, 3);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' <<std::left << std::setw(50) << "Testing cycle algorithms";
    // dfs based algorithm for all cycles not tested for directed graph
    test_johnsonAllCycles(g1, 1);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' <<std::left << std::setw(50) << "Testing flow algorithms";
    test_edmondsKarpMaximumFlow(g1, startNode, endNode, static_cast<float>(0.3000000119));
    test_pushRelabelMaximumFlow(g1, startNode, endNode, static_cast<float>(0.3000000119));
    test_all_maxFlow_algs_equivalence(g1);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' <<std::left << std::setw(50) << "Testing eulerian path and cycle algorithms";
    // changing the graph a bit so eulerian cycles/paths can exist
        g1.deleteNode(CustomClass(1, 7, 3));
    test_hierholzerFindEulerianPath(g1, CustomClass(5, 2, 6), CustomClass(2, 2, 2));
        g1.deleteNode(CustomClass(2, 2, 2));
    test_hierholzerFindEulerianCycle(g1, 4);
        g1.addEdge(CustomClass(4, 5, 6), CustomClass(1, 7, 3), 0.992f);
        g1.addEdge(CustomClass(5, 2, 6), CustomClass(2, 2, 2), 1.f);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' <<std::left << std::setw(50) << "Testing other algorithms";
    test_countTriangles(g1, 1);
    test_findIsolatedNodes(g1, 0);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    GraphClasses::Graph<CustomClass, float> g2;
    g2.configureDirections(GraphClasses::GraphDirections::Directed);
    g2.configureWeights(GraphClasses::GraphWeights::Weighted);
    const char* fileName2 = "testInputs/custom_float.txt";
    g2.readFromTxt(fileName2);
    // std::cout << "Node count: " << g2.getNodeCount() << " Edge count: " << g2.getEdgeCount() << " Density: " << g2.getDensity() << std::endl;
    // std::cout << g2 << std::endl;

    // ---- utility testing ----
    std::cout << '\t' <<std::left << std::setw(50) << "Testing utility functions";
    // complement of weighted graps is not supported and is not tested here
    std::unordered_set<CustomClass> someNodes{startNode, CustomClass(4, 5, 6), endNode};
    test_constructCompleteGraphFromNodes_with_default_weight<CustomClass, float>(someNodes, g1.getGraphDirections(), g1.getGraphWeights(), 1.0f);
    test_getSubgraphFromNodes(g1, someNodes, 3, 1);
    test_intersectGraphs(g1, g2);
    test_mergeGraphs(g1, g2);
    test_transitiveClosureOfGraph(g1, g1.getNodeCount(), 17);
    test_transitiveReductionOfGraph(g1);
    test_transposeOfGraph(g1);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << "=============================================================================" << std::endl << std::endl;
}

void test_char_ull_directed_unweighted() {
    std::cout << "==================== test_char_ull_directed_unweighted ====================" << std::endl;

    GraphClasses::Graph<char, unsigned long long> g1;
    g1.configureDirections(GraphClasses::GraphDirections::Directed);
    g1.configureWeights(GraphClasses::GraphWeights::Unweighted);
    const char* fileName1 = "testInputs/char_ull_d_u.txt";
    g1.readFromTxt(fileName1);
    // std::cout << "Node count: " << g1.getNodeCount() << " Edge count: " << g1.getEdgeCount() << " Density: " << g1.getDensity() << std::endl;
    // std::cout << g1 << std::endl;
    
    char startNode = 'a';

    assert(internal::equals(g1.getEccentricityOfNode(startNode), static_cast<unsigned long long>(5)));
    auto [radius, diameter, center] = g1.getRadiusDiameterAndCenter();
    assert(internal::equals(radius, static_cast<unsigned long long>(5)) && internal::equals(diameter, GraphClasses::MAX_WEIGHT<unsigned long long>) && center.size() == 3);
    auto [circumference, girth] = g1.getCircumferenceAndGirth();
    assert(internal::equals(circumference, static_cast<unsigned long long>(9)) && internal::equals(girth, static_cast<unsigned long long>(2)));
    auto [hasEulerianCycle, hasEulerianPath] = g1.hasEulerianCycleOrPath();
    assert(!hasEulerianCycle && !hasEulerianPath);

    // ---- alg testing ----
    std::cout << '\t' <<std::left << std::setw(50) << "Testing traversal algorithms";
    test_breadthFirstSearch(g1, startNode, 'm', true);
    test_breadthFirstSearch(g1, startNode, 'x', false);
    test_breadthFirstTraverse(g1, startNode, g1.getNodeCount());
    test_depthFirstSearch(g1, startNode, 'm', true);
    test_depthFirstSearch(g1, startNode, 'x', false);
    test_depthFirstTraverse(g1, startNode, g1.getNodeCount());
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    char endNode = 'l';

    std::cout << '\t' <<std::left << std::setw(50) << "Testing path finding algorithms";
    test_bellmanFordShortestPaths(g1, startNode, 'o', 5, static_cast<unsigned long long>(5));
    test_dijkstraAllShortestPathsFromStart(g1, startNode, 'o', 5, static_cast<unsigned long long>(5));
    test_dijkstraShortestPath(g1, startNode, endNode, 4, static_cast<unsigned long long>(4));
    test_floydWarshallAllShortestPaths(g1, 'h', 'j', static_cast<unsigned long long>(4));
    // johnsonAllShortest paths not supported for unweigted graphs and is not tested here
    test_shortestPathFasterAlgorithm(g1, startNode, 'o', 5, static_cast<unsigned long long>(5));
    // johnson and bellmanford equivalence not tested here
    test_all_path_algs_equivalence(g1);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' <<std::left << std::setw(50) << "Testing articulation point and bridge algorithms";
    test_findArticulationPoints(g1, 3);
    test_findBridges(g1, 3);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' <<std::left << std::setw(50) << "Testing topological sort algorithms";
    // removing some edges before topsort testing because this example graph is not acyclic
        g1.deleteEdge('p', 'a');
        g1.deleteEdge('o', 'p');
        g1.deleteEdge('h', 'b');
        g1.deleteEdge('m', 'h');
        g1.deleteEdge('j', 'i');
        g1.deleteEdge('l', 'h');
    test_allTopsorts(g1, 256995);
    test_dfs_and_khan_topsort(g1);
        g1.addEdge('p', 'a');
        g1.addEdge('o', 'p');
        g1.addEdge('h', 'b');
        g1.addEdge('m', 'h');
        g1.addEdge('j', 'i');
        g1.addEdge('l', 'h');
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    // std::cout << '\t' <<std::left << std::setw(50) << "Testing spanning tree algorithms";
    // MCST algorithms not supported for directed graphs
    // std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' << std::left << std::setw(50) << "Testing component algorithms";
    test_findWeaklyConnectedComponents(g1, 1);
    test_kosarajuFindStronglyConnectedComponents(g1, 3);
    // tarjan biconnected only supported for undirected graphs
    test_tarjanFindStronglyConnectedComponents(g1, 3);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;
    
    std::cout << '\t' <<std::left << std::setw(50) << "Testing cycle algorithms";
    // dfs based algorithm for all cycles not tested for directed graph
    test_johnsonAllCycles(g1, 11);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' <<std::left << std::setw(50) << "Testing flow algorithms";
    test_edmondsKarpMaximumFlow(g1, startNode, 'd', static_cast<unsigned long long>(2));
    test_pushRelabelMaximumFlow(g1, startNode, 'd', static_cast<unsigned long long>(2));
    test_all_maxFlow_algs_equivalence(g1);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' <<std::left << std::setw(50) << "Testing eulerian path and cycle algorithms";
    // changing the graph a bit so eulerian cycles/paths can exist
        g1.deleteEdge('l', 'h');
        g1.deleteEdge('c', 'd');
        g1.deleteEdge('i', 'd');
        g1.deleteEdge('j', 'i');
        g1.deleteNode('k');
    test_hierholzerFindEulerianPath(g1, 'a', 'n');
        g1.addEdge('n', 'a');
    test_hierholzerFindEulerianCycle(g1, 21);
        g1.deleteEdge('n', 'a');
        g1.addEdge('l', 'h');
        g1.addEdge('c', 'd');
        g1.addEdge('i', 'd');
        g1.addEdge('j', 'i');
        g1.addEdge('i', 'k');
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << '\t' <<std::left << std::setw(50) << "Testing other algorithms";
    test_countTriangles(g1, 2);
    test_findIsolatedNodes(g1, 0);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    GraphClasses::Graph<char, unsigned long long> g2;
    g2.configureDirections(GraphClasses::GraphDirections::Directed);
    g2.configureWeights(GraphClasses::GraphWeights::Unweighted);
    const char* fileName2 = "testInputs/char_ull_d_u.txt";
    g2.readFromTxt(fileName2);
    // std::cout << "Node count: " << g2.getNodeCount() << " Edge count: " << g2.getEdgeCount() << " Density: " << g2.getDensity() << std::endl;
    // std::cout << g2 << std::endl;

    // ---- utility testing ----
    std::cout << '\t' <<std::left << std::setw(50) << "Testing utility functions";
    test_complementOfGraph(g1);
    std::unordered_set<char> someNodes{'a', 'c', 'd', 'e', 'i', 'j'};
    test_constructCompleteGraphFromNodes_without_default_weight(someNodes, g1.getGraphDirections());
    test_getSubgraphFromNodes(g1, someNodes, 6, 6);
    test_intersectGraphs(g1, g2);
    test_mergeGraphs(g1, g2);
    test_transitiveClosureOfGraph(g1, g1.getNodeCount(), 226);
    test_transitiveReductionOfGraph(g1);
    test_transposeOfGraph(g1);
    std::cout << std::right << std::setw(10) << "SUCCESS" << std::endl;

    std::cout << "===========================================================================" << std::endl << std::endl;
}

// for quicker callgrind testing
void string_double() {
    GraphClasses::Graph<std::string, double> g;
    g.configureDirections(GraphClasses::GraphDirections::Undirected);
    g.configureWeights(GraphClasses::GraphWeights::Weighted);
    const char* fileName = "testInputs/string_double.txt";
    g.readFromTxt(fileName);

    // std::cout << "Node count: " << g.getNodeCount() << " Edge count: " << g.getEdgeCount() << " Density: " << g.getDensity() << std::endl;
    // std::cout << g << std::endl;

    // std::unordered_set<std::string> someNodes{"node1", "node2", "node5", "node7"};
    // std::string startNode = "node1";
    // std::string endNode = "node6";

    // auto start = std::chrono::high_resolution_clock::now();

    auto ret = GraphAlgorithms::countTriangles(g, GraphAlgorithms::AlgorithmBehavior::PrintAndReturn);

    // auto end = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> diff = end - start;
    // std::cout << "TIME: " << diff.count() << std::endl;
}

void int_int() {
    GraphClasses::Graph<int, int> g;
    g.configureDirections(GraphClasses::GraphDirections::Undirected);
    g.configureWeights(GraphClasses::GraphWeights::Unweighted);
    const char* fileName = "testInputs/int_int_u_u.txt";
    g.readFromTxt(fileName);

    // std::cout << "Node count: " << g.getNodeCount() << " Edge count: " << g.getEdgeCount() << " Density: " << g.getDensity() << std::endl;
    // std::cout << g << std::endl;

    // std::unordered_set<int> someNodes{2, 5, 3, 7};
    // int startNode = 1;
    // int endNode = 8;
    
    // auto start = std::chrono::high_resolution_clock::now();

    auto ret = GraphAlgorithms::countTriangles(g, GraphAlgorithms::AlgorithmBehavior::PrintAndReturn);

    // auto end = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> diff = end - start;
    // std::cout << "TIME: " << diff.count() << std::endl;
}

void custom_float() {
    GraphClasses::Graph<CustomClass, float> g;
    g.configureDirections(GraphClasses::GraphDirections::Directed);
    g.configureWeights(GraphClasses::GraphWeights::Weighted);
    const char* fileName = "testInputs/custom_float.txt";
    g.readFromTxt(fileName);

    // std::cout << "Node count: " << g.getNodeCount() << " Edge count: " << g.getEdgeCount() << " Density: " << g.getDensity() << std::endl;
    // std::cout << g << std::endl;

    // CustomClass startNode = CustomClass(1, 2, 3);
    // CustomClass endNode = CustomClass(2, 2, 2);
    
    // auto start = std::chrono::high_resolution_clock::now();

    auto ret = GraphAlgorithms::countTriangles(g, GraphAlgorithms::AlgorithmBehavior::PrintAndReturn);

    // auto end = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> diff = end - start;
    // std::cout << "TIME: " << diff.count() << std::endl;
}

void char_ull() {
    GraphClasses::Graph<char, unsigned long long> g;
    g.configureDirections(GraphClasses::GraphDirections::Directed);
    g.configureWeights(GraphClasses::GraphWeights::Unweighted);
    const char* fileName = "testInputs/char_ull_d_u.txt";
    g.readFromTxt(fileName);

    // std::cout << "Node count: " << g.getNodeCount() << " Edge count: " << g.getEdgeCount() << " Density: " << g.getDensity() << std::endl;
    // std::cout << g << std::endl;

    // char startNode = 'a';
    // char endNode = 'd';

    // auto start = std::chrono::high_resolution_clock::now();

    auto ret = GraphAlgorithms::countTriangles(g, GraphAlgorithms::AlgorithmBehavior::PrintAndReturn);

    // auto end = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> diff = end - start;
    // std::cout << "TIME: " << diff.count() << std::endl;
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