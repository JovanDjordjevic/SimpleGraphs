#ifndef __SIMPLE_GRAPHS__
#define __SIMPLE_GRAPHS__

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <optional>
#include <queue>
#include <set>
#include <sstream>
#include <stack>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#define GRAPH_ERROR(file, line, message) std::cerr << "ERROR: " << (file) << " line " << (line) << ": " << (message) << std::endl;

namespace GraphAlgorithms {
	enum class AlgorithmBehavior {
		ReturnOnly,
		PrintAndReturn
	};
}

//------------------------------------- LIBRARY CONFIGURATION -------------------------------------

// for constructig Graph without template parameters
using DefaultNodeType = int;
using DefaultWeightType = int;

// for algorithms in GraphAlgorithms namespace
GraphAlgorithms::AlgorithmBehavior defaultBehavior = GraphAlgorithms::AlgorithmBehavior::PrintAndReturn;
std::ostream& defaultOutputStream = std::cout;

// comment this define to save on a small amount of runtime performance only if you are certain that you are using functions correctly
#define CHECK_FOR_ERRORS

//------------------------------------- API -------------------------------------

namespace GraphClasses {
	template<typename WeightType>
	static constexpr WeightType MAX_WEIGHT = std::numeric_limits<WeightType>::max();

	template<typename WeightType>
	static constexpr WeightType MIN_WEIGHT = std::numeric_limits<WeightType>::lowest();

	enum class GraphDirections {
		Unset,
		Directed,
		Undirected
	};

	enum class GraphWeights {
		Unset,
		Weighted,
		Unweighted
	};

	template<typename NodeType, typename WeightType>
	struct Edge {
		public:
			template<typename N, typename W>
			friend bool operator<(const Edge<N, W>& lhs, const Edge<N, W>& rhs);

			template<typename N, typename W>
			friend bool operator==(const Edge<N, W>& lhs, const Edge<N, W>& rhs);

			explicit Edge(const NodeType neighbor, const std::optional<WeightType> weight = {});

		public:
			NodeType neighbor;
			std::optional<WeightType> weight;
	};

	template<typename NodeType = DefaultNodeType, typename WeightType = DefaultWeightType>
	class Graph {
		public:
			template<typename N, typename W>
			friend std::ostream& operator<<(std::ostream& out, const GraphClasses::Graph<N, W>& g);

			explicit Graph(const GraphDirections graphDirections = GraphDirections::Unset, const GraphWeights graphWeights = GraphWeights::Unset);

			void configureDirections(const GraphDirections graphDirections);
			void configureWeights(const GraphWeights graphWeights);

			GraphDirections getGraphDirections() const;
			GraphWeights getGraphWeights() const;

			bool isConfigured() const;

			// only empties the neighborList and does not change configuration of direction and weights
			void clearGraph();

			void readFromTxt(const char* filePath);
			void exportToTxt(const char* filePath) const;

			void addNode(const NodeType node);

			// removes a node and all edges to/from said node
			void deleteNode(const NodeType nodeToDelete);

			// NOTE: only available for unweighted graphs
			void addEdge(const NodeType startNode, const NodeType neighborNode);    

			// NOTE: only available for weighted graphs
			void addEdge(const NodeType startNode, const NodeType neighborNode, const WeightType edgeWeight); 

			void addEdge(const NodeType startNode, const Edge<NodeType, WeightType>& edge);

			// NOTE: only available for weighted graphs
			// in case of parallel edges, all their weights will be set to newWeight
			// in case of undirected graphs, both directions will be reweighed
			void reweighEdge(const NodeType startNode, const NodeType neighborNode, const WeightType newWeight);

			// deletes all edges connecting startNode and endNode
			void deleteEdge(const NodeType startNode, const NodeType endNode);	

			// deletes all edges connecting startNode and endNode with specific weight
			void deleteEdgeWithWeight(const NodeType startNode, const NodeType endNode, const WeightType weight);	
			
			size_t getNodeCount() const;
			size_t getEdgeCount() const;

			std::unordered_set<NodeType> getNodeSet() const;

			// NOTE: degree functions only available for undirected graphs
			size_t getDegreeOfNode(const NodeType node) const;
			std::unordered_map<NodeType, size_t> getDegreesOfNodes() const;

			// NOTE: in/out degree functions only available for directed graphs
			size_t getInDegreeOfNode(const NodeType node) const;
			std::unordered_map<NodeType, size_t> getInDegreesOfNodes() const;
			size_t getOutDegreeOfNode(const NodeType node) const;
			std::unordered_map<NodeType, size_t> getOutDegreesOfNodes() const;

			double getDensity() const;

			// NOTE: eccentricity for weighted graphs is calculated in terms of edge weights and not number of edges on path
			WeightType getEccentricityOfNode(const NodeType node) const;

			std::tuple<WeightType, WeightType, std::unordered_set<NodeType>> getRadiusDiameterAndCenter() const;

			// NOTE: circumference and girth for weighted graphs are calculated in terms of edge weights and not number of edges in a cycle
			std::pair<WeightType, WeightType> getCircumferenceAndGirth() const;

			bool isBiconnected() const;

			std::unordered_map<NodeType, std::vector<Edge<NodeType, WeightType>>> getNeighborList() const;

		private:
			GraphDirections m_graphDirections;
			GraphWeights m_graphWeights;
			std::unordered_map<NodeType, std::vector<Edge<NodeType, WeightType>>> m_neighborList;
	};

} // namespace GraphClasses

namespace GraphUtility {
	// NOTE: only available for unweighted graphs
	template<typename NodeType, typename WeightType>
	GraphClasses::Graph<NodeType, WeightType> complementOfGraph(const GraphClasses::Graph<NodeType, WeightType>& g);

	// NOTE: when constructing a weighted graph this way, both template arguments should be passed or WeightType will be treated as DefaultWeightType 
	// no matter what is passed inside optional as defaultWeight (???)
	template<typename NodeType, typename WeightType = DefaultWeightType>
	GraphClasses::Graph<NodeType, WeightType> constructCompleteGraphFromNodes(const std::unordered_set<NodeType>& nodes, const GraphClasses::GraphDirections graphDirections, 
		const GraphClasses::GraphWeights graphWeights = GraphClasses::GraphWeights::Unweighted, const std::optional<WeightType>& defaultWeight = {});

	template<typename NodeType, typename WeightType>
	GraphClasses::Graph<NodeType, WeightType> getSubgraphFromNodes(const GraphClasses::Graph<NodeType, WeightType>& g, const std::unordered_set<NodeType>& nodes);

	template<typename NodeType, typename WeightType>
	GraphClasses::Graph<NodeType, WeightType> intersectGraphs(const GraphClasses::Graph<NodeType, WeightType>& g1, const GraphClasses::Graph<NodeType, WeightType>& g2);

	template<typename NodeType, typename WeightType>
	GraphClasses::Graph<NodeType, WeightType> mergeGraphs(const GraphClasses::Graph<NodeType, WeightType>& g1, const GraphClasses::Graph<NodeType, WeightType>& g2);
	
	// NOTE: the algorithm assumes any node is reachable from itself and the resulting graph will contain the edge from node to itself for all nodes
	template<typename NodeType, typename WeightType>
	GraphClasses::Graph<NodeType, WeightType> transitiveClosureOfGraph(const GraphClasses::Graph<NodeType, WeightType>& g);
	
	template<typename NodeType, typename WeightType>
	GraphClasses::Graph<NodeType, WeightType> transitiveReductionOfGraph(const GraphClasses::Graph<NodeType, WeightType>& g);

	// NOTE: only available for directed graphs
	template<typename NodeType, typename WeightType>
	GraphClasses::Graph<NodeType, WeightType> transposeOfGraph(const GraphClasses::Graph<NodeType, WeightType>& g);
} // namespace GraphUtility

namespace GraphAlgorithms {
	// ----- traversal algorithms -----

	template<typename NodeType, typename WeightType>
	std::pair<bool, std::vector<NodeType>> breadthFirstSearch(const GraphClasses::Graph<NodeType, WeightType>& g, const NodeType startNode, const NodeType nodeToFind,
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream);

	template<typename NodeType, typename WeightType>
	std::vector<NodeType> breadthFirstTraverse(const GraphClasses::Graph<NodeType, WeightType>& g, const NodeType startNode,
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream);
	
	template<typename NodeType, typename WeightType>
	std::pair<bool, std::vector<NodeType>> depthFirstSearch(const GraphClasses::Graph<NodeType, WeightType>& g, const NodeType startNode, const NodeType nodeToFind,
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream);

	template<typename NodeType, typename WeightType>
	std::vector<NodeType> depthFirstTraverse(const GraphClasses::Graph<NodeType, WeightType>& g, const NodeType startNode,
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream);

	// ----- path finding algorithms -----

	template<typename NodeType, typename WeightType>
	std::unordered_map<NodeType, std::pair<std::vector<NodeType>, WeightType>> bellmanFordShortestPaths(const GraphClasses::Graph<NodeType, WeightType>& g, const NodeType startNode,
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream);

	template<typename NodeType, typename WeightType>
	std::unordered_map<NodeType, std::pair<std::vector<NodeType>, WeightType>> dijkstraAllShortestPathsFromStart(const GraphClasses::Graph<NodeType, WeightType>& g, const NodeType startNode,
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream);

	template<typename NodeType, typename WeightType>
	std::pair<std::vector<NodeType>, WeightType> dijkstraShortestPath(const GraphClasses::Graph<NodeType, WeightType>& g, const NodeType startNode, const NodeType endNode,
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream);

	// NOTE: at this time, Floyd-Warshall algorithm only returns the distances between pairs of nodes and not the paths themselves
	template<typename NodeType, typename WeightType>
	std::unordered_map<NodeType, std::unordered_map<NodeType, WeightType>> floydWarshallAllShortestPaths(const GraphClasses::Graph<NodeType, WeightType>& g,
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream);
	
	// NOTE: only available for weighted graphs
	// NOTE: In order to use this function, NodeType{} must be constructible, and NodeType{} must not be an already existing node in the graph
	// If that is not possible, the value of artificialStartValue must be passed as an argument
	template<typename NodeType, typename WeightType>
	std::unordered_map<NodeType, std::unordered_map<NodeType, std::pair<std::vector<NodeType>, WeightType>>> johnsonAllShortestsPaths(const GraphClasses::Graph<NodeType, WeightType>& g, const std::optional<NodeType> artificialStartValue = {}, 
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream);

	template<typename NodeType, typename WeightType>
	std::unordered_map<NodeType, std::pair<std::vector<NodeType>, WeightType>> shortestPathFasterAlgorithm(const GraphClasses::Graph<NodeType, WeightType>& g, const NodeType startNode,
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream);

	// ----- articulation point and bridge algorithms -----

	// NOTE: uses recursion
	template<typename NodeType, typename WeightType>
	std::unordered_set<NodeType> findArticulationPoints(const GraphClasses::Graph<NodeType, WeightType>& g,
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream);

	// NOTE: uses recursion
	template<typename NodeType, typename WeightType>
	std::vector<std::pair<NodeType, NodeType>> findBridges(const GraphClasses::Graph<NodeType, WeightType>& g,
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream);

	// ----- topological sorting algorithms -----

	// NOTE: only available for directed graphs
	// NOTE: recursive backtracking algorithm, very expensive
	template<typename NodeType, typename WeightType>
	std::vector<std::vector<NodeType>> allTopsorts(const GraphClasses::Graph<NodeType, WeightType>& g, 
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream);

	// NOTE: only available for directed graphs
	// NOTE: uses recursion
	template<typename NodeType, typename WeightType>
	std::vector<NodeType> dfsTopsort(const GraphClasses::Graph<NodeType, WeightType>& g, 
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream);

	// NOTE: only available for directed graphs
	template<typename NodeType, typename WeightType>
	std::vector<NodeType> khanTopsort(const GraphClasses::Graph<NodeType, WeightType>& g, 
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream);

	// ----- spanning tree algorithms -----

	// NOTE: only avilable for undirected graphs
	// NOTE: only works for connected graphs
	template<typename NodeType, typename WeightType>
	std::pair<WeightType, GraphClasses::Graph<NodeType, WeightType>> boruvkaMinimumSpanningTree(const GraphClasses::Graph<NodeType, WeightType>& g,
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream);
	
	// NOTE: only avilable for undirected graphs
	template<typename NodeType, typename WeightType>
	std::pair<WeightType, GraphClasses::Graph<NodeType, WeightType>> commonMinimumCostSpannigTree(const GraphClasses::Graph<NodeType, WeightType>& g1, const GraphClasses::Graph<NodeType, WeightType>& g2,
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream);

	// NOTE: only avilable for undirected graphs
	template<typename NodeType, typename WeightType>
	std::pair<WeightType, GraphClasses::Graph<NodeType, WeightType>> kruskalMinimumSpanningTree(const GraphClasses::Graph<NodeType, WeightType>& g,
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream);

	// NOTE: only avilable for undirected graphs
	// NOTE: only works for connected graphs
	template<typename NodeType, typename WeightType>
	std::pair<WeightType, GraphClasses::Graph<NodeType, WeightType>> primMinimumSpanningTree(const GraphClasses::Graph<NodeType, WeightType>& g,
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream);

	// NOTE: only avilable for undirected graphs
	// NOTE: only works for connected graphs
	template<typename NodeType, typename WeightType>
	std::pair<WeightType, GraphClasses::Graph<NodeType, WeightType>> reverseDeleteMinimumSpanningTree(const GraphClasses::Graph<NodeType, WeightType>& g,
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream);

	// ----- component algorithms -----

	template<typename NodeType, typename WeightType>
	std::vector<std::unordered_set<NodeType>> findWeaklyConnectedComponents(const GraphClasses::Graph<NodeType, WeightType>& g,
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream);

	// NOTE: only avilable for directed graphs
	template<typename NodeType, typename WeightType>
	std::vector<std::unordered_set<NodeType>> kosarajuFindStronglyConnectedComponents(const GraphClasses::Graph<NodeType, WeightType>& g,
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream);
	
	// NOTE: only available for undirected graphs
	// NOTE: uses recursion
	template<typename NodeType, typename WeightType>
	std::vector<std::unordered_set<NodeType>> tarjanFindBiconnectedComponents(const GraphClasses::Graph<NodeType, WeightType>& g,
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream);

	// NOTE: uses recursion
	template<typename NodeType, typename WeightType>
	std::vector<std::unordered_set<NodeType>> tarjanFindStronglyConnectedComponents(const GraphClasses::Graph<NodeType, WeightType>& g,
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream);

	// ----- cycle algorithms -----
	
	// NOTE: only available for undirected graphs
	// NOTE: uses recursion
	// NOTE: if graph has parallel edge cycles they will be ignored
	template<typename NodeType, typename WeightType>
	std::vector<std::pair<std::vector<NodeType>, WeightType>> findAllCycles(const GraphClasses::Graph<NodeType, WeightType>& g,
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream); 

	
	// NOTE: only available for directed graphs
	// NOTE: uses recursion
	template<typename NodeType, typename WeightType>
	std::vector<std::pair<std::vector<NodeType>, WeightType>> johnsonAllCycles(const GraphClasses::Graph<NodeType, WeightType>& g,
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream); 

	// ----- flow algorithms -----

	// NOTE: parallel edges in the same direction will be treated as one new edge with capcaity equal to the sum of parallel edge capacities
	// Edges in unweighted graphs will be treated as having a capacity of 1
	// algorithm returs maximum flow and residual graph
	template<typename NodeType, typename WeightType>
	std::pair<WeightType, GraphClasses::Graph<NodeType, WeightType>> edmondsKarpMaximumFlow(const GraphClasses::Graph<NodeType, WeightType>& g, NodeType source, NodeType sink,
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream);

	// NOTE: parallel edges in the same direction will be treated as one new edge with capcaity equal to the sum of parallel edge capacities
	// Edges in unweighted graphs will be treated as having a capacity of 1
	// algorithm returns maxium flow and graph where edges have weight that enable maximum flow (not residual graph)
	template<typename NodeType, typename WeightType>
	std::pair<WeightType, GraphClasses::Graph<NodeType, WeightType>> pushRelabelMaximumFlow(const GraphClasses::Graph<NodeType, WeightType>& g, NodeType source, NodeType sink,
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream); 

	// ----- other algorithms -----

	template<typename NodeType, typename WeightType>
	std::unordered_set<NodeType> findIsolatedNodes(const GraphClasses::Graph<NodeType, WeightType>& g,
		const AlgorithmBehavior behavior = defaultBehavior, std::ostream& out = defaultOutputStream); 

	// TODO:
	// coloring
	// pairing
	//...
} // namespace GraphAlgorithms

//------------------------------------- IMPLEMENTATION -------------------------------------

// internal namespace for helper funcitons, not inteded for end user
namespace internal {
	template<typename WeightType>
	static constexpr WeightType FLOATING_POINT_EPSIOLON = WeightType{0.000001};

	template<typename WeightType>
	std::enable_if_t<std::is_floating_point_v<WeightType>, bool>
	equals(const WeightType lhs, const WeightType rhs);

	template<typename WeightType>
	std::enable_if_t<!std::is_floating_point_v<WeightType>, bool>
	equals(const WeightType lhs, const WeightType rhs);

	template<typename WeightType>
	std::enable_if_t<std::is_floating_point_v<WeightType>, bool>
	lessThan(const WeightType lhs, const WeightType rhs);

	template<typename WeightType>
	std::enable_if_t<!std::is_floating_point_v<WeightType>, bool>
	lessThan(const WeightType lhs, const WeightType rhs);

	template<typename WeightType>
	std::enable_if_t<std::is_floating_point_v<WeightType>, bool>
	greaterThan(const WeightType lhs, const WeightType rhs);

	template<typename WeightType>
	std::enable_if_t<!std::is_floating_point_v<WeightType>, bool>
	greaterThan(const WeightType lhs, const WeightType rhs);

	template<typename NodeType, typename WeightType>
	void exportDirectedGraph(const GraphClasses::Graph<NodeType, WeightType>& g, const char* filePath);

	template<typename NodeType, typename WeightType>
	void exportUndirectedGraph(const GraphClasses::Graph<NodeType, WeightType>& g, const char* filePath);

	template<typename NodeType, typename WeightType>
	struct EdgeComparator;

	template<typename NodeType, typename WeightType>
	struct EdgeStructHasher;

	template<typename NodeType, typename WeightType>
	struct CompleteEdgeHasher;

	template<typename NodeType, typename WeightType>
	struct ArticulationHelper;

	template<typename NodeType, typename WeightType>
	void articulation__internal(const NodeType startNode, ArticulationHelper<NodeType, WeightType>& internalData);

	template<typename NodeType, typename WeightType>
	struct AllTopsortsHelper;

	template<typename NodeType, typename WeightType>
	void allTopsorts__internal(AllTopsortsHelper<NodeType, WeightType>& internalData);

	template<typename NodeType, typename WeightType>
	struct DFSTopsortHelper;

	template<typename NodeType, typename WeightType>
	bool dfsTopsort__internal (const NodeType currentNode, DFSTopsortHelper<NodeType, WeightType>& internalData);

	template<typename NodeType, typename WeightType>
	class DisjointSet;

	template<typename NodeType, typename WeightType>
	struct TarjanBCHelper;

	template<typename NodeType, typename WeightType>
	void tarjanBC__internal(const NodeType currentNode, TarjanBCHelper<NodeType, WeightType>& internalData);

	template<typename NodeType, typename WeightType>
	struct TarjanSCCHelper;

	template<typename NodeType, typename WeightType>
	void tarjanSCC__internal(const NodeType currentNode, TarjanSCCHelper<NodeType, WeightType>& internalData);

	template<typename NodeType, typename WeightType>
	struct CycleHelper;

	template<typename NodeType, typename WeightType>
	void findAllCycles__internal(const NodeType currentNode, const WeightType currentPathWeight, CycleHelper<NodeType, WeightType>& internalData);

	template<typename NodeType, typename WeightType>
	struct JohnsonAllCyclesHelper;

	template<typename NodeType, typename WeightType>
	bool johnsonCycles__internal(const NodeType currentNode, const WeightType currentCycleWeight, JohnsonAllCyclesHelper<NodeType, WeightType>& internalData);
} // namespace internal

namespace GraphClasses {
	template<typename NodeType, typename WeightType>
	bool operator<(const Edge<NodeType, WeightType>& lhs, const Edge<NodeType, WeightType>& rhs) {
		return internal::lessThan(lhs.neighbor, rhs.neighbor) || internal::lessThan(lhs.weight, rhs.weight);
	}

	template<typename NodeType, typename WeightType>
	bool operator==(const Edge<NodeType, WeightType>& lhs, const Edge<NodeType, WeightType>& rhs) {
		return internal::equals(lhs.neighbor, rhs.neighbor) && internal::equals(lhs.weight, rhs.weight);
	}

	template<typename NodeType, typename WeightType>
	Edge<NodeType, WeightType>::Edge(const NodeType neighbor, const std::optional<WeightType> weight) 
			: neighbor(neighbor), weight(weight) 
	{}

	template<typename NodeType, typename WeightType>
	std::ostream& operator<<(std::ostream& out, const GraphClasses::Graph<NodeType, WeightType>& g) {
		#ifdef CHECK_FOR_ERRORS
			if (!g.isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}
		#endif

		for (auto& [node, neighbors] : g.m_neighborList) {
			out << "Node [" << node << "] has neighbors:\n";

			if (internal::equals(g.m_graphWeights, GraphWeights::Weighted)) {
				for (auto& val : neighbors) {
					out << "|\t [" << val.neighbor << "], edge weight: " << val.weight.value() << '\n';
				}
			} 
			else { // unweighted
				for (auto& val : neighbors) {
					out << "|\t [" << val.neighbor << "]\n";
				}
			}
		}

		out << std::endl;

		return out;
	}

	template<typename NodeType, typename WeightType>
	Graph<NodeType, WeightType>::Graph(const GraphDirections graphDirections, const GraphWeights graphWeights) 
			: m_graphDirections(graphDirections), m_graphWeights(graphWeights) {
		static_assert(!std::is_reference_v<NodeType> && !std::is_pointer_v<NodeType>, "Node type must not be a poitner type or reference");
		static_assert(std::is_arithmetic_v<WeightType> && !std::is_same_v<WeightType, bool>, "Weight type must be an arithmetic type except boolean");
	}

	template<typename NodeType, typename WeightType>
	void Graph<NodeType, WeightType>::configureDirections(const GraphDirections graphDirections) {
		m_graphDirections = graphDirections;

		return;
	}

	template<typename NodeType, typename WeightType>
	void Graph<NodeType, WeightType>::configureWeights(const GraphWeights graphWeights) {
		m_graphWeights = graphWeights;

		return;
	}

	template<typename NodeType, typename WeightType>
	GraphDirections Graph<NodeType, WeightType>::getGraphDirections() const {
		return m_graphDirections;
	}

	template<typename NodeType, typename WeightType>
	GraphWeights Graph<NodeType, WeightType>::getGraphWeights() const {
		return m_graphWeights;
	}

	template<typename NodeType, typename WeightType>
	bool Graph<NodeType, WeightType>::isConfigured() const {
		return !internal::equals(m_graphDirections, GraphDirections::Unset) && !internal::equals(m_graphWeights, GraphWeights::Unset);
	}

	template<typename NodeType, typename WeightType>
	void Graph<NodeType, WeightType>::clearGraph() {
		m_neighborList.clear();

		return;
	}

	template<typename NodeType, typename WeightType>
	void Graph<NodeType, WeightType>::readFromTxt(const char* filePath) {
		#ifdef CHECK_FOR_ERRORS
			if (!isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and weight must be configured before reading from file!");
				exit(EXIT_FAILURE);
			}
		#endif

		clearGraph();

		std::ifstream file(filePath);

		#ifdef CHECK_FOR_ERRORS
			if (!file) {
				GRAPH_ERROR(__FILE__, __LINE__, "Invalid file!");
				exit(EXIT_FAILURE);
			}
		#endif

		NodeType node;
		NodeType neighbor;
		WeightType weight;

		std::string line;

		while (getline(file, line)) {
			std::istringstream lineStream(line);
			lineStream >> node;
			addNode(node); // this line is neccessary becasue of isolated nodes

			if (internal::equals(m_graphWeights, GraphWeights::Weighted)) {
				while (lineStream >> neighbor >> weight) {
					if (internal::equals(m_graphDirections, GraphDirections::Directed)) {
						addEdge(node, neighbor, weight);
					} 
					else { // undirected
						addEdge(node, neighbor, weight);
						addEdge(neighbor, node, weight);
					}
				}
			} 
			else { // unweighted
				while (lineStream >> neighbor) {
					if (internal::equals(m_graphDirections, GraphDirections::Directed)) {
						addEdge(node, neighbor);
					} 
					else { // undirected
						addEdge(node, neighbor);
						addEdge(neighbor, node);
					}
				}
			}
		}

		file.close();

		return;
	}

	template<typename NodeType, typename WeightType>
	void Graph<NodeType, WeightType>::exportToTxt(const char* filePath) const {
		#ifdef CHECK_FOR_ERRORS
			if (!isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Unconfigured graph cannot be exported!");
				exit(EXIT_FAILURE);
			}
		#endif

		if (internal::equals(m_graphDirections, GraphDirections::Directed)) {
			internal::exportDirectedGraph(*this, filePath);
		}
		else {
			internal::exportUndirectedGraph(*this, filePath);
		}

		return;
	}

	template<typename NodeType, typename WeightType>
	void Graph<NodeType, WeightType>::addNode(const NodeType node) {
		m_neighborList[node];

		return;
	}

	template<typename NodeType, typename WeightType>
	void Graph<NodeType, WeightType>::deleteNode(NodeType nodeToDelete) {
		if (internal::equals(m_neighborList.find(nodeToDelete), std::end(m_neighborList))) {
			// std::cout << "Node does not exist" << std::endl;
			return;
		}

		m_neighborList.erase(nodeToDelete);
		
		for (auto& [node, neighbors] : m_neighborList) {
			auto itBegin = std::begin(neighbors);
			auto itEnd = std::end(neighbors);
			auto itRemoved = std::remove_if(itBegin, itEnd, [&](const auto& neighborNode) { return internal::equals(neighborNode.neighbor, nodeToDelete); });
			neighbors.erase(itRemoved, itEnd);
		}

		return;
	}

	template<typename NodeType, typename WeightType>
	void Graph<NodeType, WeightType>::addEdge(const NodeType startNode, const NodeType neighborNode) {
		#ifdef CHECK_FOR_ERRORS
			if (!isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}

			if (internal::equals(m_graphWeights, GraphWeights::Weighted)) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph is weighed and edge weight must be specified");
				exit(EXIT_FAILURE);
			}
		#endif

		// this line is neccessary in case neighbor node is only mentioned as neighbor of another node
		addNode(neighborNode); 

		m_neighborList[startNode].emplace_back(neighborNode);

		return;
	}

	template<typename NodeType, typename WeightType>
	void Graph<NodeType, WeightType>::addEdge(const NodeType startNode, const NodeType neighborNode, const WeightType edgeWeight) {
		#ifdef CHECK_FOR_ERRORS
			if (!isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}

			if (internal::equals(m_graphWeights, GraphWeights::Unweighted)) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph is not weighed but edge weight is passed as an argument");
				exit(EXIT_FAILURE);
			}
		#endif

		// this line is neccessary in case neighbor node is only mentioned as neighbor of another node
		addNode(neighborNode); 

		m_neighborList[startNode].emplace_back(neighborNode, edgeWeight);

		return;
	}

	template<typename NodeType, typename WeightType>
	void Graph<NodeType, WeightType>::addEdge(const NodeType startNode, const Edge<NodeType, WeightType>& edge) {
		#ifdef CHECK_FOR_ERRORS
			if (!isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}

			if (internal::equals(m_graphWeights, GraphWeights::Unweighted) && edge.weight.has_value()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph is unweighed but edge has a weight");
				exit(EXIT_FAILURE);
			} 
			else if (internal::equals(m_graphWeights, GraphWeights::Weighted) && !edge.weight.has_value()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph is weighed but edge has no weight");
				exit(EXIT_FAILURE);
			}
		#endif

		// this line is neccessary in case neighbor node is only mentioned as neighbor of another node
		addNode(edge.neighbor); 

		m_neighborList[startNode].emplace_back(edge.neighbor, edge.weight);

		return;
	}

	template<typename NodeType, typename WeightType>
	void Graph<NodeType, WeightType>::reweighEdge(const NodeType startNode, const NodeType neighborNode, const WeightType newWeight) {
		#ifdef CHECK_FOR_ERRORS
			if (internal::equals(m_graphWeights, GraphWeights::Unweighted)) {
				GRAPH_ERROR(__FILE__, __LINE__, "Function cannot be used for unweighted graphs");
				exit(EXIT_FAILURE);
			}
		#endif

		for (auto& [neighbor, weight] : m_neighborList[startNode]) {
			if (internal::equals(neighborNode, neighbor)) {
				weight = newWeight;
			}
		}

		if (internal::equals(m_graphDirections, GraphDirections::Undirected)) {
			for (auto& [neighbor, weight] : m_neighborList[neighborNode]) {
				if (internal::equals(startNode, neighbor)) {
					weight = newWeight;
				}
			}
		}

		return;
	}

	template<typename NodeType, typename WeightType>
	void Graph<NodeType, WeightType>::deleteEdge(const NodeType startNode, const NodeType endNode) {
		#ifdef CHECK_FOR_ERRORS
			if (!isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}
		#endif

		auto itStartNode = m_neighborList.find(startNode);
		auto itEndNode = m_neighborList.find(endNode);

		if (internal::equals(itStartNode, std::end(m_neighborList)) || internal::equals(itEndNode, std::end(m_neighborList))) {
			// std::cout << "Edge does not exist" << std::endl;
			return;
		}

		auto it = std::begin((*itStartNode).second);
		auto end = std::end((*itStartNode).second);

		auto itRemoved = std::remove_if(it, end, [&](const auto& edge) { return internal::equals(edge.neighbor, endNode); });
		(*itStartNode).second.erase(itRemoved, end);

		if (internal::equals(m_graphDirections, GraphDirections::Undirected)) {
			it = std::begin((*itEndNode).second);
			end = std::end((*itEndNode).second);

			itRemoved = std::remove_if(it, end, [&](const auto& edge) { return internal::equals(edge.neighbor, startNode); });
			(*itEndNode).second.erase(itRemoved, end);
		}

		return;
	}

	template<typename NodeType, typename WeightType>
	void Graph<NodeType, WeightType>::deleteEdgeWithWeight(const NodeType startNode, const NodeType endNode, const WeightType weight) {
		#ifdef CHECK_FOR_ERRORS
			if (!isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}

			if (internal::equals(m_graphWeights, GraphWeights::Unweighted)) {
				GRAPH_ERROR(__FILE__, __LINE__, "Function cannot be used for unweighted graphs");
				exit(EXIT_FAILURE);
			}
		#endif

		auto itStartNode = m_neighborList.find(startNode);
		auto itEndNode = m_neighborList.find(endNode);

		if (internal::equals(itStartNode, std::end(m_neighborList)) || internal::equals(itEndNode, std::end(m_neighborList))) {
			// std::cout << "Edge does not exist" << std::endl;
			return;
		}

		auto it = std::begin((*itStartNode).second);
		auto end = std::end((*itStartNode).second);

		auto itRemoved = std::remove_if(it, end, [&](const auto& edge) { return internal::equals(edge.neighbor, endNode) && internal::equals(edge.weight.value(), weight); });
		(*itStartNode).second.erase(itRemoved, end);

		if (internal::equals(m_graphDirections, GraphDirections::Undirected)) {
			it = std::begin((*itEndNode).second);
			end = std::end((*itEndNode).second);

			itRemoved = std::remove_if(it, end, [&](const auto& edge) { return internal::equals(edge.neighbor, startNode) && internal::equals(edge.weight.value(), weight); });
			(*itEndNode).second.erase(itRemoved, end);
		}

		return;
	}

	template<typename NodeType, typename WeightType>
	size_t Graph<NodeType, WeightType>::getNodeCount() const {
		return m_neighborList.size();
	}

	template<typename NodeType, typename WeightType>
	size_t Graph<NodeType, WeightType>::getEdgeCount() const {
		size_t count = static_cast<size_t>(0);

		for (auto& [_, neighbors] : m_neighborList) {
			count += neighbors.size();
		}

		return count;
	}

	template<typename NodeType, typename WeightType>
	std::unordered_set<NodeType> Graph<NodeType, WeightType>::getNodeSet() const {
		std::unordered_set<NodeType> nodeSet;

		for (auto& [node, _] : m_neighborList) {
			nodeSet.emplace(node);
		}

		return nodeSet;
	}

	template<typename NodeType, typename WeightType>
	size_t Graph<NodeType, WeightType>::getDegreeOfNode(const NodeType node) const {
		#ifdef CHECK_FOR_ERRORS
			if (!isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}

			if (!internal::equals(m_graphDirections, GraphDirections::Undirected)) {
				GRAPH_ERROR(__FILE__, __LINE__, "Use getter fucntions for in/out degrees for directed graphs");
				exit(EXIT_FAILURE);
			}
		#endif

		return m_neighborList.at(node).size();
	}

	template<typename NodeType, typename WeightType>
	std::unordered_map<NodeType, size_t> Graph<NodeType, WeightType>::getDegreesOfNodes() const {
		#ifdef CHECK_FOR_ERRORS
			if (!isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}

			if (!internal::equals(m_graphDirections, GraphDirections::Undirected)) {
				GRAPH_ERROR(__FILE__, __LINE__, "Use getter fucntions for in/out degrees for directed graphs");
				exit(EXIT_FAILURE);
			}
		#endif
		
		std::unordered_map<NodeType, size_t> degrees;

		for (auto& [node, neighbors] : m_neighborList) {
			degrees[node] = neighbors.size();
		}

		return degrees;
	}

	template<typename NodeType, typename WeightType>
	size_t Graph<NodeType, WeightType>::getInDegreeOfNode(const NodeType node) const {
		#ifdef CHECK_FOR_ERRORS
			if (!isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}

			if (!internal::equals(m_graphDirections, GraphDirections::Directed)) {
				GRAPH_ERROR(__FILE__, __LINE__, "Use regular getter for degrees for undirected graphs");
				exit(EXIT_FAILURE);
			}
		#endif

		size_t inDegree = static_cast<size_t>(0);

		for (auto& [_, neighbors] : m_neighborList) {
			for (auto& [neighbor, __] : neighbors) {
				if (internal::equals(neighbor, node)) {
					++inDegree;
				}
			}
		}

		return inDegree;
	}

	template<typename NodeType, typename WeightType>
	std::unordered_map<NodeType, size_t> Graph<NodeType, WeightType>::getInDegreesOfNodes() const {
		#ifdef CHECK_FOR_ERRORS
			if (!isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}

			if (!internal::equals(m_graphDirections, GraphDirections::Directed)) {
				GRAPH_ERROR(__FILE__, __LINE__, "Use regular getter for degrees for undirected graphs");
				exit(EXIT_FAILURE);
			}
		#endif

		std::unordered_map<NodeType, size_t> inDegrees;

		for (auto& [node, _] : m_neighborList) {
			inDegrees[node] = static_cast<size_t>(0);
		}

		for (auto& [_, neighbors] : m_neighborList) {
			for (auto& [neighbor, __] : neighbors) {
				++inDegrees[neighbor];
			}
		}

		return inDegrees;
	}

	template<typename NodeType, typename WeightType>
	size_t Graph<NodeType, WeightType>::getOutDegreeOfNode(const NodeType node) const {
		#ifdef CHECK_FOR_ERRORS
			if (!isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}

			if (!internal::equals(m_graphDirections, GraphDirections::Directed)) {
				GRAPH_ERROR(__FILE__, __LINE__, "Use regula getters for degrees for undirected graphs");
				exit(EXIT_FAILURE);
			}
		#endif

		return m_neighborList.at(node).size();
	}

	template<typename NodeType, typename WeightType>
	std::unordered_map<NodeType, size_t> Graph<NodeType, WeightType>::getOutDegreesOfNodes() const {
		#ifdef CHECK_FOR_ERRORS
			if (!isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}

			if (!internal::equals(m_graphDirections, GraphDirections::Directed)) {
				GRAPH_ERROR(__FILE__, __LINE__, "Use regular getters for degrees for undirected graphs");
				exit(EXIT_FAILURE);
			}
		#endif

		std::unordered_map<NodeType, size_t> outDegrees;

		for (auto& [node, neighbors] : m_neighborList) {
			outDegrees[node] = neighbors.size();
		}

		return outDegrees;
	}

	template<typename NodeType, typename WeightType>
	double Graph<NodeType, WeightType>::getDensity() const {
		#ifdef CHECK_FOR_ERRORS
			if (!isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}
		#endif

		auto nodeCount = getNodeCount();

		double density = static_cast<double>(getEdgeCount()) / static_cast<double>(nodeCount * (nodeCount - static_cast<size_t>(1)));

		if (internal::equals(m_graphDirections, GraphDirections::Undirected)) {
			density *= static_cast<double>(2);
		}

		return density;
	}

	template<typename NodeType, typename WeightType>
	WeightType Graph<NodeType, WeightType>::getEccentricityOfNode(const NodeType node) const {
		auto shortestPaths = GraphAlgorithms::bellmanFordShortestPaths(*this, node, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
		
		// if graph has a negative cycle
		if (internal::equals(shortestPaths.size(), static_cast<size_t>(0))) {
			return MAX_WEIGHT<WeightType>;
		}

		WeightType eccentricity = MIN_WEIGHT<WeightType>;

		for (auto& [_, pathData] : shortestPaths) {
			if (internal::greaterThan(pathData.second, eccentricity)) {
				eccentricity = pathData.second;
			}
		}

		return eccentricity;
	}

	template<typename NodeType, typename WeightType>
	std::tuple<WeightType, WeightType, std::unordered_set<NodeType>> Graph<NodeType, WeightType>::getRadiusDiameterAndCenter() const {
		auto allShortestPaths = GraphAlgorithms::floydWarshallAllShortestPaths(*this, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);

		// if graph has a negative cycle
		if (internal::equals(allShortestPaths.size(), static_cast<size_t>(0))) {
			return std::make_tuple(MAX_WEIGHT<WeightType>, MAX_WEIGHT<WeightType>, std::unordered_set<NodeType>{});
		}

		WeightType radius = MAX_WEIGHT<WeightType>;
		WeightType diameter = MIN_WEIGHT<WeightType>;
		std::unordered_set<NodeType> center;

		for (auto& [startNode, pathMap] : allShortestPaths) {
			WeightType eccStartNode = MIN_WEIGHT<WeightType>;

			for (auto& [_, weight] : pathMap) {
				if (internal::greaterThan(weight, eccStartNode)) {
					eccStartNode = weight;
				}
			}

			if (internal::greaterThan(eccStartNode, diameter)) {
				diameter = eccStartNode;
			}

			if (internal::lessThan(eccStartNode, radius)) {
				radius = eccStartNode;
				center.clear();
			}

			if (internal::equals(eccStartNode, radius)) {
				center.emplace(startNode);
			}
		}

		return std::make_tuple(radius, diameter, center);
	}

	template<typename NodeType, typename WeightType>
	std::pair<WeightType, WeightType> Graph<NodeType, WeightType>::getCircumferenceAndGirth() const {
		std::vector<std::pair<std::vector<NodeType>, WeightType>> allCycles;
		
		if (internal::equals(m_graphDirections, GraphDirections::Undirected)) {
			allCycles = GraphAlgorithms::findAllCycles(*this, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
		}
		else { // directed
			allCycles = GraphAlgorithms::johnsonAllCycles(*this, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
		}

		if (internal::equals(allCycles.size(), static_cast<size_t>(0))) {
			return std::make_pair(MAX_WEIGHT<WeightType>, MAX_WEIGHT<WeightType>);
		}
		else {
			WeightType circumference = MIN_WEIGHT<WeightType>;
			WeightType girth = MAX_WEIGHT<WeightType>;

			for (auto& [cycle, cycleWeight] : allCycles) {
				if (internal::greaterThan(cycleWeight, circumference)) {
					circumference = cycleWeight;
				}

				if (internal::lessThan(cycleWeight, girth)) {
					girth = cycleWeight;
				}
			}

			return std::make_pair(circumference, girth);
		}	
	}

	template<typename NodeType, typename WeightType>
	bool Graph<NodeType, WeightType>::isBiconnected() const{	
		#ifdef CHECK_FOR_ERRORS
			if (!isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}

			if (internal::equals(getGraphDirections(), GraphClasses::GraphDirections::Directed)) {
				GRAPH_ERROR(__FILE__, __LINE__, "This funcion can only be called for undirected graphs");
				exit(EXIT_FAILURE);
			}
		#endif

		auto traversal = GraphAlgorithms::depthFirstTraverse(*this, (*std::begin(m_neighborList)).first, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);

		if (!internal::equals(getNodeCount(), traversal.size())) {
			return false;
		}

		auto articulationPoints = GraphAlgorithms::findArticulationPoints(*this, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);

		if (!internal::equals(articulationPoints.size(), static_cast<size_t>(0))) {
			return false;
		}

		return true;
	}

	template<typename NodeType, typename WeightType>
	std::unordered_map<NodeType, std::vector<Edge<NodeType, WeightType>>> Graph<NodeType, WeightType>::getNeighborList() const {
		return m_neighborList;
	}
} // namespace GraphClasses

namespace GraphUtility {
	template<typename NodeType, typename WeightType>
	GraphClasses::Graph<NodeType, WeightType> complementOfGraph(const GraphClasses::Graph<NodeType, WeightType>& g) {
		#ifdef CHECK_FOR_ERRORS
			if (!g.isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}
			
			if (internal::equals(g.getGraphWeights(), GraphClasses::GraphWeights::Weighted)) {
				GRAPH_ERROR(__FILE__, __LINE__, "Finding complement of weighted graph not supported!");
				exit(EXIT_FAILURE);
			}
		#endif

		GraphClasses::Graph<NodeType, WeightType> newGraph(g.getGraphDirections(), g.getGraphWeights());

		auto neighborList = g.getNeighborList();

		for (auto& [startNode, _] : neighborList) {
			for (auto& [endNode, __] : neighborList) {
				if (!internal::equals(startNode, endNode)) {
					newGraph.addEdge(startNode, endNode);
				}
			}
		}

		for (auto& [node, neighbors] : neighborList) {
			for (auto& [neighbor, _] : neighbors) {
				newGraph.deleteEdge(node, neighbor);
			}
		}

		return newGraph;
	}

	template<typename NodeType, typename WeightType>
	GraphClasses::Graph<NodeType, WeightType> constructCompleteGraphFromNodes(const std::unordered_set<NodeType>& nodes, const GraphClasses::GraphDirections graphDirections, const GraphClasses::GraphWeights graphWeights, const std::optional<WeightType>& defaultWeight) {
		#ifdef CHECK_FOR_ERRORS
			if (internal::equals(graphWeights, GraphClasses::GraphWeights::Unweighted)) {
				if (defaultWeight.has_value()) {
					GRAPH_ERROR(__FILE__, __LINE__, "Default edge weight must not be specified when creating an unweighted graph");
					exit(EXIT_FAILURE);
				}
			}
			else {	// weighted
				if (!defaultWeight.has_value()) {
					GRAPH_ERROR(__FILE__, __LINE__, "Default edge weight must be specified when creating a weighted graph");
					exit(EXIT_FAILURE);
				}
			
			}
		#endif

		GraphClasses::Graph<NodeType, WeightType> newGraph(graphDirections, graphWeights);

		for (auto& startNode : nodes) {
			for (auto& endNode : nodes) {
				if (!internal::equals(startNode, endNode)) {
					newGraph.addEdge(startNode, GraphClasses::Edge(endNode, defaultWeight));
				}
			}
		}

		return newGraph;
	}

	template<typename NodeType, typename WeightType>
	GraphClasses::Graph<NodeType, WeightType> getSubgraphFromNodes(const GraphClasses::Graph<NodeType, WeightType>& g, const std::unordered_set<NodeType>& nodes) {
		GraphClasses::Graph<NodeType, WeightType> newGraph(g.getGraphDirections(), g.getGraphWeights());

		auto neighborList = g.getNeighborList();

		for (auto& node : nodes) {
			newGraph.addNode(node);
			
			for (auto& edge : neighborList[node]) {
				if (!internal::equals(nodes.count(edge.neighbor), static_cast<size_t>(0))) {
					newGraph.addEdge(node, edge);
				}
			}
		}

		return newGraph;
	}

	template<typename NodeType, typename WeightType>
	GraphClasses::Graph<NodeType, WeightType> intersectGraphs(const GraphClasses::Graph<NodeType, WeightType>& g1, const GraphClasses::Graph<NodeType, WeightType>& g2) {
		#ifdef CHECK_FOR_ERRORS
			if (!g1.isConfigured() || !g2.isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}	

			if (!internal::equals(g1.getGraphDirections(), g2.getGraphDirections()) || !internal::equals(g1.getGraphWeights(), g2.getGraphWeights())) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph intersection can only be created if they have the same type (directed/undirected) and same weights (weighed/unweighed)!");
				exit(EXIT_FAILURE);
			}
		#endif

		GraphClasses::Graph<NodeType, WeightType> newGraph(g1.getGraphDirections(), g1.getGraphWeights());

		auto g1NeighborList = g1.getNeighborList();
		auto g2NeighborList = g2.getNeighborList();

		std::unordered_set<GraphClasses::Edge<NodeType, WeightType>, internal::EdgeStructHasher<NodeType, WeightType>> edges;

		for (auto& [node, _] : g1NeighborList) {
			auto it = g2NeighborList.find(node);

			if (!internal::equals(it, std::end(g2NeighborList))) {
				newGraph.addNode(node);

				auto& shorter = g1NeighborList[node];
				auto& longer = g2NeighborList[node];

				if (internal::lessThan(longer.size(), shorter.size())) {
					std::swap(shorter, longer);
				}

				for (auto& edge : shorter) {
					edges.emplace(edge.neighbor, edge.weight);
				}

				for (auto& edge : longer) {
					if (!internal::equals(edges.count(edge), static_cast<size_t>(0))) {
						newGraph.addEdge(node, edge);
					}
				}

				edges.clear();
			}
		}

		return newGraph;
	}

	template<typename NodeType, typename WeightType>
	GraphClasses::Graph<NodeType, WeightType> mergeGraphs(const GraphClasses::Graph<NodeType, WeightType>& g1, const GraphClasses::Graph<NodeType, WeightType>& g2) {
		#ifdef CHECK_FOR_ERRORS
			if (!g1.isConfigured() || !g2.isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}

			if (!internal::equals(g1.getGraphDirections(), g2.getGraphDirections()) || !internal::equals(g1.getGraphWeights(), g2.getGraphWeights())) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graphs can only be merged if they have the same type (directed/undirected) and same weights (weighed/unweighed)!");
				exit(EXIT_FAILURE);
			}
		#endif

		GraphClasses::Graph<NodeType, WeightType> newGraph(g1.getGraphDirections(), g1.getGraphWeights());

		auto g1NeighborList = g1.getNeighborList();
		auto g2NeighborList = g2.getNeighborList();

		// adding duplicate edges is avoided by putting them in a set first
		std::unordered_set<std::pair<NodeType, GraphClasses::Edge<NodeType, WeightType>>, internal::CompleteEdgeHasher<NodeType, WeightType>> edgeSet;

		for (auto& [node, neighbors] : g1NeighborList) {
			// check needed in case of isolated nodes
			if (internal::equals(neighbors.size(), static_cast<size_t>(0))) {
				newGraph.addNode(node);
				continue;
			}
			
			for (auto& edge : neighbors) {
				edgeSet.emplace(node, edge);
			}
		}

		for (auto& [node, neighbors] : g2NeighborList) {
			// check needed in case of isolated nodes
			if (internal::equals(neighbors.size(), static_cast<size_t>(0))) {
				newGraph.addNode(node);
				continue;
			}

			for (auto& edge : neighbors) {
				edgeSet.emplace(node, edge);
			}
		}

		for (auto& edge : edgeSet) {
			newGraph.addEdge(edge.first, edge.second);
		}

		return newGraph;
	}

	template<typename NodeType, typename WeightType>
	GraphClasses::Graph<NodeType, WeightType> transitiveClosureOfGraph(const GraphClasses::Graph<NodeType, WeightType>& g) {
		GraphClasses::Graph<NodeType, WeightType> closure(GraphClasses::GraphDirections::Directed, GraphClasses::GraphWeights::Unweighted);

		auto neighborList = g.getNeighborList();

		std::unordered_map<NodeType, std::unordered_map<NodeType, bool>> reachMatrix;
		
		for (auto& [node1, _] : neighborList) {
			for (auto& [node2, __] : neighborList) {
				reachMatrix[node1][node2] = false;
			}
		}

		for (auto& [node, neighbors] : neighborList) {
			reachMatrix[node][node] = true;

			for (auto& [neighbor, _] : neighbors) {
				reachMatrix[node][neighbor] = true;
			}
		}

		for (auto& [mid, _] : neighborList) {
			for (auto& [start, __] : neighborList) {
				if (reachMatrix[start][mid]) {
					for (auto& [end, ___] : neighborList) {
						if (reachMatrix[mid][end]) {
							reachMatrix[start][end] = true;
						}
					}
				}
			}
		}

		for (auto& [node1, node2List] : reachMatrix) {
			for (auto& [node2, canReach] : node2List) {
				if (canReach) {
					closure.addEdge(node1, node2);
				}
			}
		}

		return closure;
	}

	template<typename NodeType, typename WeightType>
	GraphClasses::Graph<NodeType, WeightType> transitiveReductionOfGraph(const GraphClasses::Graph<NodeType, WeightType>& g) {
		GraphClasses::Graph<NodeType, WeightType> reduction(GraphClasses::GraphDirections::Directed, GraphClasses::GraphWeights::Unweighted);

		auto neighborList = g.getNeighborList();

		std::unordered_map<NodeType, std::unordered_map<NodeType, bool>> reachMatrix;
		
		for (auto& [node1, _] : neighborList) {
			for (auto& [node2, __] : neighborList) {
				reachMatrix[node1][node2] = false;
			}
		}

		for (auto& [node, neighbors] : neighborList) {
			for (auto& [neighbor, _] : neighbors) {
				reachMatrix[node][neighbor] = true;
			}
		}

		for (auto& [mid, _] : neighborList) {
			for (auto& [start, __] : neighborList) {
				if (reachMatrix[start][mid]) {
					for (auto& [end, ___] : neighborList) {
						if (reachMatrix[mid][end]) {
							reachMatrix[start][end] = false;
						}
					}
				}
			}
		}

		for (auto& [node1, node2List] : reachMatrix) {
			for (auto& [node2, canReach] : node2List) {
				if (canReach) {
					reduction.addEdge(node1, node2);
				}
			}
		}

		return reduction;
	}

	template<typename NodeType, typename WeightType>
	GraphClasses::Graph<NodeType, WeightType> transposeOfGraph(const GraphClasses::Graph<NodeType, WeightType>& g) {
		#ifdef CHECK_FOR_ERRORS
			if (!g.isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}

			if (internal::equals(g.getGraphDirections(), GraphClasses::GraphDirections::Undirected)) {
				GRAPH_ERROR(__FILE__, __LINE__, "Transposing makes no sense for undirected graphs!");
				exit(EXIT_FAILURE);
			}
		#endif

		auto newGraphDirections = g.getGraphDirections();
		auto newGraphWeights = g.getGraphWeights();

		GraphClasses::Graph<NodeType, WeightType> newGraph(newGraphDirections, newGraphWeights);

		auto neighborList = g.getNeighborList();

		for (auto& [node, neighbors] : neighborList) {
			// needed so that isolated nodes will remain in the transposed graph
			if (internal::equals(neighbors.size(), static_cast<size_t>(0))) {
				newGraph.addNode(node);
				continue;
			}

			for (auto& [neighbor, weight] : neighbors) {
				if (internal::equals(newGraphWeights, GraphClasses::GraphWeights::Weighted)) {
					newGraph.addEdge(neighbor, node, weight.value());

					if (internal::equals(newGraphDirections, GraphClasses::GraphDirections::Undirected)) {
						newGraph.addEdge(node, neighbor, weight.value());
					} 
				}
				else { // unweighted
					newGraph.addEdge(neighbor, node);

					if (internal::equals(newGraphDirections, GraphClasses::GraphDirections::Undirected)) {
						newGraph.addEdge(node, neighbor);
					} 
				}
			}
		}

		return newGraph;
	}
} // namespace GraphUtility

namespace GraphAlgorithms {
	// ----- traversal algorithms -----
	
	template<typename NodeType, typename WeightType>
	std::pair<bool, std::vector<NodeType>> breadthFirstSearch(const GraphClasses::Graph<NodeType, WeightType>& g, const NodeType startNode, const NodeType nodeToFind, const AlgorithmBehavior behavior, std::ostream& out) {
		std::unordered_map<NodeType, bool> visited;
		std::vector<NodeType> traversalOrder;

		auto neighborList = g.getNeighborList();

		for (auto& [node, _] : neighborList) {
			visited[node] = false;
		}

		std::queue<NodeType> queue;
		queue.emplace(startNode);

		NodeType currentNode;
		while (!queue.empty()) {
			currentNode = queue.front();
			queue.pop();
			
			auto& ifVisited = visited[currentNode];
			if (!ifVisited) {
				ifVisited = true;
				traversalOrder.emplace_back(currentNode);
			}

			if (internal::equals(currentNode, nodeToFind)) {
				if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
					out << "Node [" << nodeToFind << "] found\n" << std::endl;
				}
				return std::make_pair(true, traversalOrder);
			}

			auto& currentNeighbors = neighborList[currentNode];
			for (auto& [neighbor, _] : currentNeighbors) {
				if (!visited[neighbor]) {
					queue.emplace(neighbor);
				}
			}
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			out << "Node [" << nodeToFind << "] not found\n" << std::endl;
		}

		return std::make_pair(false, traversalOrder);
	}

	template<typename NodeType, typename WeightType>
	std::vector<NodeType> breadthFirstTraverse(const GraphClasses::Graph<NodeType, WeightType>& g, const NodeType startNode, const AlgorithmBehavior behavior, std::ostream& out) {
		std::unordered_map<NodeType, bool> visited;
		std::vector<NodeType> traversalOrder;

		auto neighborList = g.getNeighborList();

		for (auto& [node, _] : neighborList) {
			visited[node] = false;
		}

		std::queue<NodeType> queue;
		queue.emplace(startNode);

		NodeType currentNode;
		while (!queue.empty()) {
			currentNode = queue.front();
			queue.pop();

			auto& ifVisited = visited[currentNode];
			if (!ifVisited) {
				ifVisited = true;
				traversalOrder.emplace_back(currentNode);
			}

			auto& currentNeighbors = neighborList[currentNode];
			for (auto& [neighbor, _] : currentNeighbors) {
				if (!visited[neighbor]) {
					queue.emplace(neighbor);
				}
			}
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			out << "Order of breadth first traversal:\n\t";

			for (auto& node : traversalOrder) {
				out << "[" << node << "] ";
			}

			out << '\n' << std::endl;
		}

		return traversalOrder;
	}

	template<typename NodeType, typename WeightType>
	std::pair<bool, std::vector<NodeType>> depthFirstSearch(const GraphClasses::Graph<NodeType, WeightType>& g, const NodeType startNode, const NodeType nodeToFind, const AlgorithmBehavior behavior, std::ostream& out) {
		std::unordered_map<NodeType, bool> visited;
		std::vector<NodeType> traversalOrder;

		auto neighborList = g.getNeighborList();

		for (auto& [node, _] : neighborList) {
			visited[node] = false;
		}

		std::stack<NodeType> stack;
		stack.emplace(startNode);

		NodeType currentNode;
		while (!stack.empty()) {
			currentNode = stack.top();
			stack.pop();

			auto& ifVisited = visited[currentNode];
			if (!ifVisited) {
				ifVisited = true;
				traversalOrder.emplace_back(currentNode);
			}

			if (internal::equals(currentNode, nodeToFind)) {
				if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
					out << "Node [" << nodeToFind << "] found\n" << std::endl;
				}

				return std::make_pair(true, traversalOrder);
			}

			auto& currentNeighbors = neighborList[currentNode];
			for (auto& [neighbor, _] : currentNeighbors) {
				if (!visited[neighbor]) {
					stack.emplace(neighbor);
				}
			}
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			out << "Node [" << nodeToFind << "] not found\n" << std::endl;
		}

		return std::make_pair(false, traversalOrder);
	}

	template<typename NodeType, typename WeightType>
	std::vector<NodeType> depthFirstTraverse(const GraphClasses::Graph<NodeType, WeightType>& g, const NodeType startNode, const AlgorithmBehavior behavior, std::ostream& out) {
		std::unordered_map<NodeType, bool> visited;
		std::vector<NodeType> traversalOrder;

		auto neighborList = g.getNeighborList();

		for (auto& [node, _] : neighborList) {
			visited[node] = false;
		}

		std::stack<NodeType> stack;
		stack.emplace(startNode);

		NodeType currentNode;
		while (!stack.empty()) {
			currentNode = stack.top();
			stack.pop();

			auto& ifVisited = visited[currentNode];
			if (!ifVisited) {
				ifVisited = true;
				traversalOrder.emplace_back(currentNode);
			}

			auto& currentNeighbors = neighborList[currentNode];
			for (auto& [neighbor, _] : currentNeighbors) {
				if (!visited[neighbor]) {
					stack.emplace(neighbor);
				}
			}
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			out << "Order of depth first traversal:\n\t";

			for (auto& node : traversalOrder) {
				out << "[" << node << "] ";
			}

			out << '\n' << std::endl;
		}

		return traversalOrder;
	}

	// ----- path finding algorithms -----

	template<typename NodeType, typename WeightType>
	std::unordered_map<NodeType, std::pair<std::vector<NodeType>, WeightType>> bellmanFordShortestPaths(const GraphClasses::Graph<NodeType, WeightType>& g, const NodeType startNode, const AlgorithmBehavior behavior, std::ostream& out) {
		std::unordered_map<NodeType, WeightType> distances;
		std::unordered_map<NodeType, std::optional<NodeType>> parents;

		auto neighborList = g.getNeighborList();

		for (auto& [node, _] : neighborList) {
			distances[node] = GraphClasses::MAX_WEIGHT<WeightType>;
		}

		distances[startNode] = static_cast<WeightType>(0);
		parents[startNode]; // only startNode will have the empty optional

		size_t relaxationCount = g.getNodeCount() - static_cast<size_t>(1);

		for (size_t r = static_cast<size_t>(0); internal::lessThan(r, relaxationCount); ++r) {
			for (auto& [node, neighbors] : neighborList) {
				if (!internal::equals(distances[node], GraphClasses::MAX_WEIGHT<WeightType>)) {
					for (auto& [neighbor, weight] : neighbors) {
						WeightType newDistnce = distances[node] + weight.value_or(static_cast<WeightType>(1));
					
						auto& neighborDist = distances[neighbor];

						if (internal::lessThan(newDistnce, neighborDist)) {
							neighborDist = newDistnce;
							parents[neighbor] = node;
						}
					}
				}
			}
		}

		// negtive cycle check
		for (auto& [node, neighbors] : neighborList) {
			for (auto& [neighbor, weight] : neighbors) {
				auto& nodeDist = distances[node];

				if (!internal::equals(nodeDist, GraphClasses::MAX_WEIGHT<WeightType>) &&
					internal::lessThan(nodeDist + weight.value_or(static_cast<WeightType>(1)), distances[neighbor])) {
					if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
						out << "Graph contains one or more negative cycles\n" << std::endl;
					}

					return {};
				}
			}
		}

		// path reconstruction
		std::unordered_map<NodeType, std::pair<std::vector<NodeType>, WeightType>> paths;

		for (auto& [node, distFromStart] : distances) {
			paths[node] = std::make_pair(std::vector<NodeType>{}, distFromStart);

			if (internal::equals(distFromStart, GraphClasses::MAX_WEIGHT<WeightType>) || internal::equals(node, startNode)) {
				continue;
			}

			NodeType pathNode = node;

			auto& pathVect = paths[node].first;
			pathVect.emplace_back(pathNode);

			std::optional<NodeType> parent = parents[pathNode];

			while (true) {
				if (!parent.has_value()) {
					break;
				}

				auto& parentVal = parent.value();

				pathVect.emplace_back(parentVal);
				pathNode = parentVal;
				
				parent = parents[pathNode];
			}

			std::reverse(std::begin(pathVect), std::end(pathVect));
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			for (auto& [node, pathAndDist] : paths) {
				// there is no path to nodes in different components
				if (internal::equals(pathAndDist.first.size(), static_cast<size_t>(0)) && !internal::equals(node, startNode)) {
					out << "There is no possible path between [" << startNode << "] and [" << node << "]\n" << std::endl;

					continue;
				}

				out << "Distance from [" << startNode << "] to [" << node << "] is: " << distances[node] << "\n\t Path: ";

				auto& pathVect = pathAndDist.first;

				for (auto& elem : pathVect) {
					out << "[" << elem << "] ";
				}

				out << '\n' << std::endl;
			}
		}

		return paths;
	}

	template<typename NodeType, typename WeightType>
	std::unordered_map<NodeType, std::pair<std::vector<NodeType>, WeightType>> dijkstraAllShortestPathsFromStart(const GraphClasses::Graph<NodeType, WeightType>& g, const NodeType startNode, const AlgorithmBehavior behavior, std::ostream& out) {
		std::unordered_map<NodeType, bool> visited;
		std::unordered_map<NodeType, WeightType> distances;
		std::unordered_map<NodeType, std::optional<NodeType>> parents;

		auto neighborList = g.getNeighborList();

		using pqData = GraphClasses::Edge<NodeType, WeightType>;
		std::priority_queue<pqData, std::vector<pqData>, internal::EdgeComparator<NodeType, WeightType>> pq;

		for (auto& [node, _] : neighborList) {
			distances[node] = GraphClasses::MAX_WEIGHT<WeightType>;
			pq.emplace(node, GraphClasses::MAX_WEIGHT<WeightType>);
			visited[node] = false;
		}

		pq.emplace(startNode, static_cast<WeightType>(0));

		distances[startNode] = static_cast<WeightType>(0);
		parents[startNode]; // only startNode will have the empty optional

		size_t numToVisit = g.getNodeCount();
		size_t numVisited = 0;

		while (!pq.empty()) {
			auto [currentNode, distToCurrentNode] = pq.top();
			pq.pop();

			if (internal::equals(numVisited, numToVisit) || internal::equals(distToCurrentNode.value(), GraphClasses::MAX_WEIGHT<WeightType>)) {
				break;
			}

			if (!visited[currentNode]) {
				visited[currentNode] = true;
				++numVisited;
				
				for (auto& [neighbor, weight] : neighborList[currentNode]) {
					if (!visited[neighbor]) {
						WeightType oldDistance = distances[neighbor];
						WeightType newDistance = distances[currentNode] + weight.value_or(static_cast<WeightType>(1));

						if (internal::lessThan(newDistance, oldDistance)) {	
							pq.emplace(neighbor, newDistance);
							distances[neighbor] = newDistance;
							parents[neighbor] = currentNode;
						}
					}
				}
			}
		}
		
		// path reconstruction
		std::unordered_map<NodeType, std::pair<std::vector<NodeType>, WeightType>> paths;

		for (auto& [node, distFromStart] : distances) {
			paths[node] = std::make_pair(std::vector<NodeType>{}, distFromStart);

			if (internal::equals(distFromStart, GraphClasses::MAX_WEIGHT<WeightType>) || internal::equals(node, startNode)) {
				continue;
			}

			NodeType pathNode = node;

			auto& pathVect = paths[node].first;
			pathVect.emplace_back(pathNode);

			std::optional<NodeType> parent = parents[pathNode];

			while (true) {
				if (!parent.has_value()) {
					break;
				}

				auto& parentVal = parent.value();

				pathVect.emplace_back(parentVal);
				pathNode = parentVal;
				
				parent = parents[pathNode];
			}

			std::reverse(std::begin(pathVect), std::end(pathVect));
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			for (auto& [node, pathAndDist] : paths) {
				// there is no path to nodes in different components
				if (internal::equals(pathAndDist.first.size(), static_cast<size_t>(0)) && !internal::equals(node, startNode)) {
					out << "There is no possible path between [" << startNode << "] and [" << node << "]\n" << std::endl;

					continue;
				}

				out << "Distance from [" << startNode << "] to [" << node << "] is: " << distances[node] << "\n\t Path: ";

				auto& pathVect = pathAndDist.first;

				for (auto& elem : pathVect) {
					out << "[" << elem << "] ";
				}

				out << '\n' << std::endl;
			}
		}

		return paths;
	}

	template<typename NodeType, typename WeightType>
	std::pair<std::vector<NodeType>, WeightType> dijkstraShortestPath(const GraphClasses::Graph<NodeType, WeightType>& g, const NodeType startNode, const NodeType endNode, const AlgorithmBehavior behavior, std::ostream& out) {
		std::unordered_map<NodeType, WeightType> distances;
		std::unordered_map<NodeType, bool> visited;
		std::unordered_map<NodeType, std::optional<NodeType>> parents;

		using pqData = GraphClasses::Edge<NodeType, WeightType>;
		std::priority_queue<pqData, std::vector<pqData>, internal::EdgeComparator<NodeType, WeightType>> pq;

		auto neighborList = g.getNeighborList();

		for (auto& [node, _] : neighborList) {
			distances[node] = GraphClasses::MAX_WEIGHT<WeightType>;
			visited[node] = false;
		}

		distances[startNode] = static_cast<WeightType>(0);
		parents[startNode]; // only startNode will have the empty optional
		pq.emplace(startNode, static_cast<WeightType>(0));

		bool pathFound = false;

		while (!pq.empty()) {
			auto [currentNode, distToCurrentNode] = pq.top();
			pq.pop();

			if (internal::equals(currentNode, endNode)) {
				pathFound = true;
				break;
			}

			visited[currentNode] = true;

			for (auto& [neighbor, weight] : neighborList[currentNode]) {
				if (!visited[neighbor]) {
					WeightType oldDistance = distances[neighbor];
					WeightType newDistance = distances[currentNode] + weight.value_or(static_cast<WeightType>(1));

					if (internal::lessThan(newDistance, oldDistance)) {	
						pq.emplace(neighbor, newDistance);
						distances[neighbor] = newDistance;
						parents[neighbor] = currentNode;
					}
				}
			}
		}

		// path reconstruction
		std::vector<NodeType> path;

		if (pathFound) {
			NodeType endCpy = endNode;
			path.emplace_back(endNode);
			
			std::optional<NodeType> parent = parents[endCpy];

			while (true) {
				if (!parent.has_value()) {
					break;
				}

				auto&  parentVal = parent.value();

				path.emplace_back(parentVal);
				endCpy = parentVal;

				parent = parents[endCpy];
			}

			std::reverse(std::begin(path), std::end(path));
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			if (pathFound) {
				out << "Found path with total distance: " << distances[endNode] << "\nPath: ";

				for (auto& elem : path) {
					out << "[" << elem << "] ";
				}
				
				out << '\n' << std::endl;
			} 
			else {
				out << "No path found between [" << startNode << "] and [" << endNode << "]\n" << std::endl;
			}
		}

		return std::make_pair(path, distances[endNode]);
	}

	template<typename NodeType, typename WeightType>
	std::unordered_map<NodeType, std::unordered_map<NodeType, WeightType>> floydWarshallAllShortestPaths(const GraphClasses::Graph<NodeType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		std::unordered_map<NodeType, std::unordered_map<NodeType, WeightType>> distances;

		auto neighborList = g.getNeighborList();

		for (auto& [node1, _] : neighborList) {
			for (auto& [node2, _] : neighborList) {
				distances[node1][node2] = GraphClasses::MAX_WEIGHT<WeightType>;
			}
		}

		for (auto& [node, neighbors] : neighborList) {
			distances[node][node] = static_cast<WeightType>(0);

			for (auto& [neighbor, weight] : neighbors) {
				// check needed in case of parallel edges
				auto w = weight.value_or(static_cast<WeightType>(1));
				auto& dist = distances[node][neighbor];

				if (internal::lessThan(w, dist)) {
					dist = w;
				}
			}
		}

		for (auto& [mid, _] : neighborList) {
			for (auto& [start, __] : neighborList) {
				auto& startMid = distances[start][mid];

				if (!internal::equals(startMid, GraphClasses::MAX_WEIGHT<WeightType>)) {
					for (auto& [end, ___] : neighborList) {
						auto& midEnd = distances[mid][end];

						if (!internal::equals(midEnd, GraphClasses::MAX_WEIGHT<WeightType>)) {
							auto& startEnd = distances[start][end];
							auto startEndDist = startMid + midEnd;
							
							if (internal::lessThan(startEndDist, startEnd)) {
								distances[start][end] = startEndDist;
							}
						}
					}
				}
			}
		}

		for (auto& [node, _] : distances) {
			if (internal::lessThan(distances[node][node], static_cast<WeightType>(0))) {
				if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
					out << "Graph contains one or more negative cycles\n" << std::endl;
				}

				return {};
			}
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			for (auto& [node, neighbors] : distances) {
				for (auto& [neighbor, distance] : neighbors) {
					if (internal::equals(distance, GraphClasses::MAX_WEIGHT<WeightType>)) {
						out << "There is no possible path between [" << node << "] and [" << neighbor << "]\n";
					}
					else {
						out << "Shortest distance between [" << node << "] and [" << neighbor << "] is: " << distance << '\n';
					}
				}

				out << std::endl;
			}
		}

		return distances;
	}

	template<typename NodeType, typename WeightType>
	std::unordered_map<NodeType, std::unordered_map<NodeType, std::pair<std::vector<NodeType>, WeightType>>> johnsonAllShortestsPaths(const GraphClasses::Graph<NodeType, WeightType>& g, const std::optional<NodeType> artificialStartValue, const AlgorithmBehavior behavior, std::ostream& out) {
		#ifdef CHECK_FOR_ERRORS
			if (internal::equals(g.getGraphWeights(), GraphClasses::GraphWeights::Unweighted)) {
				GRAPH_ERROR(__FILE__, __LINE__, "This function is not supported for unweighted graphs");
				exit(EXIT_FAILURE);
			}
		#endif

		std::unordered_map<NodeType, std::unordered_map<NodeType, std::pair<std::vector<NodeType>, WeightType>>> allShortestPaths;

		GraphClasses::Graph<NodeType, WeightType> gCopy = g;
		auto neighborList = g.getNeighborList();
		
		NodeType artificialStart = NodeType{};

		if (!artificialStartValue.has_value()) {
			#ifdef CHECK_FOR_ERRORS
				for (auto& [node, _] : neighborList) {
					if (internal::equals(node, artificialStart)) {
						GRAPH_ERROR(__FILE__, __LINE__, "NodeType{} is already a node in the graph but no value is provided for artificial start node");
						exit(EXIT_FAILURE);
					}
				}
			#endif
		}
		else {
			artificialStart = artificialStartValue.value();
		}

		for (auto& [node, _] : neighborList) {
			gCopy.addEdge(artificialStart, node, static_cast<WeightType>(0));
		}

		auto bellmanFordResult = GraphAlgorithms::bellmanFordShortestPaths(gCopy, artificialStart, AlgorithmBehavior::ReturnOnly);
		
		// if bellman ford finds a negative cycle we return empty here
		if (internal::equals(bellmanFordResult.size(), static_cast<size_t>(0))) {
			if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
				out << "Graph contains one or more negative cycles\n" << std::endl;
			}
			
			return {};
		}

		gCopy.deleteNode(artificialStart);

		for (auto& [node, neighbors] : neighborList) {
			for (auto& [neighbor, weight] : neighbors) {
				weight.value() += bellmanFordResult[node].second - bellmanFordResult[neighbor].second;
			}
		}

		for (auto& [startNode, _] : neighborList) {
			allShortestPaths[startNode] = GraphAlgorithms::dijkstraAllShortestPathsFromStart(gCopy, startNode, AlgorithmBehavior::ReturnOnly);

			for (auto& [endNode, pathVectAndWeight] : allShortestPaths[startNode]) {
				auto& pathWeight = pathVectAndWeight.second;

				if (!internal::equals(pathWeight, GraphClasses::MAX_WEIGHT<WeightType>)) {
					pathWeight += (bellmanFordResult[endNode].second - bellmanFordResult[startNode].second);
				}
			}
		}


		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			for (auto& [startNode, endNodeAndPath] : allShortestPaths) {
				for (auto& [endNode, pathVectAndWeight] : endNodeAndPath) {
					auto& pathVect = pathVectAndWeight.first;
					auto& pathWeight = pathVectAndWeight.second;

					// there is no path to nodes in different components
					if (internal::equals(pathVect.size(), static_cast<size_t>(0)) && !internal::equals(startNode, endNode)) {
						out << "There is no possible path between [" << startNode << "] and [" << endNode << "]\n";

						continue;
					}
					else {
						out << "Distance from [" << startNode << "] to [" << endNode << "] is: " << pathWeight << "\n\t Path: ";

						for (auto& elem : pathVect) {
							out << "[" << elem << "] ";
						}

						out << '\n';
					}
				}

				out << '\n';
			}

			out << std::endl;
		}

		return allShortestPaths;
	}

	template<typename NodeType, typename WeightType>
	std::unordered_map<NodeType, std::pair<std::vector<NodeType>, WeightType>> shortestPathFasterAlgorithm(const GraphClasses::Graph<NodeType, WeightType>& g, const NodeType startNode, const AlgorithmBehavior behavior, std::ostream& out) {
		std::unordered_map<NodeType, WeightType> distances;
		std::unordered_map<NodeType, std::optional<NodeType>> parents;

		auto neighborList = g.getNeighborList();

		for (auto& [node, _] : neighborList) {
			distances[node] = GraphClasses::MAX_WEIGHT<WeightType>;
		}

		distances[startNode] = static_cast<WeightType>(0);
		parents[startNode]; // only startNode will have the empty optional

		std::deque<NodeType> queue;
		queue.emplace_back(startNode);

		while (!queue.empty()) {
			NodeType currentNode = queue.front();
			queue.pop_front();

			for (auto& [neighbor, weight] : neighborList[currentNode]) {
				WeightType newDistance = distances[currentNode] + weight.value_or(1);

				if (internal::lessThan(newDistance, distances[neighbor])) {
					distances[neighbor] = newDistance;
					parents[neighbor] = currentNode;

					if (internal::equals(std::find(std::begin(queue), std::end(queue), neighbor), std::end(queue))) {
						queue.emplace_back(neighbor);
					}
				}
			}
		}

		// negtive cycle check
		for (auto& [node, neighbors] : neighborList) {
			for (auto& [neighbor, weight] : neighbors) {
				auto& nodeDist = distances[node];

				if (!internal::equals(nodeDist, GraphClasses::MAX_WEIGHT<WeightType>) &&
					internal::lessThan(nodeDist + weight.value_or(static_cast<WeightType>(1)), distances[neighbor])) {
					if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
						out << "Graph contains one or more negative cycles\n" << std::endl;
					}

					return {};
				}
			}
		}

		// path reconstruction
		std::unordered_map<NodeType, std::pair<std::vector<NodeType>, WeightType>> paths;

		for (auto& [node, distFromStart] : distances) {
			paths[node] = std::make_pair(std::vector<NodeType>{}, distFromStart);

			if (internal::equals(distFromStart, GraphClasses::MAX_WEIGHT<WeightType>) || internal::equals(node, startNode)) {
				continue;
			}

			NodeType pathNode = node;

			auto& pathVect = paths[node].first;
			pathVect.emplace_back(pathNode);

			std::optional<NodeType> parent = parents[pathNode];

			while (true) {
				if (!parent.has_value()) {
					break;
				}

				auto& parentVal = parent.value();

				pathVect.emplace_back(parentVal);
				pathNode = parentVal;
				
				parent = parents[pathNode];
			}

			std::reverse(std::begin(pathVect), std::end(pathVect));
		}
		
		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			for (auto& [node, pathAndDist] : paths) {
				// there is no path to nodes in different components
				if (internal::equals(pathAndDist.first.size(), static_cast<size_t>(0)) && !internal::equals(node, startNode)) {
					out << "There is no possible path between [" << startNode << "] and [" << node << "]\n" << std::endl;

					continue;
				}

				out << "Distance from [" << startNode << "] to [" << node << "] is: " << distances[node] << "\n\t Path: ";

				auto& pathVect = pathAndDist.first;

				for (auto& elem : pathVect) {
					out << "[" << elem << "] ";
				}

				out << '\n' << std::endl;
			}
		}

		return paths;
	}

	// ----- articulation point and bridge algorithms -----

	template<typename NodeType, typename WeightType>
	std::unordered_set<NodeType> findArticulationPoints(const GraphClasses::Graph<NodeType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		internal::ArticulationHelper<NodeType, WeightType> internalData;

		internalData.discoveryTime = static_cast<size_t>(0);
		internalData.neighborList = g.getNeighborList();

		for (auto& [node, _] : internalData.neighborList) {
			internalData.visited[node] = false;
		}

		for (auto& [startNode, _] : internalData.neighborList) {
			if (!internalData.visited[startNode]) {
				// start node will have empty optional as parent
				internalData.parents[startNode];

				internalData.previousStartNodes.emplace(startNode);
				internalData.currentStartNode = startNode;

				internal::articulation__internal(startNode, internalData);
			}
		}
		
		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			if (internal::equals(internalData.articulationPoints.size(), static_cast<size_t>(0))) {
				out << "No articulation points found\n" << std::endl;
			} 
			else {
				out << "Articulation points found:\n\t";

				for (auto& point : internalData.articulationPoints) {
					out << "[" << point << "] ";
				}

				out << '\n' << std::endl;
			}
		}

		return internalData.articulationPoints;
	}

	template<typename NodeType, typename WeightType>
	std::vector<std::pair<NodeType, NodeType>> findBridges(const GraphClasses::Graph<NodeType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		internal::ArticulationHelper<NodeType, WeightType> internalData;

		internalData.discoveryTime = static_cast<size_t>(0);
		internalData.neighborList = g.getNeighborList();

		for (auto& [node, _] : internalData.neighborList) {
			internalData.visited[node] = false;
		}

		for (auto& [startNode, _] : internalData.neighborList) {
			if (!internalData.visited[startNode]) {
				internalData.parents[startNode];

				internalData.previousStartNodes.emplace(startNode);
				internalData.currentStartNode = startNode;

				internal::articulation__internal(startNode, internalData);
			}
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			if (internal::equals(internalData.bridges.size(), static_cast<size_t>(0))) {
				out << "No bridges found\n" << std::endl;
			} 
			else {
				out << "Bridges found:\n";

				for (auto& bridge : internalData.bridges) {

					out << "\t{ [" << bridge.first << "] [" << bridge.second << "] }\n";
				}

				out << std::endl;
			}
		}

		return internalData.bridges;
	}

	// ----- topological sorting algorithms -----

	template<typename NodeType, typename WeightType>
	std::vector<std::vector<NodeType>> allTopsorts(const GraphClasses::Graph<NodeType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		#ifdef CHECK_FOR_ERRORS
			if (!g.isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}

			if (internal::equals(g.getGraphDirections(), GraphClasses::GraphDirections::Undirected)) {
				GRAPH_ERROR(__FILE__, __LINE__, "Topological sorting makes no sense for undirected graphs!");
				exit(EXIT_FAILURE);
			}
		#endif

		internal::AllTopsortsHelper<NodeType, WeightType> internalData;
		
		auto& neighborList = internalData.neighborList;
		auto& inDegrees = internalData.inDegrees;
		auto& visited = internalData.visited;
		auto& allTopsorts = internalData.allTopsorts;
		auto& currentTopsort = internalData.currentTopsort;

		neighborList = g.getNeighborList();
		inDegrees = g.getInDegreesOfNodes();

		for (auto& [node, _] : neighborList) {
			visited[node] = false;
		}

		currentTopsort.reserve(g.getNodeCount());

		allTopsorts__internal(internalData);

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			if (internal::equals(allTopsorts.size(), static_cast<size_t>(0))) {
				out << "Graph is not acyclic\n" << std::endl;
			} 
			else {
				out << "Number of possible topological orderings: " << allTopsorts.size() << "\n\n";

				size_t count = static_cast<size_t>(1);

				for (auto& topsort : allTopsorts) {
					out << "Topological ordering " << count << ":\n";

					for (auto& val : topsort) {
						out << "[" << val << "] ";
					}

					out << "\n\n";
					++count;
				}

				out << std::endl;
			}
		}

		return allTopsorts;
	}

	template<typename NodeType, typename WeightType>
	std::vector<NodeType> dfsTopsort(const GraphClasses::Graph<NodeType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		#ifdef CHECK_FOR_ERRORS
			if (!g.isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}

			if (internal::equals(g.getGraphDirections(), GraphClasses::GraphDirections::Undirected)) {
				GRAPH_ERROR(__FILE__, __LINE__, "Topological sorting makes no sense for undirected graphs!");
				exit(EXIT_FAILURE);
			}
		#endif

		internal::DFSTopsortHelper<NodeType, WeightType> internalData;
		
		auto& neighborList = internalData.neighborList;
		auto& visited = internalData.visited;
		auto& inStack = internalData.inStack;
		auto& topologicalOrdering = internalData.topologicalOrdering;
		auto& nextPos = internalData.nextPos;

		neighborList = g.getNeighborList();

		for (auto& [node, _] : neighborList) {
			visited[node] = false;
			inStack[node] = false;
		}

		auto nodeCount = g.getNodeCount();

		topologicalOrdering.resize(nodeCount);
		nextPos = nodeCount - static_cast<size_t>(1);

		for (auto& [node, _] : neighborList) {
			if (!visited[node]) {
				inStack[node] = true;

				// if false is returned, a cycle was found in the current dfs traversal
				if (!dfsTopsort__internal(node, internalData)) {
					break;
				}

				inStack[node] = false;
			}
		}

		// we signal that graph is not acyclic with an empty vector
		if (!internal::equals(topologicalOrdering.size(), nodeCount)) {
			topologicalOrdering.clear();
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			if (internal::equals(topologicalOrdering.size(), static_cast<size_t>(0))) {
				out << "Graph is not acyclic\n" << std::endl;
			} 
			else {
				out << "Topological ordering: ";

				for (auto& val : topologicalOrdering) {
					out << "[" << val << "] ";
				}

				out << '\n' << std::endl;
			}
		}

		return topologicalOrdering;
	}

	template<typename NodeType, typename WeightType>
	std::vector<NodeType> khanTopsort(const GraphClasses::Graph<NodeType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		#ifdef CHECK_FOR_ERRORS
			if (!g.isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}

			if (internal::equals(g.getGraphDirections(), GraphClasses::GraphDirections::Undirected)) {
				GRAPH_ERROR(__FILE__, __LINE__, "Topological sorting makes no sense for undirected graphs!");
				exit(EXIT_FAILURE);
			}
		#endif

		auto neighborList = g.getNeighborList();
		auto inDegrees = g.getInDegreesOfNodes();
		auto nodeCount = g.getNodeCount();

		std::vector<NodeType> topologicalOrdering;
		topologicalOrdering.reserve(nodeCount);

		std::queue<NodeType> topsortQueue;

		for (auto& [node, degree] : inDegrees) {
			if (internal::equals(degree, static_cast<size_t>(0))) {
				topsortQueue.emplace(node);
			}
		}

		NodeType current;

		while (!topsortQueue.empty()) {
			current = topsortQueue.front();
			topsortQueue.pop();

			topologicalOrdering.emplace_back(current);

			for (auto& [neighbor, _] : neighborList[current]) {
				--inDegrees[neighbor];

				if (internal::equals(inDegrees[neighbor], static_cast<size_t>(0))) {	
					topsortQueue.emplace(neighbor);
				}
			}
		}

		// we signal that graph is not acyclic with an empty vector
		if (!internal::equals(topologicalOrdering.size(), nodeCount)) {
			topologicalOrdering.clear();
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			if (internal::equals(topologicalOrdering.size(), static_cast<size_t>(0))) {
				out << "Graph is not acyclic\n" << std::endl;
			} 
			else {
				out << "Topological ordering: ";

				for (auto& val : topologicalOrdering) {
					out << "[" << val << "] ";
				}

				out << '\n' << std::endl;
			}
		}

		return topologicalOrdering;
	}

	// ----- spanning tree algorithms -----

	template<typename NodeType, typename WeightType>
	std::pair<WeightType, GraphClasses::Graph<NodeType, WeightType>> boruvkaMinimumSpanningTree(const GraphClasses::Graph<NodeType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out ) {
		#ifdef CHECK_FOR_ERRORS
			if (!g.isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}

			if (internal::equals(g.getGraphDirections(), GraphClasses::GraphDirections::Directed)) {
				GRAPH_ERROR(__FILE__, __LINE__, "Minimum cost spanning tree for directed graphs currently not supported");
				exit(EXIT_FAILURE);
			}
		#endif

		auto spanningTreeDirections = g.getGraphDirections();
		auto spanningTreeWeights = g.getGraphWeights();

		GraphClasses::Graph<NodeType, WeightType> spanningTree(spanningTreeDirections, spanningTreeWeights);

		WeightType totalCost = static_cast<WeightType>(0);

		internal::DisjointSet ds{g};

		auto neighborList = g.getNeighborList();

		size_t numberOfTrees = g.getNodeCount();;

		// <rootOfComponent/tree, <startNode, endNode, edgeWeight>>
		std::unordered_map<NodeType, std::optional<std::tuple<NodeType, NodeType, WeightType>>> cheapestEdges;

		while (internal::greaterThan(numberOfTrees, static_cast<size_t>(1))) {
			for (auto& [node, neighbors] : neighborList) {
				for (auto& [neighbor, weight] : neighbors) {
					NodeType root1 = ds.findInDisjointSet(node);
					NodeType root2 = ds.findInDisjointSet(neighbor);

					if (internal::equals(root1, root2)) {
						continue;
					}

					WeightType w = weight.value_or(static_cast<WeightType>(1));

					auto& root1CheapestEdge = cheapestEdges[root1];
					auto& root2CheapestEdge = cheapestEdges[root2];

					if (!root1CheapestEdge.has_value() || internal::lessThan(w, std::get<2>(root1CheapestEdge.value()))) {
						root1CheapestEdge = std::make_tuple(node, neighbor, w);
					}

					if (!root2CheapestEdge.has_value() || internal::lessThan(w, std::get<2>(root2CheapestEdge.value()))) {
						root2CheapestEdge = std::make_tuple(node, neighbor, w);
					}
				}
			}

			for (auto& [node, _] : neighborList) {
				NodeType root1 = ds.findInDisjointSet(node);

				if (!cheapestEdges[root1].has_value()) {
					continue;
				}

				auto [cheapEdgeStart, cheapEdgeEnd, cheapEdgeWeight] = cheapestEdges[root1].value();

				NodeType root2 = ds.findInDisjointSet(cheapEdgeEnd);

				if (internal::equals(root1, root2)) {
					continue;
				}
				
				ds.unionDisjointSets(root1, root2);

				totalCost += cheapEdgeWeight;

				if (internal::equals(spanningTreeWeights, GraphClasses::GraphWeights::Unweighted)) {
					spanningTree.addEdge(cheapEdgeStart, cheapEdgeEnd);
					spanningTree.addEdge(cheapEdgeEnd, cheapEdgeStart);
				}
				else {
					spanningTree.addEdge(cheapEdgeStart, cheapEdgeEnd, cheapEdgeWeight);
					spanningTree.addEdge(cheapEdgeEnd, cheapEdgeStart, cheapEdgeWeight);
				}

				--numberOfTrees;
			}

			cheapestEdges.clear();
		}

		if (internal::equals(behavior, GraphAlgorithms::AlgorithmBehavior::PrintAndReturn)) {
			out << "Minimum cost spanning tree:\n";

			out << spanningTree;

			out << "Total cost of spanning tree is: " << totalCost << '\n' << std::endl;
		}

		return std::make_pair(totalCost, spanningTree);
	}

	template<typename NodeType, typename WeightType>
	std::pair<WeightType, GraphClasses::Graph<NodeType, WeightType>> commonMinimumCostSpannigTree(const GraphClasses::Graph<NodeType, WeightType>& g1, const GraphClasses::Graph<NodeType, WeightType>& g2, const AlgorithmBehavior behavior, std::ostream& out) {
		GraphClasses::Graph<NodeType, WeightType> intersection = GraphUtility::intersectGraphs(g1, g2);
		
		auto [totalCost, spanningTree] = kruskalMinimumSpanningTree(intersection, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);

		if (internal::equals(behavior, GraphAlgorithms::AlgorithmBehavior::PrintAndReturn)) {
			out << "Minimum common cost spanning tree for both graphs:\n";

			out << spanningTree;

			out << "Total cost of spanning tree is: " << totalCost << '\n' << std::endl;
		}

		return std::make_pair(totalCost, spanningTree);
	}

	template<typename NodeType, typename WeightType>
	std::pair<WeightType, GraphClasses::Graph<NodeType, WeightType>> kruskalMinimumSpanningTree(const GraphClasses::Graph<NodeType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		#ifdef CHECK_FOR_ERRORS
			if (!g.isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}

			if (internal::equals(g.getGraphDirections(), GraphClasses::GraphDirections::Directed)) {
				GRAPH_ERROR(__FILE__, __LINE__, "Minimum cost spanning tree for directed graphs currently not supported");
				exit(EXIT_FAILURE);
			}
		#endif

		auto neighborList = g.getNeighborList();
		internal::DisjointSet ds{g};

		auto cmp = [](const auto& t1, const auto& t2) { return internal::lessThan(std::get<2>(t1), std::get<2>(t2)); };
		std::multiset<std::tuple<NodeType, NodeType, WeightType>, decltype(cmp)> allEdges(cmp);

		for (auto& [node, neighbors] : neighborList) {
			for (auto& [neighbor, weight] : neighbors) {
				allEdges.emplace(node, neighbor, weight.value_or(static_cast<WeightType>(1)));
			}
		}

		auto spanningTreeDirections = g.getGraphDirections();
		auto spanningTreeWeights = g.getGraphWeights();

		GraphClasses::Graph<NodeType, WeightType> spanningTree(spanningTreeDirections, spanningTreeWeights);

		WeightType totalCost = static_cast<WeightType>(0);

		size_t mcstSize = g.getNodeCount() - static_cast<size_t>(1);
		size_t addedEdges = static_cast<size_t>(0);

		for (auto& [node1, node2, edgeWeight] : allEdges) {
			if (!internal::lessThan(addedEdges, mcstSize)) {
				break;
			}

			NodeType root1 = ds.findInDisjointSet(node1);
			NodeType root2 = ds.findInDisjointSet(node2);

			if (!internal::equals(root1, root2)) {
				totalCost += edgeWeight;

				if (internal::equals(spanningTreeWeights, GraphClasses::GraphWeights::Unweighted)) {
					spanningTree.addEdge(node1, node2);
					spanningTree.addEdge(node2, node1);
				}
				else {
					spanningTree.addEdge(node1, node2, edgeWeight);
					spanningTree.addEdge(node2, node1, edgeWeight);
				}

				ds.unionDisjointSets(root1, root2);

				++addedEdges;
			}
		}

		if (internal::equals(behavior, GraphAlgorithms::AlgorithmBehavior::PrintAndReturn)) {
			out << "Minimum cost spanning tree:\n";

			out << spanningTree;

			out << "Total cost of spanning tree is: " << totalCost << '\n' << std::endl;
		}

		return std::make_pair(totalCost, spanningTree);
	}

	template<typename NodeType, typename WeightType>
	std::pair<WeightType, GraphClasses::Graph<NodeType, WeightType>> primMinimumSpanningTree(const GraphClasses::Graph<NodeType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		#ifdef CHECK_FOR_ERRORS
			if (!g.isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}

			if (internal::equals(g.getGraphDirections(), GraphClasses::GraphDirections::Directed)) {
				GRAPH_ERROR(__FILE__, __LINE__, "Minimum cost spanning tree for directed graphs currently not supported");
				exit(EXIT_FAILURE);
			}
		#endif

		std::unordered_map<NodeType, WeightType> distances;
		std::unordered_map<NodeType, bool> visited;
		std::unordered_map<NodeType, std::optional<NodeType>> parents;

		using pqData = GraphClasses::Edge<NodeType, WeightType>;
		std::priority_queue<pqData, std::vector<pqData>, internal::EdgeComparator<NodeType, WeightType>> pq;

		auto neighborList = g.getNeighborList();

		for (auto& [node, _] : neighborList) {
			distances[node] = GraphClasses::MAX_WEIGHT<WeightType>;
			visited[node] = false;
		}

		NodeType spanningTreeRoot = (*std::begin(neighborList)).first;

		distances[spanningTreeRoot] = static_cast<WeightType>(0);
		parents[spanningTreeRoot]; // only spanningTreeRoot will have the empty optional
		pq.emplace(spanningTreeRoot, static_cast<WeightType>(0));

		for (auto& [neighbor, _] : neighborList[spanningTreeRoot]) {
			pq.emplace(neighbor, GraphClasses::MAX_WEIGHT<WeightType>);
		}

		size_t nodeCount = g.getNodeCount();

		for (size_t i = static_cast<size_t>(0); internal::lessThan(i, nodeCount); ++i) {
			auto [currentNode, distToCurrentNode] = pq.top();
			pq.pop();

			if (!visited[currentNode]) {
				visited[currentNode] = true;

				for (auto& [neighbor, weight] : neighborList[currentNode]) {
					if (!visited[neighbor]) {
						WeightType w = weight.value_or(static_cast<WeightType>(1));

						if (internal::lessThan(w, distances[neighbor])) {
							distances[neighbor] = w;
							parents[neighbor] = currentNode;
							pq.emplace(neighbor, distances[neighbor]);
						}
					}
				}
			}
		}

		auto spanningTreeDirections = g.getGraphDirections();
		auto spanningTreeWeights = g.getGraphWeights();

		GraphClasses::Graph<NodeType, WeightType> spanningTree(spanningTreeDirections, spanningTreeWeights);

		WeightType totalCost = static_cast<WeightType>(0);

		for (auto& [node, _] : neighborList) {
			if (parents[node].has_value()) {
				auto& dist = distances[node];

				totalCost += dist;

				auto& parentValue = parents[node].value();

				if (internal::equals(spanningTreeWeights, GraphClasses::GraphWeights::Unweighted)) {
					spanningTree.addEdge(parentValue, node);
					spanningTree.addEdge(node, parentValue);
				}
				else {
					spanningTree.addEdge(parentValue, node, dist);
					spanningTree.addEdge(node, parentValue, dist);
				}
			}
		}

		if (internal::equals(behavior, GraphAlgorithms::AlgorithmBehavior::PrintAndReturn)) {
			out << "Minimum cost spanning tree:\n";

			out << spanningTree;

			out << "Total cost of spanning tree is: " << totalCost << '\n' << std::endl;
		}

		return std::make_pair(totalCost, spanningTree);
	}

	template<typename NodeType, typename WeightType>
	std::pair<WeightType, GraphClasses::Graph<NodeType, WeightType>> reverseDeleteMinimumSpanningTree(const GraphClasses::Graph<NodeType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		#ifdef CHECK_FOR_ERRORS
			if (!g.isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}

			if (internal::equals(g.getGraphDirections(), GraphClasses::GraphDirections::Directed)) {
				GRAPH_ERROR(__FILE__, __LINE__, "Minimum cost spanning tree for directed graphs currently not supported");
				exit(EXIT_FAILURE);
			}
		#endif

		GraphClasses::Graph<NodeType, WeightType> spanningTree = g;
		auto spanningTreeWeights = spanningTree.getGraphWeights();

		auto neighborList = spanningTree.getNeighborList();
		auto nodeCount = spanningTree.getNodeCount();

		auto cmp = [](const auto& t1, const auto& t2) { return internal::lessThan(std::get<2>(t1), std::get<2>(t2)); };
		std::multiset<std::tuple<NodeType, NodeType, WeightType>, decltype(cmp)> allEdges(cmp);

		// only one direction of each edge is needed for the main part of the algorithm
		// otherwise, number of edges checked will be doubled and totalCost must be divided by 2
		std::unordered_map<NodeType, std::unordered_map<NodeType, std::vector<WeightType>>> alreadyAdded;

		for (auto& [node, neighbors] : neighborList) {
			for (auto& [neighbor, weight] : neighbors) {
				auto& reverseDirectionAddedValues = alreadyAdded[neighbor][node];

				WeightType w = weight.value_or(static_cast<WeightType>(1));

				size_t count = std::count_if(std::begin(reverseDirectionAddedValues), std::end(reverseDirectionAddedValues), [&](const auto& val) { return internal::equals(val, w); });
		
				if (internal::equals(count, static_cast<size_t>(0))) {
					allEdges.emplace(node, neighbor, w);

					alreadyAdded[node][neighbor].emplace_back(w);
				}
			}
		}

		WeightType totalCost = static_cast<WeightType>(0);

		auto itRBegin = std::crbegin(allEdges);
		auto itREnd = std::crend(allEdges);

		while (!internal::equals(itRBegin, itREnd)) {
			auto& [node, neighbor, weight] = *itRBegin;

			if (internal::equals(spanningTreeWeights, GraphClasses::GraphWeights::Unweighted)) {
				spanningTree.deleteEdge(node, neighbor);
			}
			else {
				spanningTree.deleteEdgeWithWeight(node, neighbor, weight);
			}

			auto traversalOrder = depthFirstTraverse(spanningTree, node, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);

			if (internal::lessThan(traversalOrder.size(), nodeCount)) {
				totalCost += weight;

				if (internal::equals(spanningTreeWeights, GraphClasses::GraphWeights::Unweighted)) {
					spanningTree.addEdge(node, neighbor);
					spanningTree.addEdge(neighbor, node);
				}
				else {
					spanningTree.addEdge(node, neighbor, weight);
					spanningTree.addEdge(neighbor, node, weight);
				}
			}

			++itRBegin;
		}
		
		if (internal::equals(behavior, GraphAlgorithms::AlgorithmBehavior::PrintAndReturn)) {
			out << "Minimum cost spanning tree:\n";

			out << spanningTree;

			out << "Total cost of spanning tree is: " << totalCost << '\n' << std::endl;
		}

		return std::make_pair(totalCost, spanningTree);
	}

	// ----- component algorithms -----

	template<typename NodeType, typename WeightType>
	std::vector<std::unordered_set<NodeType>> findWeaklyConnectedComponents(const GraphClasses::Graph<NodeType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		#ifdef CHECK_FOR_ERRORS
			if (!g.isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}
		#endif

		GraphClasses::Graph<NodeType, WeightType> gCopy;
		std::unordered_map<NodeType, std::vector<GraphClasses::Edge<NodeType, WeightType>>> neighborList;

		if (internal::equals(g.getGraphDirections(), GraphClasses::GraphDirections::Directed)) {
			gCopy = g;

			neighborList = gCopy.getNeighborList();

			for (auto& [node, neighbors] : neighborList) {
				for (auto& [neighbor, weight] : neighbors) {
						gCopy.addEdge(neighbor, GraphClasses::Edge(node, weight));
				}
			}
		}
		else {
			neighborList = g.getNeighborList();
		}

		std::unordered_map<NodeType, bool> visited;

		for (auto& [node, _] : neighborList) {
			visited[node] = false;
		}

		std::vector<std::unordered_set<NodeType>> weaklyConnectedComponents;

		for (auto& [node, _] : neighborList) {
			if (!visited[node]) {
				auto& component = weaklyConnectedComponents.emplace_back(std::unordered_set<NodeType>{});

				std::stack<NodeType> stack;
				stack.emplace(node);

				NodeType currentNode;
				while (!stack.empty()) {
					currentNode = stack.top();
					stack.pop();

					auto& ifVisited = visited[currentNode];
					if (!ifVisited) {
						ifVisited = true;
						component.emplace(currentNode);
					}

					for (auto& [neighbor, __] : neighborList[currentNode]) {
						if (!visited[neighbor]) {
							stack.emplace(neighbor);
						}
					}
				}
			}
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			out << "Graph has " << weaklyConnectedComponents.size() << " weakly connected components:\n\n";

			size_t i = static_cast<size_t>(1);

			for (auto& component : weaklyConnectedComponents) {
				out << "Component " << i << " consists of " << component.size() << " nodes:\n\t";

				for (auto& node : component) {
					out << "[" << node << "] ";
				}

				out << '\n' << std::endl; 
				++i;
			}
		}

		return weaklyConnectedComponents;
	}

	template<typename NodeType, typename WeightType>
	std::vector<std::unordered_set<NodeType>> kosarajuFindStronglyConnectedComponents(const GraphClasses::Graph<NodeType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		#ifdef CHECK_FOR_ERRORS
			if (!g.isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}

			if (internal::equals(g.getGraphDirections(), GraphClasses::GraphDirections::Undirected)) {
				GRAPH_ERROR(__FILE__, __LINE__, "This algorithm is only supported for directed graphs");
				exit(EXIT_FAILURE);
			}
		#endif

		auto neighborList = g.getNeighborList();
		auto nodeCount = g.getNodeCount();

		// dfs finishing times in decreasing order
		std::vector<NodeType> dfsFinishTimeOrdering(nodeCount);
		size_t nextIndex = static_cast<size_t>(0);

		std::unordered_map<NodeType, bool> visited;
		std::unordered_map<NodeType, bool> inStack;		

		for (auto& [node, _] : neighborList) {
			visited[node] = false;
			inStack[node] = false;
		}

		std::unordered_map<NodeType, size_t> numUnvisitedChildren = g.getOutDegreesOfNodes();

		std::stack<NodeType> stack;
		NodeType current;

		for (auto& [node, _] : neighborList) {
			if (!visited[node]) {
				stack.emplace(node);
				inStack[node] = true;

				while (!stack.empty()) {
					current = stack.top();

					if (internal::equals(numUnvisitedChildren[current], static_cast<size_t>(0))) {
						visited[current] = true;
						inStack[current] = false;

						dfsFinishTimeOrdering[nextIndex] = current;
						++nextIndex;

						stack.pop();
						continue;
					}

					auto& currentNeighbors = neighborList[current];

					numUnvisitedChildren[current] = currentNeighbors.size();

					for (auto& [neighbor, __] : currentNeighbors) {
						if (!inStack[neighbor] && !visited[neighbor]) {
							stack.emplace(neighbor);
							inStack[neighbor] = true;
						}	
						else {
							--numUnvisitedChildren[current];
						}
					}
				}
			}
		}

		for (auto& [node, _] : neighborList) {
			visited[node] = false;
		}

		GraphClasses::Graph<NodeType, WeightType> transposedGraph = GraphUtility::transposeOfGraph(g);
		neighborList = transposedGraph.getNeighborList();

		std::vector<std::unordered_set<NodeType>> components;

		auto it = std::crbegin(dfsFinishTimeOrdering);
		const auto itEnd = std::crend(dfsFinishTimeOrdering);

		while (!internal::equals(it, itEnd)) {
			if (!visited[*it]) {
				stack.emplace(*it);

				auto& component = components.emplace_back(std::unordered_set<NodeType>{});

				while (!stack.empty()) {
					current = stack.top();
					stack.pop();

					visited[current] = true;
					component.emplace(current);

					for (auto& [neighbor, _] : neighborList[current]) {
						if (!visited[neighbor]) {
							stack.emplace(neighbor);
						}
					}
				}
			}

			++it;
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			out << "Graph has " << components.size() << " strongly connected components:\n\n";

			size_t i = static_cast<size_t>(1);

			for (auto& component : components) {
				out << "Component " << i << " consists of " << component.size() << " nodes:\n\t";

				for (auto& node : component) {
					out << "[" << node << "] ";
				}

				out << '\n' << std::endl;
				++i;
			}
		}

		return components;
	}

	template<typename NodeType, typename WeightType>
	std::vector<std::unordered_set<NodeType>> tarjanFindBiconnectedComponents(const GraphClasses::Graph<NodeType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		#ifdef CHECK_FOR_ERRORS
			if (!g.isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}

			if (internal::equals(g.getGraphDirections(), GraphClasses::GraphDirections::Directed)) {
				GRAPH_ERROR(__FILE__, __LINE__, "This algorithm is only supported for undirected graphs");
				exit(EXIT_FAILURE);
			}
		#endif
		
		internal::TarjanBCHelper<NodeType, WeightType> internalData;

		auto& neighborList = internalData.neighborList;
		neighborList = g.getNeighborList();

		for (auto& [node, _] : neighborList) {
			internalData.discoveryTimes[node] = static_cast<size_t>(0);
			internalData.lowerTimes[node] = static_cast<size_t>(0);
		}

		auto& stack = internalData.stack;
		auto& biconnectedComponents = internalData.biconnectedComponents;

		for (auto& [node, _] : neighborList) {
			if (internal::equals(internalData.discoveryTimes[node], static_cast<size_t>(0))) {
				// start node will have empty optional as parent
				internalData.parents[node];

				// discovery time in each component will start from 1
				internalData.discoveryTime = static_cast<size_t>(1);

				internal::tarjanBC__internal(node, internalData);

				if (!stack.empty()) {
					auto& component = biconnectedComponents.emplace_back(std::unordered_set<NodeType>{});

					while (!stack.empty()) {
						auto [node1, node2] = stack.top();

						component.emplace(node1);
						component.emplace(node2);

						stack.pop();
					}
				}
			}
		}		

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			out << "Graph has " << biconnectedComponents.size() << " possible biconnected components:\n\n";

			size_t i = static_cast<size_t>(1);

			for (auto& component : biconnectedComponents) {
				out << "Biconnected omponent " << i << " consists of " << component.size() << " nodes:\n\t";

				for (auto& node : component) {
					out << "[" << node << "] ";
				}

				out << '\n' << std::endl;
				++i;
			}
		}

		return biconnectedComponents;
	}

	template<typename NodeType, typename WeightType>
	std::vector<std::unordered_set<NodeType>> tarjanFindStronglyConnectedComponents(const GraphClasses::Graph<NodeType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		internal::TarjanSCCHelper<NodeType, WeightType> internalData;

		auto& neighborList = internalData.neighborList;
		neighborList = g.getNeighborList();

		for (auto& [node, _] : neighborList) {
			internalData.inStack[node] = false;
			// unvisited nodes will have their discovery time set to 0
			internalData.discoveryTimes[node] = static_cast<size_t>(0);
		}

		internalData.discoveryTime = static_cast<size_t>(1);

		for (auto& [node, _] : neighborList) {
			if (internal::equals(internalData.discoveryTimes[node], static_cast<size_t>(0))) {
				internal::tarjanSCC__internal(node, internalData);
			}
		}		

		auto& components = internalData.components;

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			out << "Graph has " << components.size() << " strongly connected components:\n\n";

			size_t i = static_cast<size_t>(1);

			for (auto& component : components) {
				out << "Component " << i << " consists of " << component.size() << " nodes:\n\t";

				for (auto& node : component) {
					out << "[" << node << "] ";
				}

				out << '\n' << std::endl;
				++i;
			}
		}

		return components;
	}

	// ----- cycle algorithms -----

	template<typename NodeType, typename WeightType>
	std::vector<std::pair<std::vector<NodeType>, WeightType>> findAllCycles(const GraphClasses::Graph<NodeType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		#ifdef CHECK_FOR_ERRORS
			if (!g.isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}

			if (internal::equals(g.getGraphDirections(), GraphClasses::GraphDirections::Directed)) {
				GRAPH_ERROR(__FILE__, __LINE__, "Use the johnsonAllCycles function for undirected graphs");
				exit(EXIT_FAILURE);
			}
		#endif

		internal::CycleHelper<NodeType, WeightType> internalData;
		internalData.neighborList = g.getNeighborList();

		for (auto& [node, _] : internalData.neighborList) {
			internalData.visited[node] = false;
			internalData.currentPathPrefixSum[node] = static_cast<WeightType>(0);
		}

		for (auto& [node, _] : internalData.neighborList) {
			if (!internalData.visited[node]) {
				// only start node will have empty optional
				internalData.parents[node];

				internalData.currentPath.clear();
				internalData.currentPath.emplace_back(node);

				internal::findAllCycles__internal(node, static_cast<WeightType>(0), internalData);
			}
		}
		
		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			auto& allCycles = internalData.allCycles;

			if (internal::equals(allCycles.size(), static_cast<size_t>(0))) {
				out << "Graph has no cycles\n" << std::endl;
			}
			else {
				out << "Graph has " << allCycles.size() << " cycles:\n";
				
				for (auto& [cycle, weight] : allCycles) {
					out << "{ ";
					
					size_t limit = cycle.size() - 1;
					for (size_t i = static_cast<size_t>(0); internal::lessThan(i, limit); ++i) {
						out << "[" << cycle[i] << "] -> ";
					}

					out << "[" << cycle[limit] << "] }, cycle weight: " << weight << '\n';
				}

				out << std::endl;
			}
		}

		return internalData.allCycles;
	}

	template<typename NodeType, typename WeightType>
	std::vector<std::pair<std::vector<NodeType>, WeightType>> johnsonAllCycles(const GraphClasses::Graph<NodeType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		#ifdef CHECK_FOR_ERRORS
			if (!g.isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}

			if (internal::equals(g.getGraphDirections(), GraphClasses::GraphDirections::Undirected)) {
				GRAPH_ERROR(__FILE__, __LINE__, "Use the findAllCycles function for undirected graphs");
				exit(EXIT_FAILURE);
			}
		#endif

		internal::JohnsonAllCyclesHelper<NodeType, WeightType> internalData;
		auto& blockedSet = internalData.blockedSet;
		auto& blockedMap = internalData.blockedMap;
		auto& cycleStartNode = internalData.cycleStartNode;

		GraphClasses::Graph<NodeType, WeightType> gCopy = g;
		
		bool algCanContinue = true;
		while (algCanContinue) {
			auto connectedComponents = GraphAlgorithms::tarjanFindStronglyConnectedComponents(gCopy, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);

			algCanContinue = false;
			for (auto& component : connectedComponents) {
				if (internal::greaterThan(component.size(), static_cast<size_t>(1))) {
					algCanContinue = true;

					auto subgraph = GraphUtility::getSubgraphFromNodes(gCopy, component);
					internalData.subgraphNeighborList = subgraph.getNeighborList();
					
					cycleStartNode = *std::min_element(std::begin(component), std::end(component));

					blockedSet.clear();
					blockedMap.clear();
					
					internal::johnsonCycles__internal(cycleStartNode, static_cast<WeightType>(0), internalData);

					gCopy.deleteNode(cycleStartNode);

					break;
				}	
			}
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			auto& allCycles = internalData.allCycles;

			if (internal::equals(allCycles.size(), static_cast<size_t>(0))) {
				out << "Graph has no cycles\n" << std::endl;
			}
			else {
				out << "Graph has " << allCycles.size() << " cycles:\n";
				
				for (auto& [cycle, weight] : allCycles) {
					out << "{ ";
					
					size_t limit = cycle.size() - 1;
					for (size_t i = static_cast<size_t>(0); internal::lessThan(i, limit); ++i) {
						out << "[" << cycle[i] << "] -> ";
					}

					out << "[" << cycle[limit] << "] }, cycle weight: " << weight << '\n';
				}

				out << std::endl;
			}
		}
		
		return internalData.allCycles;
	}

	// ----- flow algorithms -----

	template<typename NodeType, typename WeightType>
	std::pair<WeightType, GraphClasses::Graph<NodeType, WeightType>> edmondsKarpMaximumFlow(const GraphClasses::Graph<NodeType, WeightType>& g, NodeType source, NodeType sink, const AlgorithmBehavior behavior, std::ostream& out) {
		#ifdef CHECK_FOR_ERRORS
			if (!g.isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}
		#endif

		auto neighborList = g.getNeighborList();
		std::unordered_map<NodeType, std::unordered_map<NodeType, WeightType>> residualGraph;

		for (auto& [node, neighbors] : neighborList) {
			for (auto& [neighbor, _] : neighbors) {
				residualGraph[node][neighbor] = static_cast<WeightType>(0);
				residualGraph[neighbor][node] = static_cast<WeightType>(0);
			}
		}

		for (auto& [node, neighbors] : neighborList) {
			for (auto& [neighbor, weight] : neighbors) {
				residualGraph[node][neighbor] += weight.value_or(static_cast<WeightType>(1));
			}
		}

		std::unordered_map<NodeType, std::optional<NodeType>> parents;	 
		parents[source];	// only source will have the empty optional

		std::unordered_map<NodeType, bool> visited;

		for (auto& [node, _] : neighborList) {
			visited[node] = false;
		}
		
		WeightType maxFlow = static_cast<WeightType>(0);

		while (true) {
			// breadth first search is used to check if there is a path from source to sink
			std::queue<NodeType> queue;

			queue.emplace(source);
			visited[source] = true;

			bool foundPath = false;

			while (!queue.empty()) {
				NodeType currentNode = queue.front();
				queue.pop();

				if (internal::equals(currentNode, sink)) {
					foundPath = true;
					break;
				}

				for (auto& [neighbor, _] : neighborList[currentNode]) {
					if (!visited[neighbor] && internal::greaterThan(residualGraph[currentNode][neighbor], static_cast<WeightType>(0))) {
						queue.emplace(neighbor);
						parents[neighbor] = currentNode;
						visited[neighbor] = true;
					}
				}
			}

			if (!foundPath) {
				break;
			}

			for (auto& [node, _] : neighborList) {
				visited[node] = false;
			}

			WeightType pathFlow = GraphClasses::MAX_WEIGHT<WeightType>;

			for (NodeType pathNode = sink; parents[pathNode].has_value(); pathNode = parents[pathNode].value()) {				
				auto& parentVal = parents[pathNode].value();

				if (internal::lessThan(residualGraph[parentVal][pathNode], pathFlow)) {
					pathFlow = residualGraph[parentVal][pathNode];
				}
			}

			for (NodeType pathNode = sink; parents[pathNode].has_value(); pathNode = parents[pathNode].value()) {
				auto& parentVal = parents[pathNode].value();

				residualGraph[parentVal][pathNode] -= pathFlow;
				residualGraph[pathNode][parentVal] += pathFlow;
			}

			maxFlow += pathFlow;
		}

		GraphClasses::Graph<NodeType, WeightType> residualReturnGraph(GraphClasses::GraphDirections::Directed, GraphClasses::GraphWeights::Weighted);

		for (auto& [node, neighbors] : residualGraph) {
			for (auto& [neighbor, weight] : neighbors) {
				residualReturnGraph.addEdge(node, neighbor, weight);
			}
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			if (internal::equals(maxFlow, static_cast<WeightType>(0))) {
				out << "No possible path exists between source [" << source << "] and sink [" << sink << "]\n" << std::endl;
			}
			else {
				out << "Maximum possible flow from [" << source << "] to [" << sink << "] is: " << maxFlow << '\n';
				out << "Residual graph:\n";
				out << residualReturnGraph;
			}
		}

		return std::make_pair(maxFlow, residualReturnGraph);
	}

	template<typename NodeType, typename WeightType>
	std::pair<WeightType, GraphClasses::Graph<NodeType, WeightType>> pushRelabelMaximumFlow(const GraphClasses::Graph<NodeType, WeightType>& g, NodeType source, NodeType sink, const AlgorithmBehavior behavior, std::ostream& out) {
		#ifdef CHECK_FOR_ERRORS
			if (!g.isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}
		#endif
	
		std::unordered_map<NodeType, std::unordered_map<NodeType, WeightType>> flow;
		std::unordered_map<NodeType, std::unordered_map<NodeType, WeightType>> capacity;

		std::unordered_map<NodeType, size_t> height;
		std::unordered_map<NodeType, WeightType> excessFlow;

		auto neighborList = g.getNeighborList();

		for (auto& [node, neighbors] : neighborList) {
			height[node] = static_cast<size_t>(0);
			excessFlow[node] = static_cast<size_t>(0);

			for (auto& [neighbor, weight] : neighbors) {
				flow[node][neighbor] = static_cast<size_t>(0);
				flow[neighbor][node] = static_cast<size_t>(0);
				capacity[node][neighbor] += weight.value_or(static_cast<WeightType>(1));
			}
		}  

		height[source] = g.getNodeCount();
		excessFlow[source] = GraphClasses::MAX_WEIGHT<WeightType>;

    	for (auto& [node, _] : neighborList) {
			if (!internal::equals(node, source)) {
				// push(source, node)
				WeightType min = std::min(excessFlow[source], capacity[source][node] - flow[source][node]);

				flow[source][node] += min;
				flow[node][source] -= min;
				excessFlow[source] -= min;
				excessFlow[node] += min;
			}
		}

		while (true) {
			std::vector<NodeType> maxHeightNodes;
			size_t maxHeight = std::numeric_limits<size_t>::min();

			for (auto& [node, _] : neighborList) {
				if (internal::greaterThan(excessFlow[node], static_cast<WeightType>(0)) && !internal::equals(node, source) && !internal::equals(node, sink)) {
					if (!maxHeightNodes.empty() && internal::greaterThan(height[node], maxHeight)) {
						maxHeight = height[node];
						maxHeightNodes.clear();
					}
					if (maxHeightNodes.empty() || internal::equals(height[node], maxHeight)) {
						maxHeightNodes.emplace_back(node);
					}
				}
			}

			if (maxHeightNodes.empty()) {
				break;
			}

			for (auto& maxHeightNode : maxHeightNodes) {
				bool pushed = false;

				for (auto& [node, _] : neighborList) { 
					if (internal::equals(excessFlow[maxHeightNode], static_cast<WeightType>(0))) {
						break;
					}
					else if (internal::greaterThan(capacity[maxHeightNode][node] - flow[maxHeightNode][node], static_cast<WeightType>(0)) 
							 && internal::equals(height[maxHeightNode], height[node] + static_cast<size_t>(1))) {
						// push (maxHeightNode, node)
						WeightType min = std::min(excessFlow[maxHeightNode], capacity[maxHeightNode][node] - flow[maxHeightNode][node]);

						flow[maxHeightNode][node] += min;
						flow[node][maxHeightNode] -= min;
						excessFlow[maxHeightNode] -= min;
						excessFlow[node] += min;

						pushed = true;
					}	
				}

				if (!pushed) {
					// relabel(i);
					size_t minHeight = std::numeric_limits<size_t>::max();

					for (auto& [node, _] : neighborList) {
						if (internal::greaterThan(capacity[maxHeightNode][node] - flow[maxHeightNode][node], static_cast<WeightType>(0))) {
							minHeight = std::min(minHeight, height[node]);
						}
					}

					if (internal::lessThan(minHeight, std::numeric_limits<size_t>::max())) {
						height[maxHeightNode] = minHeight + static_cast<size_t>(1);
					}

					break;
				}
			}
		}

    	WeightType maxFlow = static_cast<WeightType>(0);

		for (auto& [node, _] : neighborList) {
			maxFlow += flow[node][sink];
		}

		GraphClasses::Graph<NodeType, WeightType> flowGraph(GraphClasses::GraphDirections::Directed, GraphClasses::GraphWeights::Weighted);

		for (auto& [node, neighbors] : neighborList) {
			for (auto& [neighbor, _] : neighbors) {
				flowGraph.addEdge(node, neighbor, flow[node][neighbor]);
			}
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			if (internal::equals(maxFlow, static_cast<WeightType>(0))) {
				out << "No possible path exists between source [" << source << "] and sink [" << sink << "]\n" << std::endl;
			}
			else {
				out << "Maximum possible flow from [" << source << "] to [" << sink << "] is: " << maxFlow << '\n';
				out << "Flow graph:\n";
				out << flowGraph;
			}
		}

		return std::make_pair(maxFlow, flowGraph);
	}

	// ----- other algorithms -----

	template<typename NodeType, typename WeightType>
	std::unordered_set<NodeType> findIsolatedNodes(const GraphClasses::Graph<NodeType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		#ifdef CHECK_FOR_ERRORS
			if (!g.isConfigured()) {
				GRAPH_ERROR(__FILE__, __LINE__, "Graph type and graph weights must be configured before calling this function");
				exit(EXIT_FAILURE);
			}
		#endif
		
		std::unordered_set<NodeType> isolatedNodes;

		if (internal::equals(g.getGraphDirections(), GraphClasses::GraphDirections::Undirected)) {
			auto degrees = g.getDegreesOfNodes();

			for (auto& [node, degree] : degrees) {
				if (internal::equals(degree, static_cast<size_t>(0))) {
					isolatedNodes.emplace(node);
				}
			}
		}
		else { // directed
			auto inDegrees = g.getInDegreesOfNodes();
			auto outDegrees = g.getOutDegreesOfNodes();
			auto neighborList = g.getNeighborList();

			for (auto& [node, _] : neighborList) {
				if (internal::equals(inDegrees[node], static_cast<size_t>(0)) && internal::equals(outDegrees[node], static_cast<size_t>(0))) {
					isolatedNodes.emplace(node);
				}
			}
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			if (internal::equals(isolatedNodes.size(), static_cast<size_t>(0))) {
				out << "Graph contains no isolated nodes\n" << std::endl;
			}
			else {
				out << "Found " << isolatedNodes.size() << " isolated nodes:\n";

				for (auto& node : isolatedNodes) {
						out << "[" << node << "] ";
				}

				out << '\n' << std::endl;
			}
		}

		return isolatedNodes;
	}
} // namespace GraphAlgorithms

// internal namespace for helper funcitons, not inteded for end user
namespace internal {
	template<typename WeightType>
	std::enable_if_t<std::is_floating_point_v<WeightType>, bool>
	equals(const WeightType lhs, const WeightType rhs) {
		return std::fabs(rhs - lhs) < FLOATING_POINT_EPSIOLON<WeightType>;
	}

	template<typename WeightType>
	std::enable_if_t<!std::is_floating_point_v<WeightType>, bool>
	equals(const WeightType lhs, const WeightType rhs) {
		return lhs == rhs;
	}

	template<typename WeightType>
	std::enable_if_t<std::is_floating_point_v<WeightType>, bool>
	lessThan(const WeightType lhs, const WeightType rhs) {
		return lhs < (rhs - FLOATING_POINT_EPSIOLON<WeightType>);
	}

	template<typename WeightType>
	std::enable_if_t<!std::is_floating_point_v<WeightType>, bool>
	lessThan(const WeightType lhs, const WeightType rhs) {
		return lhs < rhs;
	}

	template<typename WeightType>
	std::enable_if_t<std::is_floating_point_v<WeightType>, bool>
	greaterThan(const WeightType lhs, const WeightType rhs) {
		return (lhs - FLOATING_POINT_EPSIOLON<WeightType>) > rhs;
	}

	template<typename WeightType>
	std::enable_if_t<!std::is_floating_point_v<WeightType>, bool>
	greaterThan(const WeightType lhs, const WeightType rhs) {
		return lhs > rhs;
	}

	template<typename NodeType, typename WeightType>
	void exportDirectedGraph(const GraphClasses::Graph<NodeType, WeightType>& g, const char* filePath) {
		std::ofstream file(filePath);

		#ifdef CHECK_FOR_ERRORS
			if (!file) {
				GRAPH_ERROR(__FILE__, __LINE__, "Invalid file!");
				exit(EXIT_FAILURE);
			}
		#endif

		auto neighborList = g.getNeighborList();
		for (auto& [node, neighbors] : neighborList) {
			file << node << " ";

			for (auto& [neighbor, weight] : neighbors) {
				file << neighbor << " ";

				if (weight.has_value()) {
					file << weight.value() << " ";
				}
			}

			file << std::endl;
		}
		
		file.close();

		return;
	}

	template<typename NodeType, typename WeightType>
	void exportUndirectedGraph(const GraphClasses::Graph<NodeType, WeightType>& g, const char* filePath) {
		std::ofstream file(filePath);
		
		#ifdef CHECK_FOR_ERRORS
			if (!file) {
				GRAPH_ERROR(__FILE__, __LINE__, "Invalid file!");
				exit(EXIT_FAILURE);
			}
		#endif

		// for undirected graphs, we must only write one direction of an edge, or else on next read from file the number of edges will be doubled
		std::unordered_map<NodeType, std::unordered_set<GraphClasses::Edge<NodeType, WeightType>, internal::EdgeStructHasher<NodeType, WeightType>>> doNotAdd;
		auto neighborList = g.getNeighborList();

		for (auto& [node, neighbors] : neighborList) {
			file << node << " ";

			auto& alreadyAddedEdges = doNotAdd[node];
			for (auto& edge : neighbors) {
				if (internal::equals(alreadyAddedEdges.count(edge), static_cast<size_t>(0))) {
					file << edge.neighbor << " ";

					if (edge.weight.has_value()) {
						file << edge.weight.value() << " ";
					}

					doNotAdd[edge.neighbor].emplace(node, edge.weight);
				}
			}

			file << std::endl;
		}
		
		file.close();

		return;
	}

	template<typename NodeType, typename WeightType>
	struct EdgeComparator {
		bool operator()(const GraphClasses::Edge<NodeType, WeightType>& e1, const GraphClasses::Edge<NodeType, WeightType>& e2) const { 
			return internal::greaterThan(e1.weight.value(), e2.weight.value()); 
		}
	};

	template<typename NodeType, typename WeightType>
	struct EdgeStructHasher {
		size_t operator()(const GraphClasses::Edge<NodeType, WeightType>& obj) const {
			std::hash<NodeType> nHash;
			std::hash<WeightType> wHash;
			// TODO:  try finding a better alternative
			return nHash(obj.neighbor) + wHash(obj.weight.value_or(static_cast<size_t>(0)));
		}
	};

	template<typename NodeType, typename WeightType>
	struct CompleteEdgeHasher {
		size_t operator()(const std::pair<NodeType, GraphClasses::Edge<NodeType, WeightType>>& obj) const {
			std::hash<NodeType> nHash;
			std::hash<WeightType> wHash;
			// TODO:  try finding a better alternative
			return nHash(obj.first) + nHash(obj.second.neighbor) + wHash(obj.second.weight.value_or(static_cast<size_t>(0)));
		}
	};

	template<typename NodeType, typename WeightType>
	struct ArticulationHelper {
		public:
			size_t discoveryTime;
			std::unordered_map<NodeType, size_t> discoveryTimes;
			std::unordered_map<NodeType, size_t> lowerTimes;
			std::unordered_map<NodeType, bool> visited;
			std::unordered_map<NodeType, std::optional<NodeType>> parents;
			std::unordered_map<NodeType, std::vector<GraphClasses::Edge<NodeType, WeightType>>> neighborList;
			std::unordered_set<NodeType> previousStartNodes;
			NodeType currentStartNode;
			std::unordered_set<NodeType> articulationPoints;
			std::vector<std::pair<NodeType, NodeType>> bridges;
	};

	// this one internal function is used both for findArticulationPoints and findBridges as these two algorithms are very simmilar
	template<typename NodeType, typename WeightType>
	void articulation__internal(const NodeType startNode, ArticulationHelper<NodeType, WeightType>& internalData) {
		internalData.visited[startNode] = true;
		internalData.discoveryTimes[startNode] = internalData.discoveryTime;
		internalData.lowerTimes[startNode] = internalData.discoveryTime;
		++internalData.discoveryTime;

		size_t numChildren = static_cast<size_t>(0);

		auto& startNodeParent = internalData.parents[startNode];
		auto& startNodeTime = internalData.discoveryTimes[startNode];
		auto& startNodeLowerTime = internalData.lowerTimes[startNode];
		auto& neighborList = internalData.neighborList;
		auto& currentStartNode = internalData.currentStartNode;
		auto& previousStartNodes = internalData.previousStartNodes;

		for (auto& [neighbor, _] : neighborList[startNode]) {
			auto& neighborLowerTime = internalData.lowerTimes[neighbor];

			if (!internalData.visited[neighbor] 
				 || (!internal::equals(neighbor, currentStartNode) && internal::greaterThan(previousStartNodes.count(neighbor), static_cast<size_t>(0)))) {
				++numChildren;
				internalData.parents[neighbor] = startNode;

				articulation__internal(neighbor, internalData);

				if (internal::lessThan(neighborLowerTime, startNodeLowerTime)) {
					startNodeLowerTime = neighborLowerTime;
				}

				// for articulation points
				bool startNodeHasParent = startNodeParent.has_value();

				if ((!startNodeHasParent && internal::greaterThan(numChildren, static_cast<size_t>(1)))
					 || (startNodeHasParent && (internal::greaterThan(neighborLowerTime, startNodeTime)
					 ||	internal::equals(neighborLowerTime, startNodeTime)))) {
					internalData.articulationPoints.emplace(startNode);
				}

				// for bridges
				if (internal::greaterThan(neighborLowerTime, startNodeTime)) {
					internalData.bridges.emplace_back(startNode, neighbor);
				}
			} 
			else {
				auto& neighborTime = internalData.discoveryTimes[neighbor];

				if (startNodeParent.has_value() && !internal::equals(neighbor, startNodeParent.value()) && internal::lessThan(neighborTime, startNodeLowerTime)) {
					startNodeLowerTime = neighborTime;
				}
			}
		}

		return;
	}
	
	template<typename NodeType, typename WeightType>
	struct AllTopsortsHelper {
		std::unordered_map<NodeType, std::vector<GraphClasses::Edge<NodeType, WeightType>>> neighborList;
		std::unordered_map<NodeType, size_t> inDegrees;
		std::unordered_map<NodeType, bool> visited;
		std::vector<std::vector<NodeType>> allTopsorts;
		std::vector<NodeType> currentTopsort;
	};

	template<typename NodeType, typename WeightType>
	void allTopsorts__internal(AllTopsortsHelper<NodeType, WeightType>& internalData) {
		auto& neighborList = internalData.neighborList;
		auto& inDegrees = internalData.inDegrees;
		auto& visited = internalData.visited;
		auto& allTopsorts = internalData.allTopsorts;
		auto& currentTopsort = internalData.currentTopsort;

		for (auto& [node, inDegree] : inDegrees) {
			if (internal::equals(inDegree, static_cast<size_t>(0)) && !visited[node]) {
				auto& neighbors = neighborList[node];

				for (auto& [neighbor, _] : neighbors) {
					--inDegrees[neighbor];
				}

				currentTopsort.emplace_back(node);
				visited[node] = true;

				allTopsorts__internal(internalData);

				// backtrack
				for (auto& [neighbor, _] : neighbors) {
					++inDegrees[neighbor];
				}

				currentTopsort.pop_back();
				visited[node] = false;
			}
		}

		if (internal::equals(currentTopsort.size(), neighborList.size())) {
			// TODO: see if there exists a more efficient way
			allTopsorts.push_back(currentTopsort);
		}
	}

	template<typename NodeType, typename WeightType>
	struct DFSTopsortHelper {
		std::unordered_map<NodeType, std::vector<GraphClasses::Edge<NodeType, WeightType>>> neighborList;
		std::unordered_map<NodeType, bool> visited;
		std::unordered_map<NodeType, bool> inStack;
		std::vector<NodeType> topologicalOrdering;
		size_t nextPos;
	};

	template<typename NodeType, typename WeightType>
	bool dfsTopsort__internal (const NodeType currentNode, DFSTopsortHelper<NodeType, WeightType>& internalData) {
		auto& neighborList = internalData.neighborList;
		auto& visited = internalData.visited;
		auto& inStack = internalData.inStack;
		auto& topologicalOrdering = internalData.topologicalOrdering;
		auto& nextPos = internalData.nextPos;

		visited[currentNode] = true;
		inStack[currentNode] = true;

		for (auto& [neighbor, _] : neighborList[currentNode]) {
			if (inStack[neighbor]) {
				return false;
			}

			if (!visited[neighbor]) {
				if (!dfsTopsort__internal(neighbor, internalData)) {
					return false;
				}
			}
		}

		topologicalOrdering[nextPos] = currentNode;
		--nextPos;

		inStack[currentNode] = false;

		return true;
	}

	template<typename NodeType, typename WeightType>
	class DisjointSet {
		public:
			explicit DisjointSet(const GraphClasses::Graph<NodeType, WeightType>& g) {
				auto neighborList = g.getNeighborList();

				for (auto& [node, _] : neighborList) {
					parent[node] = node;
					rank[node] = static_cast<size_t>(0);
				}
			}

			NodeType findInDisjointSet(const NodeType node) {
				NodeType nodeCpy = node;
				NodeType root = node;

				while (!internal::equals(root, parent[root])) {
					root = parent[root];
				}
				
				while (!internal::equals(nodeCpy, root)) {
					NodeType tmp = parent[nodeCpy];
					parent[nodeCpy] = root;
					nodeCpy = tmp;
				}

				return root;
			}

			void unionDisjointSets(const NodeType root1, const NodeType root2) {
				if (internal::greaterThan(rank[root1], rank[root2])) {
					parent[root2] = root1;
				} 
				else if (internal::lessThan(rank[root1], rank[root2])) {
					parent[root1] = root2;
				} 
				else {
					parent[root1] = root2;
					++rank[root2];
				}
			}

		private:
			std::unordered_map<NodeType, NodeType> parent;
			std::unordered_map<NodeType, size_t> rank;
	};

	template<typename NodeType, typename WeightType>
	struct TarjanBCHelper {
		public:
			size_t discoveryTime;
			std::unordered_map<NodeType, size_t> discoveryTimes;
			std::unordered_map<NodeType, size_t> lowerTimes;
			std::unordered_map<NodeType, std::optional<NodeType>> parents;
			std::stack<std::pair<NodeType, NodeType>> stack;
			std::unordered_map<NodeType, std::vector<GraphClasses::Edge<NodeType, WeightType>>> neighborList;
			std::vector<std::unordered_set<NodeType>> biconnectedComponents;
	};

	template<typename NodeType, typename WeightType>
	void tarjanBC__internal(const NodeType currentNode, TarjanBCHelper<NodeType, WeightType>& internalData) {
		auto& discoveryTime = internalData.discoveryTime;
		auto& discoveryTimes = internalData.discoveryTimes;
		auto& lowerTimes = internalData.lowerTimes;
		auto& parents = internalData.parents;
		auto& stack = internalData.stack;
		auto& neighborList = internalData.neighborList;

		discoveryTimes[currentNode] = discoveryTime;
		lowerTimes[currentNode] = discoveryTime;
		++discoveryTime;

		size_t numChildren = static_cast<size_t>(0);

		auto& currentNodeDiscoveryTime = discoveryTimes[currentNode];
		auto& currentNodeLowerTime = lowerTimes[currentNode];

		for (auto& [neighbor, _] : neighborList[currentNode]) {
			auto& neighborDiscoveryTime = discoveryTimes[neighbor];

			if (internal::equals(neighborDiscoveryTime, static_cast<size_t>(0))) {
				++numChildren;

				parents[neighbor] = currentNode;

				stack.emplace(currentNode, neighbor);
				internal::tarjanBC__internal(neighbor, internalData);

				auto& neighborLowerTime = lowerTimes[neighbor];
				
				if (internal::lessThan(neighborLowerTime, currentNodeLowerTime)) {
					currentNodeLowerTime = neighborLowerTime;
				}
				
				// if found articulation point
				if ((internal::equals(currentNodeDiscoveryTime, static_cast<size_t>(1)) && internal::greaterThan(numChildren, static_cast<size_t>(1))) ||
					(internal::greaterThan(currentNodeDiscoveryTime, static_cast<size_t>(1)) && !internal::lessThan(neighborLowerTime, currentNodeDiscoveryTime))
				   ) {
					auto& component = internalData.biconnectedComponents.emplace_back(std::unordered_set<NodeType>{});

					while (!internal::equals(stack.top().first, currentNode) || !internal::equals(stack.top().second, neighbor)) {
						component.emplace(stack.top().first);
						component.emplace(stack.top().second);
						
						stack.pop();
					}

					component.emplace(stack.top().first);
					component.emplace(stack.top().second);

					stack.pop();
				}
			}
			else {
				auto& currentNodeParent = parents[currentNode];

				if (currentNodeParent.has_value() && !internal::equals(currentNodeParent.value(), neighbor)) {
					if (internal::lessThan(neighborDiscoveryTime, currentNodeLowerTime)) {
						currentNodeLowerTime = neighborDiscoveryTime;

						if (internal::lessThan(neighborDiscoveryTime, currentNodeDiscoveryTime)) {
							stack.emplace(currentNode, neighbor);
						}
					}
				}
			} 
		}

		return;
	}

	template<typename NodeType, typename WeightType>
	struct TarjanSCCHelper {
		public:
			size_t discoveryTime;
			std::unordered_map<NodeType, size_t> discoveryTimes;
			std::unordered_map<NodeType, size_t> lowerTimes;
			std::stack<NodeType> traversalOrder;
			std::unordered_map<NodeType, bool> inStack;
			std::unordered_map<NodeType, std::vector<GraphClasses::Edge<NodeType, WeightType>>> neighborList;
			std::vector<std::unordered_set<NodeType>> components;
	};

	template<typename NodeType, typename WeightType>
	void tarjanSCC__internal(const NodeType currentNode, TarjanSCCHelper<NodeType, WeightType>& internalData) {
		auto& discoveryTime = internalData.discoveryTime;
		auto& discoveryTimes = internalData.discoveryTimes;
		auto& lowerTimes = internalData.lowerTimes;
		auto& inStack = internalData.inStack;
		auto& neighborList = internalData.neighborList;
		
		discoveryTimes[currentNode] = discoveryTime;
		lowerTimes[currentNode] = discoveryTime;
		++discoveryTime;

		internalData.traversalOrder.emplace(currentNode);
		inStack[currentNode] = true;

		for (auto& [neighbor, _] : neighborList[currentNode]) {
			auto& currentNodeLowerTime = lowerTimes[currentNode];
			auto& neighborDiscoveryTime = discoveryTimes[neighbor];
			auto& neighborLowerTime = lowerTimes[neighbor];

			if (internal::equals(neighborDiscoveryTime, static_cast<size_t>(0))) {
				tarjanSCC__internal(neighbor, internalData);

				if (internal::lessThan(neighborLowerTime, currentNodeLowerTime)) {
					currentNodeLowerTime = neighborLowerTime;
				}
			} 
			else if (inStack[neighbor] && internal::lessThan(neighborDiscoveryTime, currentNodeLowerTime)) {
				currentNodeLowerTime = neighborDiscoveryTime;
			}
		}

		// component found
		if (internal::equals(discoveryTimes[currentNode], lowerTimes[currentNode])) {
			auto& component = internalData.components.emplace_back(std::unordered_set<NodeType>{});
			auto& traversalOrder = internalData.traversalOrder;

			NodeType componentNode;

			while (true) {
				componentNode = traversalOrder.top();
				traversalOrder.pop();

				component.emplace(componentNode);
				inStack[componentNode] = false;

				if (internal::equals(componentNode, currentNode)) {
					break;
				}
			}
		}

		return;
	}

	template<typename NodeType, typename WeightType>
	struct CycleHelper {
		public:
			std::unordered_map<NodeType, std::optional<NodeType>> parents;
			std::unordered_map<NodeType, bool> visited;
			std::vector<std::pair<std::vector<NodeType>, WeightType>> allCycles;
			std::unordered_map<NodeType, std::vector<GraphClasses::Edge<NodeType, WeightType>>> neighborList;
			std::vector<NodeType> currentPath;
			std::unordered_map<NodeType, WeightType> currentPathPrefixSum;
	};

	template<typename NodeType, typename WeightType>
	void findAllCycles__internal(const NodeType currentNode, const WeightType currentPathWeight, CycleHelper<NodeType, WeightType>& internalData) {
		auto& parents = internalData.parents;
		auto& allCycles = internalData.allCycles;
		auto& neighborList = internalData.neighborList;
		auto& currentPath = internalData.currentPath;
		auto& visited = internalData.visited;
		auto& currentPathPrefixSum = internalData.currentPathPrefixSum;

		visited[currentNode] = true;
		currentPathPrefixSum[currentNode] = currentPathWeight;

		for (auto& [neighbor, weight] : neighborList[currentNode]) {
			if (!parents[currentNode].has_value() || !internal::equals(neighbor, parents[currentNode].value())) {
				// if neighbor node is detected on currentPath before this point, a cycle is found
				auto itFindNeighbor = std::find(std::begin(currentPath), std::end(currentPath), neighbor);

				if (!internal::equals(itFindNeighbor, std::end(currentPath))) {
					std::vector<NodeType> cycle(itFindNeighbor, std::end(currentPath));

					auto cycleBegin = std::begin(cycle);
					auto cycleEnd = std::end(cycle);

					std::rotate(cycleBegin, std::min_element(cycleBegin, cycleEnd), cycleEnd);
					cycle.emplace_back(*std::begin(cycle));

					// iterators must be renewed because emplace_back may invalidate them 
					cycleBegin = std::begin(cycle);
					cycleEnd = std::end(cycle);

					auto cmp = [&](const auto& elem) { return internal::equals(elem.first, cycle); };

					auto itFindCycle = std::find_if(std::begin(allCycles), std::end(allCycles), cmp);
					if (!internal::equals(itFindCycle, std::end(allCycles))) {
						continue;
					}

					std::reverse(cycleBegin, cycleEnd);

					auto itFindReverseOfCycle = std::find_if(std::begin(allCycles), std::end(allCycles), cmp);
					if (!internal::equals(itFindReverseOfCycle, std::end(allCycles))) {
						continue;
					}

					WeightType cycleWeight = currentPathWeight + weight.value_or(static_cast<WeightType>(1)) - currentPathPrefixSum[neighbor];
					allCycles.emplace_back(cycle, cycleWeight);
					continue;
				}

				parents[neighbor] = currentNode;
				currentPath.emplace_back(neighbor);

				findAllCycles__internal(neighbor, currentPathWeight + weight.value_or(static_cast<WeightType>(1)), internalData);
				currentPath.pop_back();
			}
		}

		return;
	}

	template<typename NodeType, typename WeightType>
	struct JohnsonAllCyclesHelper {
		public:
			NodeType cycleStartNode;
			std::unordered_set<NodeType> blockedSet;
			std::unordered_map<NodeType, std::unordered_set<NodeType>> blockedMap;
			std::deque<NodeType> cycleStack;
			std::vector<std::pair<std::vector<NodeType>, WeightType>> allCycles;
			std::unordered_map<NodeType, std::vector<GraphClasses::Edge<NodeType, WeightType>>> subgraphNeighborList;
	};

	template<typename NodeType, typename WeightType>
	bool johnsonCycles__internal(const NodeType currentNode, const WeightType currentCycleWeight, JohnsonAllCyclesHelper<NodeType, WeightType>& internalData) {
		auto& blockedSet = internalData.blockedSet;
		auto& blockedMap = internalData.blockedMap;
		auto& cycleStack = internalData.cycleStack;
		auto& allCycles = internalData.allCycles;
		auto& subgraphNeighborList = internalData.subgraphNeighborList;
		auto& cycleStartNode = internalData.cycleStartNode;

		bool foundCycle = false;

		cycleStack.emplace_back(currentNode);
		blockedSet.emplace(currentNode);

		for (auto& [neighbor, weight] : subgraphNeighborList[currentNode]) {
			if (internal::equals(neighbor, cycleStartNode)) {
				std::vector<NodeType> cycle;

				for (auto& node : cycleStack) {
					cycle.emplace_back(node);
				}

				cycle.emplace_back(cycleStartNode);
				allCycles.emplace_back(cycle, currentCycleWeight + weight.value_or(static_cast<WeightType>(1)));

				foundCycle = true;
			}
			else if (internal::equals(blockedSet.count(neighbor), static_cast<size_t>(0))) {
				bool gotCycle = johnsonCycles__internal(neighbor, currentCycleWeight + weight.value_or(static_cast<WeightType>(1)), internalData);
				foundCycle = foundCycle || gotCycle;
			}
		}

		if (foundCycle) {
			// unblock currentNode
			std::deque<NodeType> forRemoval;
			forRemoval.emplace_back(currentNode);

			while (!forRemoval.empty()) {
				NodeType nodeToUnblock = forRemoval.front();
				forRemoval.pop_front();

				for (auto& dependentNode : blockedMap[nodeToUnblock]) {
					forRemoval.emplace_back(dependentNode);
				}

				blockedSet.erase(nodeToUnblock);
				blockedMap.erase(nodeToUnblock);
			}
		}
		else {
			for (auto& [neighbor, _] : subgraphNeighborList[currentNode]) {
				blockedMap[neighbor].emplace(currentNode);
			}
		}

		cycleStack.pop_back();
		return foundCycle; 
	}
} // namespace internal

#endif //__SIMPLE_GRAPHS__