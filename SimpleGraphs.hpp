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

#define GRAPH_ERROR(message) std::cerr << "ERROR: " << (message) << std::endl;

//------------------------------------- API -------------------------------------
namespace GraphClasses {
	template<typename WeightType>
	static constexpr WeightType MAX_WEIGHT = std::numeric_limits<WeightType>::max();

	template<typename WeightType>
	static constexpr WeightType MIN_WEIGHT = std::numeric_limits<WeightType>::lowest();

	enum class GraphType {
		Unset,
		Directed,
		Undirected
	};

	enum class GraphWeights {
		Unset,
		Weighted,
		Unweighted
	};

	template<typename DataType, typename WeightType>
	struct Edge {
		public:
			explicit Edge(const DataType neighbor, const std::optional<WeightType> weight = {});

		public:
			DataType neighbor;
			std::optional<WeightType> weight;
	};

	template<typename DataType, typename WeightType = int>
	class Graph {
		public:
			explicit Graph(const GraphType graphType = GraphType::Unset, const GraphWeights graphWeights = GraphWeights::Unset);

			void configureDirections(const GraphType graphType);
			void configureWeights(const GraphWeights graphWeights);
			bool isConfigured() const;
			void clearGraph();

			void readFromTxt(const char* filePath);
			void exportToTxt(const char* filePath) const;

			void addNode(const DataType node);
			void addEdge(const DataType startNode, const DataType neighborNode);    // for unweighted graphs
			void addEdge(const DataType startNode, const DataType neighborNode, const WeightType edgeWeight); // for weighted graphs
			void addEdge(const DataType startNode, const Edge<DataType, WeightType>& edge);
			// deletes all edges from startNode to endNode in case of a multigraph
			void deleteEdge(const DataType startNode, const DataType endNode);	
			// removes a node and all edges to/from said node
			void deleteNode(const DataType nodeToDelete);
			size_t getNodeCount() const;
			size_t getEdgeCount() const;

			std::unordered_set<DataType> getNodeSet() const;

			size_t getDegreeOfNode(const DataType node) const;	// only for undirected
			std::unordered_map<DataType, size_t> getDegreesOfNodes() const;	// only for undirected
			size_t getInDegreeOfNode(const DataType node) const;	// only for directed
			std::unordered_map<DataType, size_t> getInDegreesOfNodes() const;		// only for directed
			size_t getOutDegreeOfNode(const DataType node) const;	// only for directed
			std::unordered_map<DataType, size_t> getOutDegreesOfNodes() const;	// only for directed

			double getDensity() const;
			// NOTE: for weighted graphs, eccentricity is calculated in terms of edge weights and not number of edges on path
			WeightType getEccentricityOfNode(const DataType node) const;
			std::tuple<WeightType, WeightType, std::unordered_set<DataType>> getRadiusDiameterAndCenter() const;
			// ...

			GraphType getGraphType() const;
			GraphWeights getGraphWeights() const;
			std::unordered_map<DataType, std::vector<Edge<DataType, WeightType>>> getNeighbors() const;

			template<typename D, typename W>
			friend std::ostream& operator<<(std::ostream& out, const GraphClasses::Graph<D, W>& g);

		private:
			GraphType m_graphType;
			GraphWeights m_graphWeights;
			std::unordered_map<DataType, std::vector<Edge<DataType, WeightType>>> m_neighbors;
	};

} // namespace GraphClasses

namespace GraphUtility {
	template<typename DataType, typename WeightType>
	GraphClasses::Graph<DataType, WeightType> mergeGraphs(const GraphClasses::Graph<DataType, WeightType>& g1, const GraphClasses::Graph<DataType, WeightType>& g2);

	template<typename DataType, typename WeightType>
	GraphClasses::Graph<DataType, WeightType> intersectGraphs(const GraphClasses::Graph<DataType, WeightType>& g1, const GraphClasses::Graph<DataType, WeightType>& g2);

	template<typename DataType, typename WeightType>
	GraphClasses::Graph<DataType, WeightType> getSubgraphFromNodes(const GraphClasses::Graph<DataType, WeightType>& g, const std::unordered_set<DataType>& nodes);
	
	template<typename DataType, typename WeightType>
	GraphClasses::Graph<DataType, WeightType> transposeOfGraph(const GraphClasses::Graph<DataType, WeightType>& g);

	// constructs unweighted graph
	template<typename DataType>
	GraphClasses::Graph<DataType> constructCompleteGraphFromNodes(const std::unordered_set<DataType>& nodes, const GraphClasses::GraphType graphType);

	// cosntructs weighted graph
	template<typename DataType, typename WeightType>
	GraphClasses::Graph<DataType, WeightType> constructCompleteGraphFromNodes(const std::unordered_set<DataType>& nodes, const GraphClasses::GraphType graphType, const WeightType defaultWeight);

	template<typename DataType, typename WeightType>
	GraphClasses::Graph<DataType, WeightType> complementOfGraph(const GraphClasses::Graph<DataType, WeightType>& g);
	
	// NOTE: the algorithm assumes any node is reachable from itself and the resulting graph will contain the edge from node to itself for all nodes
	template<typename DataType, typename WeightType>
	GraphClasses::Graph<DataType, WeightType> transitiveClosureOfGraph(const GraphClasses::Graph<DataType, WeightType>& g);
	
	template<typename DataType, typename WeightType>
	GraphClasses::Graph<DataType, WeightType> transitiveReductionOfGraph(const GraphClasses::Graph<DataType, WeightType>& g);
} // namespace GraphUtility

namespace GraphAlgorithms {
	enum class AlgorithmBehavior {
		ReturnOnly,
		PrintAndReturn
	};

	template<typename DataType, typename WeightType>
	std::vector<DataType> depthFirstTraverse(const GraphClasses::Graph<DataType, WeightType>& g, const DataType startNode,
		const AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	template<typename DataType, typename WeightType>
	std::pair<bool, std::vector<DataType>> depthFirstSearch(const GraphClasses::Graph<DataType, WeightType>& g, const DataType startNode, const DataType nodeToFind,
		const AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	template<typename DataType, typename WeightType>
	std::vector<DataType> breadthFirstTraverse(const GraphClasses::Graph<DataType, WeightType>& g, const DataType startNode,
		const AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	template<typename DataType, typename WeightType>
	std::pair<bool, std::vector<DataType>> breadthFirstSearch(const GraphClasses::Graph<DataType, WeightType>& g, const DataType startNode, const DataType nodeToFind,
		const AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	template<typename DataType, typename WeightType>
	std::pair<std::vector<DataType>, WeightType> dijkstraShortestPath(const GraphClasses::Graph<DataType, WeightType>& g, const DataType startNode, const DataType endNode,
		const AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	template<typename DataType, typename WeightType>
	std::unordered_map<DataType, std::pair<std::vector<DataType>, WeightType>> bellmanFordShortestPaths(const GraphClasses::Graph<DataType, WeightType>& g, const DataType startNode,
		const AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	// NOTE: at this time, Floyd-Warshall algorithm only returns the distances between pairs of nodes and not the paths themselves
	template<typename DataType, typename WeightType>
	std::unordered_map<DataType, std::unordered_map<DataType, WeightType>> floydWarshallAllShortestPaths(const GraphClasses::Graph<DataType, WeightType>& g,
		const AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	// without start node (only available for undirected graphs)
	template<typename DataType, typename WeightType>
	std::unordered_set<DataType> findArticulationPoints(const GraphClasses::Graph<DataType, WeightType>& g,
		const AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	// with start node (available for both undirected and directed)
	// NOTE: when using this function for directed graphs, only nodes in the corresponding dfs tree will be checked
	template<typename DataType, typename WeightType>
	std::unordered_set<DataType> findArticulationPoints(const GraphClasses::Graph<DataType, WeightType>& g, const DataType startNode,
		const AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	// without start node (only available for undirected graphs)
	template<typename DataType, typename WeightType>
	std::vector<std::pair<DataType, DataType>> findBridges(const GraphClasses::Graph<DataType, WeightType>& g,
		const AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	// with start node (available for both undirected and directed)
	// NOTE: when using this function for directed graphs, only nodes in the corresponding dfs tree will be checked
	template<typename DataType, typename WeightType>
	std::vector<std::pair<DataType, DataType>> findBridges(const GraphClasses::Graph<DataType, WeightType>& g, const DataType startNode,
		const AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	template<typename DataType, typename WeightType>
	std::vector<DataType> topsortKhan(const GraphClasses::Graph<DataType, WeightType>& g, 
		const AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	template<typename DataType, typename WeightType>
	WeightType mcstPrimTotalCostOnly(const GraphClasses::Graph<DataType, WeightType>& g, 
		const AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	template<typename DataType, typename WeightType>
	std::vector<std::tuple<DataType, DataType, WeightType>> mcstPrim(const GraphClasses::Graph<DataType, WeightType>& g,
		const AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	template<typename DataType, typename WeightType>
	std::vector<std::tuple<DataType, DataType, WeightType>> mcstKruskal(const GraphClasses::Graph<DataType, WeightType>& g,
		const AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	// without start node (only available for undirected graphs)
	template<typename DataType, typename WeightType>
	std::vector<std::unordered_set<DataType>> findStronglyConnectedComponentsTarjan(const GraphClasses::Graph<DataType, WeightType>& g,
		const AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	// NOTE: when using this function for directed graphs, only nodes in the corresponding dfs tree will be checked
	template<typename DataType, typename WeightType>
	std::vector<std::unordered_set<DataType>> findStronglyConnectedComponentsTarjan(const GraphClasses::Graph<DataType, WeightType>& g, const DataType startNode,
		const AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	template<typename DataType, typename WeightType>
	std::vector<std::unordered_set<DataType>> findWeaklyConnectedComponents(const GraphClasses::Graph<DataType, WeightType>& g,
		const AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	template<typename DataType, typename WeightType>
	std::unordered_set<DataType> findIsolatedNodes(const GraphClasses::Graph<DataType, WeightType>& g,
		const AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout); 

	// TODO:
	// cycles
	// coloring
	// maximum flow (ford-fulkerson, edmonds-karp)
	// pairing
	//...
} // namespace GraphAlgorithms


//------------------------------------- IMPLEMENTATION -------------------------------------
// forward decleare here or else clang fails on linux (but not on windows)
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

	template<typename DataType, typename WeightType>
	void exportDirectedGraph(const GraphClasses::Graph<DataType, WeightType>& g, const char* filePath);

	template<typename DataType, typename WeightType>
	void exportUndirectedGraph(const GraphClasses::Graph<DataType, WeightType>& g, const char* filePath);

	template<typename DataType, typename WeightType>
	struct EdgeComparator;

	template<typename DataType, typename WeightType>
	struct EdgeStructHasher;

	template<typename DataType, typename WeightType>
	struct CompleteEdgeHasher;

	template<typename DataType, typename WeightType>
	struct ArticulationHelper;

	template<typename DataType, typename WeightType>
	void articulation__internal(const GraphClasses::Graph<DataType, WeightType>& g, const DataType startNode, ArticulationHelper<DataType, WeightType>& internalData);

	template<typename DataType, typename WeightType>
	class DisjointSet;

	template<typename DataType, typename WeightType>
	struct TarjanHelper;

	template<typename DataType, typename WeightType>
	void tarjan__internal(const GraphClasses::Graph<DataType, WeightType>& g, const DataType startNode, TarjanHelper<DataType, WeightType>& internalData);
} // namespace internal


namespace GraphClasses {
	template<typename DataType, typename WeightType>
	std::ostream& operator<<(std::ostream& out, const GraphClasses::Graph<DataType, WeightType>& g) {
		for (auto& [node, neighbors] : g.m_neighbors) {
			out << "Node [" << node << "] has neighbors:\n";

			if (internal::equals(g.m_graphWeights, GraphWeights::Weighted)) {
				for (auto& val : neighbors) {
					out << "|\t [" << val.neighbor << "], edge weight: " << val.weight.value() << '\n';
				}
			} else { // unweighted
				for (auto& val : neighbors) {
					out << "|\t [" << val.neighbor << "]\n";
				}
			}
		}

		out << std::endl;

		return out;
	}

	template<typename DataType, typename WeightType>
	bool operator<(const Edge<DataType, WeightType>& lhs, const Edge<DataType, WeightType>& rhs) {
		return internal::lessThan(lhs.neighbor, rhs.neighbor) || internal::lessThan(lhs.weight, rhs.weight);
	}

	template<typename DataType, typename WeightType>
	bool operator==(const Edge<DataType, WeightType>& lhs, const Edge<DataType, WeightType>& rhs) {
		return internal::equals(lhs.neighbor, rhs.neighbor) && internal::equals(lhs.weight, rhs.weight);
	}

	template<typename DataType, typename WeightType>
	Edge<DataType, WeightType>::Edge(const DataType neighbor, const std::optional<WeightType> weight) 
			: neighbor(neighbor), weight(weight) 
	{}

	template<typename DataType, typename WeightType>
	Graph<DataType, WeightType>::Graph(const GraphType graphType, const GraphWeights graphWeights) 
			: m_graphType(graphType), m_graphWeights(graphWeights) {
		static_assert(std::is_arithmetic_v<WeightType> && !std::is_same_v<WeightType, bool>, "Weight type must be an arithmetic type except boolean");
		// std::cout << MAX_WEIGHT<WeightType> << " " << MIN_WEIGHT<WeightType> << std::endl;
	}

	template<typename DataType, typename WeightType>
	void Graph<DataType, WeightType>::configureDirections(const GraphType graphType) {
		m_graphType = graphType;

		return;
	}

	template<typename DataType, typename WeightType>
	void Graph<DataType, WeightType>::configureWeights(const GraphWeights graphWeights) {
		m_graphWeights = graphWeights;

		return;
	}

	template<typename DataType, typename WeightType>
	bool Graph<DataType, WeightType>::isConfigured() const {
		return !internal::equals(m_graphType, GraphType::Unset) && !internal::equals(m_graphWeights, GraphWeights::Unset);
	}

	template<typename DataType, typename WeightType>
	void Graph<DataType, WeightType>::clearGraph() {
		m_neighbors.clear();

		return;
	}

	template<typename DataType, typename WeightType>
	void Graph<DataType, WeightType>::readFromTxt(const char* filePath) {
		if (!isConfigured()) {
			GRAPH_ERROR("Graph type and weight must be configured before reading from file!");
			exit(EXIT_FAILURE);
		}

		clearGraph();

		std::ifstream file(filePath);
		if (!file) {
			GRAPH_ERROR("Invalid file!");
			exit(EXIT_FAILURE);
		}

		DataType node;
		DataType neighbor;
		WeightType weight;

		std::string line;

		while (getline(file, line)) {
			std::istringstream lineStream(line);
			lineStream >> node;
			addNode(node); // this line is neccessary becasue of isolated nodes

			if (internal::equals(m_graphWeights, GraphWeights::Weighted)) {
				while (lineStream >> neighbor >> weight) {
					if (internal::equals(m_graphType, GraphType::Directed)) {
						addEdge(node, neighbor, weight);
					} else { // undirected
						addEdge(node, neighbor, weight);
						addEdge(neighbor, node, weight);
					}
				}
			} else { // unweighted
				while (lineStream >> neighbor) {
					if (internal::equals(m_graphType, GraphType::Directed)) {
						addEdge(node, neighbor);
					} else { // undirected
						addEdge(node, neighbor);
						addEdge(neighbor, node);
					}
				}
			}
		}

		file.close();

		return;
	}

	template<typename DataType, typename WeightType>
	void Graph<DataType, WeightType>::exportToTxt(const char* filePath) const {
		if (!isConfigured()) {
			GRAPH_ERROR("Unconfigured graph cannot be exported!");
			exit(EXIT_FAILURE);
		}

		if (internal::equals(m_graphType, GraphType::Directed)) {
			internal::exportDirectedGraph(*this, filePath);
		}
		else {
			internal::exportUndirectedGraph(*this, filePath);
		}

		return;
	}

	template<typename DataType, typename WeightType>
	void Graph<DataType, WeightType>::addNode(const DataType node) {
		m_neighbors[node];

		return;
	}

	template<typename DataType, typename WeightType>
	void Graph<DataType, WeightType>::addEdge(const DataType startNode, const DataType neighborNode) {
		if (internal::equals(m_graphWeights, GraphWeights::Weighted)) {
			GRAPH_ERROR("Graph is weighed and edge weight must be specified! ");
			exit(EXIT_FAILURE);
		}

		// this line is neccessary in case neighbor node is only mentioned as neighbor of another node
		addNode(neighborNode); 

		m_neighbors[startNode].emplace_back(neighborNode);

		return;
	}

	template<typename DataType, typename WeightType>
	void Graph<DataType, WeightType>::addEdge(const DataType startNode, const DataType neighborNode, const WeightType edgeWeight) {
		if (internal::equals(m_graphWeights, GraphWeights::Unweighted)) {
			GRAPH_ERROR("Graph is not weighed but you are trying to specify edge weight!");
			exit(EXIT_FAILURE);
		}

		// this line is neccessary in case neighbor node is only mentioned as neighbor of another node
		addNode(neighborNode); 

		m_neighbors[startNode].emplace_back(neighborNode, edgeWeight);

		return;
	}

	template<typename DataType, typename WeightType>
	void Graph<DataType, WeightType>::addEdge(const DataType startNode, const Edge<DataType, WeightType>& edge) {
		if (internal::equals(m_graphWeights, GraphWeights::Unweighted) && edge.weight.has_value()) {
			GRAPH_ERROR("Graph is unweighed but edge has a weight!");
			exit(EXIT_FAILURE);
		} else if (internal::equals(m_graphWeights, GraphWeights::Weighted) && !edge.weight.has_value()) {
			GRAPH_ERROR("Graph is weighed but edge has no weight!");
			exit(EXIT_FAILURE);
		}

		// this line is neccessary in case neighbor node is only mentioned as neighbor of another node
		addNode(edge.neighbor); 

		m_neighbors[startNode].emplace_back(edge.neighbor, edge.weight);

		return;
	}

	template<typename DataType, typename WeightType>
	void Graph<DataType, WeightType>::deleteEdge(const DataType startNode, const DataType endNode) {
		auto itStartNode = m_neighbors.find(startNode);
		auto itEndNode = m_neighbors.find(endNode);
		if (internal::equals(itStartNode, std::end(m_neighbors)) || internal::equals(itEndNode, std::end(m_neighbors))) {
			// std::cout << "Edge does not exist" << std::endl;
			return;
		}

		auto it = std::begin((*itStartNode).second);
		auto end = std::end((*itStartNode).second);
		auto itRemoved = std::remove_if(it, end, [&](const auto& neighborNode){ return internal::equals(neighborNode.neighbor, endNode); });
		(*itStartNode).second.erase(itRemoved, end);

		if (internal::equals(m_graphType, GraphType::Undirected)) {
			it = std::begin((*itEndNode).second);
			end = std::end((*itEndNode).second);
			itRemoved = std::remove_if(it, end, [&](const auto& neighborNode){ return internal::equals(neighborNode.neighbor, startNode); });
			(*itEndNode).second.erase(itRemoved, end);
		}

		return;
	}

	template<typename DataType, typename WeightType>
	void Graph<DataType, WeightType>::deleteNode(DataType nodeToDelete) {
		if (internal::equals(m_neighbors.find(nodeToDelete), std::end(m_neighbors))) {
			// std::cout << "Node does not exist" << std::endl;
			return;
		}

		m_neighbors.erase(nodeToDelete);
		
		for (auto& [node, neighbors] : m_neighbors) {
			auto itBegin = std::begin(neighbors);
			auto itEnd = std::end(neighbors);
			auto itRemoved = std::remove_if(itBegin, itEnd, [&](const auto& neighborNode){ return internal::equals(neighborNode.neighbor, nodeToDelete); });
			neighbors.erase(itRemoved, itEnd);
		}

		return;
	}

	template<typename DataType, typename WeightType>
	size_t Graph<DataType, WeightType>::getNodeCount() const {
		return m_neighbors.size();
	}

	template<typename DataType, typename WeightType>
	size_t Graph<DataType, WeightType>::getEdgeCount() const {
		size_t count = static_cast<size_t>(0);

		for (auto& [node, neighbors] : m_neighbors) {
			count += neighbors.size();
		}

		return count;
	}

	template<typename DataType, typename WeightType>
	std::unordered_set<DataType> Graph<DataType, WeightType>::getNodeSet() const {
		std::unordered_set<DataType> nodeSet;

		for (auto& [node, neighbors] : m_neighbors) {
			nodeSet.emplace(node);
		}

		return nodeSet;
	}

	template<typename DataType, typename WeightType>
	size_t Graph<DataType, WeightType>::getDegreeOfNode(const DataType node) const {
		if (!internal::equals(m_graphType, GraphType::Undirected)) {
			GRAPH_ERROR("Use getter fucntions for in/out degrees for directed graphs!");
			exit(EXIT_FAILURE);
		}

		return m_neighbors.at(node).size();
	}

	template<typename DataType, typename WeightType>
	std::unordered_map<DataType, size_t> Graph<DataType, WeightType>::getDegreesOfNodes() const {
		if (!internal::equals(m_graphType, GraphType::Undirected)) {
			GRAPH_ERROR("Use getter fucntions for in/out degrees for directed graphs!");
			exit(EXIT_FAILURE);
		}
		
		std::unordered_map<DataType, size_t> degrees;

		for (auto& [node, neighbors] : m_neighbors) {
			degrees[node] = neighbors.size();
		}

		return degrees;
	}

	template<typename DataType, typename WeightType>
	size_t Graph<DataType, WeightType>::getInDegreeOfNode(const DataType node) const {
		if (!internal::equals(m_graphType, GraphType::Directed)) {
			GRAPH_ERROR("Use regular getters for degrees for undirected graphs!");
			exit(EXIT_FAILURE);
		}

		size_t inDegree = static_cast<size_t>(0);

		for (auto& [someNode, neighbors] : m_neighbors) {
			for (auto& [neighbor, weight] : neighbors) {
				if (internal::equals(neighbor, node)) {
					++inDegree;
				}
			}
		}

		return inDegree;
	}

	template<typename DataType, typename WeightType>
	std::unordered_map<DataType, size_t> Graph<DataType, WeightType>::getInDegreesOfNodes() const {
		if (!internal::equals(m_graphType, GraphType::Directed)) {
			GRAPH_ERROR("Use regular getters for degrees for undirected graphs!");
			exit(EXIT_FAILURE);
		}

		std::unordered_map<DataType, size_t> inDegrees;

		for (auto& [node, neighbors] : m_neighbors) {
			inDegrees[node] = static_cast<size_t>(0);
		}

		for (auto& [node, neighbors] : m_neighbors) {
			for (auto& [neighbor, weight] : neighbors) {
				++inDegrees[neighbor];
			}
		}

		return inDegrees;
	}

	template<typename DataType, typename WeightType>
	size_t Graph<DataType, WeightType>::getOutDegreeOfNode(const DataType node) const {
		if (!internal::equals(m_graphType, GraphType::Directed)) {
			GRAPH_ERROR("Use regular getters for degrees for undirected graphs!");
			exit(EXIT_FAILURE);
		}

		return m_neighbors.at(node).size();
	}

	template<typename DataType, typename WeightType>
	std::unordered_map<DataType, size_t> Graph<DataType, WeightType>::getOutDegreesOfNodes() const {
		if (!internal::equals(m_graphType, GraphType::Directed)) {
			GRAPH_ERROR("Use regular getters for degrees for undirected graphs!");
			exit(EXIT_FAILURE);
		}

		std::unordered_map<DataType, size_t> outDegrees;

		for (auto& [node, neighbors] : m_neighbors) {
			outDegrees[node] = neighbors.size();
		}

		return outDegrees;
	}

	template<typename DataType, typename WeightType>
	double Graph<DataType, WeightType>::getDensity() const {
		auto nodeCount = getNodeCount();

		double density = static_cast<double>(getEdgeCount()) / (nodeCount * (nodeCount - static_cast<size_t>(1)));

		if (internal::equals(m_graphType, GraphType::Undirected)) {
			density *= 2;
		}

		return density;
	}

	template<typename DataType, typename WeightType>
	WeightType Graph<DataType, WeightType>::getEccentricityOfNode(const DataType node) const {
		auto shortestPaths = GraphAlgorithms::bellmanFordShortestPaths(*this, node, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
		
		WeightType eccentricity = MIN_WEIGHT<WeightType>;

		for (auto& [neighbor, pathData] : shortestPaths) {
			if (internal::greaterThan(pathData.second, eccentricity)) {
				eccentricity = pathData.second;
			}
		}

		return eccentricity;
	}

	template<typename DataType, typename WeightType>
	std::tuple<WeightType, WeightType, std::unordered_set<DataType>> Graph<DataType, WeightType>::getRadiusDiameterAndCenter() const {
		auto allShortestPaths = GraphAlgorithms::floydWarshallAllShortestPaths(*this, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);

		WeightType radius = MAX_WEIGHT<WeightType>;
		WeightType diameter = MIN_WEIGHT<WeightType>;
		std::unordered_set<DataType> center;

		for (auto& [startNode, pathMap] : allShortestPaths) {
			WeightType eccStartNode = MIN_WEIGHT<WeightType>;

			for (auto& [endNode, weight] : pathMap) {
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

	template<typename DataType, typename WeightType>
	GraphType Graph<DataType, WeightType>::getGraphType() const {
		return m_graphType;
	}

	template<typename DataType, typename WeightType>
	GraphWeights Graph<DataType, WeightType>::getGraphWeights() const {
		return m_graphWeights;
	}

	template<typename DataType, typename WeightType>
	std::unordered_map<DataType, std::vector<Edge<DataType, WeightType>>> Graph<DataType, WeightType>::getNeighbors() const {
		return m_neighbors;
	}

} // namespace GraphClasses

namespace GraphUtility {
	template<typename DataType, typename WeightType>
	GraphClasses::Graph<DataType, WeightType> mergeGraphs(const GraphClasses::Graph<DataType, WeightType>& g1, const GraphClasses::Graph<DataType, WeightType>& g2) {
		GraphClasses::Graph<DataType, WeightType> newGraph;

		if (!internal::equals(g1.getGraphType(), g2.getGraphType()) || !internal::equals(g1.getGraphWeights(), g2.getGraphWeights())) {
			GRAPH_ERROR("Graphs can only be merged if they have the same type (directed/undirected) and same weights (weighed/unweighed)!");
			exit(EXIT_FAILURE);
		}

		newGraph.configureDirections(g1.getGraphType());
		newGraph.configureWeights(g1.getGraphWeights());

		auto g1NeighborList = g1.getNeighbors();
		auto g2NeighborList = g2.getNeighbors();

		// adding duplicate edges is avoided by putting them in a set first
		std::unordered_set<std::pair<DataType, GraphClasses::Edge<DataType, WeightType>>, internal::CompleteEdgeHasher<DataType, WeightType>> edgeSet;

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

	template<typename DataType, typename WeightType>
	GraphClasses::Graph<DataType, WeightType> intersectGraphs(const GraphClasses::Graph<DataType, WeightType>& g1, const GraphClasses::Graph<DataType, WeightType>& g2) {
		GraphClasses::Graph<DataType, WeightType> newGraph;

		if (!internal::equals(g1.getGraphType(), g2.getGraphType()) || !internal::equals(g1.getGraphWeights(), g2.getGraphWeights())) {
			GRAPH_ERROR("Graph intersection can only be created if they have the same type (directed/undirected) and same weights (weighed/unweighed)!");
			exit(EXIT_FAILURE);
		}

		newGraph.configureDirections(g1.getGraphType());
		newGraph.configureWeights(g1.getGraphWeights());

		auto g1NeighborList = g1.getNeighbors();
		auto g2NeighborList = g2.getNeighbors();

		std::unordered_set<GraphClasses::Edge<DataType, WeightType>, internal::EdgeStructHasher<DataType, WeightType>> edges;

		for (auto& [node, neighbors] : g1NeighborList) {
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

	template<typename DataType, typename WeightType>
	GraphClasses::Graph<DataType, WeightType> getSubgraphFromNodes(const GraphClasses::Graph<DataType, WeightType>& g, const std::unordered_set<DataType>& nodes) {
		GraphClasses::Graph<DataType, WeightType> newGraph;

		newGraph.configureDirections(g.getGraphType());
		newGraph.configureWeights(g.getGraphWeights());

		auto neighborList = g.getNeighbors();

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

	template<typename DataType, typename WeightType>
	GraphClasses::Graph<DataType, WeightType> transposeOfGraph(const GraphClasses::Graph<DataType, WeightType>& g) {
		if (internal::equals(g.getGraphType(), GraphClasses::GraphType::Undirected)) {
			GRAPH_ERROR("Transposing makes no sense for undirected graphs!");
			exit(EXIT_FAILURE);
		}

		GraphClasses::Graph<DataType, WeightType> newGraph;

		newGraph.configureDirections(g.getGraphType());
		newGraph.configureWeights(g.getGraphWeights());

		auto neighborList = g.getNeighbors();

		for (auto& [node, neighbors] : neighborList) {
			// needed so that isolated nodes will remain in the transposed graph
			if (internal::equals(neighbors.size(), static_cast<size_t>(0))) {
				newGraph.addNode(node);
				continue;
			}

			for (auto& [neighbor, weight] : neighbors) {
				if (internal::equals(newGraph.getGraphWeights(), GraphClasses::GraphWeights::Weighted)) {
					newGraph.addEdge(neighbor, node, weight.value());

					if (internal::equals(newGraph.getGraphType(), GraphClasses::GraphType::Undirected)) {
						newGraph.addEdge(node, neighbor, weight.value());
					} 
				}
				else { // unweighted
					newGraph.addEdge(neighbor, node);

					if (internal::equals(newGraph.getGraphType(), GraphClasses::GraphType::Undirected)) {
						newGraph.addEdge(node, neighbor);
					} 
				}
			}
		}

		return newGraph;
	}

	template<typename DataType>
	GraphClasses::Graph<DataType> constructCompleteGraphFromNodes(const std::unordered_set<DataType>& nodes, const GraphClasses::GraphType graphType) {
		GraphClasses::Graph<DataType> newGraph;

		newGraph.configureDirections(graphType);
		newGraph.configureWeights(GraphClasses::GraphWeights::Unweighted);

		for (auto& startNode : nodes) {
			for (auto& endNode : nodes) {
				if (!internal::equals(startNode, endNode)) {
					newGraph.addEdge(startNode, endNode);
				}
			}
		}

		return newGraph;
	}

	template<typename DataType, typename WeightType>
	GraphClasses::Graph<DataType, WeightType> constructCompleteGraphFromNodes(const std::unordered_set<DataType>& nodes, const GraphClasses::GraphType graphType, const WeightType defaultWeight) {
		GraphClasses::Graph<DataType, WeightType> newGraph;

		newGraph.configureDirections(graphType);
		newGraph.configureWeights(GraphClasses::GraphWeights::Weighted);

		for (auto& startNode : nodes) {
			for (auto& endNode : nodes) {
				if (!internal::equals(startNode, endNode)) {
					newGraph.addEdge(startNode, endNode, defaultWeight);
				}
			}
		}

		return newGraph;
	}

	template<typename DataType, typename WeightType>
	GraphClasses::Graph<DataType, WeightType> complementOfGraph(const GraphClasses::Graph<DataType, WeightType>& g) {
		if (internal::equals(g.getGraphWeights(), GraphClasses::GraphWeights::Weighted)) {
			GRAPH_ERROR("Finding complement of weighted graph not supported!");
			exit(EXIT_FAILURE);
		}

		GraphClasses::Graph<DataType, WeightType> newGraph;

		newGraph.configureDirections(g.getGraphType());
		newGraph.configureWeights(g.getGraphWeights());

		auto neighborList = g.getNeighbors();

		for (auto& [startNode, startNodeNeighbors] : neighborList) {
			for (auto& [endNode, endNodeNeighbors] : neighborList) {
				if (!internal::equals(startNode, endNode)) {
					newGraph.addEdge(startNode, endNode);
				}
			}
		}

		for (auto& [node, neighbors] : neighborList) {
			for(auto& [neighbor, weight] : neighbors) {
				newGraph.deleteEdge(node, neighbor);
			}
		}

		return newGraph;
	}

	template<typename DataType, typename WeightType>
	GraphClasses::Graph<DataType, WeightType> transitiveClosureOfGraph(const GraphClasses::Graph<DataType, WeightType>& g) {
		GraphClasses::Graph<DataType, WeightType> closure;

		closure.configureDirections(GraphClasses::GraphType::Directed);
		closure.configureWeights(GraphClasses::GraphWeights::Unweighted);

		auto neighborList = g.getNeighbors();

		std::unordered_map<DataType, std::unordered_map<DataType, bool>> reachMatrix;
		
		for (auto& [node1, neighbors1] : neighborList) {
			for (auto& [node2, neighbors2] : neighborList) {
				reachMatrix[node1][node2] = false;
			}
		}

		for (auto& [node, neighbors] : neighborList) {
			reachMatrix[node][node] = true;

			for (auto& [neighbor, weight] : neighbors) {
				reachMatrix[node][neighbor] = true;
			}
		}

		for (auto& [mid, n1] : neighborList) {
			for (auto& [start, n2] : neighborList) {
				if (reachMatrix[start][mid]) {
					for (auto& [end, n3] : neighborList) {
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

	template<typename DataType, typename WeightType>
	GraphClasses::Graph<DataType, WeightType> transitiveReductionOfGraph(const GraphClasses::Graph<DataType, WeightType>& g) {
		GraphClasses::Graph<DataType, WeightType> reduction;

		reduction.configureDirections(GraphClasses::GraphType::Directed);
		reduction.configureWeights(GraphClasses::GraphWeights::Unweighted);

		auto neighborList = g.getNeighbors();

		std::unordered_map<DataType, std::unordered_map<DataType, bool>> reachMatrix;
		
		for (auto& [node1, neighbors1] : neighborList) {
			for (auto& [node2, neighbors2] : neighborList) {
				reachMatrix[node1][node2] = false;
			}
		}

		for (auto& [node, neighbors] : neighborList) {
			for (auto& [neighbor, weight] : neighbors) {
				reachMatrix[node][neighbor] = true;
			}
		}

		for (auto& [mid, n1] : neighborList) {
			for (auto& [start, n2] : neighborList) {
				if (reachMatrix[start][mid]) {
					for (auto& [end, n3] : neighborList) {
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
} // namespace GraphUtility


namespace GraphAlgorithms {
	template<typename DataType, typename WeightType>
	std::vector<DataType> depthFirstTraverse(const GraphClasses::Graph<DataType, WeightType>& g, const DataType startNode, const AlgorithmBehavior behavior, std::ostream& out) {
		std::unordered_map<DataType, bool> visited;
		std::vector<DataType> traversalOrder;

		auto neighborList = g.getNeighbors();

		for (auto& [node, neighbors] : neighborList) {
			visited[node] = false;
		}

		std::stack<DataType> stack;
		stack.emplace(startNode);

		DataType currentNode;
		while (!stack.empty()) {
			currentNode = stack.top();
			stack.pop();

			auto& ifVisited = visited[currentNode];
			if (!ifVisited) {
				ifVisited = true;
				traversalOrder.emplace_back(currentNode);
			}

			auto& currentNeighbors = neighborList[currentNode];
			for (auto& [neighbor, weight] : currentNeighbors) {
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
			out << std::endl;
		}

		return traversalOrder;
	}

	template<typename DataType, typename WeightType>
	std::pair<bool, std::vector<DataType>> depthFirstSearch(const GraphClasses::Graph<DataType, WeightType>& g, const DataType startNode, const DataType nodeToFind, const AlgorithmBehavior behavior, std::ostream& out) {
		std::unordered_map<DataType, bool> visited;
		std::vector<DataType> traversalOrder;

		auto neighborList = g.getNeighbors();

		for (auto& [node, neighbors] : neighborList) {
			visited[node] = false;
		}

		std::stack<DataType> stack;
		stack.emplace(startNode);

		DataType currentNode;
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
					out << "Node [" << nodeToFind << "] found" << std::endl;
				}
				return std::make_pair(true, traversalOrder);
			}

			auto& currentNeighbors = neighborList[currentNode];
			for (auto& [neighbor, weight] : currentNeighbors) {
				if (!visited[neighbor]) {
					stack.emplace(neighbor);
				}
			}
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			out << "Node [" << nodeToFind << "] not found" << std::endl;
		}

		return std::make_pair(false, traversalOrder);
	}

	template<typename DataType, typename WeightType>
	std::vector<DataType> breadthFirstTraverse(const GraphClasses::Graph<DataType, WeightType>& g, const DataType startNode, const AlgorithmBehavior behavior, std::ostream& out) {
		std::unordered_map<DataType, bool> visited;
		std::vector<DataType> traversalOrder;

		auto neighborList = g.getNeighbors();

		for (auto& [node, neighbors] : neighborList) {
			visited[node] = false;
		}

		std::queue<DataType> queue;
		queue.emplace(startNode);

		DataType currentNode;
		while (!queue.empty()) {
			currentNode = queue.front();
			queue.pop();

			auto& ifVisited = visited[currentNode];
			if (!ifVisited) {
				ifVisited = true;
				traversalOrder.emplace_back(currentNode);
			}

			auto& currentNeighbors = neighborList[currentNode];
			for (auto& [neighbor, weight] : currentNeighbors) {
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
			out << std::endl;
		}

		return traversalOrder;
	}

	template<typename DataType, typename WeightType>
	std::pair<bool, std::vector<DataType>> breadthFirstSearch(const GraphClasses::Graph<DataType, WeightType>& g, const DataType startNode, const DataType nodeToFind, const AlgorithmBehavior behavior, std::ostream& out) {
		std::unordered_map<DataType, bool> visited;
		std::vector<DataType> traversalOrder;

		auto neighborList = g.getNeighbors();

		for (auto& [node, neighbors] : neighborList) {
			visited[node] = false;
		}

		std::queue<DataType> queue;
		queue.emplace(startNode);

		DataType currentNode;
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
					out << "Node [" << nodeToFind << "] found" << std::endl;
				}
				return std::make_pair(true, traversalOrder);
			}

			auto& currentNeighbors = neighborList[currentNode];
			for (auto& [neighbor, weight] : currentNeighbors) {
				if (!visited[neighbor]) {
					queue.emplace(neighbor);
				}
			}
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			out << "Node [" << nodeToFind << "] not found" << std::endl;
		}

		return std::make_pair(false, traversalOrder);
	}

	template<typename DataType, typename WeightType>
	std::pair<std::vector<DataType>, WeightType> dijkstraShortestPath(const GraphClasses::Graph<DataType, WeightType>& g, const DataType startNode, const DataType endNode, const AlgorithmBehavior behavior, std::ostream& out) {
		std::unordered_map<DataType, WeightType> distances;
		std::unordered_map<DataType, bool> visited;
		std::unordered_map<DataType, std::optional<DataType>> parents;

		using pqData = GraphClasses::Edge<DataType, WeightType>;
		std::priority_queue<pqData, std::vector<pqData>, internal::EdgeComparator<DataType, WeightType>> pq;

		auto neighborList = g.getNeighbors();

		for (auto& [node, neighbors] : neighborList) {
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
		std::vector<DataType> path;

		if (pathFound) {
			DataType endCpy = endNode;
			path.emplace_back(endNode);
			
			std::optional<DataType> parent = parents[endCpy];

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
				for (auto& elem : path) {
					out << "[" << elem << "] ";
				}
				
				out << "\nwith total distance: " << distances[endNode] << std::endl;
			} else {
				out << "No path found between [" << startNode << "] and [" << endNode << "]" << std::endl;
			}
		}

		return std::make_pair(path, distances[endNode]);
	}
	
	template<typename DataType, typename WeightType>
	std::unordered_map<DataType, std::pair<std::vector<DataType>, WeightType>> bellmanFordShortestPaths(const GraphClasses::Graph<DataType, WeightType>& g, const DataType startNode, const AlgorithmBehavior behavior, std::ostream& out) {
		std::unordered_map<DataType, WeightType> distances;
		std::unordered_map<DataType, std::optional<DataType>> parents;

		auto neighborsList = g.getNeighbors();

		for (auto& [node, neighbors] : neighborsList) {
			distances[node] = GraphClasses::MAX_WEIGHT<WeightType>;
		}

		distances[startNode] = static_cast<WeightType>(0);
		parents[startNode]; // only startNode will have the empty optional

		size_t relaxationCount = g.getNodeCount() - static_cast<size_t>(1);

		for (size_t r = static_cast<size_t>(0); internal::lessThan(r, relaxationCount); ++r) {
			for (auto& [node, neighbors] : neighborsList) {
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
		for (auto& [node, neighbors] : neighborsList) {
			for (auto& [neighbor, weight] : neighbors) {
				auto& nodeDist = distances[node];

				if (!internal::equals(nodeDist, GraphClasses::MAX_WEIGHT<WeightType>) &&
					internal::lessThan(nodeDist + weight.value_or(static_cast<WeightType>(1)), distances[neighbor])) {
						GRAPH_ERROR("Graph contins negative cycle");
						exit(EXIT_FAILURE);
				}
			}
		}

		// path reconstruction
		std::unordered_map<DataType, std::pair<std::vector<DataType>, WeightType>> paths;

		for (auto& [node, distFromStart] : distances) {
			paths[node] = std::make_pair(std::vector<DataType>{}, distFromStart);

			if (internal::equals(distFromStart, GraphClasses::MAX_WEIGHT<WeightType>) || internal::equals(node, startNode)) {
				continue;
			}

			DataType pathNode = node;

			auto& pathVect = paths[node].first;
			pathVect.emplace_back(pathNode);

			std::optional<DataType> parent = parents[pathNode];

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
					out << "There is no possible path between [" << startNode << "] and [" << node << "]" << std::endl;
					continue;
				}

				out << "Distance from [" << startNode << "] to [" << node << "] is: " << distances[node] << "\n\t Path: ";

				auto& pathVect = pathAndDist.first;

				for (auto& elem : pathVect) {
					out << "[" << elem << "] ";
				}

				out << std::endl;
			}
		}

		return paths;
	}

	template<typename DataType, typename WeightType>
	std::unordered_map<DataType, std::unordered_map<DataType, WeightType>> floydWarshallAllShortestPaths(const GraphClasses::Graph<DataType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		std::unordered_map<DataType, std::unordered_map<DataType, WeightType>> distances;

		auto neighborList = g.getNeighbors();

		for (auto& [node1, neighbors1] : neighborList) {
			for (auto& [node2, neighbors2] : neighborList) {
				distances[node1][node2] = GraphClasses::MAX_WEIGHT<WeightType>;
			}
		}

		for (auto& [node, neighbors] : neighborList) {
			distances[node][node] = static_cast<WeightType>(0);
			for (auto& [neighbor, weight] : neighbors) {
				// check needed in case of multigraph
				auto w = weight.value_or(static_cast<WeightType>(1));
				auto& dist = distances[node][neighbor];
				if (w < dist) {
					dist = w;
				}
			}
		}

		for (auto& [mid, n1] : neighborList) {
			for (auto& [start, n2] : neighborList) {
				auto& startMid = distances[start][mid];

				if (!internal::equals(startMid, GraphClasses::MAX_WEIGHT<WeightType>)) {
					for (auto& [end, n3] : neighborList) {
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

		for (auto& [node, distToOthers] : distances) {
			if (internal::lessThan(distances[node][node], static_cast<WeightType>(0))) {
				GRAPH_ERROR("Graph contins negative cycle"); // should this be an error ?
				exit(EXIT_FAILURE);
			}
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			for (auto& [node, neighbors] : distances) {
				for (auto& [neighbor, distance] : neighbors) {
					if (internal::equals(distance, GraphClasses::MAX_WEIGHT<WeightType>)) {
						out << "There is no possible path between [" << node << "] and [" << neighbor << "]" << std::endl;
					}
					else {
						out << "Shortest distance between [" << node << "] and [" << neighbor << "] is: " << distance << std::endl;
					}
				}

				out << std::endl;
			}
		}

		return distances;
	}

	template<typename DataType, typename WeightType>
	std::unordered_set<DataType> findArticulationPoints(const GraphClasses::Graph<DataType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		if (internal::equals(g.getGraphType(), GraphClasses::GraphType::Directed)) {
			GRAPH_ERROR("Must specify startNode for directed graphs. Call the appropriate overload of this function!");
			exit(EXIT_FAILURE);
		}

		DataType startNode = (*std::begin(g.getNeighbors())).first;

		return findArticulationPoints(g, startNode, behavior, out);
	}

	template<typename DataType, typename WeightType>
	std::unordered_set<DataType> findArticulationPoints(const GraphClasses::Graph<DataType, WeightType>& g, const DataType startNode, const AlgorithmBehavior behavior, std::ostream& out) {
		internal::ArticulationHelper<DataType, WeightType> internalData;
		internalData.time = static_cast<size_t>(0);
		internalData.parents[startNode];

		auto neighborList = g.getNeighbors();

		for (auto& [node, neighbors] : neighborList) {
			internalData.visited[node] = false;
		}

		internal::articulation__internal(g, startNode, internalData);

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			if (internal::equals(internalData.articulationPoints.size(), static_cast<size_t>(0))) {
				out << "No articulation points found" << std::endl;
			} else {
				out << "Articulation points found:\n\t";

				for (auto& point : internalData.articulationPoints) {
					out << "[" << point << "] ";
				}

				out << std::endl;
			}
		}

		return internalData.articulationPoints;
	}

	template<typename DataType, typename WeightType>
	std::vector<std::pair<DataType, DataType>> findBridges(const GraphClasses::Graph<DataType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		if (internal::equals(g.getGraphType(), GraphClasses::GraphType::Directed)) {
			GRAPH_ERROR("Must specify startNode for directed graphs. Call the appropriate overload of this function!");
			exit(EXIT_FAILURE);
		}

		DataType startNode = (*std::begin(g.getNeighbors())).first;

		return findBridges(g, startNode, behavior, out);
	}

	template<typename DataType, typename WeightType>
	std::vector<std::pair<DataType, DataType>> findBridges(const GraphClasses::Graph<DataType, WeightType>& g, const DataType startNode, const AlgorithmBehavior behavior, std::ostream& out) {
		internal::ArticulationHelper<DataType, WeightType> internalData;
		internalData.time = static_cast<size_t>(0);
		internalData.parents[startNode];

		auto neighborList = g.getNeighbors();

		for (auto& [node, neighbors] : neighborList) {
			internalData.visited[node] = false;
		}

		internal::articulation__internal(g, startNode, internalData);

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			if (internal::equals(internalData.bridges.size(), static_cast<size_t>(0))) {
				out << "No bridges found" << std::endl;
			} else {
				out << "Bridges found:\n";

				for (auto& bridge : internalData.bridges) {

					out << "\t{ [" << bridge.first << "] [" << bridge.second << "] }\n";
				}
			}
		}

		return internalData.bridges;
	}

	template<typename DataType, typename WeightType>
	std::vector<DataType> topsortKhan(const GraphClasses::Graph<DataType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		if (internal::equals(g.getGraphType(), GraphClasses::GraphType::Undirected)) {
			GRAPH_ERROR("Topological sorting makes no sense for undirected graphs!");
			exit(EXIT_FAILURE);
		}

		auto neighborList = g.getNeighbors();
		auto inDegrees = g.getInDegreesOfNodes();

		std::vector<DataType> topologicalOrdering;
		size_t numVisited = static_cast<size_t>(0);

		std::queue<DataType> topsortQueue;
		for (auto& [node, degree] : inDegrees) {
			if (internal::equals(inDegrees[node], static_cast<size_t>(0))) {
				topsortQueue.emplace(node);
			}
		}

		DataType current;
		while (!topsortQueue.empty()) {
			current = topsortQueue.front();
			topsortQueue.pop();

			topologicalOrdering.emplace_back(current);
			++numVisited;

			for (auto& [neighbor, weight] : neighborList[current]) {
				--inDegrees[neighbor];

				if (internal::equals(inDegrees[neighbor], static_cast<size_t>(0))) {	
					topsortQueue.emplace(neighbor);
				}
			}
		}

		// we signal that graph is not acyclic with an empty vector
		if (!internal::equals(numVisited, g.getNodeCount())) {
			topologicalOrdering.clear();
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			if (!internal::equals(numVisited, g.getNodeCount())) {
				out << "Graph is not acyclic!" << std::endl;
			} else {
				out << "Topological ordering:\n\t";

				for (auto& val : topologicalOrdering) {
					out << "[" << val << "] ";
				}

				out << "\n" << std::endl;
			}
		}

		return topologicalOrdering;
	}

	template<typename DataType, typename WeightType>
	WeightType mcstPrimTotalCostOnly(const GraphClasses::Graph<DataType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		auto ret = mcstPrim(g, AlgorithmBehavior::ReturnOnly, out);

		WeightType totalCost = static_cast<WeightType>(0);
		for (auto& [node1, node2, weight] : ret) {
			totalCost += weight;
		}

		if (internal::equals(behavior, GraphAlgorithms::AlgorithmBehavior::PrintAndReturn)) {
			out << "Total cost of minimum cost spanning tree is: " << totalCost << std::endl;
		}

		return totalCost;
	}

	template<typename DataType, typename WeightType>
	std::vector<std::tuple<DataType, DataType, WeightType>> mcstPrim(const GraphClasses::Graph<DataType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		if (internal::equals(g.getGraphType(), GraphClasses::GraphType::Directed)) {
			GRAPH_ERROR("Minimum cost spanning tree for directed graphs currently not supported");
			exit(EXIT_FAILURE);
		}

		std::unordered_map<DataType, WeightType> distances;
		std::unordered_map<DataType, bool> visited;
		std::unordered_map<DataType, std::optional<DataType>> parents;

		using pqData = GraphClasses::Edge<DataType, WeightType>;
		std::priority_queue<pqData, std::vector<pqData>, internal::EdgeComparator<DataType, WeightType>> pq;

		auto neighborList = g.getNeighbors();

		for (auto& [node, neighbors] : neighborList) {
			distances[node] = GraphClasses::MAX_WEIGHT<WeightType>;
			visited[node] = false;
		}

		DataType startNode = (*std::begin(neighborList)).first;

		distances[startNode] = static_cast<WeightType>(0);
		parents[startNode]; // only startNode will have the empty optional
		pq.emplace(startNode, static_cast<WeightType>(0));

		for (auto& [neighbor, weight] : neighborList[startNode]) {
			pq.emplace(neighbor, GraphClasses::MAX_WEIGHT<WeightType>);
		}

		size_t nodeCount = g.getNodeCount();
		for (size_t i = static_cast<size_t>(0); internal::lessThan(i, nodeCount); ++i) {
			auto [currentNode, distToCurrentNode] = pq.top();
			pq.pop();

			// distances[currentNode] = distToCurrentNode.value_or(1);    not needed?

			if (!visited[currentNode]) {
				visited[currentNode] = true;

				for (auto& [neighbor, weight] : neighborList[currentNode]) {
					if (!visited[neighbor]) {
						if (internal::lessThan(weight.value_or(static_cast<WeightType>(1)), distances[neighbor])) {
							distances[neighbor] = weight.value_or(static_cast<WeightType>(1));
							parents[neighbor] = currentNode;
							pq.emplace(neighbor, distances[neighbor]);
						}
					}
				}
			}
		}

		std::vector<std::tuple<DataType, DataType, WeightType>> mcst;
		for (auto& [node, neighbors] : neighborList) {
			if (parents[node].has_value()) {
				mcst.emplace_back(parents[node].value(), node, distances[node]);
			}
		}

		if (internal::equals(behavior, GraphAlgorithms::AlgorithmBehavior::PrintAndReturn)) {
			WeightType totalCost = static_cast<WeightType>(0);

			out << "Minimum cost spanning tree consists of the following edges:\n";

			for (auto& [node1, node2, weight] : mcst) {
				out << "\t(" << node1 << ", " << node2 << ") weight " << weight << "\n";
				totalCost += weight;
			}

			out << "\nTotal cost of minimum cost spanning tree is: " << totalCost << "\n" << std::endl;
		}

		return mcst;
	}

	template<typename DataType, typename WeightType>
	std::vector<std::tuple<DataType, DataType, WeightType>> mcstKruskal(const GraphClasses::Graph<DataType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		if (internal::equals(g.getGraphType(), GraphClasses::GraphType::Directed)) {
			GRAPH_ERROR("Minimum cost spanning tree for directed graphs currently not supported");
			exit(EXIT_FAILURE);
		}

		auto neighborList = g.getNeighbors();
		internal::DisjointSet ds{g};

		auto cmp = [](const auto& t1, const auto& t2) { return internal::lessThan(std::get<2>(t1), std::get<2>(t2)); };
		std::multiset<std::tuple<DataType, DataType, WeightType>, decltype(cmp)> allEdges(cmp);

		for (auto& [node, neighbors] : neighborList) {
			for (auto& [neighbor, weight] : neighbors) {
				allEdges.emplace(node, neighbor, weight.value_or(static_cast<WeightType>(1)));
			}
		}

		std::vector<std::tuple<DataType, DataType, WeightType>> mcst;
		size_t mcstSize = g.getNodeCount() - static_cast<size_t>(1);
		size_t addedEdges = static_cast<size_t>(0);

		for (auto& [node1, node2, edgeWeight] : allEdges) {
			if (internal::greaterThan(addedEdges, mcstSize) || internal::equals(addedEdges, mcstSize)) {
				break;
			}

			DataType root1 = ds.findInDisjointSet(node1);
			DataType root2 = ds.findInDisjointSet(node2);

			if (!internal::equals(root1, root2)) {
				mcst.emplace_back(node1, node2, edgeWeight);
				ds.unionDisjointSets(root1, root2);
				++addedEdges;
			}
		}

		if (internal::equals(behavior, GraphAlgorithms::AlgorithmBehavior::PrintAndReturn)) {
			WeightType totalCost = static_cast<WeightType>(0);

			out << "Minimum cost spanning tree consists of the following edges:\n";

			for (auto& [node1, node2, weight] : mcst) {
				out << "\t(" << node1 << ", " << node2 << ") weight " << weight << "\n";
				totalCost += weight;
			}

			out << "\nTotal cost of minimum cost spanning tree is: " << totalCost << "\n" << std::endl;
		}

		return mcst;
	}


	template<typename DataType, typename WeightType>
	std::vector<std::unordered_set<DataType>> findStronglyConnectedComponentsTarjan(const GraphClasses::Graph<DataType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		if (internal::equals(g.getGraphType(), GraphClasses::GraphType::Directed)) {
			GRAPH_ERROR("Must specify startNode for directed graphs. Call the appropriate overload of this function!");
			exit(EXIT_FAILURE);
		}

		DataType startNode = (*std::begin(g.getNeighbors())).first;
		return findStronglyConnectedComponentsTarjan(g, startNode, behavior, out);
	}


	template<typename DataType, typename WeightType>
	std::vector<std::unordered_set<DataType>> findStronglyConnectedComponentsTarjan(const GraphClasses::Graph<DataType, WeightType>& g, const DataType startNode, const AlgorithmBehavior behavior, std::ostream& out) {
		internal::TarjanHelper<DataType, WeightType> internalData;
		internalData.time = static_cast<size_t>(1);
		
		auto neighborList = g.getNeighbors();

		for (auto& [node, neighbors] : neighborList) {
			internalData.inStack[node] = false;
			// unvisited nodes will have their time set to 0
			internalData.times[node] = static_cast<size_t>(0);
		}

		internal::tarjan__internal(g, startNode, internalData);

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			size_t i = static_cast<size_t>(1);

			// auto& components = internalData.components;

			for (auto& component : internalData.components) {
				out << "Component " << i << " consists of " << component.size() << " nodes:\n\t";

				for (auto& node : component) {
					out << "[" << node << "] ";
				}

				out << std::endl;
				++i;
			}

			out << std::endl;
		}

		return internalData.components;
	}

	template<typename DataType, typename WeightType>
	std::vector<std::unordered_set<DataType>> findWeaklyConnectedComponents(const GraphClasses::Graph<DataType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		GraphClasses::Graph<DataType, WeightType> gCopy = g;
		auto neighborList = gCopy.getNeighbors();
		
		if (internal::equals(gCopy.getGraphType(), GraphClasses::GraphType::Directed)) {
			for (auto& [node, neighbors] : neighborList) {
				for (auto& [neighbor, weight] : neighbors) {
						gCopy.addEdge(neighbor, GraphClasses::Edge(node, weight));
				}
			}
		}

		std::unordered_map<DataType, bool> visited;

		for (auto& [node, neighbor] : neighborList) {
			visited[node] = false;
		}

		std::vector<std::unordered_set<DataType>> weaklyConnectedComponents;

		for (auto& [node, neighbor] : neighborList) {
			if (!visited[node]) {
				auto dfTraverseOrder = GraphAlgorithms::depthFirstTraverse(gCopy, node, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);

				std::unordered_set<DataType> component;

				for (auto& dfsNode : dfTraverseOrder) {
					visited[dfsNode] = true;
					component.emplace(dfsNode);
				}
				
				weaklyConnectedComponents.emplace_back(component);
			}
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			size_t i = static_cast<size_t>(1);

			for (auto& component : weaklyConnectedComponents) {
				out << "Component " << i << " consists of " << component.size() << " nodes:\n\t";

				for (auto& node : component) {
					out << "[" << node << "] ";
				}

				out << "\n";
				++i;
			}

			out << std::endl;
		}

		return weaklyConnectedComponents;
	}

	template<typename DataType, typename WeightType>
	std::unordered_set<DataType> findIsolatedNodes(const GraphClasses::Graph<DataType, WeightType>& g, const AlgorithmBehavior behavior, std::ostream& out) {
		std::unordered_set<DataType> isolatedNodes;

		if (internal::equals(g.getGraphType(), GraphClasses::GraphType::Undirected)) {
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
			auto neighborList = g.getNeighbors();

			for (auto& [node, neighbors] : neighborList) {
				if (internal::equals(inDegrees[node], static_cast<size_t>(0)) && internal::equals(outDegrees[node], static_cast<size_t>(0))) {
					isolatedNodes.emplace(node);
				}
			}
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			if (internal::equals(isolatedNodes.size(), static_cast<size_t>(0))) {
				out << "Graph contains no isolated nodes" << std::endl;
			}
			else {
				out << "Found " << isolatedNodes.size() << " isolated nodes:\n";

				for (auto& node : isolatedNodes) {
						out << "[" << node << "] ";
				}

				out << "\n" << std::endl;
			}
		}

		return isolatedNodes;
	}

} // namespace GraphAlgorithms


// internal namesapce for helper funcitons, not inteded for end user
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

	template<typename DataType, typename WeightType>
	void exportDirectedGraph(const GraphClasses::Graph<DataType, WeightType>& g, const char* filePath) {
		std::ofstream file(filePath);

		if (!file) {
			GRAPH_ERROR("Invalid file!");
			exit(EXIT_FAILURE);
		}

		auto neighborList = g.getNeighbors();
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

	template<typename DataType, typename WeightType>
	void exportUndirectedGraph(const GraphClasses::Graph<DataType, WeightType>& g, const char* filePath) {
		std::ofstream file(filePath);
		if (!file) {
			GRAPH_ERROR("Invalid file!");
			exit(EXIT_FAILURE);
		}

		// for undirected graphs, we must only write one direction of an edge, or else on next read from file the number of edges will be doubled
		std::unordered_map<DataType, std::unordered_set<GraphClasses::Edge<DataType, WeightType>, internal::EdgeStructHasher<DataType, WeightType>>> doNotAdd;
		auto neighborList = g.getNeighbors();

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

	template<typename DataType, typename WeightType>
	struct EdgeComparator {
		bool operator()(const GraphClasses::Edge<DataType, WeightType>& e1, const GraphClasses::Edge<DataType, WeightType>& e2) const { 
			return internal::greaterThan(e1.weight.value(), e2.weight.value()); 
		}
	};

	template<typename DataType, typename WeightType>
	struct EdgeStructHasher {
		size_t operator()(const GraphClasses::Edge<DataType, WeightType>& obj) const {
			std::hash<DataType> nHash;
			std::hash<WeightType> wHash;
			// TODO:  try finding a better alternative
			return nHash(obj.neighbor) + wHash(obj.weight.value_or(static_cast<size_t>(0)));
		}
	};

	template<typename DataType, typename WeightType>
	struct CompleteEdgeHasher {
		size_t operator()(const std::pair<DataType, GraphClasses::Edge<DataType, WeightType>>& obj) const {
			std::hash<DataType> nHash;
			std::hash<WeightType> wHash;
			// TODO:  try finding a better alternative
			return nHash(obj.first) + nHash(obj.second.neighbor) + wHash(obj.second.weight.value_or(static_cast<size_t>(0)));
		}
	};

	template<typename DataType, typename WeightType>
	struct ArticulationHelper {
		public:
			size_t time;
			std::unordered_map<DataType, size_t> times;
			std::unordered_map<DataType, size_t> lowerTimes;
			std::unordered_map<DataType, bool> visited;
			std::unordered_map<DataType, std::optional<DataType>> parents;
			std::unordered_set<DataType> articulationPoints;
			std::vector<std::pair<DataType, DataType>> bridges;
	};

	// this one internal function is used both for findArticulationPoints and findBridges as these to algorithms are very simmilar
	template<typename DataType, typename WeightType>
	void articulation__internal(const GraphClasses::Graph<DataType, WeightType>& g, const DataType startNode, ArticulationHelper<DataType, WeightType>& internalData) {
		internalData.visited[startNode] = true;
		internalData.times[startNode] = internalData.time;
		internalData.lowerTimes[startNode] = internalData.time;
		++internalData.time;

		auto neighborList = g.getNeighbors();
		size_t numChildren = static_cast<size_t>(0);

		auto& startNodeParent = internalData.parents[startNode];
		auto& startNodeTime = internalData.times[startNode];
		auto& startNodeLowerTime = internalData.lowerTimes[startNode];

		for (auto& [neighbor, weight] : neighborList[startNode]) {
			auto& neighborLowerTime = internalData.lowerTimes[neighbor];

			if (!internalData.visited[neighbor]) {
				++numChildren;
				internalData.parents[neighbor] = startNode;

				articulation__internal(g, neighbor, internalData);

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

			} else {
				auto& neighborTime = internalData.times[neighbor];

				if (startNodeParent.has_value() && !internal::equals(neighbor, startNodeParent.value()) &&
						internal::lessThan(neighborTime, startNodeLowerTime)) {
					startNodeLowerTime = neighborTime;
				}
			}
		}

		return;
	}

	template<typename DataType, typename WeightType>
	class DisjointSet {
		public:
			explicit DisjointSet(const GraphClasses::Graph<DataType, WeightType>& g) {
				auto neighborList = g.getNeighbors();

				for (auto& [node, neighbors] : neighborList) {
					parent[node] = node;
					rank[node] = static_cast<size_t>(0);
				}
			}

			DataType findInDisjointSet(const DataType node) {
				DataType nodeCpy = node;
				DataType root = node;

				while (!internal::equals(root, parent[root])) {
					root = parent[root];
				}
				
				while (!internal::equals(nodeCpy, root)) {
					DataType tmp = parent[nodeCpy];
					parent[nodeCpy] = root;
					nodeCpy = tmp;
				}

				return root;
			}

			void unionDisjointSets(const DataType root1, const DataType root2) {
				if (internal::greaterThan(rank[root1], rank[root2])) {
					parent[root2] = root1;
				} else if (internal::lessThan(rank[root1], rank[root2])) {
					parent[root1] = root2;
				} else {
					parent[root1] = root2;
					++rank[root2];
				}
			}

		private:
			std::unordered_map<DataType, DataType> parent;
			std::unordered_map<DataType, size_t> rank;
	};

	template<typename DataType, typename WeightType>
	struct TarjanHelper {
		public:
			size_t time;
			std::unordered_map<DataType, size_t> times;
			std::unordered_map<DataType, size_t> lowerTimes;
			std::stack<DataType> traversalOrder;
			std::unordered_map<DataType, bool> inStack;
			std::vector<std::unordered_set<DataType>> components;
	};

	template<typename DataType, typename WeightType>
	void tarjan__internal(const GraphClasses::Graph<DataType, WeightType>& g, const DataType startNode, TarjanHelper<DataType, WeightType>& internalData) {
		internalData.times[startNode] = internalData.time;
		internalData.lowerTimes[startNode] = internalData.time;
		++internalData.time;
		internalData.traversalOrder.emplace(startNode);
		internalData.inStack[startNode] = true;

		auto neighborList = g.getNeighbors();
		for (auto& [neighbor, weight] : neighborList[startNode]) {
			auto& startNodeLowerTime = internalData.lowerTimes[startNode];
			auto& neighborTime = internalData.times[neighbor];
			auto& neighborLowerTime = internalData.lowerTimes[neighbor];

			if (internal::equals(neighborTime, static_cast<size_t>(0))) {
				tarjan__internal(g, neighbor, internalData);

				if (internal::lessThan(neighborLowerTime, startNodeLowerTime)) {
					startNodeLowerTime = neighborLowerTime;
				}
			} else if (internalData.inStack[neighbor] && internal::lessThan(neighborTime, startNodeLowerTime)) {
				startNodeLowerTime = neighborTime;
			}
		}

		// component found
		if (internal::equals(internalData.times[startNode], internalData.lowerTimes[startNode])) {
			DataType componentNode;
			std::unordered_set<DataType> component;

			while (true) {
				componentNode = internalData.traversalOrder.top();
				internalData.traversalOrder.pop();

				component.emplace(componentNode);
				internalData.inStack[componentNode] = false;

				if (internal::equals(componentNode, startNode)) {
					internalData.components.emplace_back(component);
					break;
				}
			}
		}

		return;
	}

} // namespace internal


#endif //__SIMPLE_GRAPHS__