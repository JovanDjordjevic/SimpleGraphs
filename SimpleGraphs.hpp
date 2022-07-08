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
	static WeightType MAX_WEIGHT = std::numeric_limits<WeightType>::max();

	template<typename WeightType>
	static WeightType MIN_WEIGHT = std::numeric_limits<WeightType>::lowest();

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
			explicit Edge(DataType neighbor, std::optional<WeightType> weight = {}) 
				: neighbor(neighbor), weight(weight) 
			{}

		public:
			DataType neighbor;
			std::optional<WeightType> weight;
	};

	template<typename DataType, typename WeightType = int>
	class Graph {
		public:
			explicit Graph(GraphType graphType = GraphType::Unset, GraphWeights graphWeights = GraphWeights::Unset);

			void configureDirections(GraphType graphType);
			void configureWeights(GraphWeights graphWeights);
			bool isConfigured();
			void clearGraph();

			void readFromTxt(const char* filePath);
			void writeToTxt(const char* filePath);
			void exportToTxt(const char* filePath); // TODO: export in format that SimpleGraphs can read

			void addNode(DataType node);
			void addEdge(DataType startNode, DataType neighborNode);                        // for unweighted graphs
			void addEdge(DataType startNode, DataType neighborNode, WeightType edgeWeight); // for weighted graphs
			void addEdge(DataType startNode, Edge<DataType, WeightType> edge);
			void deleteEdge(DataType startNode, DataType endNode);
			void deleteNode(DataType node); // TODO
			size_t getNodeCount();
			size_t getEdgeCount();

			double getDensity();
			// ...

			GraphType getGraphType();
			GraphWeights getGraphWeights();
			std::unordered_map<DataType, std::vector<Edge<DataType, WeightType>>> getNeighbors();

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
	GraphClasses::Graph<DataType, WeightType> mergeGraphs(GraphClasses::Graph<DataType, WeightType>& g1, GraphClasses::Graph<DataType, WeightType>& g2);

	template<typename DataType, typename WeightType>
	GraphClasses::Graph<DataType, WeightType> intersectGraphs(GraphClasses::Graph<DataType, WeightType>& g1, GraphClasses::Graph<DataType, WeightType>& g2);

	template<typename DataType, typename WeightType>
	GraphClasses::Graph<DataType, WeightType> getSubgraphFromNodes(GraphClasses::Graph<DataType, WeightType>& g, std::unordered_set<DataType>& nodes);

	// ...
} // namespace GraphUtility

namespace GraphAlgorithms {
	enum class AlgorithmBehavior {
		ReturnOnly,
		PrintAndReturn
	};

	template<typename DataType, typename WeightType>
	std::vector<DataType> dfs(GraphClasses::Graph<DataType, WeightType>& g, DataType startNode,
		AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	template<typename DataType, typename WeightType>
	std::vector<DataType> bfs(GraphClasses::Graph<DataType, WeightType>& g, DataType startNode,
		AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	template<typename DataType, typename WeightType>
	std::vector<DataType> dijkstra(GraphClasses::Graph<DataType, WeightType>& g, DataType startNode, DataType endNode,
		AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	template<typename DataType, typename WeightType>
	std::unordered_map<DataType, std::vector<DataType>> bellmanFord(GraphClasses::Graph<DataType, WeightType>& g, DataType startNode,
		AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	// NOTE: at this time, Floyd-Warshall algorithm only returns the distances between pairs of nodes and not the paths themselves
	// TODO: implement returning of paths
	template<typename DataType, typename WeightType>
	std::unordered_map<DataType, std::unordered_map<DataType, WeightType>> floydWarshall(GraphClasses::Graph<DataType, WeightType>& g,
		AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	// without start node (only available for undirected graphs)
	template<typename DataType, typename WeightType>
	std::unordered_set<DataType> findArticulationPoints(GraphClasses::Graph<DataType, WeightType>& g,
		AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	// with start node (available for both undirected and directed)
	// NOTE: when using this function for directed graphs, only nodes in the corresponding dfs tree will be checked
	template<typename DataType, typename WeightType>
	std::unordered_set<DataType> findArticulationPoints(GraphClasses::Graph<DataType, WeightType>& g, DataType startNode,
		AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	// without start node (only available for undirected graphs)
	template<typename DataType, typename WeightType>
	std::vector<std::pair<DataType, DataType>> findBridges(GraphClasses::Graph<DataType, WeightType>& g,
		AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	// with start node (available for both undirected and directed)
	// NOTE: when using this function for directed graphs, only nodes in the corresponding dfs tree will be checked
	template<typename DataType, typename WeightType>
	std::vector<std::pair<DataType, DataType>> findBridges(GraphClasses::Graph<DataType, WeightType>& g, DataType startNode,
		AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	template<typename DataType, typename WeightType>
	std::vector<DataType> topsortKhan(GraphClasses::Graph<DataType, WeightType>& g, AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn,
		std::ostream& out = std::cout);

	template<typename DataType, typename WeightType>
	WeightType mcstPrimTotalCostOnly(GraphClasses::Graph<DataType, WeightType>& g, AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn,
		std::ostream& out = std::cout);

	template<typename DataType, typename WeightType>
	std::vector<std::tuple<DataType, DataType, WeightType>> mcstPrim(GraphClasses::Graph<DataType, WeightType>& g,
		AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	template<typename DataType, typename WeightType>
	std::vector<std::tuple<DataType, DataType, WeightType>> mcstKruskal(GraphClasses::Graph<DataType, WeightType>& g,
		AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	// without start node (only available for undirected graphs)
	template<typename DataType, typename WeightType>
	std::vector<std::unordered_set<DataType>> findStronglyConnectedComponentsTarjan(GraphClasses::Graph<DataType, WeightType>& g,
		AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	// NOTE: when using this function for directed graphs, only nodes in the corresponding dfs tree will be checked
	template<typename DataType, typename WeightType>
	std::vector<std::unordered_set<DataType>> findStronglyConnectedComponentsTarjan(GraphClasses::Graph<DataType, WeightType>& g, DataType startNode,
		AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

	// NOTE: currently only works for undirected graphs
	// TODO: implement a funciton that also works for directed graphs
	template<typename DataType, typename WeightType>
	std::vector<std::unordered_set<DataType>> findWeaklyConnectedComponents(GraphClasses::Graph<DataType, WeightType>& g,
		AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

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
	// static WeightType FLOATING_POINT_EPSIOLON = static_cast<WeightType>(0.000001);
	static WeightType FLOATING_POINT_EPSIOLON = WeightType{0.000001};

	template<typename WeightType>
	std::enable_if_t<std::is_floating_point_v<WeightType>, bool>
	equals(WeightType lhs, WeightType rhs);

	template<typename WeightType>
	std::enable_if_t<!std::is_floating_point_v<WeightType>, bool>
	equals(WeightType lhs, WeightType rhs);

	template<typename WeightType>
	std::enable_if_t<std::is_floating_point_v<WeightType>, bool>
	lessThan(WeightType lhs, WeightType rhs);

	template<typename WeightType>
	std::enable_if_t<!std::is_floating_point_v<WeightType>, bool>
	lessThan(WeightType lhs, WeightType rhs);

	template<typename WeightType>
	std::enable_if_t<std::is_floating_point_v<WeightType>, bool>
	greaterThan(WeightType lhs, WeightType rhs);

	template<typename WeightType>
	std::enable_if_t<!std::is_floating_point_v<WeightType>, bool>
	greaterThan(WeightType lhs, WeightType rhs);

	template<typename DataType, typename WeightType>
	struct EdgeHasher;

	template<typename DataType, typename WeightType>
	struct ArticulationHelper;

	template<typename DataType, typename WeightType>
	void articulation__internal(GraphClasses::Graph<DataType, WeightType>& g, DataType startNode, GraphAlgorithms::AlgorithmBehavior behavior,
		std::ostream& out, ArticulationHelper<DataType, WeightType>& internalData);

	template<typename DataType, typename WeightType>
	class DisjointSet;

	template<typename DataType, typename WeightType>
	struct TarjanHelper;

	template<typename DataType, typename WeightType>
	void tarjan__internal(GraphClasses::Graph<DataType, WeightType>& g, DataType startNode, TarjanHelper<DataType, WeightType>& internalData);
} // namespace internal


namespace GraphClasses {
	template<typename DataType, typename WeightType>
	std::ostream& operator<<(std::ostream& out, const GraphClasses::Graph<DataType, WeightType>& g) {
		for (auto& [node, neighbors] : g.m_neighbors) {
			out << "Node [" << node << "] has neighbours:" << std::endl;

			if (internal::equals(g.m_graphWeights, GraphWeights::Weighted)) {
				for (auto& val : neighbors) {
					out << "|\t [" << val.neighbor << "], edge weight: " << val.weight.value() << std::endl;
				}
			} else { // unweighted
				for (auto& val : neighbors) {
					out << "|\t [" << val.neighbor << "]" << std::endl;
				}
			}
		}
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
	Graph<DataType, WeightType>::Graph(GraphType graphType, GraphWeights graphWeights) 
			: m_graphType(graphType), m_graphWeights(graphWeights) {
		static_assert(std::is_arithmetic_v<WeightType> && !std::is_same_v<WeightType, bool>, "Weight type must be an arithmetic type except boolean");
		// std::cout << MAX_WEIGHT<WeightType> << " " << MIN_WEIGHT<WeightType> << std::endl;
	}

	template<typename DataType, typename WeightType>
	void Graph<DataType, WeightType>::configureDirections(GraphType graphType) {
		m_graphType = graphType;
	}

	template<typename DataType, typename WeightType>
	void Graph<DataType, WeightType>::configureWeights(GraphWeights graphWeights) {
		m_graphWeights = graphWeights;
	}

	template<typename DataType, typename WeightType>
	bool Graph<DataType, WeightType>::isConfigured() {
		return !internal::equals(m_graphType, GraphType::Unset) && !internal::equals(m_graphWeights, GraphWeights::Unset);
	}

	template<typename DataType, typename WeightType>
	void Graph<DataType, WeightType>::clearGraph() {
		m_neighbors.clear(); // TODO: check if this is enough
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
					addNode(neighbor); // this line is neccessary in case neighbor node is only mentioned as neighbor of another node
					addEdge(node, neighbor, weight);
				}
			} else { // unweighted
				while (lineStream >> neighbor) {
					addNode(neighbor); // this line is neccessary in case neighbor node is only mentioned as neighbor of another node
					addEdge(node, neighbor);
				}
			}
		}

		file.close();
	}

	template<typename DataType, typename WeightType>
	void Graph<DataType, WeightType>::writeToTxt(const char* filePath) {
		std::ofstream file(filePath);

		file << (*this);

		file.close();
	}

	template<typename DataType, typename WeightType>
	void Graph<DataType, WeightType>::addNode(DataType node) {
		m_neighbors[node];
	}

	template<typename DataType, typename WeightType>
	void Graph<DataType, WeightType>::addEdge(DataType startNode, DataType neighborNode) {
		if (internal::equals(m_graphWeights, GraphWeights::Weighted)) {
			GRAPH_ERROR("Graph is weighed and edge weight must be specified! ");
			exit(EXIT_FAILURE);
		}

		if (internal::equals(m_graphType, GraphType::Directed)) {
			m_neighbors[startNode].emplace_back(neighborNode);
		} else { // undirected
			m_neighbors[startNode].emplace_back(neighborNode);
			m_neighbors[neighborNode].emplace_back(startNode);
		}
	}

	template<typename DataType, typename WeightType>
	void Graph<DataType, WeightType>::addEdge(DataType startNode, DataType neighborNode, WeightType edgeWeight) {
		if (internal::equals(m_graphWeights, GraphWeights::Unweighted)) {
			GRAPH_ERROR("Graph is not weighed but you are trying to specify edge weight!");
			exit(EXIT_FAILURE);
		}

		if (internal::equals(m_graphType, GraphType::Directed)) {
			m_neighbors[startNode].emplace_back(neighborNode, edgeWeight);
		} else { // undirected
			m_neighbors[startNode].emplace_back(neighborNode, edgeWeight);
			m_neighbors[neighborNode].emplace_back(startNode, edgeWeight);
		}
	}

	template<typename DataType, typename WeightType>
	void Graph<DataType, WeightType>::addEdge(DataType startNode, Edge<DataType, WeightType> edge) {
		if (internal::equals(m_graphWeights, GraphWeights::Unweighted) && edge.weight.has_value()) {
			GRAPH_ERROR("Graph is unweighed but edge has a weight!");
			exit(EXIT_FAILURE);
		} else if (internal::equals(m_graphWeights, GraphWeights::Weighted) && !edge.weight.has_value()) {
			GRAPH_ERROR("Graph is weighed but edge has no weight!");
			exit(EXIT_FAILURE);
		}

		m_neighbors[startNode].emplace_back(edge.neighbor, edge.weight);
		return;
	}

	// NOTE: will delete all edges that connect start and end nodes in case of a multigraph
	template<typename DataType, typename WeightType>
	void Graph<DataType, WeightType>::deleteEdge(DataType startNode, DataType endNode) {
		auto it_start = m_neighbors.find(startNode);
		auto it_end = m_neighbors.find(endNode);
		if (internal::equals(it_start, std::end(m_neighbors)) || internal::equals(it_end, std::end(m_neighbors))) {
			// std::cout << "Edge does not exist" << std::endl;
			return;
		}

		auto it  = std::begin((*it_start).second);
		auto end = std::end((*it_start).second);
		while (!internal::equals(it, end)) {
			if (internal::equals((*it).neighbor, endNode)) {
				// std::cout << "start->end edge erased" << std::endl;
				((*it_start).second).erase(it);
			}
			++it;
		}

		if (internal::equals(m_graphType, GraphType::Undirected)) {
			auto it  = std::begin((*it_end).second);
			auto end = std::end((*it_end).second);
			while (!internal::equals(it, end)) {
				if (internal::equals((*it).neighbor, startNode)) {
					// std::cout << "end->start edge erased" << std::endl;
					((*it_end).second).erase(it);
				}
				++it;
			}
		}

		return;
	}

	// removes a node and all edges to/from said node
	template<typename DataType, typename WeightType>
	void Graph<DataType, WeightType>::deleteNode(DataType node) {
		if (internal::equals(m_neighbors.find(node), std::end(m_neighbors))) {
			// std::cout << "Node does not exist" << std::endl;
			return;
		}

		m_neighbors[node].clear(); // needed?
		m_neighbors.erase(node);

		for (auto& [node, neighbors] : m_neighbors) {
			auto it_begin = std::begin(neighbors);
			auto it_end = std::end(neighbors);
			while (internal::equals(it_begin, it_end)) {
				if (internal::equals((*it_begin).neighbor, node)) {
					neighbors.erase(it_begin);
				}
				++it_begin;
			}
		}

		return;
	}

	template<typename DataType, typename WeightType>
	size_t Graph<DataType, WeightType>::getNodeCount() {
		return m_neighbors.size();
	}

	template<typename DataType, typename WeightType>
	size_t Graph<DataType, WeightType>::getEdgeCount() {
		size_t count = 0;
		for (auto& [node, neighbors] : m_neighbors) {
			count += neighbors.size();
		}
		return count;
	}

	template<typename DataType, typename WeightType>
	double Graph<DataType, WeightType>::getDensity() {
		double density = static_cast<double>(getEdgeCount()) / (getNodeCount() * (getNodeCount() - 1));

		if (m_graphType == GraphType::Undirected) {
			density *= 2;
		}

		return density;
	}

	template<typename DataType, typename WeightType>
	GraphType Graph<DataType, WeightType>::getGraphType() {
		return m_graphType;
	}

	template<typename DataType, typename WeightType>
	GraphWeights Graph<DataType, WeightType>::getGraphWeights() {
		return m_graphWeights;
	}

	template<typename DataType, typename WeightType>
	std::unordered_map<DataType, std::vector<Edge<DataType, WeightType>>> Graph<DataType, WeightType>::getNeighbors() {
		return m_neighbors;
	}

} // namespace GraphClasses


namespace GraphUtility {
	template<typename DataType, typename WeightType>
	GraphClasses::Graph<DataType, WeightType> mergeGraphs(GraphClasses::Graph<DataType, WeightType>& g1, GraphClasses::Graph<DataType, WeightType>& g2) {
		GraphClasses::Graph<DataType, WeightType> newGraph;

		if (!internal::equals(g1.getGraphType(), g2.getGraphType()) || !internal::equals(g1.getGraphWeights(), g2.getGraphWeights())) {
			GRAPH_ERROR("Graphs can only be merged if they have the same type (directed/undirected) and same weights (weighed/unweighed)!");
			exit(EXIT_FAILURE);
		}

		newGraph.configureDirections(g1.getGraphType());
		newGraph.configureWeights(g1.getGraphWeights());

		auto g1NeighborList = g1.getNeighbors();
		for (auto& [node, neighbors] : g1NeighborList) {
			newGraph.addNode(node);
		}

		auto g2NeighborList = g2.getNeighbors();
		for (auto& [node, neighbors] : g2NeighborList) {
			newGraph.addNode(node);
		}

		auto neighborList = newGraph.getNeighbors();
		for (auto& [node, neighbors] : neighborList) {
			auto it1 = g1NeighborList.find(node);
			auto it2 = g2NeighborList.find(node);

			if (!internal::equals(it1, std::end(g1NeighborList)) && !internal::equals(it2, std::end(g2NeighborList))) { // node is in both graphs
				// we avoid adding duplicate edges by putting them in a set first
				std::unordered_set<GraphClasses::Edge<DataType, WeightType>, internal::EdgeHasher<DataType, WeightType>> edgeSet;
				for (auto& edge : g1NeighborList[node]) {
					edgeSet.emplace(edge);
				}

				for (auto& edge : g2NeighborList[node]) {
					edgeSet.emplace(edge);
				}

				for (auto& edge : edgeSet) {
					newGraph.addEdge(node, edge);
				}
			} else if (!internal::equals(it1, std::end(g1NeighborList)) && internal::equals(it2, std::end(g2NeighborList))) { // node is only in g1
				for (auto& edge : g1NeighborList[node]) {
					newGraph.addEdge(node, edge);
				}
			} else if (internal::equals(it1, std::end(g1NeighborList)) && !internal::equals(it2, std::end(g2NeighborList))) { // is only in g2
				for (auto& edge : g2NeighborList[node]) {
					newGraph.addEdge(node, edge);
				}
			}
		}

		return newGraph;
	}

	template<typename DataType, typename WeightType>
	GraphClasses::Graph<DataType, WeightType> intersectGraphs(GraphClasses::Graph<DataType, WeightType>& g1, GraphClasses::Graph<DataType, WeightType>& g2) {
		GraphClasses::Graph<DataType, WeightType> newGraph;

		if (!internal::equals(g1.getGraphType(), g2.getGraphType()) || !internal::equals(g1.getGraphWeights(), g2.getGraphWeights())) {
			GRAPH_ERROR("Graph intersection can only be created if they have the same type (directed/undirected) and same weights (weighed/unweighed)!");
			exit(EXIT_FAILURE);
		}

		newGraph.configureDirections(g1.getGraphType());
		newGraph.configureWeights(g1.getGraphWeights());

		auto g1NeighborList = g1.getNeighbors();
		auto g2NeighborList = g2.getNeighbors();

		for (auto& [node, neighbors] : g1NeighborList) {
			auto it = g2NeighborList.find(node);
			if (!internal::equals(it, std::end(g2NeighborList))) {
				newGraph.addNode(node);

				std::unordered_set<GraphClasses::Edge<DataType, WeightType>, internal::EdgeHasher<DataType, WeightType>> edges;
				auto& shorter = g1NeighborList[node];
				auto& longer = g2NeighborList[node];
				if (internal::lessThan(g2NeighborList[node].size(), g1NeighborList[node].size())) {
					shorter = g2NeighborList[node];
					longer = g1NeighborList[node];
				}

				for (auto& edge : shorter) {
					edges.emplace(edge.neighbor, edge.weight);
				}

				for (auto& edge : longer) {
					if (!internal::equals(edges.find(edge), std::end(edges))) {
						newGraph.addEdge(node, edge);
					}
				}
			}
		}

		return newGraph;
	}

	template<typename DataType, typename WeightType>
	GraphClasses::Graph<DataType, WeightType> getSubgraphFromNodes(GraphClasses::Graph<DataType, WeightType>& g, std::unordered_set<DataType>& nodes) {
		GraphClasses::Graph<DataType, WeightType> newGraph;

		newGraph.configureDirections(g.getGraphType());
		newGraph.configureWeights(g.getGraphWeights());

		auto neighborList = g.getNeighbors();

		for (auto& node : nodes) {
			newGraph.addNode(node);
			for (auto& edge : neighborList[node]) {
				if (!internal::equals(nodes.find(edge.neighbor), std::end(nodes))) {
					newGraph.addEdge(node, edge);
				}
			}
		}

		return newGraph;
	}
} // namespace GraphUtility


namespace GraphAlgorithms {
	template<typename DataType, typename WeightType>
	std::vector<DataType> dfs(GraphClasses::Graph<DataType, WeightType>& g, DataType startNode, AlgorithmBehavior behavior, std::ostream& out) {
		std::unordered_map<DataType, bool> visited;
		std::vector<DataType> dfsSearchTree;

		auto neighborList = g.getNeighbors();

		for (auto& [node, neighbors] : neighborList) {
			visited[node] = false;
		}

		std::stack<DataType> stack;
		stack.emplace(startNode);

		while (!stack.empty()) {
			DataType currentNode = stack.top();
			stack.pop();

			if (!visited[currentNode]) {
				visited[currentNode] = true;
				dfsSearchTree.emplace_back(currentNode);
			}

			auto it = std::cbegin(neighborList[currentNode]);
			auto end = std::cend(neighborList[currentNode]);
			while (!internal::equals(it, end)) {
				if (!visited[(*it).neighbor]) {
					stack.emplace((*it).neighbor);
				}
				++it;
			}
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			out << "Order of DFS traversal:\n\t";
			for (auto& node : dfsSearchTree) {
				out << "[" << node << "] ";
			}
			out << std::endl;
		}

		return dfsSearchTree;
	}

	template<typename DataType, typename WeightType>
	std::vector<DataType> bfs(GraphClasses::Graph<DataType, WeightType>& g, DataType startNode, AlgorithmBehavior behavior, std::ostream& out) {
		std::unordered_map<DataType, bool> visited;
		std::vector<DataType> bfsTraversalOrder;

		auto neighborList = g.getNeighbors();

		for (auto& [node, neighbors] : neighborList) {
			visited[node] = false;
		}

		std::queue<DataType> queue;
		queue.emplace(startNode);

		while (!queue.empty()) {
			DataType currentNode = queue.front();
			queue.pop();

			if (!visited[currentNode]) {
				bfsTraversalOrder.emplace_back(currentNode);
				visited[currentNode] = true;
			}

			auto it = std::cbegin(neighborList[currentNode]);
			auto end = std::cend(neighborList[currentNode]);
			while (!internal::equals(it, end)) {
				if (!visited[(*it).neighbor]) {
					queue.emplace((*it).neighbor);
				}
				++it;
			}
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			out << "Order of BFS traversal:\n\t";
			for (auto& node : bfsTraversalOrder) {
				out << "[" << node << "] ";
			}
			out << std::endl;
		}

		return bfsTraversalOrder;
	}

	template<typename DataType, typename WeightType>
	std::vector<DataType> dijkstra(GraphClasses::Graph<DataType, WeightType>& g, DataType startNode, DataType endNode, AlgorithmBehavior behavior, std::ostream& out) {
		std::unordered_map<DataType, WeightType> distances;
		std::unordered_map<DataType, bool> visited;
		std::unordered_map<DataType, std::optional<DataType>> parents;

		using pqData = GraphClasses::Edge<DataType, WeightType>;
		struct Comparator {
				bool operator()(pqData& e1, pqData& e2) { return internal::greaterThan(e1.weight.value(), e2.weight.value()); }
		};
		std::priority_queue<pqData, std::vector<pqData>, Comparator> pq;

		auto neighborList = g.getNeighbors();

		for (auto& [node, neighbors] : neighborList) {
			distances[node] = GraphClasses::MAX_WEIGHT<WeightType>;
			visited[node] = false;
		}

		distances[startNode] = 0;
		parents[startNode]; // only startNode will have the empty optional
		pq.emplace(startNode, 0);

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
			while (true) {
				std::optional<DataType> parent = parents[endCpy];
				if (!parent.has_value()) {
					break;
				}
				path.emplace_back(parent.value());
				endCpy = parent.value();
			}

			std::reverse(std::begin(path), std::end(path));
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			if (pathFound) {
				auto it = std::cbegin(path);
				auto end = std::cend(path);
				out << "Path found: \n\t";
				while (!internal::equals(it, end)) {
					out << "[" << (*it) << "] ";
					++it;
				}
				out << "\nwith total distance: " << distances[endNode] << std::endl;
			} else {
				out << "No path found between [" << startNode << "] and [" << endNode << "]" << std::endl;
			}
		}

		return path;
	}

	template<typename DataType, typename WeightType>
	std::unordered_map<DataType, std::vector<DataType>> bellmanFord(GraphClasses::Graph<DataType, WeightType>& g, DataType startNode, AlgorithmBehavior behavior, std::ostream& out) {
		std::unordered_map<DataType, WeightType> distances;
		std::unordered_map<DataType, std::optional<DataType>> parents;

		auto neighborsList = g.getNeighbors();

		for (auto& [node, neighbors] : neighborsList) {
			distances[node] = GraphClasses::MAX_WEIGHT<WeightType>;
		}

		distances[startNode] = 0;
		parents[startNode]; // only startNode will have the empty optional

		size_t relaxationCount = g.getNodeCount() - 1;

		for (size_t r = 0; internal::lessThan(r, relaxationCount); ++r) {
			for (auto& [node, neighbors] : neighborsList) {
				for (auto& [neighbor, weight] : neighbors) {
					WeightType newDistnce = distances[node] + weight.value_or(static_cast<WeightType>(1));
					if (internal::lessThan(newDistnce, distances[neighbor])) {
						distances[neighbor] = newDistnce;
						parents[neighbor] = node;
					}
				}
			}
		}

		// negtive cycle check
		for (auto& [node, neighbors] : neighborsList) {
			for (auto& [neighbor, weight] : neighbors) {
				if (internal::lessThan(distances[node] + weight.value_or(static_cast<WeightType>(1)), distances[neighbor])) {
					GRAPH_ERROR("Graph contins negative cycle");
					exit(EXIT_FAILURE);
				}
			}
		}

		// path reconstruction
		std::unordered_map<DataType, std::vector<DataType>> paths;

		for (auto& [node, distFromStart] : distances) {
			paths[node] = {};

			if (internal::equals(distFromStart, GraphClasses::MAX_WEIGHT<WeightType>) || internal::equals(node, startNode)) {
				continue;
			}

			DataType pathNode = node;
			paths[node].emplace_back(pathNode);

			while (true) {
				std::optional<DataType> parent = parents[pathNode];
				if (!parent.has_value()) {
					break;
				}
				paths[node].emplace_back(parent.value());
				pathNode = parent.value();
			}

			std::reverse(std::begin(paths[node]), std::end(paths[node]));
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			for (auto& [node, path] : paths) {
				// path to start node itself is irrelevant
				if (internal::equals(node, startNode)) {
					continue;
				}

				// there is no path to nodes in different components
				if (internal::equals(path.size(), static_cast<size_t>(0))) {
					out << "There is no possible path between [" << startNode << "] and [" << node << "]" << std::endl;
					continue;
				}

				out << "Distance from [" << startNode << "] to [" << node << "] is: " << distances[node] << "\n\t Path: ";
				auto it = std::cbegin(path);
				auto end = std::cend(path);
				while (!internal::equals(it, end)) {
					out << "[" << (*it) << "] ";
					++it;
				}
				out << std::endl;
			}
		}

		return paths;
	}

	template<typename DataType, typename WeightType>
	std::unordered_map<DataType, std::unordered_map<DataType, WeightType>> floydWarshall(GraphClasses::Graph<DataType, WeightType>& g, AlgorithmBehavior behavior, std::ostream& out) {
		std::unordered_map<DataType, std::unordered_map<DataType, WeightType>> distances;

		auto neighborList = g.getNeighbors();

		for (auto& [node1, neighbors1] : neighborList) {
			for (auto& [node2, neighbors2] : neighborList) {
				distances[node1][node2] = GraphClasses::MAX_WEIGHT<WeightType>;
			}
		}

		for (auto& [node, neighbors] : neighborList) {
			for (auto& [neighbor, weight] : neighbors) {
				if (internal::equals(node, neighbor)) {
					distances[node][neighbor] = 0;
				} else {
					distances[node][neighbor] = weight.value_or(static_cast<WeightType>(1));
				}
			}
		}

		for (auto& [mid, n1] : neighborList) {
			for (auto& [start, n2] : neighborList) {
				for (auto& [end, n3] : neighborList) {
					auto& startMid = distances[start][mid];
					auto& midEnd = distances[mid][end];
					auto& startEnd = distances[start][end];
					if (!internal::equals(startMid, GraphClasses::MAX_WEIGHT<WeightType>) && !internal::equals(midEnd, GraphClasses::MAX_WEIGHT<WeightType>) && internal::lessThan((startMid + midEnd), startEnd)) {
						distances[start][end] = startMid + midEnd;
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
					if (internal::equals(distance, GraphClasses::MAX_WEIGHT<WeightType>) || internal::equals(node, neighbor)) {
						continue;
					}
					out << "Shortest distance between [" << node << "] and [" << neighbor << "] is: " << distance << std::endl;
				}

				// FIXME: for string nodes, sometimes 2 new line characters are printed between groups
				out << std::endl;
			}
		}

		return distances;
	}

	template<typename DataType, typename WeightType>
	std::unordered_set<DataType> findArticulationPoints(GraphClasses::Graph<DataType, WeightType>& g, AlgorithmBehavior behavior, std::ostream& out) {
		if (internal::equals(g.getGraphType(), GraphClasses::GraphType::Directed)) {
			GRAPH_ERROR("Must specify startNode for directed graphs. Call the appropriate overload of this function!");
			exit(EXIT_FAILURE);
		}

		DataType startNode = (*std::begin(g.getNeighbors())).first;
		return findArticulationPoints(g, startNode, behavior, out);
	}

	template<typename DataType, typename WeightType>
	std::unordered_set<DataType> findArticulationPoints(GraphClasses::Graph<DataType, WeightType>& g, DataType startNode, AlgorithmBehavior behavior, std::ostream& out) {
		internal::ArticulationHelper<DataType, WeightType> internalData;

		auto neighborList = g.getNeighbors();

		internalData.time = 0u;
		internalData.parents[startNode];
		for (auto& [node, neighbors] : neighborList) {
			internalData.visited[node] = false;
		}

		internal::articulation__internal(g, startNode, behavior, out, internalData);

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			if (internal::equals(internalData.articulationPoints.size(), static_cast<size_t>(0))) {
				out << "No articulation points found" << std::endl;
			} else {
				out << "Articulation points found:" << std::endl;
				for (auto& point : internalData.articulationPoints) {
					out << "[" << point << "] ";
				}
				out << std::endl;
			}
		}

		return internalData.articulationPoints;
	}

	template<typename DataType, typename WeightType>
	std::vector<std::pair<DataType, DataType>> findBridges(GraphClasses::Graph<DataType, WeightType>& g, AlgorithmBehavior behavior, std::ostream& out) {
		if (internal::equals(g.getGraphType(), GraphClasses::GraphType::Directed)) {
			GRAPH_ERROR("Must specify startNode for directed graphs. Call the appropriate overload of this function!");
			exit(EXIT_FAILURE);
		}

		DataType startNode = (*std::begin(g.getNeighbors())).first;
		return findBridges(g, startNode, behavior, out);
	}

	template<typename DataType, typename WeightType>
	std::vector<std::pair<DataType, DataType>> findBridges(GraphClasses::Graph<DataType, WeightType>& g, DataType startNode, AlgorithmBehavior behavior, std::ostream& out) {
		internal::ArticulationHelper<DataType, WeightType> internalData;

		auto neighborList = g.getNeighbors();

		internalData.time = 0u;
		internalData.parents[startNode];
		for (auto& [node, neighbors] : neighborList) {
			internalData.visited[node] = false;
		}

		internal::articulation__internal(g, startNode, behavior, out, internalData);

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			if (internal::equals(internalData.bridges.size(), static_cast<size_t>(0))) {
				out << "No bridges found" << std::endl;
			} else {
				out << "Bridges found:" << std::endl;
				for (auto& bridge : internalData.bridges) {
					out << "\t{ [" << bridge.first << "] [" << bridge.second << "] }" << std::endl;
				}
			}
		}

		return internalData.bridges;
	}

	template<typename DataType, typename WeightType>
	std::vector<DataType> topsortKhan(GraphClasses::Graph<DataType, WeightType>& g, AlgorithmBehavior behavior, std::ostream& out) {
		if (internal::equals(g.getGraphType(), GraphClasses::GraphType::Undirected)) {
			GRAPH_ERROR("Topological sorting makes no sense for undirected graphs!");
			exit(EXIT_FAILURE);
		}

		auto neighborList = g.getNeighbors();

		std::unordered_map<DataType, unsigned> inDegrees;
		for (auto& [node, neighbors] : neighborList) {
			inDegrees[node] = 0u;
		}

		std::vector<DataType> topologicalOrdering;
		unsigned numVisited = 0u;

		for (auto& [node, neighbors] : neighborList) {
			for (auto& [neighbor, weight] : neighbors) {
				++inDegrees[neighbor];
			}
		}

		std::queue<DataType> topsortQueue;
		for (auto& [node, degree] : inDegrees) {
			if (internal::equals(inDegrees[node], 0u)) {
				topsortQueue.emplace(node);
			}
		}

		while (!topsortQueue.empty()) {
			DataType current = topsortQueue.front();
			topsortQueue.pop();
			topologicalOrdering.emplace_back(current);
			++numVisited;

			for (auto& [neighbor, weight] : neighborList[current]) {
				--inDegrees[neighbor];
				if (internal::equals(inDegrees[neighbor], 0u)) {	
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
				out << std::endl;
			}
		}

		return topologicalOrdering;
	}

	template<typename DataType, typename WeightType>
	WeightType mcstPrimTotalCostOnly(GraphClasses::Graph<DataType, WeightType>& g, AlgorithmBehavior behavior, std::ostream& out) {
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
	std::vector<std::tuple<DataType, DataType, WeightType>> mcstPrim(GraphClasses::Graph<DataType, WeightType>& g, AlgorithmBehavior behavior, std::ostream& out) {
		if (internal::equals(g.getGraphType(), GraphClasses::GraphType::Directed)) {
			GRAPH_ERROR("Minimum cost spanning tree for directed graphs currently not supported");
			exit(EXIT_FAILURE);
		}

		std::unordered_map<DataType, WeightType> distances;
		std::unordered_map<DataType, bool> visited;
		std::unordered_map<DataType, std::optional<DataType>> parents;

		using pqData = GraphClasses::Edge<DataType, WeightType>;
		struct Comparator {
				bool operator()(pqData& e1, pqData& e2) { return internal::greaterThan(e1.weight.value(), e2.weight.value()); }
		};
		std::priority_queue<pqData, std::vector<pqData>, Comparator> pq;

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
		for (size_t i = 0; internal::lessThan(i, nodeCount); ++i) {
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
			out << "\nTotal cost of minimum cost spanning tree is: " << totalCost << std::endl;
		}

		return mcst;
	}

	template<typename DataType, typename WeightType>
	std::vector<std::tuple<DataType, DataType, WeightType>> mcstKruskal(GraphClasses::Graph<DataType, WeightType>& g, AlgorithmBehavior behavior, std::ostream& out) {
		if (internal::equals(behavior, GraphAlgorithms::AlgorithmBehavior::PrintAndReturn)) {
			GRAPH_ERROR("Minimum cost spanning tree for directed graphs currently not supported");
			exit(EXIT_FAILURE);
		}

		auto neighborList = g.getNeighbors();
		internal::DisjointSet ds{g};
		std::vector<std::tuple<DataType, DataType, WeightType>> allEdges;

		for (auto& [node, neighbors] : neighborList) {
			for (auto& [neighbor, weight] : neighbors) {
				allEdges.emplace_back(node, neighbor, weight.value_or(static_cast<WeightType>(1)));
			}
		}

		std::sort(std::begin(allEdges), std::end(allEdges), [](auto& t1, auto& t2) { return internal::lessThan(std::get<2>(t1), std::get<2>(t2)); });

		std::vector<std::tuple<DataType, DataType, WeightType>> mcst;
		unsigned mcstSize = g.getNodeCount() - 1u;
		unsigned addedEdges = 0u;

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
			WeightType totalCost = 0;

			out << "Minimum cost spanning tree consists of the following edges:\n";
			for (auto& [node1, node2, weight] : mcst) {
				out << "\t(" << node1 << ", " << node2 << ") weight " << weight << "\n";
				totalCost += weight;
			}
			out << "\nTotal cost of minimum cost spanning tree is: " << totalCost << std::endl;
		}

		return mcst;
	}


	template<typename DataType, typename WeightType>
	std::vector<std::unordered_set<DataType>> findStronglyConnectedComponentsTarjan(GraphClasses::Graph<DataType, WeightType>& g, AlgorithmBehavior behavior, std::ostream& out) {
		if (internal::equals(g.getGraphType(), GraphClasses::GraphType::Directed)) {
			GRAPH_ERROR("Must specify startNode for directed graphs. Call the appropriate overload of this function!");
			exit(EXIT_FAILURE);
		}

		DataType startNode = (*std::begin(g.getNeighbors())).first;
		return findStronglyConnectedComponentsTarjan(g, startNode, behavior, out);
	}


	template<typename DataType, typename WeightType>
	std::vector<std::unordered_set<DataType>> findStronglyConnectedComponentsTarjan(GraphClasses::Graph<DataType, WeightType>& g, DataType startNode, AlgorithmBehavior behavior, std::ostream& out) {
		internal::TarjanHelper<DataType, WeightType> internalData;

		auto neighborList = g.getNeighbors();

		internalData.time = 1u;
		for (auto& [node, neighbors] : neighborList) {
			internalData.inStack[node] = false;
			// unvisited nodes will have their time set to 0
			internalData.times[node] = 0u;
		}

		internal::tarjan__internal(g, startNode, internalData);

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			unsigned i = 1u;
			for (auto& component : internalData.components) {
				out << "Component " << i << " consists of " << component.size() << " nodes:\n\t";
				for (auto& node : component) {
					out << "[" << node << "] ";
				}
				std::cout << std::endl;
				++i;
			}
		}

		return internalData.components;
	}

	template<typename DataType, typename WeightType>
	std::vector<std::unordered_set<DataType>> findWeaklyConnectedComponents(GraphClasses::Graph<DataType, WeightType>& g, AlgorithmBehavior behavior, std::ostream& out) {
		if (internal::equals(g.getGraphType(), GraphClasses::GraphType::Directed)) {
			GRAPH_ERROR("Finding weakly connected components in directed graphs currently not supported!");
			exit(EXIT_FAILURE);
		}

		std::unordered_map<DataType, bool> visited;

		auto neighborList = g.getNeighbors();

		for (auto& [node, neighbor] : neighborList) {
			visited[node] = false;
		}

		std::vector<std::unordered_set<DataType>> weaklyConnectedComponents;

		for (auto& [node, neighbor] : neighborList) {
			if (!visited[node]) {
				auto dfsSearchTree = GraphAlgorithms::dfs(g, node, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
				std::unordered_set<DataType> component;
				for (auto& dfsNode : dfsSearchTree) {
					visited[dfsNode] = true;
					component.emplace(dfsNode);
				}
				weaklyConnectedComponents.emplace_back(component);
			}
		}

		if (internal::equals(behavior, AlgorithmBehavior::PrintAndReturn)) {
			unsigned i = 1u;
			for (auto& component : weaklyConnectedComponents) {
				out << "Component " << i << " consists of " << component.size() << " nodes:\n\t";
				for (auto& node : component) {
					out << "[" << node << "] ";
				}
				std::cout << std::endl;
				++i;
			}
		}

		return weaklyConnectedComponents;
	}
} // namespace GraphAlgorithms


// internal namesapce for helper funcitons, not inteded for end user
namespace internal {
	template<typename WeightType>
	std::enable_if_t<std::is_floating_point_v<WeightType>, bool>
	equals(WeightType lhs, WeightType rhs) {
		return std::fabs(rhs - lhs) < FLOATING_POINT_EPSIOLON<WeightType>;
	}

	template<typename WeightType>
	std::enable_if_t<!std::is_floating_point_v<WeightType>, bool>
	equals(WeightType lhs, WeightType rhs) {
		return lhs == rhs;
	}

	template<typename WeightType>
	std::enable_if_t<std::is_floating_point_v<WeightType>, bool>
	lessThan(WeightType lhs, WeightType rhs) {
		return lhs < (rhs - FLOATING_POINT_EPSIOLON<WeightType>);
	}

	template<typename WeightType>
	std::enable_if_t<!std::is_floating_point_v<WeightType>, bool>
	lessThan(WeightType lhs, WeightType rhs) {
		return lhs < rhs;
	}

	template<typename WeightType>
	std::enable_if_t<std::is_floating_point_v<WeightType>, bool>
	greaterThan(WeightType lhs, WeightType rhs) {
		return (lhs - FLOATING_POINT_EPSIOLON<WeightType>) > rhs;
	}

	template<typename WeightType>
	std::enable_if_t<!std::is_floating_point_v<WeightType>, bool>
	greaterThan(WeightType lhs, WeightType rhs) {
		return lhs > rhs;
	}

	template<typename DataType, typename WeightType>
	struct EdgeHasher {
			size_t operator()(const GraphClasses::Edge<DataType, WeightType>& obj) const {
				std::hash<DataType> nHash;
				std::hash<WeightType> wHash;
				// TODO:  try finding a better alternative
				return nHash(obj.neighbor) + wHash(obj.weight.value_or(0));
			}
	};

	template<typename DataType, typename WeightType>
	struct ArticulationHelper {
		public:
			unsigned time;
			std::unordered_map<DataType, unsigned> times;
			std::unordered_map<DataType, unsigned> lowerTimes;
			std::unordered_map<DataType, bool> visited;
			std::unordered_map<DataType, std::optional<DataType>> parents;
			std::unordered_set<DataType> articulationPoints;
			std::vector<std::pair<DataType, DataType>> bridges;
	};

	// this one internal function is used both for findArticulationPoints and findBridges as these to algorithms are very simmilar
	template<typename DataType, typename WeightType>
	void articulation__internal(GraphClasses::Graph<DataType, WeightType>& g, DataType startNode, GraphAlgorithms::AlgorithmBehavior behavior, std::ostream& out, ArticulationHelper<DataType, WeightType>& internalData) {
		internalData.visited[startNode] = true;
		internalData.times[startNode] = internalData.time;
		internalData.lowerTimes[startNode] = internalData.time;
		++internalData.time;

		auto neighborList = g.getNeighbors();
		unsigned numChildren = 0u;

		for (auto& [neighbor, weight] : neighborList[startNode]) {
			if (!internalData.visited[neighbor]) {
				++numChildren;
				internalData.parents[neighbor] = startNode;

				articulation__internal(g, neighbor, behavior, out, internalData);

				if (internal::lessThan(internalData.lowerTimes[neighbor], internalData.lowerTimes[startNode])) {
					internalData.lowerTimes[startNode] = internalData.lowerTimes[neighbor];
				}

				// for articulation points
				if ((!internalData.parents[startNode].has_value() && internal::greaterThan(numChildren, 1u)) ||
						(internalData.parents[startNode].has_value() && ( internal::greaterThan(internalData.lowerTimes[neighbor], internalData.times[startNode]) ||
						internal::equals(internalData.lowerTimes[neighbor], internalData.times[startNode]) ))) {
					internalData.articulationPoints.emplace(startNode);
				}

				// for bridges
				if (internal::greaterThan(internalData.lowerTimes[neighbor], internalData.times[startNode])) {
					internalData.bridges.emplace_back(startNode, neighbor);
				}

			} else {
				if (internalData.parents[startNode].has_value() && !internal::equals(neighbor, internalData.parents[startNode].value()) &&
						internal::lessThan(internalData.times[neighbor], internalData.lowerTimes[startNode])) {
					internalData.lowerTimes[startNode] = internalData.times[neighbor];
				}
			}
		}

		return;
	}

	template<typename DataType, typename WeightType>
	class DisjointSet {
		public:
			explicit DisjointSet(GraphClasses::Graph<DataType, WeightType>& g) {
				auto neighborList = g.getNeighbors();
				for (auto& [node, neighbors] : neighborList) {
					parent[node] = node;
					rank[node] = 0u;
				}
			}

			DataType findInDisjointSet(DataType node) {
				DataType root = node;
				while (!internal::equals(root, parent[root])) {
					root = parent[root];
				}
				while (!internal::equals(node, root)) {
					DataType tmp = parent[node];
					parent[node] = root;
					node = tmp;
				}
				return root;
			}

			void unionDisjointSets(DataType root1, DataType root2) {
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
			std::unordered_map<DataType, unsigned> rank;
	};

	template<typename DataType, typename WeightType>
	struct TarjanHelper {
		public:
			unsigned time;
			std::unordered_map<DataType, unsigned> times;
			std::unordered_map<DataType, unsigned> lowerTimes;
			std::stack<DataType> traversalOrder;
			std::unordered_map<DataType, bool> inStack;
			std::vector<std::unordered_set<DataType>> components;
	};

	template<typename DataType, typename WeightType>
	void tarjan__internal(GraphClasses::Graph<DataType, WeightType>& g, DataType startNode, TarjanHelper<DataType, WeightType>& internalData) {
		internalData.times[startNode] = internalData.time;
		internalData.lowerTimes[startNode] = internalData.time;
		++internalData.time;
		internalData.traversalOrder.emplace(startNode);
		internalData.inStack[startNode] = true;

		auto neighborList = g.getNeighbors();
		for (auto& [neighbor, weight] : neighborList[startNode]) {
			if (internal::equals(internalData.times[neighbor], 0u)) {
				tarjan__internal(g, neighbor, internalData);

				if (internal::lessThan(internalData.lowerTimes[neighbor], internalData.lowerTimes[startNode])) {
					internalData.lowerTimes[startNode] = internalData.lowerTimes[neighbor];
				}
			} else if (internalData.inStack[neighbor] && internal::lessThan(internalData.times[neighbor], internalData.lowerTimes[startNode])) {
				internalData.lowerTimes[startNode] = internalData.times[neighbor];
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