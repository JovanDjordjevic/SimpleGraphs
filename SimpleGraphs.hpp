#ifndef __SIMPLE_GRAPHS__
#define __SIMPLE_GRAPHS__

#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <optional>
#include <fstream>
#include <sstream>
#include <stack>
#include <queue>
#include <utility>
#include <set>
#include <algorithm>
#include <tuple>

#define GRAPH_ERROR(message) std::cerr << "ERROR: " << message << std::endl; 

//------------------------------------- API -------------------------------------
namespace GraphClasses {
    // NOTE: this is good enough for now but it limits possible types that can be used as WeightType
    template<typename WeightType>
    WeightType MAX_WEIGHT = std::numeric_limits<WeightType>::max();

    template<typename WeightType>
    WeightType MIN_WEIGHT = std::numeric_limits<WeightType>::lowest();

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

    template <typename DataType, typename WeightType>
    struct Edge {
        public:
            Edge(DataType neighbor, std::optional<WeightType> weight = {})
                : neighbor(neighbor), weight(weight)
            {}

            template<typename DataType, typename WeightType> 
            friend std::ostream& operator<(const Edge& lhs, const Edge& rhs);
            template<typename DataType, typename WeightType> 
            friend std::ostream& operator==(const Edge& lhs, const Edge& rhs);

        public:
            DataType neighbor;
            std::optional<WeightType> weight;
    };

    template <typename DataType, typename WeightType = int>
    class Graph {
        public:
            Graph(GraphType graphType = GraphType::Unset, GraphWeights graphWeights = GraphWeights::Unset);

            void configureDirections(GraphType graphType);
            void configureWeights(GraphWeights graphWeights);
            bool isConfigured();
            void clearGraph();

            void readFromTxt(const char* filePath);
            void writeToTxt(const char* filePath);
            void exportToTxt(const char* filePath); // TODO: export in format that SimpleGraphs can read
            void readFromDimacs(const char* filePath);

            void addNode(DataType node);
            void addEdge(DataType startNode, DataType neighborNode); // for unweighted graphs 
            void addEdge(DataType startNode, DataType neighborNode, WeightType edgeWeight); // for weighted graphs
            void addEdge(DataType startNode, Edge<DataType, WeightType> edge);
            void deleteEdge(DataType startNode, DataType endNode);
            void deleteNode(DataType node); // TODO
            size_t getNodeCount();
            size_t getEdgeCount();
            
            // ...

            GraphType getGraphType();
            GraphWeights getGraphWeights();
            std::unordered_map<DataType, std::vector<Edge<DataType, WeightType>>> getNeighbors();

            template<typename DataType, typename WeightType> 
            friend std::ostream& operator<<(std::ostream& out, const GraphClasses::Graph<DataType, WeightType>& g);
            
        private:
            GraphType m_graphType;
            GraphWeights m_graphWeights;
            std::unordered_map<DataType, std::vector<Edge<DataType, WeightType>>> m_neighbors;
    }; 

} //namespace GraphClasses

namespace GraphUtility {
    template<typename DataType, typename WeightType> 
    GraphClasses::Graph<DataType, WeightType> mergeGraphs(GraphClasses::Graph<DataType, WeightType>& g1, GraphClasses::Graph<DataType, WeightType>& g2);

    template<typename DataType, typename WeightType> 
    GraphClasses::Graph<DataType, WeightType> intersectGraphs(GraphClasses::Graph<DataType, WeightType>& g1, GraphClasses::Graph<DataType, WeightType>& g2);

    template<typename DataType, typename WeightType> 
    GraphClasses::Graph<DataType, WeightType> getSubgraphFromNodes(GraphClasses::Graph<DataType, WeightType>& g, std::unordered_set<DataType>& nodes);

    // ...
} //namespace GraphUtility

namespace GraphAlgorithms {
    enum class AlgorithmBehavior {   
        ReturnOnly,
        PrintAndReturn
    };
    
    template<typename DataType, typename WeightType> 
    std::vector<DataType> dfs(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, 
                    AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

    template<typename DataType, typename WeightType> 
    std::vector<DataType> bfs(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, 
                    AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

    template<typename DataType, typename WeightType> 
    std::vector<DataType> dijkstra(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, DataType endNode, 
                    AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

    template<typename DataType, typename WeightType> 
    std::unordered_map<DataType, std::vector<DataType>> bellmanFord(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, 
                    AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

    // NOTE: at this time, Floyd-Warshall algorithm only returns the distances between pairs of nodes and not the paths themselves
    // TODO: implement returning of paths
    template<typename DataType, typename WeightType> 
    std::unordered_map<DataType, std::unordered_map<DataType, WeightType>> floydWarshall(GraphClasses::Graph<DataType, WeightType> &g,
                    AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

    // without start node (only available for undirected graphs)
    template<typename DataType, typename WeightType> 
    std::unordered_set<DataType> findArticulationPoints(GraphClasses::Graph<DataType, WeightType> &g, AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn,
                    std::ostream& out = std::cout);

    // with start node (available for both undirected and directed)
    // NOTE: when using this function for directed graphs, only nodes in the corresponding dfs tree will be checked
    template<typename DataType, typename WeightType> 
    std::unordered_set<DataType> findArticulationPoints(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn,
                    std::ostream& out = std::cout);

    // without start node (only available for undirected graphs)
    template<typename DataType, typename WeightType> 
    std::vector<std::pair<DataType, DataType>> findBridges(GraphClasses::Graph<DataType, WeightType> &g, AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn,
                    std::ostream& out = std::cout);

    // with start node (available for both undirected and directed)
    // NOTE: when using this function for directed graphs, only nodes in the corresponding dfs tree will be checked
    template<typename DataType, typename WeightType> 
    std::vector<std::pair<DataType, DataType>> findBridges(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn,
                    std::ostream& out = std::cout);

    template<typename DataType, typename WeightType> 
    std::vector<DataType> topsortKhan(GraphClasses::Graph<DataType, WeightType> &g, AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn,
                    std::ostream& out = std::cout);

    template<typename DataType, typename WeightType> 
    WeightType mcstPrimTotalCostOnly(GraphClasses::Graph<DataType, WeightType> &g, AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, 
                    std::ostream& out = std::cout);

    template<typename DataType, typename WeightType> 
    std::vector<std::tuple<DataType, DataType, WeightType>> mcstPrim(GraphClasses::Graph<DataType, WeightType> &g, 
                    AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

    template<typename DataType, typename WeightType> 
    std::vector<std::tuple<DataType, DataType, WeightType>> mcstKruskal(GraphClasses::Graph<DataType, WeightType> &g, 
                    AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

    // without start node (only available for undirected graphs)
    template<typename DataType, typename WeightType> 
    std::vector<std::unordered_set<DataType>> findStronglyConnectedComponentsTarjan(GraphClasses::Graph<DataType, WeightType> &g,
                    AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

    // NOTE: when using this function for directed graphs, only nodes in the corresponding dfs tree will be checked
    template<typename DataType, typename WeightType> 
    std::vector<std::unordered_set<DataType>> findStronglyConnectedComponentsTarjan(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, 
                    AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

    // NOTE: currently only works for undirected graphs
    // TODO: implement a funciton that also works for directed graphs
    template<typename DataType, typename WeightType> 
    std::vector<std::unordered_set<DataType>> findStronglyWeaklyComponents(GraphClasses::Graph<DataType, WeightType> &g,
                    AlgorithmBehavior behavior = AlgorithmBehavior::PrintAndReturn, std::ostream& out = std::cout);

    // TODO:    
    // cycles
    // coloring
    // maximum flow (ford-fulkerson, edmonds-karp)
    // pairing
    //...
} // namespace GraphAlgorithms



//------------------------------------- IMPLEMENTATION -------------------------------------
namespace GraphClasses {
    template<typename DataType, typename WeightType>
    std::ostream& operator<<(std::ostream& out, const GraphClasses::Graph<DataType, WeightType>& g) {   
        for(auto& kvPair : g.m_neighbors) {   
            out << "Node [" << kvPair.first << "] has neighbours:" << std::endl;

            if (g.m_graphWeights == GraphWeights::Weighted) {
                for(auto&  val : kvPair.second) {
                    out << "|\t [" << val.neighbor << "], edge weight: " << val.weight.value() << std::endl;
                }
            }
            else { // unweighted
                for(auto&  val : kvPair.second) {
                   out << "|\t [" << val.neighbor << "]" << std::endl;
                }
            }
        }
        return out;
    }

    template <typename DataType, typename WeightType>
    bool operator<(const Edge<DataType, WeightType>& lhs, const Edge<DataType, WeightType>& rhs) {
        return lhs.neighbor < rhs.neighbor || lhs.weight < rhs.weight;
    }

    template <typename DataType, typename WeightType>
    bool operator==(const Edge<DataType, WeightType>& lhs, const Edge<DataType, WeightType>& rhs) {
        return lhs.neighbor == rhs.neighbor && lhs.weight == rhs.weight;
    }

    template <typename DataType, typename WeightType>
    Graph<DataType, WeightType>::Graph(GraphType graphType, GraphWeights graphWeights)
        : m_graphType(graphType), m_graphWeights(graphWeights) {
            // std::cout << MAX_WEIGHT<WeightType> << " " << MIN_WEIGHT<WeightType> << std::endl;
        }

    template <typename DataType, typename WeightType>
    void Graph<DataType, WeightType>::configureDirections(GraphType graphType) {
        m_graphType = graphType;
    }

    template <typename DataType, typename WeightType>
    void Graph<DataType, WeightType>::configureWeights(GraphWeights graphWeights) {
        m_graphWeights = graphWeights;
    }

    template <typename DataType, typename WeightType>
    bool Graph<DataType, WeightType>::isConfigured() {
        if (m_graphType != GraphType::Unset || m_graphWeights != GraphWeights::Unset) {
            return true;
        }
        return false;
    }

    template <typename DataType, typename WeightType>
    void Graph<DataType, WeightType>::clearGraph() {
        m_neighbors.clear(); // TODO: check if this is enough
    }

    template <typename DataType, typename WeightType>
    void Graph<DataType, WeightType>::readFromTxt(const char* filePath) {   
        if (!isConfigured())  {   
            GRAPH_ERROR("Graph type and weight must be configured before reading from file!");
            exit(EXIT_FAILURE);
        }

        clearGraph();
        
        std::ifstream file(filePath);
        if (!file)  {
            GRAPH_ERROR("Invalid file!");
            exit(EXIT_FAILURE);
        }

        DataType node;
        DataType neighbor;
        WeightType weight;

        std::string line;

        while(getline(file, line)) {   
            std::istringstream lineStream(line);
            lineStream >> node;
            m_neighbors[node]; // this line is neccessary becasue of isolated nodes

            if (m_graphWeights == GraphWeights::Weighted) {
                while(lineStream >> neighbor >> weight) {
                    m_neighbors[neighbor]; // this line is neccessary in case neighbor node is only mentioned as neighbor of another node
                    if (m_graphType == GraphType::Directed) {
                          m_neighbors[node].emplace_back(neighbor, weight);  
                    }
                    else { // undirected
                        m_neighbors[node].emplace_back(neighbor, weight);
                        m_neighbors[neighbor].emplace_back(node, weight);
                    }
                }
            } 
            else { // unweighted
                while(lineStream >> neighbor) {
                    m_neighbors[neighbor]; // this line is neccessary in case neighbor node is only mentioned as neighbor of another node
                    if (m_graphType == GraphType::Directed) {
                          m_neighbors[node].emplace_back(neighbor);
                    }
                    else  { // undirected
                        m_neighbors[node].emplace_back(neighbor);
                        m_neighbors[neighbor].emplace_back(node);
                    }
                }
            }
        }

        file.close();
    }

    template <typename DataType, typename WeightType>
    void Graph<DataType, WeightType>::writeToTxt(const char* filePath) {
        std::ofstream file(filePath);

        file << (*this);

        file.close();
    }

    template <typename DataType, typename WeightType>
    void Graph<DataType, WeightType>::readFromDimacs(const char* filePath) {
        if (!isConfigured())  {   
            GRAPH_ERROR("Graph type and weight must be configured before reading from file!");
            exit(EXIT_FAILURE);
        }

        clearGraph();
        
        std::ifstream file(filePath);
        if (!file)  {
            GRAPH_ERROR("Invalid file!");
            exit(EXIT_FAILURE);
        }

        DataType node;
        DataType neighbor;
        WeightType weight;

        std::string line;
        char lineInfo;
        
        while(getline(file, line)) {
            std::istringstream lineStream(line);
            lineStream >> lineInfo;

            if (lineInfo == 'c' || lineInfo == 'p') {
                continue;
            }

            lineStream >> node;
            m_neighbors[node]; // this line is neccessary becasue of isolated nodes

            if (m_graphWeights == GraphWeights::Weighted) {
                while(lineStream >> neighbor >> weight) {
                    m_neighbors[neighbor]; // this line is neccessary in case neighbor node is only mentioned as neighbor of another node
                    if (m_graphType == GraphType::Directed) {
                          m_neighbors[node].emplace_back(neighbor, weight);  
                    }
                    else { // undirected
                        m_neighbors[node].emplace_back(neighbor, weight);
                        m_neighbors[neighbor].emplace_back(node, weight);
                    }
                }
            } 
            else { // unweighted
                while(lineStream >> neighbor) {
                    m_neighbors[neighbor]; // this line is neccessary in case neighbor node is only mentioned as neighbor of another node
                    if (m_graphType == GraphType::Directed) {
                          m_neighbors[node].emplace_back(neighbor);
                    }
                    else  { // undirected
                        m_neighbors[node].emplace_back(neighbor);
                        m_neighbors[neighbor].emplace_back(node);
                    }
                }
            }
        }   

        file.close();
    }

    template <typename DataType, typename WeightType>
    void Graph<DataType, WeightType>::addNode(DataType node) {
        m_neighbors[node];
    }

    template <typename DataType, typename WeightType>
    void Graph<DataType, WeightType>::addEdge(DataType startNode, DataType neighborNode) {
        if (m_graphWeights == GraphWeights::Weighted) {
            GRAPH_ERROR("Graph is weighed and edge weight must be specified! ");
            exit(EXIT_FAILURE);
        }

        if (m_graphType == GraphType::Directed) {
            m_neighbors[startNode].emplace_back(neighborNode);
        }
        else { // undirected
            m_neighbors[startNode].emplace_back(neighborNode);
            m_neighbors[neighborNode].emplace_back(startNode);
        }
    }

    template <typename DataType, typename WeightType>
    void Graph<DataType, WeightType>::addEdge(DataType startNode, DataType neighborNode, WeightType edgeWeight) {
        if (m_graphWeights == GraphWeights::Unweighted) {
            GRAPH_ERROR("Graph is not weighed but you are trying to specify edge weight!");
            exit(EXIT_FAILURE);
        }

        if (m_graphType == GraphType::Directed) {
            m_neighbors[startNode].emplace_back(neighborNode, edgeWeight);
        }
        else { // undirected
            m_neighbors[startNode].emplace_back(neighborNode, edgeWeight);
            m_neighbors[neighborNode].emplace_back(startNode, edgeWeight);
        }
    }

    template <typename DataType, typename WeightType>
    void Graph<DataType, WeightType>::addEdge(DataType startNode, Edge<DataType, WeightType> edge) {
        if (m_graphWeights == GraphWeights::Unweighted && edge.weight.has_value() ) {
            GRAPH_ERROR("Graph is unweighed but edge has a weight!");
            exit(EXIT_FAILURE);
        } else if (m_graphWeights == GraphWeights::Weighted && !edge.weight.has_value()) {
            GRAPH_ERROR("Graph is weighed but edge has no weight!");
            exit(EXIT_FAILURE);
        }

        m_neighbors[startNode].emplace_back(edge.neighbor, edge.weight);
        return;
    }

    // NOTE: will delete all edges that connect start and end nodes in case of a multigraph
    template <typename DataType, typename WeightType>
    void Graph<DataType, WeightType>::deleteEdge(DataType startNode, DataType endNode) {
        auto it_start = m_neighbors.find(startNode);
        auto it_end = m_neighbors.find(endNode);
        if (it_start == std::end(m_neighbors) || it_end == std::end(m_neighbors)) {
            // std::cout << "Edge does not exist" << std::endl;
            return;
        }

        auto it = std::begin((*it_start).second);
        auto end = std::end((*it_start).second);
        while(it != end) {
            if ((*it).neighbor == endNode) {
                // std::cout << "start->end edge erased" << std::endl;
                ((*it_start).second).erase(it);
            } 
            ++it;
        }
         
        if (m_graphType == GraphType::Undirected) {
            auto it = std::begin((*it_end).second);
            auto end = std::end((*it_end).second);
            while(it != end) {
                if ((*it).neighbor == startNode) {
                    // std::cout << "end->start edge erased" << std::endl;
                    ((*it_end).second).erase(it);
                } 
                ++it;
            }
        }
        
        return;
    }
    
    // removes a node and all edges to/from said node
    template <typename DataType, typename WeightType>
    void Graph<DataType, WeightType>::deleteNode(DataType node) {
        if (m_neighbors.find(node) == m_neighbors.end()) {
            // std::cout << "Node does not exist" << std::endl;
            return;
        }   

        m_neighbors[node].clear(); // needed?
        m_neighbors.erase(node);

        for(auto& kv : m_neighbors) {
            auto it_begin = std::begin(kv.second);
            auto it_end = std::end(kv.second);
            while(it_begin != it_end) {
                if ((*it_begin).neighbor == node) {
                    kv.second.erase(it_begin);
                }
                ++it_begin;
            }
        }

        return;
    }

    template <typename DataType, typename WeightType>
    size_t Graph<DataType, WeightType>::getNodeCount() {
        return m_neighbors.size();
    }

    template <typename DataType, typename WeightType>
    size_t Graph<DataType, WeightType>::getEdgeCount() {
        size_t count = 0;
        for(auto& kv : m_neighbors) {
            count += kv.second.size();
        }
        return count;
    }

    template <typename DataType, typename WeightType>
    GraphType Graph<DataType, WeightType>::getGraphType() {
        return m_graphType;
    }

    template <typename DataType, typename WeightType>
    GraphWeights Graph<DataType, WeightType>::getGraphWeights() {
        return m_graphWeights;
    }

    template <typename DataType, typename WeightType>
    std::unordered_map<DataType, std::vector<Edge<DataType, WeightType>>> Graph<DataType, WeightType>::getNeighbors() {
        return m_neighbors;
    }

} //namespace GraphClasses


namespace GraphUtility {
    template<typename DataType, typename WeightType> 
    GraphClasses::Graph<DataType, WeightType> mergeGraphs(GraphClasses::Graph<DataType, WeightType>& g1, GraphClasses::Graph<DataType, WeightType>& g2) {
        GraphClasses::Graph<DataType, WeightType> newGraph;

        if (g1.getGraphType() != g2.getGraphType() || g1.getGraphWeights() != g2.getGraphWeights()) {
            GRAPH_ERROR("Graphs can only be merged if they have the same type (directed/undirected) and same weights (weighed/unweighed)!");
            exit(EXIT_FAILURE);
        }

        newGraph.configureDirections(g1.getGraphType());
        newGraph.configureWeights(g1.getGraphWeights());

        auto g1NeighborList = g1.getNeighbors();
        for (auto& kv : g1NeighborList) {
            newGraph.addNode(kv.first);
        }

        auto g2NeighborList = g2.getNeighbors();
        for (auto& kv : g2NeighborList) {
            newGraph.addNode(kv.first);
        }

        auto neightborList = newGraph.getNeighbors();
        for (auto& kv : neightborList) {
            auto it1 = g1NeighborList.find(kv.first);
            auto it2 = g2NeighborList.find(kv.first);
            
            if (it1 != g1NeighborList.end() && it2 != g2NeighborList.end()) {   // node is in both graphs
                // we avoid adding duplicate edges by putting them in a set first
                std::set<GraphClasses::Edge<DataType, WeightType>> edges;
                for (auto& edge : g1NeighborList[kv.first]) {
                    edges.emplace(edge);
                }

                for (auto& edge : g2NeighborList[kv.first]) {
                    edges.emplace(edge);
                }

                for(auto& edge : edges) {
                    newGraph.addEdge(kv.first, edge);
                }
            } else if (it1 != g1NeighborList.end() && it2 == g2NeighborList.end()) {  // node is only in g1
                for(auto& edge : g1NeighborList[kv.first]) {
                    newGraph.addEdge(kv.first, edge);
                }
            } else if (it1 == g1NeighborList.end() && it2 != g2NeighborList.end()) { // is only in g2
                for(auto& edge : g2NeighborList[kv.first]) {
                    newGraph.addEdge(kv.first, edge);
                }
            }
        }

        return newGraph;
    }

    template<typename DataType, typename WeightType> 
    GraphClasses::Graph<DataType, WeightType> intersectGraphs(GraphClasses::Graph<DataType, WeightType>& g1, GraphClasses::Graph<DataType, WeightType>& g2) {
        GraphClasses::Graph<DataType, WeightType> newGraph;

        if (g1.getGraphType() != g2.getGraphType() || g1.getGraphWeights() != g2.getGraphWeights()) {
            GRAPH_ERROR("Graph intersection can only be created if they have the same type (directed/undirected) and same weights (weighed/unweighed)!");
            exit(EXIT_FAILURE);
        }

        newGraph.configureDirections(g1.getGraphType());
        newGraph.configureWeights(g1.getGraphWeights());

        auto g1NeighborList = g1.getNeighbors();
        auto g2NeighborList = g2.getNeighbors();

        for (auto& kv : g1NeighborList) {
            auto it = g2NeighborList.find(kv.first);
            if (it != std::end(g2NeighborList)) {  
                newGraph.addNode(kv.first);

                std::unordered_set<GraphClasses::Edge<DataType, WeightType>, internal::EdgeHasher<DataType, WeightType>> edges;
                auto& shorter = g1NeighborList[kv.first];
                auto& longer = g2NeighborList[kv.first];
                if (g2NeighborList[kv.first].size() < g2NeighborList[kv.first].size()) {
                    shorter = g2NeighborList[kv.first];
                    longer = g1NeighborList[kv.first];
                }

                for(auto& edge : shorter) {
                    edges.emplace(edge.neighbor, edge.weight);
                }

                for (auto& edge : longer) {
                    if (edges.find(edge) != std::end(edges)) {
                        newGraph.addEdge(kv.first, edge);
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

        for(auto& node : nodes) {
            newGraph.addNode(node);
            for(auto& [neighbor, weight] : neighborList[node]) {
                if (nodes.find(neighbor) != nodes.end()) {
                    if (newGraph.getGraphWeights() == GraphClasses::GraphWeights::Weighted) {
                        newGraph.addEdge(node, neighbor, weight.value());
                    } else {
                        newGraph.addEdge(node, neighbor);
                    }
                }
            }
        }

        return newGraph;
    }
} //namespace GraphUtility


namespace GraphAlgorithms {
    template<typename DataType, typename WeightType> 
    std::vector<DataType> dfs(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, AlgorithmBehavior behavior, std::ostream& out) {
        std::unordered_map<DataType, bool> visited;
        std::vector<DataType> dfsSearchTree;

        auto neighborList = g.getNeighbors();

        for(auto& kv : neighborList) {
            visited[kv.first] = false;
        }
        
        std::stack<DataType> stack;
        stack.emplace(startNode);

        while (!stack.empty()) {
            DataType currentNode = stack.top();
            stack.pop();

            dfsSearchTree.emplace_back(currentNode);

            if (!visited[currentNode]) {
                visited[currentNode] = true;
            }

            auto it = std::cbegin(neighborList[currentNode]);
            auto end = std::cend(neighborList[currentNode]);
            while (it != end) {
                if (!visited[(*it).neighbor]) {
                    stack.emplace((*it).neighbor);
                }
                ++it;
            }
        }

        if (behavior == AlgorithmBehavior::PrintAndReturn) {
            out << "Order of DFS traversal:\n\t";
            for(auto& node : dfsSearchTree) {
                out << "[" << node << "] ";
            }
            out << std::endl;
        }

        return dfsSearchTree;
    }

    template<typename DataType, typename WeightType> 
    std::vector<DataType> bfs(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, AlgorithmBehavior behavior, std::ostream& out) {
        std::unordered_map<DataType, bool> visited;
        std::vector<DataType> bfsTraversalOrder;

        auto neighborList = g.getNeighbors();

        for(auto& kv : neighborList) {
            visited[kv.first] = false;
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
            while (it != end) {
                if (!visited[(*it).neighbor]) {
                    queue.emplace((*it).neighbor);
                }
                ++it;
            }
        }

        if (behavior == GraphAlgorithms::AlgorithmBehavior::PrintAndReturn) {
            out << "Order of BFS traversal:\n\t";
            for(auto& node : bfsTraversalOrder) {
                out << "[" << node << "] ";
            }
            out << std::endl;
        }

        return bfsTraversalOrder;
    }

    template<typename DataType, typename WeightType> 
    std::vector<DataType> dijkstra(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, DataType endNode, AlgorithmBehavior behavior, std::ostream& out) {
        std::unordered_map<DataType, WeightType> distances;
        std::unordered_map<DataType, bool> visited;
        std::unordered_map<DataType, std::optional<DataType>> parents; 

        using pqData = GraphClasses::Edge<DataType, WeightType>;
        struct Comparator {
            bool operator()(pqData& e1, pqData& e2) {
                return e1.weight.value() > e2.weight.value();
            }
        };
        std::priority_queue<pqData, std::vector<pqData>, Comparator> pq;

        auto neighborList = g.getNeighbors();

        for(auto& kv : neighborList) {
            distances[kv.first] = GraphClasses::MAX_WEIGHT<WeightType>;
            visited[kv.first] = false;
        }

        distances[startNode] = 0;
        parents[startNode]; // only startNode will have the empty optional
        pq.emplace(startNode, 0);

        bool pathFound = false;

        while (!pq.empty()) {
            auto [currentNode, weight] = pq.top();
            pq.pop();

            if (currentNode == endNode) {
                pathFound = true;
                break;
            }

            visited[currentNode] = true;

            for(auto& [neighbor, weight] : neighborList[currentNode]) {
                if (!visited[neighbor]) {
                    WeightType oldDistance = distances[neighbor];
                    WeightType newDistance = distances[currentNode] + weight.value_or(1); // i don't like this
                    if (newDistance < oldDistance) {
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
                if(!parent.has_value()) {
                    break;
                }
                path.emplace_back(parent.value());
                endCpy = parent.value();
            }

            std::reverse(std::begin(path), std::end(path));
        }

        if(behavior == AlgorithmBehavior::PrintAndReturn) {
            if (pathFound) {
                auto it = std::cbegin(path);
                auto end = std::cend(path);
                out << "Path found: \n\t";
                while(it != end) {
                    out << "[" << (*it) << "] ";
                    ++it;
                }
                out << "\nwith total distance: " << distances[endNode] << std::endl;
            }
            else {
                out << "No path found between [" << startNode <<"] and [" << endNode << "]" << std::endl;
            }
        }

        return path;
    }

    template<typename DataType, typename WeightType> 
    std::unordered_map<DataType, std::vector<DataType>> bellmanFord(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, AlgorithmBehavior behavior, std::ostream& out) {
        std::unordered_map<DataType, WeightType> distances;
        std::unordered_map<DataType, std::optional<DataType>> parents; 

        auto neighborsList = g.getNeighbors();

        for(auto& kv : neighborsList) {
            distances[kv.first] = GraphClasses::MAX_WEIGHT<WeightType>;
        }

        distances[startNode] = 0;
        parents[startNode]; // only startNode will have the empty optional

        size_t relaxationCount = g.getNodeCount() - 1;

        for(size_t r = 0; r < relaxationCount; ++r) {
            for(auto& kv : neighborsList) {
                DataType currentNode = kv.first;
                auto neighbors = kv.second;
                for(auto& [neighbor, weight] : neighbors) {
                    WeightType newDistnce = distances[currentNode] + weight.value_or(1);
                    if (newDistnce < distances[neighbor]) {
                        distances[neighbor] = newDistnce;
                        parents[neighbor] = currentNode;
                    }
                }
            }
        }

        // negtive cycle check
        for (auto& kv : neighborsList) {
            DataType currentNode = kv.first;
            auto neighbors = kv.second;
            for (auto& [neighbor, weight] : neighbors) {
                if (distances[currentNode] + weight.value_or(1) < distances[neighbor]) {
                    GRAPH_ERROR("Graph contins negative cycle");
                    exit(EXIT_FAILURE);
                }
            }
        }

        // path reconstruction
        std::unordered_map<DataType, std::vector<DataType>> paths;

        for(auto& kv : distances) {
            paths[kv.first] = {};

            if (kv.second == GraphClasses::MAX_WEIGHT<WeightType> || kv.first == startNode) {
                continue;
            }

            DataType pathNode = kv.first;
            paths[kv.first].emplace_back(pathNode);

            while (true) {
                std::optional<DataType> parent = parents[pathNode];
                if(!parent.has_value()) {
                    break;
                }
                paths[kv.first].emplace_back(parent.value());
                pathNode = parent.value();
            }

            std::reverse(std::begin(paths[kv.first]), std::end(paths[kv.first]));
        }

        if (behavior == AlgorithmBehavior::PrintAndReturn) {
            for(auto& kv : paths) {
                // path to start node itself is irrelevant
                if (kv.first == startNode) {
                    continue;
                }

                // there is no path to nodes in different components
                if (kv.second.size() == 0) {
                    out << "There is no possible path between [" << startNode << "] and [" << kv.first << "]" << std::endl;
                    continue;
                }

                out << "Distance from [" << startNode << "] to [" <<  kv.first << "] is: " << distances[kv.first] <<"\n\t Path: ";
                auto it = std::cbegin(kv.second);
                auto end = std::cend(kv.second);
                while(it != end) {
                    out << "[" << (*it) << "] ";
                    ++it;
                }
                out << std::endl;
            }
        }

        return paths;
    }

    template<typename DataType, typename WeightType> 
    std::unordered_map<DataType, std::unordered_map<DataType, WeightType>> floydWarshall(GraphClasses::Graph<DataType, WeightType> &g, AlgorithmBehavior behavior, std::ostream& out) {
        std::unordered_map<DataType, std::unordered_map<DataType, WeightType>> distances;

        auto neighborList = g.getNeighbors();

        for(auto& kv1 : neighborList) {
            for(auto& kv2 : neighborList) {
                distances[kv1.first][kv2.first] = GraphClasses::MAX_WEIGHT<WeightType>;
            }
        }

        for(auto& kv : neighborList) {
            for(auto& [neighbor, weight] : kv.second) {
                if (kv.first == neighbor) {
                    distances[kv.first][neighbor] = 0;
                }
                else {
                    distances[kv.first][neighbor] = weight.value_or(1);
                }             
            }
        }

        for(auto& [mid, n1] : neighborList) {
            for(auto& [start, n2] : neighborList) {
                for(auto& [end, n3] : neighborList) {
                    if (distances[start][mid] != GraphClasses::MAX_WEIGHT<WeightType>
                        && distances[mid][end] != GraphClasses::MAX_WEIGHT<WeightType>
                        && distances[start][mid] + distances[mid][end] < distances[start][end]) {
                            distances[start][end] = distances[start][mid] + distances[mid][end];
                    }
                }   
            }
        }

        for(auto& kv : distances) {
            if (distances[kv.first][kv.first] < 0) {
                GRAPH_ERROR("Graph contins negative cycle"); // should this be an error ?
                exit(EXIT_FAILURE);
            }
        }

        if (behavior == AlgorithmBehavior::PrintAndReturn) {
            for(auto& [node, neighbors] : distances) {
                for(auto& [neighbor, distance] : neighbors) {
                    if (distance == GraphClasses::MAX_WEIGHT<WeightType> || node == neighbor) {
                        continue;
                    }
                    out << "Shortest distance between [" << node <<"] and [" << neighbor << "] is: " << distance << std::endl; 
                }
                
                // FIXME: for string nodes, sometimes 2 new line characters are printed between groups
                out << std::endl;
            }
        }

        return distances;
    }

    template<typename DataType, typename WeightType> 
    std::unordered_set<DataType> findArticulationPoints(GraphClasses::Graph<DataType, WeightType> &g, AlgorithmBehavior behavior, std::ostream& out) {
        if (g.getGraphType() == GraphClasses::GraphType::Directed) {        
            GRAPH_ERROR("Must specify startNode for directed graphs. Call the appropriate overload of this function!");
            exit(EXIT_FAILURE);
        }  

        DataType startNode = (*std::begin(g.getNeighbors())).first;
        return findArticulationPoints(g, startNode, behavior, out);
    }

    template<typename DataType, typename WeightType> 
    std::unordered_set<DataType> findArticulationPoints(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, AlgorithmBehavior behavior, std::ostream& out) {
        internal::ArticulationHelper<DataType, WeightType> internalData;

        auto neighborList = g.getNeighbors();

        internalData.time = 0u;
        internalData.parents[startNode];
        for(auto& kv : neighborList) {
            internalData.visited[kv.first] = false;
        }

        internal::articulation__internal(g, startNode, behavior, out, internalData);

        if (behavior == AlgorithmBehavior::PrintAndReturn) {
            if (internalData.articulationPoints.size() == 0) {
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
    std::vector<std::pair<DataType, DataType>> findBridges(GraphClasses::Graph<DataType, WeightType> &g, AlgorithmBehavior behavior, std::ostream& out) {
        if (g.getGraphType() == GraphClasses::GraphType::Directed) {        
            GRAPH_ERROR("Must specify startNode for directed graphs. Call the appropriate overload of this function!");
            exit(EXIT_FAILURE);
        }  

        DataType startNode = (*std::begin(g.getNeighbors())).first;
        return findBridges(g, startNode, behavior, out);
    }

    template<typename DataType, typename WeightType> 
    std::vector<std::pair<DataType, DataType>> findBridges(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, AlgorithmBehavior behavior, std::ostream& out) {
        internal::ArticulationHelper<DataType, WeightType> internalData;

        auto neighborList = g.getNeighbors();

        internalData.time = 0u;
        internalData.parents[startNode];
        for(auto& kv : neighborList) {
            internalData.visited[kv.first] = false;
        }

        internal::articulation__internal(g, startNode, behavior, out, internalData);

        if (behavior == AlgorithmBehavior::PrintAndReturn) {
            if (internalData.bridges.size() == 0) {
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
    std::vector<DataType> topsortKhan(GraphClasses::Graph<DataType, WeightType> &g, AlgorithmBehavior behavior, std::ostream& out) {
        if (g.getGraphType() == GraphClasses::GraphType::Undirected) {
            GRAPH_ERROR("Topological sorting makes no sense for undirected graphs!");
            exit(EXIT_FAILURE);
        }

        auto neighborList = g.getNeighbors();

        std::unordered_map<DataType, unsigned> inDegrees;
        for(auto& kv : neighborList) {
            inDegrees[kv.first] = 0u;
        }

        std::vector<DataType> topologicalOrdering;
        unsigned numVisited = 0u;

        for(auto& kv : neighborList) {
            for (auto& [neighbor, weight] : neighborList[kv.first]) {
                ++inDegrees[neighbor];
            }
        }

        std::queue<DataType> topsortQueue;
        for(auto& kv : inDegrees) {
            if (inDegrees[kv.first] == 0) {
                topsortQueue.emplace(kv.first);
            }
        }

        while (!topsortQueue.empty()) {
            DataType current = topsortQueue.front();
            topsortQueue.pop();
            topologicalOrdering.emplace_back(current);
            ++numVisited;

            for (auto& [neighbor, weight] : neighborList[current]) {
                --inDegrees[neighbor];
                if (inDegrees[neighbor] == 0) {
                    topsortQueue.emplace(neighbor);
                }
            }
        }

        // we signal that graph is not acyclic with an empty vector
        if (numVisited != g.getNodeCount()) {
            topologicalOrdering.clear();
        }

        if (behavior == AlgorithmBehavior::PrintAndReturn) {
            if (numVisited != g.getNodeCount()) {
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
    WeightType mcstPrimTotalCostOnly(GraphClasses::Graph<DataType, WeightType> &g, AlgorithmBehavior behavior, std::ostream& out) {
        auto ret = mcstPrim(g, AlgorithmBehavior::ReturnOnly, out);

        WeightType totalCost = 0;
        for(auto& [node1, node2, weight] : ret) {
            totalCost += weight;
        }

        if (behavior == GraphAlgorithms::AlgorithmBehavior::PrintAndReturn) {
            out << "Total cost of minimum cost spanning tree is: " << totalCost << std::endl;
        }

        return totalCost;
    }

    template<typename DataType, typename WeightType> 
    std::vector<std::tuple<DataType, DataType, WeightType>> mcstPrim(GraphClasses::Graph<DataType, WeightType> &g, AlgorithmBehavior behavior, std::ostream& out) {
        if (g.getGraphType() == GraphClasses::GraphType::Directed) {
            GRAPH_ERROR("Minimum cost spanning tree for directed graphs currently not supported");
            exit(EXIT_FAILURE);
        }

        std::unordered_map<DataType, WeightType> distances;
        std::unordered_map<DataType, bool> visited;
        std::unordered_map<DataType, std::optional<DataType>> parents; 

        using pqData = GraphClasses::Edge<DataType, WeightType>;
        struct Comparator {
            bool operator()(pqData& e1, pqData& e2) {
                return e1.weight.value() > e2.weight.value();
            }
        };
        std::priority_queue<pqData, std::vector<pqData>, Comparator> pq;
        
        auto neighborList = g.getNeighbors();

        for(auto& kv : neighborList) {
            distances[kv.first] = GraphClasses::MAX_WEIGHT<WeightType>;
            visited[kv.first] = false;
        }
        
        DataType startNode = (*std::begin(g.getNeighbors())).first;

        distances[startNode] = 0;
        parents[startNode]; // only startNode will have the empty optional
        pq.emplace(startNode, 0);

        for (auto& [neighbor, weight] : neighborList[startNode]) {
            pq.emplace(neighbor, GraphClasses::MAX_WEIGHT<WeightType>);
        }

        size_t nodeCount = g.getNodeCount();
        for(size_t i = 0; i < nodeCount; ++i) {
            auto [currentNode, weight] = pq.top();
            pq.pop();

            // distances[currentNode] = weight.value_or(1);    not needed?

            if (!visited[currentNode]) {
                visited[currentNode] = true;  

                for(auto& [neighbor, weight] : neighborList[currentNode]) {
                    if (!visited[neighbor]) {
                        if (weight.value_or(1) < distances[neighbor]) {
                            distances[neighbor] = weight.value_or(1);
                            parents[neighbor] = currentNode;
                            pq.emplace(neighbor, distances[neighbor]);
                        }
                    }
                }
            } 
        }

        std::vector<std::tuple<DataType, DataType, WeightType>> mcst;
        for (auto& kv : neighborList) {
            if (parents[kv.first].has_value()) {
                mcst.emplace_back(parents[kv.first].value(), kv.first, distances[kv.first]);
            }
        }

        if (behavior == GraphAlgorithms::AlgorithmBehavior::PrintAndReturn) {
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
    std::vector<std::tuple<DataType, DataType, WeightType>> mcstKruskal(GraphClasses::Graph<DataType, WeightType> &g, AlgorithmBehavior behavior, std::ostream& out) {
        if (g.getGraphType() == GraphClasses::GraphType::Directed) {
            GRAPH_ERROR("Minimum cost spanning tree for directed graphs currently not supported");
            exit(EXIT_FAILURE);
        }

        auto neighborList = g.getNeighbors();
        internal::DisjointSet ds{g};
        std::vector<std::tuple<DataType, DataType, WeightType>> allEdges;

        for (auto& [node, neighbors] : neighborList) {
            for (auto& [neighbor, weight] : neighbors) {
                allEdges.emplace_back(node, neighbor, weight.value_or(1));
            }
        }

        std::sort(std::begin(allEdges), std::end(allEdges), [](auto& t1, auto& t2){ return std::get<2>(t1) < std::get<2>(t2); });

        std::vector<std::tuple<DataType, DataType, WeightType>> mcst;
        unsigned mcstSize = g.getNodeCount() - 1;
        unsigned addedEdges = 0u;

        for(auto& [node1, node2, edgeWeight] : allEdges) {
            if (addedEdges >= mcstSize) {
                break;
            }

            DataType root1 = ds.findInDisjointSet(node1);
            DataType root2 = ds.findInDisjointSet(node2);

            if (root1 != root2) {
                mcst.emplace_back(node1, node2, edgeWeight);
                ds.unionDisjointSets(root1, root2);
                ++addedEdges;
            }
        }

        if (behavior == GraphAlgorithms::AlgorithmBehavior::PrintAndReturn) {
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
    std::vector<std::unordered_set<DataType>> findStronglyConnectedComponentsTarjan(GraphClasses::Graph<DataType, WeightType> &g, AlgorithmBehavior behavior, std::ostream& out) {
        if (g.getGraphType() == GraphClasses::GraphType::Directed) {        
            GRAPH_ERROR("Must specify startNode for directed graphs. Call the appropriate overload of this function!");
            exit(EXIT_FAILURE);
        }  

        DataType startNode = (*std::begin(g.getNeighbors())).first;
        return findStronglyConnectedComponentsTarjan(g, startNode, behavior, out);
    }


    template<typename DataType, typename WeightType> 
    std::vector<std::unordered_set<DataType>> findStronglyConnectedComponentsTarjan(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, AlgorithmBehavior behavior, std::ostream& out) {
        internal::TarjanHelper<DataType, WeightType> internalData;

        auto neighborList = g.getNeighbors();

        internalData.time = 1u;
        for(auto& kv : neighborList) {
            internalData.inStack[kv.first] = false;
            // unvisited nodes will have their time set to 0
            internalData.times[kv.first] = 0u;
        }

        internal::tarjan__internal(g, startNode, internalData);

        if (behavior == AlgorithmBehavior::PrintAndReturn) {
            unsigned i = 1u;
            for (auto& component : internalData.components) {
                out << "Component " << i << " consists of " <<  component.size() << " nodes:\n\t";
                for(auto& node : component) {
                    out << "[" << node << "] ";
                }
                std::cout << std::endl;
                ++i;
            }
        }

        return internalData.components;
    }

    template<typename DataType, typename WeightType> 
    std::vector<std::unordered_set<DataType>> findStronglyWeaklyComponents(GraphClasses::Graph<DataType, WeightType> &g, AlgorithmBehavior behavior, std::ostream& out) {
        if (g.getGraphType() == GraphClasses::GraphType::Directed) {
            GRAPH_ERROR("Finding weakly connected components in directed graphs currently not supported!");
            exit(EXIT_FAILURE);
        }
        
        std::unordered_map<DataType, bool> visited;

        auto neighborList = g.getNeighbors();

        for(auto& kv : neighborList) {
            visited[kv.first] = false;
        }

        std::vector<std::unordered_set<DataType>> weaklyConnectedComponents;

        for(auto& kv : neighborList) {
            if (!visited[kv.first]) {
                auto dfsSearchTree = GraphAlgorithms::dfs(g, kv.first, GraphAlgorithms::AlgorithmBehavior::ReturnOnly);
                std::unordered_set<DataType> component;
                    for (auto& node : dfsSearchTree) {
                        visited[node] = true;
                        component.emplace(node);                        
                    }
                weaklyConnectedComponents.emplace_back(component);
            }
        }

        if (behavior == AlgorithmBehavior::PrintAndReturn) {
            unsigned i = 1u;
            for (auto& component : weaklyConnectedComponents) {
                out << "Component " << i << " consists of " <<  component.size() << " nodes:\n\t";
                for(auto& node : component) {
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
    template <typename DataType, typename WeightType>
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
    void articulation__internal(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, GraphAlgorithms::AlgorithmBehavior behavior, std::ostream& out, ArticulationHelper<DataType, WeightType>& internalData) {
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

                if (internalData.lowerTimes[neighbor] < internalData.lowerTimes[startNode]) {
                    internalData.lowerTimes[startNode] = internalData.lowerTimes[neighbor];
                }

                // for articulation points
                if ( (!internalData.parents[startNode].has_value() && numChildren > 1) 
                  || (internalData.parents[startNode].has_value() && (internalData.lowerTimes[neighbor] >= internalData.times[startNode])) ){
                    internalData.articulationPoints.emplace(startNode);
                }

                // for bridges
                if (internalData.lowerTimes[neighbor] > internalData.times[startNode]) {
                    internalData.bridges.emplace_back(startNode, neighbor);
                }

            } else {
                if (internalData.parents[startNode].has_value() 
                  && neighbor != internalData.parents[startNode].value() 
                  && internalData.times[neighbor] < internalData.lowerTimes[startNode]) {
                    internalData.lowerTimes[startNode] = internalData.times[neighbor];
                }
            }
        }

        return;
    }

    template<typename DataType, typename WeightType>
    class DisjointSet {
        public:
            DisjointSet (GraphClasses::Graph<DataType, WeightType>& g) {
                auto neighborList = g.getNeighbors();
                for(auto& kv : neighborList) {
                    parent[kv.first] = kv.first;
                    rank[kv.first] = 0u;
                }
            }

            DataType findInDisjointSet (DataType node) {
                DataType root = node;
                while (root != parent[root]) {
                    root = parent[root];
                }
                while (node != root) {
                    DataType tmp = parent[node];
                    parent[node] = root;
                    node = tmp;
                }
                return root;
            }

            void unionDisjointSets (DataType root1, DataType root2) {
                if (rank[root1] > rank[root2]) {
                    parent[root2] = root1;
                } else if (rank[root1] < rank[root2]) {
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
    void tarjan__internal(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, TarjanHelper<DataType, WeightType>& internalData) {
        internalData.times[startNode] = internalData.time;
        internalData.lowerTimes[startNode] = internalData.time;
        ++internalData.time;
        internalData.traversalOrder.emplace(startNode);
        internalData.inStack[startNode] = true;

        auto neighborList = g.getNeighbors();
        for (auto& [neighbor, weight] : neighborList[startNode]) {
            if (internalData.times[neighbor] == 0u) {
                tarjan__internal(g, neighbor, internalData);

                if (internalData.lowerTimes[neighbor] < internalData.lowerTimes[startNode]) {
                    internalData.lowerTimes[startNode] = internalData.lowerTimes[neighbor];
                }
            } else if (internalData.inStack[neighbor] && (internalData.times[neighbor] < internalData.lowerTimes[startNode])) {
                    internalData.lowerTimes[startNode] = internalData.times[neighbor];
            }
        }  
        
        // component found
        if (internalData.times[startNode] == internalData.lowerTimes[startNode]) {
            DataType componentNode;
            std::unordered_set<DataType> component;
            while (true) {
                componentNode = internalData.traversalOrder.top();
                internalData.traversalOrder.pop();
                component.emplace(componentNode);
                internalData.inStack[componentNode] = false;
                if (componentNode == startNode) {
                    internalData.components.emplace_back(component);
                    break;
                }
            }
        }

        return;
    }

} // namespace internal


#endif  //__SIMPLE_GRAPHS__
