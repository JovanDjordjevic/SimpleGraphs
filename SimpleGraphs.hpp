#ifndef __SIMPLE_GRAPHS__
#define __SIMPLE_GRAPHS__

#include <iostream>
#include <unordered_map>
#include <vector>
#include <optional>

#define GRAPH_ERROR(message) std::cerr << "ERROR: " << message << std::endl; 

//------------------------------------- API -------------------------------------
namespace GraphClasses {
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
    class Graph {
        public:
            Graph(GraphType graphType = GraphType::Unset, GraphWeights graphWeights = GraphWeights::Unset);

            void configureDirections(GraphType graphType);
            void configureWeights(GraphWeights graphWeights);
            bool isConfigured();

            void readFromTxt(const char* filePath);
            void writeToTxt(const char* filePath);

            void addNode(DataType node);
            void addEdge(DataType startNode, DataType neighborNode); // for unweighted graphs 
            void addEdge(DataType startNode, DataType neighborNode, WeightType edgeWeight); // for weighted graphs
            
            // ...

            template<typename DataType, typename WeightType> 
            friend std::ostream& operator<<(std::ostream& out, const GraphClasses::Graph<DataType, WeightType>& g);
            
        private:
            GraphType m_graphType;
            GraphWeights m_graphWeights;
            
            struct Edge{
                Edge(DataType neighbor, std::optional<WeightType> weight = {}) 
                    : neighbor(neighbor), weight(weight) 
                {}

                DataType neighbor;
                std::optional<WeightType> weight;
            };

            std::unordered_map<DataType, std::vector<Edge>> m_neighbors;
    }; 

} //namespace GraphClasses


namespace GraphAlgorithms {
    // TODO:
    // bfs, dfs
    // dijkstra
    // belman-ford
    // flojd-varsal
    // cycles
    // mst  (prim, kruskal)
    // components (tarjan, kosaraju)
    // topsort (kan)
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
            else {
                for(auto&  val : kvPair.second) {
                   out << "|\t [" << val.neighbor << "]" << std::endl;
                }
            }
        }
        return out;
    }

    template <typename DataType, typename WeightType>
    Graph<DataType, WeightType>::Graph(GraphType graphType, GraphWeights graphWeights)
        : m_graphType(graphType), m_graphWeights(graphWeights) {}

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

    // samo ubacuje cvor, bez ikakvih grana i tezina
    template <typename DataType, typename WeightType>
    void Graph<DataType, WeightType>::addNode(DataType node) {
        m_neighbors[node];
    }

    // mozda spoji oba addEdge overloada u jednu funkc
    template <typename DataType, typename WeightType>
    void Graph<DataType, WeightType>::addEdge(DataType startNode, DataType neighborNode) {
        if (m_graphWeights == GraphWeights::Weighted)
        {
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
        if (m_graphWeights == GraphWeights::Unweighted)
        {
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

} //namespace GraphClasses


namespace GraphAlgorithms {
    // ...
} // namespace GraphAlgorithms

#endif  //__SIMPLE_GRAPHS__
