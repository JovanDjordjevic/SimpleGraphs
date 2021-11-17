#ifndef __SIMPLE_GRAPHS__
#define __SIMPLE_GRAPHS__

#include <unordered_map>
#include <vector>
#include <optional>

//------------------------------------- API -------------------------------------
namespace GraphClasses {
    enum class GraphType {   
        Directed,
        Undirected
    };

    enum class GraphWeights {   
        Weighted,
        Unweighted
    };

    template <typename DataType, typename WeightType>
    class Graph {
        public:
            Graph(GraphType graphType, GraphWeights graphWeights);

            void configureDirections(GraphType graphType);
            void configureWeights(GraphWeights graphWeights);

            void readFromTxt(const char* filePath);
            void writeToTxt(const char* filePath);

            void addNode(DataType node);
            void addEdge(DataType startNode, DataType neighborNode); // for unweighted graphs 
            void addEdge(DataType startNode, DataType neighborNode, WeightType edgeWeight); // for weighted graphs
            
            // ...
            
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
    // ....

} //namespace GraphClasses


namespace GraphAlgorithms {
    // ...
} // namespace GraphAlgorithms

#endif  //__SIMPLE_GRAPHS__
