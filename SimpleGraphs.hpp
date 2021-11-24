#ifndef __SIMPLE_GRAPHS__
#define __SIMPLE_GRAPHS__

#include <iostream>
#include <unordered_map>
#include <vector>
#include <optional>
#include <fstream>
#include <sstream>
#include <stack>
#include <queue>

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

            void addNode(DataType node);
            void addEdge(DataType startNode, DataType neighborNode); // for unweighted graphs 
            void addEdge(DataType startNode, DataType neighborNode, WeightType edgeWeight); // for weighted graphs
            void deleteEdge(DataType startNode, DataType endNode);
            
            // ...

            template<typename DataType, typename WeightType> 
            friend std::ostream& operator<<(std::ostream& out, const GraphClasses::Graph<DataType, WeightType>& g);
            
        public:
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
    template<typename DataType, typename WeightType> 
    void dfs(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, std::ostream& out = std::cout);

    template<typename DataType, typename WeightType> 
    void bfs(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, std::ostream& out = std::cout);

    // TODO:    
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
            else { // unweighted
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

} //namespace GraphClasses


namespace GraphAlgorithms {
    template<typename DataType, typename WeightType> 
    void dfs(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, std::ostream& out) {
        std::unordered_map<DataType, bool> visited;
        for(auto& kv : g.m_neighbors) {
            visited[kv.first] = false;
        }
        
        std::stack<DataType> stack;
        stack.emplace(startNode);

        while (!stack.empty()) {
            DataType currentNode = stack.top();
            stack.pop();

            if (!visited[currentNode]) {
                out << "[" << currentNode << "] ";
                visited[currentNode] = true;
            }

            auto it = std::cbegin(g.m_neighbors[currentNode]);
            auto end = std::cend(g.m_neighbors[currentNode]);
            while (it != end) {
                if (!visited[(*it).neighbor]) {
                    stack.emplace((*it).neighbor);
                }

                ++it;
            }
        }
        out << std::endl;

        return;
    }

    template<typename DataType, typename WeightType> 
    void bfs(GraphClasses::Graph<DataType, WeightType> &g, DataType startNode, std::ostream& out) {
        std::unordered_map<DataType, bool> visited;
        for(auto& kv : g.m_neighbors) {
            visited[kv.first] = false;
        }
        
        std::queue<DataType> queue;
        queue.emplace(startNode);

        while (!queue.empty()) {
            DataType currentNode = queue.front();
            queue.pop();

            if (!visited[currentNode]) {
                out << "[" << currentNode << "] ";
                visited[currentNode] = true;
            }

            auto it = std::cbegin(g.m_neighbors[currentNode]);
            auto end = std::cend(g.m_neighbors[currentNode]);
            while (it != end) {
                if (!visited[(*it).neighbor]) {
                    queue.emplace((*it).neighbor);
                }

                ++it;
            }
        }
        out << std::endl;

        return;
    }

} // namespace GraphAlgorithms

#endif  //__SIMPLE_GRAPHS__
