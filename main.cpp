/******************************************************************************

                              Online C++ Compiler.
               Code, Compile, Run and Debug C++ program online.
Write your code in this editor and press "Run" button to compile and execute it.

*******************************************************************************/

#include <iostream>
#include <vector>
#include <queue>
#include <utility>   // For std::pair
#include <limits>    // For std::numeric_limits
#include <algorithm> // For std::reverse

// A constant to represent infinity, used for nodes that are not yet reached.
const int INF = std::numeric_limits<int>::max();

// A type alias for a pair of integers (int, int).
// This will be used to store {weight, vertex} pairs in the priority queue.
using iPair = std::pair<int, int>;

/**
 * @class Graph
 * @brief Represents a weighted, bidirectional graph.
 *
 * This class uses an adjacency list to store the graph structure.
 * It provides methods to add edges and find the shortest path between
 * two vertices using Dijkstra's algorithm.
 */
class Graph {
private:
    int V; // Number of vertices in the graph
    std::vector<std::vector<iPair>> adj; // Adjacency list representation

public:
    /**
     * @brief Constructs a Graph with a given number of vertices.
     * @param vertices The total number of vertices in the graph.
     */
    Graph(int vertices);

    /**
     * @brief Adds a weighted, bidirectional edge between two vertices.
     * @param u The first vertex.
     * @param v The second vertex.
     * @param w The weight of the edge between u and v.
     */
    void addEdge(int u, int v, int w);

    /**
     * @brief Finds the shortest path and minimum cost between a source and a destination vertex.
     * @param src The starting vertex.
     * @param dest The destination vertex.
     * @return A pair containing the minimum cost (int) and the path (vector<int>).
     * If no path exists, the cost will be INF and the path will be empty.
     */
    std::pair<int, std::vector<int>> shortestPath(int src, int dest);
};

// Constructor implementation
Graph::Graph(int vertices) {
    this->V = vertices;
    adj.resize(V); // Resize the adjacency list to hold V vertices
}

// Method to add a bidirectional edge
void Graph::addEdge(int u, int v, int w) {
    // Add an edge from u to v with weight w
    adj[u].push_back({v, w});
    // Since the graph is bidirectional, add an edge from v to u with the same weight
    adj[v].push_back({u, w});
}

// Method to find the shortest path using Dijkstra's algorithm
std::pair<int, std::vector<int>> Graph::shortestPath(int src, int dest) {
    // A min-priority queue to store vertices that are being processed.
    // The pair is {distance, vertex}. std::greater makes it a min-heap.
    std::priority_queue<iPair, std::vector<iPair>, std::greater<iPair>> pq;

    // 'dist' vector to store the shortest distance from the source to each vertex.
    std::vector<int> dist(V, INF);

    // 'parent' vector to reconstruct the path from source to destination.
    std::vector<int> parent(V, -1);

    // Insert the source vertex into the priority queue and initialize its distance as 0.
    pq.push({0, src});
    dist[src] = 0;

    // Main loop of Dijkstra's algorithm
    while (!pq.empty()) {
        // Extract the vertex with the smallest distance from the priority queue.
        int u = pq.top().second;
        pq.pop();

        // Optimization: If we have reached the destination, we can stop.
        if (u == dest) break;

        // Iterate through all adjacent vertices of the dequeued vertex 'u'.
        for (auto& edge : adj[u]) {
            int v = edge.first;
            int weight = edge.second;

            // Relaxation step: If we found a shorter path to 'v' through 'u'.
            if (dist[v] > dist[u] + weight) {
                // Update the distance of 'v'.
                dist[v] = dist[u] + weight;
                // Push the updated distance and vertex to the priority queue.
                pq.push({dist[v], v});
                // Set 'u' as the parent of 'v' in the shortest path tree.
                parent[v] = u;
            }
        }
    }

    // If the destination was not reached, its distance will remain INF.
    if (dist[dest] == INF) {
        return {INF, {}}; // Return infinity cost and an empty path
    }

    // Reconstruct the path by backtracking from the destination using the parent array.
    std::vector<int> path;
    int currentNode = dest;
    while (currentNode != -1) {
        path.push_back(currentNode);
        currentNode = parent[currentNode];
    }
    std::reverse(path.begin(), path.end()); // Reverse the path to get the correct order (src -> dest)

    return {dist[dest], path};
}

// Main function to demonstrate the usage of the Graph class
int main() {
    // Create a graph with 9 vertices (0-8)
    int V = 9;
    Graph g(V);

    // Build the graph by adding edges
    // addEdge(node1, node2, weight)
    g.addEdge(0, 1, 4);
    g.addEdge(0, 7, 8);
    g.addEdge(1, 2, 8);
    g.addEdge(1, 7, 11);
    g.addEdge(2, 3, 7);
    g.addEdge(2, 8, 2);
    g.addEdge(2, 5, 4);
    g.addEdge(3, 4, 9);
    g.addEdge(3, 5, 14);
    g.addEdge(4, 5, 10);
    g.addEdge(5, 6, 2);
    g.addEdge(6, 7, 1);
    g.addEdge(6, 8, 6);
    g.addEdge(7, 8, 7);

    // --- Test Case 1 ---
    int source = 0;
    int destination = 4;
    std::cout << "--- Finding shortest path from node " << source << " to node " << destination << " ---" << std::endl;

    // Call the shortestPath function
    std::pair<int, std::vector<int>> result = g.shortestPath(source, destination);
    int cost = result.first;
    std::vector<int> path = result.second;

    // Print the results
    if (cost == INF) {
        std::cout << "No path found between " << source << " and " << destination << "." << std::endl;
    } else {
        std::cout << "Minimum Cost: " << cost << std::endl;
        std::cout << "Path: ";
        for (size_t i = 0; i < path.size(); ++i) {
            std::cout << path[i] << (i == path.size() - 1 ? "" : " -> ");
        }
        std::cout << std::endl;
    }

    // --- Test Case 2 ---
    destination = 3;
    std::cout << "\n--- Finding shortest path from node " << source << " to node " << destination << " ---" << std::endl;

    result = g.shortestPath(source, destination);
    cost = result.first;
    path = result.second;

    if (cost == INF) {
        std::cout << "No path found between " << source << " and " << destination << "." << std::endl;
    } else {
        std::cout << "Minimum Cost: " << cost << std::endl;
        std::cout << "Path: ";
        for (size_t i = 0; i < path.size(); ++i) {
            std::cout << path[i] << (i == path.size() - 1 ? "" : " -> ");
        }
        std::cout << std::endl;
    }

    return 0;
}
