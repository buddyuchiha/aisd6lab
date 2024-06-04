#include <iostream>
#include "..\algorithm\algorithm.h"

using namespace std;
using namespace algorithm;

int main() {
    Graph<int> g;

    g.add_vertex(1);
    g.add_vertex(2);
    g.add_vertex(3);

    cout << "Graph has vertex 1: " << g.has_vertex(1) << endl;
    cout << "Graph has vertex 4: " << g.has_vertex(4) << endl;

    g.add_edge(1, 2, 1.0);
    g.add_edge(1, 3, 2.5);
    g.add_edge(2, 3, 1.5);

    cout << "Graph has edge 1->2: " << g.has_edge(1, 2) << endl;
    cout << "Graph has edge 2->1: " << g.has_edge(2, 1) << endl;

    vector<int> vertex_list = g.vertices();
    cout << "Vertices in graph: ";
    for (int v : vertex_list) {
        cout << v << " ";
    }
    cout << endl;

    vector<Graph<int>::Edge> edge_list = g.edges(1);
    cout << "Edges from vertex 1: ";
    for (const auto& edge : edge_list) {
        cout << "(" << edge.from << "->" << edge.to << ", " << edge.distance << ") ";
    }
    cout << endl;

    g.remove_edge(1, 2);
    cout << "Graph has edge 1->2 after removal: " << g.has_edge(1, 2) << endl;

    g.remove_vertex(3);
    cout << "Graph has vertex 3 after removal: " << g.has_vertex(3) << endl;

    cout << "Degree of vertex 1: " << g.degree(1) << endl;

    vector<int> bfs = g.walk(1);
    cout << "BFS walk starting from vertex 1: ";
    for (int v : bfs) {
        cout << v << " ";
    }
    cout << endl;

    g.add_edge(1, 2, 1.0);
    g.add_edge(2, 3, 1.5);
    vector<Graph<int>::Edge> shortest_path = g.shortest_path(1, 3);
    cout << "Shortest path from 1 to 3: ";
    for (const auto& edge : shortest_path) {
        cout << "(" << edge.from << " -> " << edge.to << ", " << edge.distance << ") ";
    }
    cout << endl;

    int optimal_warehouse = g.find_optimal_warehouse();
    cout << "Optimal warehouse location: " << optimal_warehouse << endl;

    return 0;
}
