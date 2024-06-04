#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <limits>
#include <functional>
#include <queue>
using namespace std;


template<typename Vertex, typename Distance = double>
class Graph {
public:
    struct Edge {
        Vertex from;
        Vertex to;
        Distance distance;

        bool operator==(const Edge& other) const {
            return from == other.from && to == other.to && distance == other.distance;
        }
    };
    bool has_vertex(const Vertex& v) const {
        return adjacency_list.find(v) != adjacency_list.end();
    }
    void add_vertex(const Vertex& v) {
        adjacency_list[v]; 
    }
    bool remove_vertex(const Vertex& v) {
        if (!has_vertex(v)) return false;
        adjacency_list.erase(v);

        for (auto& [vertex, edges] : adjacency_list) {
            edges.erase(v);
        }

        return true;
    }
    vector<Vertex> vertices() const {
        vector<Vertex> v;
        for (const auto& [vertex, _] : adjacency_list) {
            v.push_back(vertex);
        }
        return v;
    }

    void add_edge(const Vertex& from, const Vertex& to,const Distance& d) {
        add_vertex(from);
        add_vertex(to);
        adjacency_list[from][to] = d;
    }
    bool remove_edge(const Vertex& from, const Vertex& to) {
        if (!has_vertex(from) || !has_vertex(to)) return false;
        auto it = adjacency_list[from].find(to);
        if (it == adjacency_list[from].end()) return false;
        adjacency_list[from].erase(it);
        return true;
    }
    bool remove_edge(const Edge& e) {
        return remove_edge(e.from, e.to);
    }
    bool has_edge(const Vertex& from, const Vertex& to) const {
        if (!has_vertex(from) || !has_vertex(to)) return false;
        return adjacency_list.at(from).find(to) != adjacency_list.at(from).end();
    }
    bool has_edge(const Edge& e) const {
        return has_edge(e.from, e.to);
    }

    vector<Edge> edges(const Vertex& vertex) {
        vector<Edge> edge_list;
        if (!has_vertex(vertex)) return edge_list;
        for (const auto& [to, distance] : adjacency_list[vertex]) {
            edge_list.push_back({ vertex, to, distance });
        }
        return edge_list;
    }

    size_t order() const {
        return adjacency_list.size();
    }

    size_t degree(const Vertex& v) const {
        if (!has_vertex(v)) return 0;
        return adjacency_list.at(v).size();
    }


    vector<Edge> shortest_path(const Vertex& from, const Vertex& to) const {
        unordered_map<Vertex, Distance> distances;
        unordered_map<Vertex, Vertex> predecessors;

        for (const auto& [vertex, _] : adjacency_list) {
            distances[vertex] = numeric_limits<Distance>::infinity();
        }
        distances[from] = 0;

        size_t V = adjacency_list.size();
        for (size_t i = 1; i < V; ++i) {
            for (const auto& [u, edges] : adjacency_list) {
                for (const auto& [v, w] : edges) {
                    if (distances[u] + w < distances[v]) {
                        distances[v] = distances[u] + w;
                        predecessors[v] = u;
                    }
                }
            }
        }

        for (const auto& [u, edges] : adjacency_list) {
            for (const auto& [v, w] : edges) {
                if (distances[u] + w < distances[v]) {
                    std::cerr << "Graph contains a negative-weight cycle\n";
                    return {};
                }
            }
        }

        vector<Edge> path;
        for (Vertex v = to; v != from; v = predecessors[v]) {
            if (predecessors.find(v) == predecessors.end()) return {};
            path.push_back({ predecessors[v], v, adjacency_list.at(predecessors[v]).at(v) });
        }
        reverse(path.begin(), path.end());
        return path;
    }

    vector<Vertex> walk(const Vertex& start_vertex) const {
        vector<Vertex> result;
        if (!has_vertex(start_vertex)) return result;

        unordered_set<Vertex> visited;
        queue<Vertex> q;

        q.push(start_vertex);
        visited.insert(start_vertex);

        while (!q.empty()) {
            Vertex current = q.front();
            q.pop();
            result.push_back(current);

            for (const auto& [neighbor, _] : adjacency_list.at(current)) {
                if (visited.find(neighbor) == visited.end()) {
                    visited.insert(neighbor);
                    q.push(neighbor);
                }
            }
        }

        return result;
    }


private:
    unordered_map<Vertex, unordered_map<Vertex, Distance>> adjacency_list;
};

int main() {
    Graph<int, double> graph;

    // Добавляем вершины и рёбра
    graph.add_edge(1, 2, 5.0);
    graph.add_edge(1, 3, 3.0);
    graph.add_edge(2, 3, 2.0);
    graph.add_edge(2, 4, 7.0);
    graph.add_edge(3, 4, 4.0);

    // Проверяем методы
    cout << "Vertices in the graph: ";
    for (const auto& vertex : graph.vertices()) {
        cout << vertex << " ";
    }
    cout << endl;

    cout << "Edges from vertex 1: ";
    for (const auto& edge : graph.edges(1)) {
        cout << "(" << edge.from << "->" << edge.to << ", distance: " << edge.distance << ") ";
    }
    cout << endl;

    cout << "Shortest path from 1 to 4: ";
    vector<Graph<int, double>::Edge> shortest_path = graph.shortest_path(1, 4);
    for (const auto& edge : shortest_path) {
        cout << "(" << edge.from << "->" << edge.to << ", distance: " << edge.distance << ") ";
    }
    cout << endl;

    return 0;
}