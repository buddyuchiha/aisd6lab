#pragma once
#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <limits>
#include <queue>
using namespace std;

namespace algorithm {
    template<typename Vertex, typename Distance = double>
    class Graph {
    public:
        struct Edge {
            Vertex from;
            Vertex to;
            Distance distance;
        };

        bool has_vertex(const Vertex& v) const {
            return adjacency_list.count(v) > 0;
        }

        void add_vertex(const Vertex& v) {
            if (!has_vertex(v)) {
                adjacency_list[v] = {};
            }
        }

        bool remove_vertex(const Vertex& v) {
            if (has_vertex(v)) {
                adjacency_list.erase(v);
                for (auto& pair : adjacency_list) {
                    pair.second.erase(v);
                }
                return true;
            }
            return false;
        }

        vector<Vertex> vertices() const {
            vector<Vertex> v_list;
            for (const auto& pair : adjacency_list) {
                v_list.push_back(pair.first);
            }
            return v_list;
        }

        void add_edge(const Vertex& from, const Vertex& to, const Distance& d) {
            add_vertex(from);
            add_vertex(to);
            adjacency_list[from][to] = d;
        }

        bool remove_edge(const Vertex& from, const Vertex& to) {
            if (has_vertex(from) && has_vertex(to) && adjacency_list[from].count(to) > 0) {
                adjacency_list[from].erase(to);
                return true;
            }
            return false;
        }

        bool remove_edge(const Edge& e) {
            return remove_edge(e.from, e.to);
        }

        bool has_edge(const Vertex& from, const Vertex& to) const {
            if (has_vertex(from) && has_vertex(to)) {
                return adjacency_list.at(from).count(to) > 0;
            }
            return false;
        }

        bool has_edge(const Edge& e) const {
            return has_edge(e.from, e.to);
        }

        vector<Edge> edges(const Vertex& vertex) {
            vector<Edge> edge_list;
            if (has_vertex(vertex)) {
                for (const auto& pair : adjacency_list[vertex]) {
                    edge_list.push_back({ vertex, pair.first, pair.second });
                }
            }
            return edge_list;
        }

        size_t order() const {
            return adjacency_list.size();
        }

        size_t degree(const Vertex& v) const {
            if (has_vertex(v)) {
                return adjacency_list.at(v).size();
            }
            return 0;
        }

        vector<Edge> shortest_path(const Vertex& from, const Vertex& to) const {
            unordered_map<Vertex, Distance> distances;
            unordered_map<Vertex, Vertex> predecessors;
            for (const auto& pair : adjacency_list) {
                distances[pair.first] = numeric_limits<Distance>::infinity();
            }
            distances[from] = 0;

            for (size_t i = 0; i < adjacency_list.size() - 1; ++i) {
                for (const auto& pair : adjacency_list) {
                    Vertex u = pair.first;
                    for (const auto& neighbor : pair.second) {
                        Vertex v = neighbor.first;
                        Distance weight = neighbor.second;
                        if (distances[u] + weight < distances[v]) {
                            distances[v] = distances[u] + weight;
                            predecessors[v] = u;
                        }
                    }
                }
            }

            vector<Edge> path;
            for (Vertex at = to; at != from; at = predecessors[at]) {
                if (predecessors.find(at) == predecessors.end()) {
                    return {};
                }
                Vertex prev = predecessors[at];
                path.push_back({ prev, at, adjacency_list.at(prev).at(at) });
            }
            reverse(path.begin(), path.end());
            return path;
        }

        vector<Vertex> walk(const Vertex& start_vertex) const {
            vector<Vertex> result;
            if (has_vertex(start_vertex)) {
                unordered_set<Vertex> visited;
                queue<Vertex> q;
                q.push(start_vertex);
                visited.insert(start_vertex);

                while (!q.empty()) {
                    Vertex current = q.front();
                    q.pop();
                    result.push_back(current);
                    for (const auto& neighbor : adjacency_list.at(current)) {
                        if (visited.count(neighbor.first) == 0) {
                            visited.insert(neighbor.first);
                            q.push(neighbor.first);
                        }
                    }
                }
            }
            return result;
        }

        Vertex find_optimal_warehouse() const {
            Vertex optimal_warehouse;
            Distance min_max_distance = numeric_limits<Distance>::infinity();

            for (const auto& start_pair : adjacency_list) {
                Vertex start = start_pair.first;
                unordered_map<Vertex, Distance> distances;
                for (const auto& pair : adjacency_list) {
                    distances[pair.first] = numeric_limits<Distance>::infinity();
                }
                distances[start] = 0;

                for (size_t i = 0; i < adjacency_list.size() - 1; ++i) {
                    for (const auto& pair : adjacency_list) {
                        Vertex u = pair.first;
                        for (const auto& neighbor : pair.second) {
                            Vertex v = neighbor.first;
                            Distance weight = neighbor.second;
                            if (distances[u] + weight < distances[v]) {
                                distances[v] = distances[u] + weight;
                            }
                        }
                    }
                }

                Distance max_distance = 0;
                for (const auto& distance_pair : distances) {
                    if (distance_pair.second != numeric_limits<Distance>::infinity() && distance_pair.second > max_distance) {
                        max_distance = distance_pair.second;
                    }
                }

                if (max_distance < min_max_distance) {
                    min_max_distance = max_distance;
                    optimal_warehouse = start;
                }
            }

            return optimal_warehouse;
        }

    private:
        unordered_map<Vertex, unordered_map<Vertex, Distance>> adjacency_list;
    };
}