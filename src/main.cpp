#include <iostream>
#include <vector>
#include <unordered_map>
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


    ////поиск кратчайшего пути
    //std::vector<Edge> shortest_path(const Vertex& from,
    //    const Vertex& to) const;
    ////обход
    //std::vector<Vertex>  walk(const Vertex& start_vertex)const;


private:
    unordered_map<Vertex, std::unordered_map<Vertex, Distance>> adjacency_list;
};

int main() {
    return 0;
}