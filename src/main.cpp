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


    ////проверка-добавление-удаление ребер
    //void add_edge(const Vertex& from, const Vertex& to, 
    //    const Distance& d);
    //bool remove_edge(const Vertex& from, const Vertex& to);
    //bool remove_edge(const Edge& e); //c учетом расстояния
    //bool has_edge(const Vertex& from, const Vertex& to) const;
    //bool has_edge(const Edge& e) const; //c учетом расстояния в Edge

    ////получение всех ребер, выходящих из вершины
    //std::vector<Edge> edges(const Vertex& vertex);

    //size_t order() const; //порядок 
    //size_t degree(const Vertex& v) const; //степень вершины


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