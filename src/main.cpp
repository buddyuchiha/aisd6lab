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
    //��������-����������-�������� ������
    bool has_vertex(const Vertex& v) const;
    void add_vertex(const Vertex& v);
    bool remove_vertex(const Vertex& v);
    std::vector<Vertex> vertices() const;


    //��������-����������-�������� �����
    void add_edge(const Vertex& from, const Vertex& to,
        const Distance& d);
    bool remove_edge(const Vertex& from, const Vertex& to);
    bool remove_edge(const Edge& e); //c ������ ����������
    bool has_edge(const Vertex& from, const Vertex& to) const;
    bool has_edge(const Edge& e) const; //c ������ ���������� � Edge

    //��������� ���� �����, ��������� �� �������
    std::vector<Edge> edges(const Vertex& vertex);

    size_t order() const; //������� 
    size_t degree(const Vertex& v) const; //������� �������


    //����� ����������� ����
    std::vector<Edge> shortest_path(const Vertex& from,
        const Vertex& to) const;
    //�����
    std::vector<Vertex>  walk(const Vertex& start_vertex)const;


private:
    unordered_map<Vertex, std::unordered_map<Vertex, Distance>> adjacency_list;
};

