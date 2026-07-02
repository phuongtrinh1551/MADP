#pragma once
#include <vector>
#include <string>

class GomoryHuTree {
public:
    GomoryHuTree(int number_vertices, std::vector<std::vector<int>> graph);

    int sz;
    std::vector<std::vector<int>> graph;

    struct Edge;
    struct Vertex;

    typedef std::vector<int> Row;
    typedef std::vector<Row> Matrix;
    typedef std::vector<Vertex*> VertexList;
    typedef std::vector<Edge*> EdgeList;

    struct Vertex {
        int id;
        VertexList group;
        EdgeList edges;
        int flag;
        Vertex* parent;
        Vertex(int id);
        Vertex(const VertexList &group);
    };

    struct Edge {
        Vertex* vertex;
        int c;
        int f;
        Edge(Vertex* vertex, int c);
    };

    struct MinCut {
        int f;
        Vertex* s;
        Vertex* t;
        VertexList A;
        VertexList B;
        MinCut(Vertex* s, Vertex* t);
    };

    VertexList::iterator findVertex(VertexList &set, Vertex* v);
    VertexList setMinus(VertexList &set1, VertexList &set2);
    VertexList setMul(VertexList &set1, VertexList &set2);
    EdgeList::iterator findEdge(Vertex* v1, Vertex* v2);
    void addEdge(Vertex* v1, Vertex* v2, const int c, bool addReverse = true);
    Edge* getEdge(Vertex* v1, Vertex* v2);
    void deleteEdge(Vertex* v1, Vertex* v2);
    void deleteVertex(VertexList &set, Vertex* v);
    void deleteVertexList(VertexList &set);
    VertexList extractGroups(const VertexList &set);
    Matrix vertexListToMatrix(const VertexList &set);
    void sortListById(VertexList &set);
    std::string vertexToStr(const Vertex* vertex);
    std::string vertexListToStr(const VertexList &set);
    std::string matrixToStr(const Matrix &m);
    VertexList findPath(const VertexList &set, Vertex* s, Vertex* t);
    MinCut findMinCut(VertexList &set);
    Matrix buildGomoryHuTree(const Matrix &g);
    std::vector<std::vector<int>> buildGomoryHuTree();
};