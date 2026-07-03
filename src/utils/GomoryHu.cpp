#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <climits> 
#include "GomoryHu.hpp"

using namespace std;

#define debug 0

GomoryHuTree::GomoryHuTree(int number_vertices, std::vector<std::vector<int>> graph){
    this->sz = number_vertices;
    this->graph = graph;
}

GomoryHuTree::Vertex::Vertex(int id) : id(id) { }
GomoryHuTree::Vertex::Vertex(const VertexList &group) : id(-1), group(group) { }

GomoryHuTree::Edge::Edge(Vertex* vertex, int c) : vertex(vertex), c(c), f(0) { }

GomoryHuTree::MinCut::MinCut(Vertex* s, Vertex* t) : s(s), t(t), f(0) {}

GomoryHuTree::VertexList::iterator GomoryHuTree::findVertex(VertexList &set, Vertex* v) 
{
    VertexList::iterator i = set.begin();
    VertexList::iterator j = set.end();
    while(i != j && (*i) != v) i++;
    return i;
}

GomoryHuTree::VertexList GomoryHuTree::setMinus(VertexList &set1, VertexList &set2) 
{
    VertexList result;
    for (Vertex* v : set1)
    {
        if(findVertex(set2, v) == set2.end()) result.push_back(v);
    }
    return result;
}

GomoryHuTree::VertexList GomoryHuTree::setMul(VertexList &set1, VertexList &set2) 
{
    VertexList result;
    for(Vertex* v:set2)
    {
        if(findVertex(set1, v) != set1.end()) result.push_back(v);
    }
    return result;
}

GomoryHuTree::EdgeList::iterator GomoryHuTree::findEdge(Vertex* v1, Vertex* v2) 
{
    EdgeList::iterator i = v1->edges.begin();
    EdgeList::iterator j = v1->edges.end();
    while(i != j && (*i)->vertex != v2) i++;
    return i;
}

void GomoryHuTree::addEdge(Vertex* v1, Vertex* v2, const int c, bool addReverse)
{
    EdgeList::iterator i = findEdge(v1, v2);
    if(i != v1->edges.end()) (*i)->c += c;
    else v1->edges.push_back(new Edge(v2, c));

    if(!addReverse) return;
    
    i = findEdge(v2, v1);
    if(i != v2->edges.end()) (*i)->c += c;
    else v2->edges.push_back(new Edge(v1, c));
}

GomoryHuTree::Edge* GomoryHuTree::getEdge(Vertex* v1, Vertex* v2)
{
    EdgeList::iterator i = findEdge(v1, v2);
    return (i == v1->edges.end() ? NULL : *i);
}

void GomoryHuTree::deleteEdge(Vertex* v1, Vertex* v2)
{
    EdgeList::iterator i = findEdge(v1, v2);
    if(i == v1->edges.end()) return;

    delete (*i);
    v1->edges.erase(i);
}
   
void GomoryHuTree::deleteVertex(VertexList &set, Vertex* v) 
{
    VertexList::iterator i = findVertex(set, v);
    if(i == set.end()) return;

    for(Edge* e:v->edges) 
    {
        deleteEdge(e->vertex, v);
        delete e;
    }

    delete (*i);
    set.erase(i);
}

void GomoryHuTree::deleteVertexList(VertexList &set)
{
    for(Vertex* v:set)
    {
        for(Edge* e:v->edges) delete e;
        delete v;
    }
    set.clear();
}
   
GomoryHuTree::VertexList GomoryHuTree::extractGroups(const VertexList &set) 
{
    VertexList result;
    for(Vertex* v:set)
    {
        if(v->group.size() > 0)
        {
            for(Vertex* subv:v->group) result.push_back(subv);
        }
        else result.push_back(v);
    }
    return result;
}

GomoryHuTree::Matrix GomoryHuTree::vertexListToMatrix(const VertexList &set)
{
    int n = static_cast<int>(set.size());
    Matrix m(n, Row(n, 0));

    for(int i = 0; i < n; i++) set[i]->flag = i;
    for(Vertex* v:set)
    {
        for(Edge* e:v->edges) m[v->flag][e->vertex->flag] = e->c;
    }
    return m;
}

void GomoryHuTree::sortListById(VertexList &set)
{
    size_t n = set.size();
    for(size_t i = 0; i < n - 1; i++) 
    {
        size_t k = i;
        for(size_t j = i + 1; j < n; j++) 
        {
            if(set[j]->id < set[k]->id) k = j;
        }
        if(k != i)
        {
            Vertex* v = set[k];
            set[k] = set[i];
            set[i] = v;
        }
    }
}

string GomoryHuTree::vertexToStr(const Vertex* vertex)
{
    stringstream s;
    s << "{";
    if(vertex->group.size() == 0)
    {
        s << vertex->id << "}";
        return s.str();
    }
    else
    {
        for(Vertex* v:vertex->group) s << vertexToStr(v) << ",";
        return s.str().substr(0, s.str().size() - 1) + "}";
    }
}

string GomoryHuTree::vertexListToStr(const VertexList &set)
{
    stringstream s;
    s << "[ ";
    for(Vertex* v:set) s << vertexToStr(v) << ", ";
    return s.str().substr(0, s.str().size() - 2) + " ]";
}

string GomoryHuTree::matrixToStr(const Matrix &m)
{
    int n = static_cast<int>(m.size());
    stringstream s;
    for(int i = 0; i < n; i++)
    {
        s << "    ";
        for(int j = 0; j < n; j++) s << setw(3) << m[i][j];
        s << endl;
    }
    return s.str();
}

GomoryHuTree::VertexList GomoryHuTree::findPath(const VertexList &set, Vertex* s, Vertex* t)
{
    for(Vertex* v:set)
    {
        v->parent = NULL;
        v->flag = 0;
    }
    
    size_t i = 0;
    VertexList queue;
    s->flag = 1;
    queue.push_back(s);

    while(i < queue.size())
    {
        for(Edge* e:queue[i]->edges) 
        {
            Vertex* v = e->vertex;
            if(v->flag == 0 && e->c > 0) 
            {
                v->parent = queue[i];
                v->flag = 1;
                queue.push_back(v);
                if(v == t) return queue;
            }
        }
        i++;
    }
    return queue;
}

GomoryHuTree::MinCut GomoryHuTree::findMinCut(VertexList &set)
{
    MinCut result(set[0], set[1]);
    for(;;)
    {
        VertexList path = findPath(set, result.s, result.t);
        if(result.t->parent == NULL) break;

        int min_c = INT_MAX;
        Vertex* v = result.t;
        EdgeList edges;
        while(v->parent != NULL)
        {
            edges.push_back(getEdge(v, v->parent));
            Edge* e = getEdge(v->parent, v);
            edges.push_back(e);
            if(e->c < min_c) min_c = e->c;
            v = v->parent;
        }

        result.f += min_c;
        for(Edge* e:edges) 
        {
            e->f += min_c;
            e->c -= min_c;
        }
    }

    result.B = findPath(set, result.t, result.s);
    result.A = setMinus(set, result.B);

    for(Vertex* v:set)
    {
        for(Edge* e:v->edges)
        {
            e->c += e->f;
            e->f = 0;
        }
    }
    return result;
}

GomoryHuTree::Matrix GomoryHuTree::buildGomoryHuTree(const Matrix &g)
{
    VertexList Vg = VertexList();
    VertexList Vt = VertexList();
    
    size_t n = g.size();
    for(size_t i = 0; i < n; i++)
    {
        Vg.push_back(new Vertex(static_cast<int>(i)));
        for(size_t j = 0; j < i; j++)
        {
            if(g[i][j] != 0) addEdge(Vg[i], Vg[j], g[i][j]);
        }
    }
    
    Vt.push_back(new Vertex(Vg));

    for(;;)
    {
        VertexList::iterator i = Vt.begin();
        VertexList::iterator j = Vt.end();
        while(i != j && (*i)->group.size() < 2) i++;
        if(i == j) break;
        
        Vertex* x = *i;
        VertexList G = VertexList();

        for(Vertex* v:x->group) 
        {
            v->parent = new Vertex(VertexList(1, v));
            G.push_back(v->parent);
        }

        for(Edge* e:x->edges)
        {
            deleteEdge(e->vertex, x);
            Vertex* z = new Vertex(extractGroups(findPath(Vt, e->vertex, x)));
            addEdge(e->vertex, x, e->c, false);

            for(Vertex* v:z->group) v->parent = z;
            G.push_back(z);
        }

        for(Vertex* z:G)
        {
            for(Vertex* v:z->group)
            {
                for(Edge* e:v->edges) 
                {
                    if(z != e->vertex->parent) addEdge(z, e->vertex->parent, e->c, false);
                }
            }
        }

        MinCut cut = findMinCut(G);
        cut.A = extractGroups(cut.A);
        cut.B = extractGroups(cut.B);

        deleteVertexList(G);

        Vertex* XA = new Vertex(setMul(x->group, cut.A));
        Vertex* XB = new Vertex(setMul(x->group, cut.B));
        addEdge(XA, XB, cut.f);

        for(Edge* e:x->edges)
        {
            Vertex* v = e->vertex;
            addEdge((findVertex(cut.A, v->group[0]) != cut.A.end() ? XA : XB), v, e->c);
        }

        Vt.insert(Vt.insert(i, XB), XA);
        deleteVertex(Vt, x);
    }

    for(Vertex* v:Vt) 
    {
        v->id = v->group[0]->id;
        v->group.clear();
    }

    sortListById(Vt);
    Matrix m = vertexListToMatrix(Vt);
    
    deleteVertexList(Vg);
    deleteVertexList(Vt);
    return m;
}

std::vector<std::vector<int>> GomoryHuTree::buildGomoryHuTree()
{
    setlocale(LC_ALL, "Russian");
    std::cout << "\n----------------------------------------" << std::endl;
    std::cout << "#> Computing Gomory-Hu Tree" << std::endl;
    for(int j = 0; j < sz; j++)
    {
        if(graph[j][j] != 0) return graph;
        for(int i = 0; i < j; i++)
        {
            if(graph[i][j] != graph[j][i] || graph[i][j] < 0) return graph;
        }
    }

    Matrix g(sz, Row(sz));
    for(int i = 0; i < sz; i++) for(int j = 0; j < sz; j++) g[i][j] = graph[i][j];
    Matrix m = buildGomoryHuTree(g);
    std::cout << "\n----------------------------------------" << std::endl;
    return m;
}

