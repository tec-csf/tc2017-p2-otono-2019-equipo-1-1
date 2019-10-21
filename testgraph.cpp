//Insertar Vertice tiene una complejidad O(1)
//Eliminar Vertice tiene una complejidad O(n)
//Insertar Arista tiene una complejidad de O(1)
//Eliminar Arista tiene una compledijad de O(n)
//DFS tiene complejidad O(V+E) y la tecnica de diseno es avido
//BFS tiene complejidad O(V+E) y la tecnica de diseno es avido
//Kruskal tiene complejidad O(E log E) y la tecnica de diseno es avido
//Dijkstra tiene complejidad O(V^@) y la tecnica de diseno es avido
//Floyd Warshall tiene complejidad O(V^3) y la tecnica de diseno es programacion dinamica

//El codigo se hizo en base a:
//https://github.com/margotduek/AnalisisDisenioDeAlgoritmos/blob/master/tarea4/Otra/tutorials/main.cpp 
//https://www.boost.org/doc/libs/1_41_0/libs/graph/example/dijkstra-example.cpp 
//https://www.boost.org/doc/libs/1_51_0/libs/graph/doc/kruskal_min_spanning_tree.html
//https://www.boost.org/doc/libs/1_51_0/libs/graph/example/kruskal-example.cpp
//http://snap.standford.edu/snap/quick.htmk

#include "/home/fernando/Snap-5.0/snap-core/Snap.h"
#include "stdafx.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <limits>
#include <queue>
#include <iomanip>
#include <chrono>
#define INF 1000000

using namespace std;
using namespace TSnap;
using namespace std::chrono;

void Kruskal(TPt<TNodeEDatNet<TInt, TFlt> > G, int vertices);
void Dijkstra(TPt<TNodeEDatNet<TInt, TFlt> >  G, const int &SrcNId);
void FloydWarshall();
bool sortFunction(int a, int b, vector<float> distancias);
int find(int a, int padre[]);
void setUnion(int a, int b, int padre[]);

typedef TNodeEDatNet<TInt,TFlt>::TNodeI SnapNode;
typedef TNodeEDatNet<TInt,TFlt>::TEdgeI SnapEdge;

class graph {
public:
    graph() { }
    graph(const int& Nodes){ }
    void DiscoverNode(int a) {}
    void FinishNode(const int& a) { }
    void ExamineEdge(const int& a, const int& b) { }
    void TreeEdge(const int& a, const int& b) { }
    void BackEdge(const int& a, const int& b) { }
    void FwdEdge(const int& a, const int& b) { }

};


class distanciaId
{
private:
public:
  double distancia;
  int id;
  int padre;
  distanciaId() { distancia = -1; id = 0; }
  double getDistancia() { return distancia; }
};

class grafos {
  vector<float> type_;
public:
  grafos(vector<float> type) : type_(type) {}
  bool operator()(int a, int b) const
  {
    return sortFunction(a, b, type_ );
  }
};

struct {

  bool operator()(distanciaId* a, distanciaId* b)
  {
    return a->getDistancia() < b->getDistancia();
  }
} comparaDistancia;

TPt<TNodeEDatNet<TInt, TFlt> >  G = TNodeEDatNet<TInt, TFlt>::New();

bool vertice(int a)
{
    for (SnapNode NI = G->BegNI(); NI < G->EndNI(); NI++)
    {
        if(NI.GetDat() == a)
        {
            return true;
        }
    }
    return false;
}

bool edge(int a, int b)
{
    for (SnapEdge EI = G->BegEI(); EI < G->EndEI(); EI++)
    {
        if(EI.GetDstNDat() == b && EI.GetSrcNDat() == a)
            return true;
    }
    return false;
}

bool edge(int a, int b, TFlt weight)
{
    for (SnapEdge EI = G->BegEI(); EI < G->EndEI(); EI++)
    {
        if(EI.GetDstNDat() == b && EI.GetSrcNDat() == a && EI.GetDat() == weight)
            return true;
    }
    return false;
}

int find(int a, int padre[]) {
  if (padre[a-1] != a)
    padre[a-1] = find(padre[a-1], padre);
  return padre[a-1];
}

void setUnion(int a, int b, int padre[]) {
  a = find(a, padre);
  b = find(b, padre);
  padre[a-1] = b;
}

bool sortFunction(int a, int b, vector<float> distancias){
  return distancias[a-1] < distancias[b-1];
}



void insertarVertice(int a){
  G->AddNode(a, a);
}

void insertarArista(int a, int b, float weight){
  G->AddEdge(a, b, weight);
}

void eliminarVertice(int v){
  G->DelNode(v);
}

void eliminarArista(int a, int b){
  G->DelEdge(a, b);    
}

void DFS(){
    graph vis(G->GetNodes());
    TCnCom::GetDfsVisitor(G, vis);
}

void BFS(){
    PNGraph GBFS = TSnap::GetBfsTree(G, 1, true, true);
}


void Kruskal() {
  int padre[G->GetNodes()];
  vector<pair<int, TPt<TNodeEDatNet<TInt, TFlt> >::TObj::TEdgeI> > aristas;
  vector<TPt<TNodeEDatNet<TInt, TFlt> >::TObj::TEdgeI> tree;

  for (TPt<TNodeEDatNet<TInt, TFlt> >::TObj::TNodeI NI = G->BegNI(); NI < G->EndNI(); NI++) {
    int id = NI.GetId();
    padre[id-1] = id;
  }

  for (TPt<TNodeEDatNet<TInt, TFlt> >::TObj::TEdgeI gg = G->BegEI(); gg < G->EndEI(); gg++)
    aristas.push_back(make_pair(gg.GetDat(), gg));

  sort(aristas.begin(), aristas.end());

  for (unsigned int i = 0; i < aristas.size(); ++i) {
    int a = aristas[i].second.GetSrcNId();
    int b = aristas[i].second.GetDstNId();
    if (find(a, padre) != find(b, padre)) {
      tree.push_back(aristas[i].second);
      setUnion(a, b, padre);
    }
  }

  while (!tree.empty()) {
    tree.erase(tree.begin());
  }
}


void Dijkstra(){
  int s = 1;
  deque<int> nV;
  vector<int> v;

  vector<float> distances (G->GetNodes(),INFINITY);
  vector<int> parents (G->GetNodes(),-1);

  for(int i=0; i<G->GetNodes(); i++){
    nV.push_back(i+1);
  }

  distances[s-1] = 0;

  while(!nV.empty()){

    sort(nV.begin(), nV.end(), grafos(distances));
    int a= nV[0];
    int sourceNode = a;

    nV.pop_front();
    v.push_back(a);

    TNodeEDatNet<TInt, TFlt>::TNodeI NI = G->GetNI(sourceNode);

    for (int b = 0; b < NI.GetOutDeg(); b++){

      int destNode = NI.GetOutNId(b);
      TNodeEDatNet<TInt, TFlt>::TEdgeI EI = G->GetEI(sourceNode,destNode);
      float edgeWeight = (float)EI.GetDat();

      if(distances[destNode-1] > distances[sourceNode-1] + edgeWeight){
        distances[destNode-1] = distances[sourceNode-1] + edgeWeight;
        parents[destNode-1] = sourceNode;
      }
    }
  }
}

const int NUM = 15;
int vVal[NUM][NUM];

void FloydWarshall()
{
    int table[NUM][NUM];
    for (int j = 1; j < NUM; j++){
        for (int i = 1; i < NUM; i++){
            table[i][j] = vVal[i][j];
            if (table[i][j] == 0) table[i][j] = INF;
            if (i == j) table[i][j] = 0;
        }
    }

    for (int z = 1; z < NUM; z++){
        for (int j = 1; j < NUM; j++){
            for (int i = 1; i < NUM; i++){
                if (table[z][j] + table[i][z]< table[i][j]){
                    table[i][j] = table[z][j] + table[i][z];
                }
            }
        }
    }

    for (int j = 1; j < NUM; j++){
        for (int i = 1; i < NUM; i++){
            if (table[i][j] == INF) cout << "||";
            else{
                if (table[i][j] == 0) cout << "  ";
                else cout << setfill('0') << setw(2) << table[i][j];
            }
            cout << " ";
        }
        cout << endl;
    }

}

int main(int argc, char* argv[]) {
	
    srand((unsigned) time(0));
    TPt<TNodeEDatNet<TInt, TFlt> >  G = TNodeEDatNet<TInt, TFlt>::New();

    for(int i=1; i<15; i++)
    {
        G->AddNode(i,i);
    }

    G->AddEdge(1,4,8);
    G->AddEdge(1,3,8);
    G->AddEdge(2,5,7);
    G->AddEdge(3,2,7);
    G->AddEdge(3,10,4);
    G->AddEdge(3,5,8);
    G->AddEdge(4,7,3);
    G->AddEdge(4,8,2);
    G->AddEdge(4,5,1);
    G->AddEdge(5,6,9);
    G->AddEdge(6,13,4);
    G->AddEdge(7,4,6);
    G->AddEdge(8,7,3);
    G->AddEdge(8,9,3);
    G->AddEdge(9,10,2);
    G->AddEdge(9,12,4);
    G->AddEdge(10,6,6);
    G->AddEdge(10,3,10);
    G->AddEdge(11,12,6);
    G->AddEdge(12,9,2);
    G->AddEdge(12,11,8);
    G->AddEdge(12,14,9);
    G->AddEdge(13,14,6);
    G->AddEdge(14,13,2);

    auto start = high_resolution_clock::now();

    cout << "Insertar Vertice" << endl;
    insertarVertice(G);

    auto stop = high_resolution_clock::();
    auto durationMili = duration_cast<miliseconds>(stop-start);
    cout << "\n";
    cout << "Le tomo" << durationMili.count() << " milisegundos\n";
    cout << "\n";

    auto start = high_resolution_clock::now();

    cout << "Insertar Arista" << endl;
    insertarArista(G);

    auto stop = high_resolution_clock::();
    auto durationMili = duration_cast<miliseconds>(stop-start);
    cout << "\n";
    cout << "Le tomo" << durationMili.count() << " milisegundos\n";
    cout << "\n";

    auto start = high_resolution_clock::now();

    cout << "Eliminar Vertice" << endl;
    eliminarVertice(G);

    auto stop = high_resolution_clock::();
    auto durationMili = duration_cast<miliseconds>(stop-start);
    cout << "\n";
    cout << "Le tomo" << durationMili.count() << " milisegundos\n";
    cout << "\n";

    auto start = high_resolution_clock::now();

    cout << "Eliminar Arista" << endl;
    eliminarArista(G);

    auto stop = high_resolution_clock::();
    auto durationMili = duration_cast<miliseconds>(stop-start);
    cout << "\n";
    cout << "Le tomo" << durationMili.count() << " milisegundos\n";
    cout << "\n";

    auto start = high_resolution_clock::now();

    cout << "DFS" << endl;
    DFS(G);

    auto stop = high_resolution_clock::();
    auto durationMili = duration_cast<miliseconds>(stop-start);
    cout << "\n";
    cout << "Le tomo" << durationMili.count() << " milisegundos\n";
    cout << "\n";

    auto start = high_resolution_clock::now();

    cout << "BFS" << endl;
    BFS(G);

    auto stop = high_resolution_clock::();
    auto durationMili = duration_cast<miliseconds>(stop-start);
    cout << "\n";
    cout << "Le tomo" << durationMili.count() << " milisegundos\n";
    cout << "\n";

    auto start = high_resolution_clock::now();

    cout << "Kruskal" << endl;
    Kruskal(G);

    auto stop = high_resolution_clock::();
    auto durationMili = duration_cast<miliseconds>(stop-start);
    cout << "\n";
    cout << "Le tomo" << durationMili.count() << " milisegundos\n";
    cout << "\n";

    auto start = high_resolution_clock::now();

    cout << "Dijkstra" << endl;
    Dijkstra(G);

    auto stop = high_resolution_clock::();
    auto durationMili = duration_cast<miliseconds>(stop-start);
    cout << "\n";
    cout << "Le tomo" << durationMili.count() << " milisegundos\n";
    cout << "\n";

    auto start = high_resolution_clock::now();

    cout << "Floyd Warshall" << endl;
    FloydWarshall(G);

    auto stop = high_resolution_clock::();
    auto durationMili = duration_cast<miliseconds>(stop-start);
    cout << "\n";
    cout << "Le tomo" << durationMili.count() << " milisegundos\n";
    cout << "\n";  
    
}
