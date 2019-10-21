//Problema practico 2
//Boost graph library

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/detail/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/graph/exterior_property.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/concept/assert.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/floyd_warshall_shortest.hpp>
#include <iostream>
#include <stdlib.h>
#include <bits/stdc++.h>
#include <string>
#include <vector>
#include <iterator>
#include <chrono>


using namespace std;
using namespace boost;
using namespace std::chrono;


typedef property<edge_weight_t, int> EdgeWeightProperty;
typedef adjacency_list<vecS, vecS, bidirectionalS,property<vertex_distance_t, int>, EdgeWeightProperty > p;
typedef graph_traits<p>::vertex_descriptor Vertex;
typedef graph_traits<p>::edge_descriptor Edge;
typedef property<vertex_distance_t, int> Distancia;
typedef property<vertex_predecessor_t, int> Predecessor;
typedef boost::exterior_vertex_property<p, int> DistanceProperty;
typedef DistanceProperty::matrix_type DistanceMatrix;
typedef DistanceProperty::matrix_map_type DistanceMatrixMap;

p g;
property_map<p, edge_weight_t>::type weightmap = get(edge_weight, g);
property_map<p, vertex_index_t>::type indexmap = get(vertex_index, g);   





#define numVertices 14;
 

class VisitorBFS : public boost::default_bfs_visitor{
    public:
    void discover_vertex(Vertex a, const p& g) const{
        std::cerr <<a<< " ";
        return;
    }
};

class VisitorDFS : public boost::default_dfs_visitor{
    public:
    void discover_vertex(Vertex a, const p& g) const{
        std::cerr <<a<< " ";
        return;
    }
};


//O(1)
void insertarVertice(){
    add_vertex(g);
}//End of insertarVertice

//(1)
void insertarEdge(unsigned long fuente, unsigned long destino, int peso){
    auto e = add_edge(fuente,destino,peso, g).first;
    weightmap[e]=  peso;
}

//O(n)
void eliminarVertice(unsigned long vertice){
    clear_vertex(vertice, g);
    remove_vertex(vertice, g);
}

//O(n)
void eliminarEdge(unsigned long fuente, unsigned long destino){
    remove_edge(fuente, destino, g);
}

//BFS
//técnica de diseño: Ávido
//Complejidad O(|V|+|E|)
void BFS(unsigned long comienzoBusqueda){
    VisitorBFS visitorBFS;
    breadth_first_search(g, comienzoBusqueda, visitor(visitorBFS));
}

//DFS
//Tecnica de diseño: avido (con una pila) y backtracking (recursivo)
//Complejidad O(|V|+|E|)
void DFS(){
    VisitorDFS visitorDFS;
    depth_first_search(g, visitor(visitorDFS));
}

void imprimirEdges(){
    auto it = edges(g);
    for(auto i = it.first; i != it.second; i++)
        cout << "Nodo: "<< " fuente= " << source(*i, g) << " destino= " << target(*i, g) << " peso= " << weightmap[*i] << "\n";
}


//Djkstra
//Tecnica de diseño: avido
//Complejidad: O(|V|^2) ya que utiliza una cola de prioridad
void dijkstra(){
    std:: vector<Vertex> v (num_vertices(g));
    std:: vector<int> d(num_vertices(g));

    Vertex descriptorDijkstra = vertex(1, g);

    
    std:: vector<Edge> b;
   
    dijkstra_shortest_paths(g, descriptorDijkstra, &v[0], &d[0], weightmap, indexmap, std::less<int>(), closed_plus<int>(),(std::numeric_limits<int>::max)(), 0,default_dijkstra_visitor());
    
}//End of djkstra

//Floyd warshall
//Tecnica de diseño: programacion dinamica
//Complejidad: O(|V|^3)
void floydWarshall(){
    // set the distance matrix to receive the floyd warshall output
  //Distance matrix
DistanceMatrix distances(num_vertices(g));
DistanceMatrixMap dm(distances, g);

  // find all pairs shortest paths
  bool valid = floyd_warshall_all_pairs_shortest_paths(g, dm, boost::weight_map(weightmap));

  // check if there no negative cycles
//   if (!valid) {
//     std::cerr << "Error - Negative cycle in matrix" << std::endl;
//     return -1;
//   }
    cout<<"Floyd warshall"<<endl;
  //print upper triangular part of the distance matrix
  std::cout << "Distance matrix: " << std::endl;
  for (std::size_t i = 1; i < num_vertices(g); ++i) {
    for (std::size_t j = 1; j < num_vertices(g); ++j) {
      std::cout << "From vertex " << i+1 << " to " << j+1 << " : ";
      if(distances[i][j] == std::numeric_limits<edge_weight_t>::max()){
        std::cout << "inf" << std::endl; }
      else{
        std::cout << distances[i][j] << std::endl;}
    }
    std::cout << std::endl;
  }
}//End of floyd warshal


int main (){

    //Insertar vertice
    auto start = high_resolution_clock::now();
    insertarVertice();
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop-start);
    cout<<"Insertar vertice: "<<duration.count()<<" microsegundos"<<endl;
    
    //Insertar edge
    start = high_resolution_clock::now();
    insertarEdge(2, 5, 3);
    stop = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stop-start);
    cout<<"Insertar vertice: "<<duration.count()<<" microsegundos"<<endl;

    //Eliminar vertice
    start = high_resolution_clock::now();
    eliminarVertice(1);
    stop = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stop-start);
    cout<<"Eliminar vertice: "<<duration.count()<<" microsegundos"<<endl;

    //Eliminar edge
    start = high_resolution_clock::now();
    eliminarEdge(2, 5);
    stop = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stop-start);
    cout<<"Eliminar edge: "<<duration.count()<<" microsegundos"<<endl;
    
    //Haciendo el grafo especificado
    insertarEdge(1,4,8);
    insertarEdge(1,3,8);
    insertarEdge(2,5,7);
    insertarEdge(3,2,7);
    insertarEdge(3,5,8);
    insertarEdge(3,10,4);
    insertarEdge(4,7,3);
    insertarEdge(4,8,2);
    insertarEdge(4,5,1);
    insertarEdge(5,6,9);
    insertarEdge(6,13,4);
    insertarEdge(7,4,6);
    insertarEdge(8,7,3);
    insertarEdge(8,9,3);
    insertarEdge(9,10,2);
    insertarEdge(9,12,4);
    insertarEdge(10,6,6);
    insertarEdge(10,3,10);
    insertarEdge(11,12,6);
    insertarEdge(12,11,8);
    insertarEdge(12,9,2);
    insertarEdge(12,14,9);
    insertarEdge(13,14,6);
    insertarEdge(14,13,2);


   imprimirEdges();
    
    //BFS
    start = high_resolution_clock::now();
    BFS(1);
    stop = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stop-start);
    cout<<"BFS : "<<duration.count()<<" microsegundos"<<endl;
    cout<<""<<endl;

    //DFS
    start = high_resolution_clock::now();
    DFS();
    stop = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stop-start);
    cout<<"DFS : "<<duration.count()<<" microsegundos"<<endl;
    cout<<""<<endl;

   //Djkstra
   start = high_resolution_clock::now();
    dijkstra();
    stop = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stop-start);
    cout<<"Djkstra : "<<duration.count()<<" microsegundos"<<endl;
    cout<<""<<endl;

    //Floyd warshall
    start = high_resolution_clock::now();
    floydWarshall();
    stop = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stop-start);
    cout<<"Floyd warshall : "<<duration.count()<<" microsegundos"<<endl;
    cout<<""<<endl;

    return 0;
}

//https://www.youtube.com/watch?v=uYvBH7TZlFk
//Los códigos fueron obtenidos de la libreria de boost, que contiene distintos ejemplos
//Para floyd warshall https://stackoverflow.com/questions/26855184/floyd-warshall-all-pairs-shortest-paths-on-weighted-undirected-graph-boost-g