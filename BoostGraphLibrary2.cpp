
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
typedef adjacency_list<vecS, vecS, undirectedS,property<vertex_distance_t, int>, EdgeWeightProperty > p;
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
//bfs_<null_visitor> vis;
 
//adjacency_list<> g;
//std::pair<adjacency_list<>::edge_descriptor, bool> p;

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



//Insertar vertice con numero como nombre
//Complejidad: 
void insertarVertice(){
    add_vertex(g);
}//End of insertarVertice

void insertarEdge(unsigned long fuente, unsigned long destino, int peso){
    auto e = add_edge(fuente,destino,peso, g).first;
    weightmap[e]=  peso;
}

void eliminarVertice(unsigned long vertice){
    clear_vertex(vertice, g);
    remove_vertex(vertice, g);
}

void eliminarEdge(unsigned long fuente, unsigned long destino){
    remove_edge(fuente, destino, g);
}

//Prim
//Tecnica de diseño: avido
//Complejidad: en la matriz de adyacencia O(|V|^2) ya que utiliza un indexMap
void prim(p g){
    std:: vector<Vertex> a(num_vertices(g));
    
    property_map<p, vertex_distance_t>::type distance = get(vertex_distance, g); 
    prim_minimum_spanning_tree(g, *vertices(g).first, &a[0], distance, weightmap, indexmap, default_dijkstra_visitor());
    // for(std:: size_t i = 1; i!= a.size(); i++){
    //     cout<<"parent["<<i<<"] = "<<a[i]<<endl;
    // }//End of for
}

//Kruskal
//Tecnica de diseño: avido
//Complejidad: O(|E| log|E|)

void kruskal(p g){
    std:: vector<Edge> a;
    kruskal_minimum_spanning_tree(g, std::back_inserter(a));

    for(std::vector <Edge>::iterator it = a.begin(); it!=a.end(); ++it){
        cout<<source(*it, g)<<" <--> " <<target(*it, g)<<" con peso " << weightmap[*it]<<endl;
    }   //End of for

}


int main (){
    //Aqui no se muestran todos los metodos ya que unicamente se mostrara prim y kruskal que necesitan un grafo no dirigido
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
    
    
    //Prim
    auto start = high_resolution_clock::now();
    prim(g);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop-start);
    cout<<"Prim : "<<duration.count()<<" microsegundos"<<endl;
    cout<<""<<endl;

   //Kruskal
   start = high_resolution_clock::now();
   kruskal(g);
    stop = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stop-start);
    cout<<"Kruskal : "<<duration.count()<<" microsegundos"<<endl;
    cout<<""<<endl;
    return 0;
}

//https://www.youtube.com/watch?v=uYvBH7TZlFk
//Los códigos fueron obtenidos de la libreria de boost, que contiene distintos ejemplos
//Para floyd warshall https://stackoverflow.com/questions/26855184/floyd-warshall-all-pairs-shortest-paths-on-weighted-undirected-graph-boost-g
