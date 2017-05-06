#ifndef GRAPH_HPP
#define GRAPH_HPP

#include "tags.hpp"

#include <cassert>
#include <list>
#include <vector>
#include <algorithm>
#include <map>
#include <climits>
#include <queue>

namespace graph {
template<typename Tag>
struct diGraph {
public:
	struct Vertex {
		int id;

		Vertex(int n) : id(n) {}
	};
	
	struct Edge {
		Vertex src;
		Vertex des;
		int w;

		Edge(Vertex source, Vertex destination, int weight): src(source), des(destination), w(weight) {}
	};
	using DirectedTag = Tag;
	using Vertices = std::vector<Vertex>;
	using Edges = std::vector<Edge>;
	using Neighbors = std::map<int,std::vector<Edge>>;
private:
	Vertices vertices;
	Neighbors neighbors;	
	Edges edges;

public:
	friend Vertex addVertex(diGraph &g, int c){
		auto v = Vertex(c);
		g.vertices.push_back(v);
		return v;
	}

	friend Edge addEdge(diGraph &g, Vertex src, Vertex des, int w){
		auto e = Edge(src,des,w);
		g.neighbors[src.id].push_back(e);
		if(std::is_same<DirectedTag,tags::Undirected>::value){
			g.neighbors[des.id].push_back(e);
		}
		g.edges.push_back(e);
		return e;
	}

	friend int getNeighbors(diGraph &g, Vertex src){
		return g.neighbors[src.id].size();
	}

	friend Vertex getSource(Edge e){
		return e.src;
	}

	friend Vertex getDestination(Edge e){
		return e.des;
	}

	friend int getWeight(Edge e){
		return e.w;
	}
	
	friend int getWeight(diGraph &g, Vertex u, Vertex v){
		int counter = 0;
		for(auto e: g.edges){
			if(e.src.id == u.id && e.des.id == v.id){
				break;
			} else {
				counter++;
			}
		}
		return g.edges[counter].w;
	}
	
	friend void bellmanFord(diGraph &g, Vertex &src){
		std::vector<int> distance(g.vertices.size()+1);

		for(int i = 0; i <= g.vertices.size(); i++){
			distance[i] = INT_MAX;
		}
		distance[src.id] = 0;

		for(int i = 0; i < g.vertices.size()-1; i++){
			for(int j = 0; j < g.edges.size(); j++){
				int u = g.edges[j].src.id;
				int v = g.edges[j].des.id;
				int w = g.edges[j].w;
				if(distance[u] != INT_MAX && (distance[u] + w) < distance[v]){
					distance[v] = distance[u]+w;
				}
			}
		}
		for(int i = 0; i < g.edges.size(); i++){
			int u = g.edges[i].src.id;
			int v = g.edges[i].des.id;
			int w = g.edges[i].w;
			if(distance[u] != INT_MAX && (distance[u] + w) < distance[v]){
				printf("Error: Graph contains negative weight cycle.");
			}
		}

		for(int i = 1; i < distance.size(); i++){
			printf("From %d -> %d : %d\n", src.id, i, distance[i]);
		}
	}
}; //end of diGraph

} // end of namespace graph

#endif // GRAPH_HPP    
    
    
