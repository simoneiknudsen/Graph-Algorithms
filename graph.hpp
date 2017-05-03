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
}; //end of diGraph

} // end of namespace graph

#endif // GRAPH_HPP    
    
    
