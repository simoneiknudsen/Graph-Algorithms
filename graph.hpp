#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <cassert>
#include <list>
#include <vector>
#include <algorithm>
#include <map>
#include <climits>
#include <queue>

namespace graph {
struct diGraph {
public:
	struct Vertex {
		char name;

		Vertex(char n) : name(n) {}

		int d;
		char p;
		int dis;
	};

	struct Edge {
		Vertex src;
		Vertex des;
		int c;

		Edge(Vertex source, Vertex destination, int cost): src(source), des(destination), c(cost) {}
	};

	using Vertices = std::vector<Vertex>;
	using Edges = std::vector<Edge>;
	using Neighbors = std::map<char,std::vector<Edge>>;
	using Queue = std::priority_queue<char,std::vector<char>, std::greater<char>>;

private:
	Vertices vertices;
	Neighbors neighbors;
	Queue minQ;	
	Edges edges;

public:
	friend Vertex addVertex(diGraph &g, char c){
		auto v = Vertex(c);
		g.vertices.push_back(v);
		return v;
	}

	friend Edge addEdge(diGraph &g, char src, char des, int cost){
		auto e = Edge(src,des,cost);
		g.neighbors[src].push_back(e);
		g.edges.push_back(e);
		return e;
	}

	friend int getNeighbors(diGraph &g, char src){
		return g.neighbors[src].size();
	}

	friend Vertex getSource(Edge e){
		return e.src;
	}

	friend Vertex getDestination(Edge e){
		return e.des;
	}

	friend int getCost(Edge e){
		return e.c;
	}
}; //end of diGraph

} // end of namespace graph

#endif // GRAPH_HPP    
    
    
