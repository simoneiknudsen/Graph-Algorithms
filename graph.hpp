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

	struct cmp {
    	bool operator()( const Edge& a, const Edge& b ) { 
    		return a.w < b.w; 
    	}
	};

	using DirectedTag = Tag;
	using Vertices = std::vector<Vertex>;
	using InboundNeighbors = std::map<int,std::vector<Vertex>>;
	using Edges = std::vector<Edge>;
	using Neighbors = std::map<int,std::vector<Vertex>>;

private:
	Vertices vertices;
	Neighbors neighbors;
	InboundNeighbors inboundNeighbors;
	Edges edges;

public:
	friend bool operator!=(const Vertex &a, const Vertex &b){
		return a.id != b.id;
	}

	friend bool operator==(const Vertex &a, const Vertex &b){
		return a.id == b.id;
	}

	friend Vertex addVertex(diGraph &g, int c){
		auto v = Vertex(c);
		g.vertices.push_back(v);
		return v;
	}

	friend Edge addEdge(diGraph &g, Vertex src, Vertex des, int w){
		auto e = Edge(src,des,w);
		g.neighbors[src.id].push_back(des.id);
		if(std::is_same<DirectedTag,tags::Undirected>::value){
			g.neighbors[des.id].push_back(src.id);
		} else {
			g.inboundNeighbors[des.id].push_back(src.id);
		}
		g.edges.push_back(e);
		return e;
	}

	// addEdge for graphs without edge weights
	friend Edge addEdge(diGraph &g, Vertex src, Vertex des){
		auto e = Edge(src,des,0);
		g.neighbors[src.id].push_back(des.id);
		if(std::is_same<DirectedTag,tags::Undirected>::value){
			g.neighbors[des.id].push_back(src.id);
		} else {
			g.inboundNeighbors[des.id].push_back(src.id);
		}
		g.edges.push_back(e);
		return e;
	}

	friend int outDegree(diGraph &g, Vertex &src){
		return g.neighbors[src.id].size();
	}

	friend int inDegree(diGraph &g, Vertex &src){
		return g.inboundNeighbors[src.id].size();
	}

	friend std::vector<Vertex> outboundNeighbors(diGraph &g, Vertex &src){
		return g.neighbors[src.id];
	}

	friend std::vector<Vertex> inboundNeighbors(diGraph &g, Vertex &src){
		return g.inboundNeighbors[src.id];
	}

	friend int getNumberVertices(diGraph &g){
		return g.vertices.size();
	}

	friend int getNumberEdges(diGraph &g){
		return g.edges.size();
	}

	friend Vertex getSource(Edge e){
		return e.src.id;
	}

	friend Vertex getDestination(Edge e){
		return e.des.id;
	}

	friend int getWeight(Edge e){
		return e.w;
	}

	friend int getWeight(diGraph &g, int u, int v){
		int counter = 0;
		if(std::is_same<DirectedTag,tags::Directed>::value){
			for(auto e: g.edges){
				if((e.src.id == u && e.des.id == v)){
					break;
				} else {
					counter++;
				}
			}
		} else {
			for(auto e: g.edges){
				if((e.src.id == u && e.des.id == v) || (e.des.id == u && e.src.id == v)){
					break;
				} else {
					counter++;
				}
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

	friend void prim(diGraph &g, Vertex &src){
		std::vector<int> distance(g.vertices.size()+1);
		std::vector<bool> visited(g.vertices.size()+1);
		std::vector<int> tree(g.vertices.size()+1);
		std::vector<std::pair<int,int>> MST;


		for(int i = 0; i <= g.vertices.size(); i++){
			distance[i] = INT_MAX;
			visited[i] = false;
		}
		distance[src.id] = 0;

		for(int i = 0; i < distance.size(); i++){
			auto u = minVertex(g,distance,visited);

			visited[u] = true;

			for(int j = 0; j < g.neighbors[u].size(); j++){
				auto v = g.neighbors[u][j].id;
				auto d = getWeight(g,u,v);
				if(distance[v] > d && visited[v] == false){
					distance[v] = d;
					tree[v] = u;
				}
			}
		}
		
		for(int i = 2; i <= g.vertices.size(); i++){
			MST.push_back(std::make_pair(tree[i],i));
			printf("%d - %d : %d\n", tree[i], i, getWeight(g,i,tree[i]));
		}
	}

	friend void makeSet(diGraph &g, Vertex &x, std::vector<Vertex> &parent, std::vector<int> &rank){
		parent[x.id] = x;
		rank[x.id] = 0;
	}

	friend Vertex findSet(diGraph &g, Vertex &x, std::vector<Vertex> &parent){
		if(x.id != parent[x.id].id){
			parent[x.id] = findSet(g,parent[x.id],parent);
		}
		return parent[x.id];
	}

	friend void setUnion(diGraph &g, Vertex &x, Vertex &y, std::vector<Vertex> &parent, std::vector<int> &rank){
		auto findX = findSet(g,x,parent);
		auto findY = findSet(g,y,parent);

		if(rank[findX.id] > rank[findY.id]){
			parent[findY.id] = findX;
		} else {
			parent[findX.id] = findY;
			if(rank[findX.id] == rank[findY.id]){
				rank[findY.id]++;
			}
		}
	}

	friend void kruskal(diGraph &g){
		std::vector<Edge> A;
		std::vector<int> rank(g.vertices.size()+1);
		std::vector<Vertex> p;
		p.reserve(g.vertices.size()+1);
		std::vector<Edge> edges;

		for(auto v : g.vertices){
			makeSet(g,v,p,rank);
		}
		for(auto edge : g.edges){
			edges.push_back(edge);
		}
		std::sort(edges.begin(),edges.end(),cmp());

		for(auto e : edges){
			if(findSet(g,e.src,p).id != findSet(g,e.des,p).id){
				A.push_back(e);
				setUnion(g,e.src,e.des,p,rank);
			}
		}

		for(int i = 0; i < A.size(); i++){
			printf("%d - %d : %d\n", A[i].src.id, A[i].des.id, A[i].w);
		}
	}

	friend int minVertex(diGraph &g, std::vector<int> d, std::vector<bool> p){
		int index;
		int min = INT_MAX;
		for(int i = 1; i <= g.vertices.size(); i++){
			if(p[i] == false && d[i] <= min){
				min = d[i];
				index = i;
			}
		}
		return index;
	}

	friend void dijkstra(diGraph &g, Vertex &s){
		std::vector<int> distance(g.vertices.size()+1);
		std::vector<int> pi(g.vertices.size()+1);
		std::vector<bool> visited(g.vertices.size()+1);

		for(int i = 0; i <= g.vertices.size(); i++){
			distance[i] = INT_MAX;
			visited[i] = false;
		}
		distance[s.id] = 0;

		for(int i = 0; i < g.vertices.size(); i++){
			auto u = minVertex(g,distance,visited);

			visited[u] = true;

			for(int j = 0; j < g.neighbors[u].size(); j++){
				auto v = g.neighbors[u][j].id;
				auto d = distance[u]+getWeight(g,u,v);
				if(distance[v] > d && visited[v] == false){
					distance[v] = d;
					pi[v] = u;
				}
			}
		}

		for(int i = 1; i < distance.size(); i++){
			printf("%d - %d : %d\n", s.id, i, distance[i]);
		}
	}
	
	friend void bfs(diGraph &g, Vertex &src){
		std::vector<std::string> color(g.vertices.size()+1);
		std::vector<int> dist(g.vertices.size()+1);
		std::vector<Vertex> p;
		p.reserve(g.vertices.size()+1);
		std::vector<Vertex> Q;

		for(int i = 0; i <= g.vertices.size(); i++){
			color[i] = "white";
			dist[i] = INT_MAX;
		}
		color[src.id] = "gray";
		dist[src.id] = 0;
		Q.push_back(src);
		while(!Q.empty()){
			auto u = Q[0];
			Q.erase(Q.begin());
			for(int j = 0; j < g.neighbors[u.id].size(); j++){
				auto v = g.neighbors[u.id][j];
				if(color[v.id] == "white"){
					color[v.id] = "gray";
					dist[v.id] = dist[u.id]+1;
					p[v.id] = u;
					Q.push_back(v);
				}
			}
			color[u.id] = "black";
		}
		for(int i = 1; i < dist.size(); i++){
			printf("Vertex %d: %d\n", i, dist[i]);
		}
	}

}; //end of diGraph

} // end of namespace graph

#endif // GRAPH_HPP
