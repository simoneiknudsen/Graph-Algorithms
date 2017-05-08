#include "graph.hpp"

using namespace graph;
using namespace tags;
using namespace std;

int main(){
	auto g = diGraph<Directed>();

	//Testing Bellman Ford algorithm on directed graph
	auto v1 = addVertex(g,1);
	auto v2 = addVertex(g,2);
	auto v3 = addVertex(g,3);
	auto v4 = addVertex(g,4);
	auto v5 = addVertex(g,5);

	addEdge(g,v1,v2,6);
	addEdge(g,v1,v5,7);
	addEdge(g,v2,v3,5);
	addEdge(g,v2,v4,-4);
	addEdge(g,v2,v5,8);
	addEdge(g,v3,v2,-2);
	addEdge(g,v4,v3,7);
	addEdge(g,v4,v1,2);
	addEdge(g,v5,v4,9);
	addEdge(g,v5,v3,-3);

	bellmanFord(g,v1);
	
	//Testing of Prim's Algorithm on undirected graph
	auto g1 = diGraph<Undirected>();
	auto v1 = addVertex(g1,1);
	auto v2 = addVertex(g1,2);
	auto v3 = addVertex(g1,3);
	auto v4 = addVertex(g1,4);
	auto v5 = addVertex(g1,5);
	auto v6 = addVertex(g1,6);
	auto v7 = addVertex(g1,7);
	auto v8 = addVertex(g1,8);
	auto v9 = addVertex(g1,9);
	
	addEdge(g1,v1,v2,4);
	addEdge(g1,v1,v8,8);
	addEdge(g1,v2,v3,8);
	addEdge(g1,v2,v8,11);
	addEdge(g1,v3,v4,7);
	addEdge(g1,v3,v9,2);
	addEdge(g1,v3,v6,4);
	addEdge(g1,v4,v5,9);
	addEdge(g1,v4,v6,14);
	addEdge(g1,v5,v6,10);
	addEdge(g1,v6,v7,2);
	addEdge(g1,v7,v8,1);
	addEdge(g1,v7,v9,6);
	addEdge(g1,v8,v9,7);

	prim(g1,v1);
	kruskal(g1);
  
  return 0;
}
