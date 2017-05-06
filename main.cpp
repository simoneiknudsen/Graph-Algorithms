#include "graph.hpp"
using namespace graph;
using namespace tags;
using namespace std;

int main(){
	auto g = diGraph<Directed>();

    //Testing Bellman Ford algorithm
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
  
  return 0;
}
