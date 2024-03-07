#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "Graph.h"
#include <stack>

int main() {
	setlocale(LC_ALL, "Russian");
	Graph<1, 1> graph;
	std::ifstream fin("DWGraph.txt");
	GraphIO::Input(graph, fin);
	auto temp = graph.Johnson();
}