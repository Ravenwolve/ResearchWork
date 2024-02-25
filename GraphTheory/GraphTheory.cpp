#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "Graph.h"
#include <stack>

int main() {
	setlocale(LC_ALL, "Russian");
	std::cout << "Задача о максимальном потоке. Решение алгоритмом Форда-Фалкерсона" << std::endl;
	Graph<1, 1> forFordFulkerson;
	forFordFulkerson.AddVertex("1"), forFordFulkerson.AddVertex("2");
	forFordFulkerson.AddVertex("3"), forFordFulkerson.AddVertex("4");
	forFordFulkerson.AddVertex("5");
	forFordFulkerson.AddArc("1", "4", 10), forFordFulkerson.AddArc("1", "2", 20);
	forFordFulkerson.AddArc("1", "3", 30), forFordFulkerson.AddArc("2", "3", 40);
	forFordFulkerson.AddArc("2", "5", 30), forFordFulkerson.AddArc("3", "4", 10);
	forFordFulkerson.AddArc("3", "5", 20), forFordFulkerson.AddArc("4", "5", 20);
	GraphIO::Output(forFordFulkerson, std::cout);
	std::string s, t;
	std::cout << "Введите вершины истока и стока: ";
	std::cin >> s >> t;
	try {
		std::cout << "Максимальный поток: ";
		//std::cout << forFordFulkerson.MaximalFlow("1", "5") << std::endl;
	}
	catch (const std::string& ex) {
		std::cout << ex << std::endl;
	}
}