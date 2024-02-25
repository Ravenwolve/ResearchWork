#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "Graph.h"
#include <stack>

//int main() {
//	setlocale(LC_ALL, "Russian");
//	Graph<1, 1> graphForDijkstra;
//	graphForDijkstra.AddVertex("1"), graphForDijkstra.AddVertex("2");
//	graphForDijkstra.AddVertex("3"), graphForDijkstra.AddVertex("4");
//	graphForDijkstra.AddVertex("5"), graphForDijkstra.AddArc("1", "2", 10);
//	graphForDijkstra.AddArc("1", "4", 30), graphForDijkstra.AddArc("1", "5", 100);
//	graphForDijkstra.AddArc("2", "3", 50), graphForDijkstra.AddArc("3", "5", 10);
//	graphForDijkstra.AddArc("4", "3", 20), graphForDijkstra.AddArc("4", "5", 60);
//	GraphIO::Output(graphForDijkstra, std::cout);
//	std::cout << "Кратчайший путь из u1 и u2 до v (Dijkstra): ";
//	std::string u1, u2, v;
//	std::cin >> u1 >> u2 >> v;
//	try {
//		auto path1 = graphForDijkstra.ShortestPath(u1, v);
//		for (auto iter = path1.begin(); iter != path1.end(); ++iter)
//			std::cout << *iter << ' ';
//		std::cout << std::endl;
//	}
//	catch (const std::string& ex) {
//		std::cout << ex << std::endl;
//	}
//	try {
//		auto path1 = graphForDijkstra.ShortestPath(u2, v);
//		for (auto iter = path1.begin(); iter != path1.end(); ++iter)
//			std::cout << *iter << ' ';
//		std::cout << std::endl;
//	}
//	catch (const std::string& ex) {
//		std::cout << ex << std::endl;
//	}
//}

//int main() {
//	setlocale(LC_ALL, "Russian");
//	Graph<1, 1> graphForFloyd;
//	graphForFloyd.AddVertex("1"), graphForFloyd.AddVertex("2"), graphForFloyd.AddVertex("3");
//	graphForFloyd.AddVertex("4"), graphForFloyd.AddVertex("5"), graphForFloyd.AddVertex("6");
//	graphForFloyd.AddArc("1", "2", 2), graphForFloyd.AddArc("1", "4", 5);
//	graphForFloyd.AddArc("2", "3", 4), graphForFloyd.AddArc("3", "4", 1);
//	graphForFloyd.AddArc("3", "6", 7), graphForFloyd.AddArc("4", "1", -3);
//	graphForFloyd.AddArc("4", "2", 4), graphForFloyd.AddArc("4", "3", 6);
//	graphForFloyd.AddArc("4", "5", 5), graphForFloyd.AddArc("5", "3", 8);
//	graphForFloyd.AddArc("5", "6", -1);
//	GraphIO::Output(graphForFloyd, std::cout);
//	std::cout << "Длины всех кратчайших путей от u до остальных вершин (Floyd): ";
//	std::string u1;
//	std::cin >> u1;
//	auto V = graphForFloyd.GetVertices();
//	for (auto v = V.begin(); v != V.end(); ++v)
//		if (*v != u1) {
//			try {
//				double dist = graphForFloyd.Distance(u1, *v, ShortestPathAlgorithm::Floyd);
//				std::cout << *v << ' ' << dist << std::endl;
//			}
//			catch (const std::string& ex) {
//				std::cout << ex << std::endl;
//			}
//		}
//}

//int main() {
//	setlocale(LC_ALL, "Russian");
//	// Ford-Bellman
//	std::cout << "Вывести кратчайший путь из вершины u до вершины v (Ford-Bellman)" << std::endl;
//	std::string u1, v;
//	std::cin >> u1 >> v;
//	Graph<1, 1> graphForFordBellman;
//	graphForFordBellman.AddVertex("1"), graphForFordBellman.AddVertex("2");
//	graphForFordBellman.AddVertex("3"), graphForFordBellman.AddVertex("4");
//	graphForFordBellman.AddVertex("5"), graphForFordBellman.AddArc("1", "2", 10);
//	graphForFordBellman.AddArc("1", "4", 30), graphForFordBellman.AddArc("1", "5", 100);
//	graphForFordBellman.AddArc("2", "3", 50), graphForFordBellman.AddArc("3", "5", 10);
//	graphForFordBellman.AddArc("4", "3", 20), graphForFordBellman.AddArc("4", "5", 60);
//	try {
//		auto path = graphForFordBellman.ShortestPath(u1, v, ShortestPathAlgorithm::Bellman_Ford);
//		for (auto v = path.begin(); v != path.end(); ++v)
//			std::cout << *v << ' ';
//		std::cout << std::endl;
//	}
//	catch (const std::string& ex) {
//		std::cout << ex << std::endl;
//	}
//	// Ford-Bellman with negative cycle
//	Graph<1, 1> graphWithNegativeCycle;
//	graphWithNegativeCycle.AddVertex("0");
//	graphWithNegativeCycle.AddVertex("1");
//	graphWithNegativeCycle.AddVertex("2");
//	graphWithNegativeCycle.AddVertex("3");
//	graphWithNegativeCycle.AddVertex("4");
//	graphWithNegativeCycle.AddArc("0", "1", 6);
//	graphWithNegativeCycle.AddArc("0", "4", 7);
//	graphWithNegativeCycle.AddArc("1", "2", 5);
//	graphWithNegativeCycle.AddArc("1", "3", -4);
//	graphWithNegativeCycle.AddArc("1", "4", -8);
//	graphWithNegativeCycle.AddArc("2", "1", -2);
//	graphWithNegativeCycle.AddArc("3", "2", 7);
//	graphWithNegativeCycle.AddArc("3", "0", 2);
//	graphWithNegativeCycle.AddArc("4", "2", -3);
//	graphWithNegativeCycle.AddArc("4", "3", 9);
//	std::cout << "Вывести кратчайший путь из вершины u до вершины v (Ford-Bellman)" << std::endl;
//	GraphIO::Output(graphWithNegativeCycle, std::cout);
//	std::cin >> u1 >> v;
//	try {
//		auto path = graphWithNegativeCycle.ShortestPath(u1, v, ShortestPathAlgorithm::Bellman_Ford);
//		for (auto v = path.begin(); v != path.end(); ++v)
//			std::cout << *v << ' ';
//		std::cout << std::endl;
//	}
//	catch (const std::string& ex) {
//		std::cout << ex << std::endl;
//	}
//	catch (std::pair<std::vector<std::string>, std::string> ex) {
//		std::cout << ex.second << std::endl;
//		for (auto v = ex.first.begin(); v != ex.first.end(); ++v)
//			std::cout << *v << ' ';
//		std::cout << std::endl;
//	}
//}

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
		std::cout << forFordFulkerson.MaximalFlow("1", "5") << std::endl;
	}
	catch (const std::string& ex) {
		std::cout << ex << std::endl;
	}
}

//int main() {
//	setlocale(LC_ALL, "Russian");
//	Graph<1, 1> graphForDijkstra;
	//graphForDijkstra.AddVertex("1"), graphForDijkstra.AddVertex("2");
	//graphForDijkstra.AddVertex("3"), graphForDijkstra.AddVertex("4");
	//graphForDijkstra.AddVertex("5"), graphForDijkstra.AddArc("1", "2", 10);
	//graphForDijkstra.AddArc("1", "4", 30), graphForDijkstra.AddArc("1", "5", 100);
	//graphForDijkstra.AddArc("2", "3", 50), graphForDijkstra.AddArc("3", "5", 10);
	//graphForDijkstra.AddArc("4", "3", 20), graphForDijkstra.AddArc("4", "5", 60);
//	GraphIO::Output(graphForDijkstra, std::cout);
//	std::cout << "Кратчайший путь из u1 и u2 до v (Dijkstra): ";
//	std::string u1, u2, v;
//	std::cin >> u1 >> u2 >> v;
//	// Dijkstra
//	try {
//		auto path1 = graphForDijkstra.ShortestPath(u1, v);
//		for (auto iter = path1.begin(); iter != path1.end(); ++iter)
//			std::cout << *iter << ' ';
//		std::cout << std::endl;
//	}
//	catch (const std::string& ex) {
//		std::cout << ex << std::endl;
//	}
//	try {
//		auto path1 = graphForDijkstra.ShortestPath(u2, v);
//		for (auto iter = path1.begin(); iter != path1.end(); ++iter)
//			std::cout << *iter << ' ';
//		std::cout << std::endl;
//	}
//	catch (const std::string& ex) {
//		std::cout << ex << std::endl;
//	}
//	// Floyd
//	Graph<1, 1> graphForFloyd;
//	graphForFloyd.AddVertex("1"), graphForFloyd.AddVertex("2"), graphForFloyd.AddVertex("3");
//	graphForFloyd.AddVertex("4"), graphForFloyd.AddVertex("5"), graphForFloyd.AddVertex("6");
//	graphForFloyd.AddArc("1", "2", 2), graphForFloyd.AddArc("1", "4", 5);
//	graphForFloyd.AddArc("2", "3", 4), graphForFloyd.AddArc("3", "4", 1);
//	graphForFloyd.AddArc("3", "6", 7), graphForFloyd.AddArc("4", "1", -3);
//	graphForFloyd.AddArc("4", "2", 4), graphForFloyd.AddArc("4", "3", 6);
//	graphForFloyd.AddArc("4", "5", 5), graphForFloyd.AddArc("5", "3", 8);
//	graphForFloyd.AddArc("5", "6", -1);
//	GraphIO::Output(graphForFloyd, std::cout);
//	std::cout << "Длины всех кратчайших путей от u до остальных вершин (Floyd): ";
//	std::cin >> u1;
//	auto V = graphForFloyd.GetVertices();
//	for (auto v = V.begin(); v != V.end(); ++v)
//		if (*v != u1) {
//			try {
//				double dist = graphForFloyd.Distance(u1, *v, ShortestPathAlgorithm::Floyd);
//				std::cout << *v << ' ' << dist << std::endl;
//			}
//			catch (const std::string& ex) {
//				std::cout << ex << std::endl;
//			}
//		}
//	// Ford-Bellman
//	std::cout << "Вывести кратчайший путь из вершины u до вершины v (Ford-Bellman)" << std::endl;
//	std::cin >> u1 >> v;
//	Graph<1, 1> graphForFordBellman = graphForDijkstra;
//	try {
//		auto path = graphForFordBellman.ShortestPath(u1, v, ShortestPathAlgorithm::Bellman_Ford);
//		for (auto v = path.begin(); v != path.end(); ++v)
//			std::cout << *v << ' ';
//		std::cout << std::endl;
//	}
//	catch (const std::string& ex) {
//		std::cout << ex << std::endl;
//	}
//	// Ford-Bellman with negative cycle
//	Graph<1, 1> graphWithNegativeCycle;
//	graphWithNegativeCycle.AddVertex("0");
//	graphWithNegativeCycle.AddVertex("1");
//	graphWithNegativeCycle.AddVertex("2");
//	graphWithNegativeCycle.AddVertex("3");
//	graphWithNegativeCycle.AddVertex("4");
//	graphWithNegativeCycle.AddArc("0", "1", 6);
//	graphWithNegativeCycle.AddArc("0", "4", 7);
//	graphWithNegativeCycle.AddArc("1", "2", 5);
//	graphWithNegativeCycle.AddArc("1", "3", -4);
//	graphWithNegativeCycle.AddArc("1", "4", -8);
//	graphWithNegativeCycle.AddArc("2", "1", -2);
//	graphWithNegativeCycle.AddArc("3", "2", 7);
//	graphWithNegativeCycle.AddArc("3", "0", 2);
//	graphWithNegativeCycle.AddArc("4", "2", -3);
//	graphWithNegativeCycle.AddArc("4", "3", 9);
//	std::cout << "Вывести кратчайший путь из вершины u до вершины v (Ford-Bellman)" << std::endl;
//	GraphIO::Output(graphWithNegativeCycle, std::cout);
//	std::cin >> u1 >> v;
//	try {
//		auto path = graphWithNegativeCycle.ShortestPath(u1, v, ShortestPathAlgorithm::Bellman_Ford);
//		for (auto v = path.begin(); v != path.end(); ++v)
//			std::cout << *v << ' ';
//		std::cout << std::endl;
//	}
//	catch (const std::string& ex) {
//		std::cout << ex << std::endl;
//	}
//	catch (std::pair<std::vector<std::string>, std::string> ex) {
//		std::cout << ex.second << std::endl;
//		for (auto v = ex.first.begin(); v != ex.first.end(); ++v)
//			std::cout << *v << ' ';
//		std::cout << std::endl;
//	}
//	// Ford-Fulkerson
//	std::cout << "Задача о максимальном потоке. Решение алгоритмом Форда-Фалкерсона" << std::endl;
//	Graph<1, 1> forFordFulkerson;
//	forFordFulkerson.AddVertex("1"), forFordFulkerson.AddVertex("2"), forFordFulkerson.AddVertex("3"), forFordFulkerson.AddVertex("4"), forFordFulkerson.AddVertex("5");
//	forFordFulkerson.AddArc("1", "4", 10), forFordFulkerson.AddArc("1", "2", 20), forFordFulkerson.AddArc("1", "3", 30);
//	forFordFulkerson.AddArc("2", "3", 40), forFordFulkerson.AddArc("2", "5", 30);
//	forFordFulkerson.AddArc("3", "4", 10), forFordFulkerson.AddArc("3", "5", 20);
//	forFordFulkerson.AddArc("4", "5", 20);
//	GraphIO::Output(forFordFulkerson, std::cout);
//	try {
//		std::cout << "Максимальный поток: ";
//		std::cout << forFordFulkerson.MaximalFlow("1", "5") << std::endl;
//	}
//	catch (const std::string& ex) {
//		std::cout << ex << std::endl;
//	}
//}