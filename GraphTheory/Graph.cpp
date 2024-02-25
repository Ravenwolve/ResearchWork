#pragma once
#include <fstream>
#include <queue>
#include <stack>
#include <functional>
#include "Graph.h"

template Graph<true, true>;
template Graph<false, false>;
template Graph<true, false>;
template Graph<false, true>;
template void GraphIO::Input<true, true>(Graph<true, true>&, std::ifstream&);
template void GraphIO::Input<false, false>(Graph<false, false>&, std::ifstream&);
template void GraphIO::Input<true, false>(Graph<true, false>&, std::ifstream&);
template void GraphIO::Input<false, true>(Graph<false, true>&, std::ifstream&);
template void GraphIO::Output<true, true>(const Graph<true, true>&, std::ostream&);
template void GraphIO::Output<false, false>(const Graph<false, false>&, std::ostream&);
template void GraphIO::Output<false, true>(const Graph<false, true>&, std::ostream&);
template void GraphIO::Output<true, false>(const Graph<true, false>&, std::ostream&);

template <bool Directing, bool Weighting>
inline size_t Graph<Directing, Weighting>::Size() const noexcept {
	return adjacencyList.size();
}

template <bool Directing, bool Weighting>
inline Graph<Directing, Weighting>::Graph(const Graph<false, Weighting>& other) noexcept : adjacencyList(other.adjacencyList), visited(other.visited), cachedFloyd(other.cachedFloyd) {}

template <bool Directing, bool Weighting>
inline Graph<Directing, Weighting>::Graph() noexcept {}


template <bool Directing, bool Weighting>
inline Graph<Directing, Weighting>::Graph(const Graph<true, Weighting>& other) noexcept : adjacencyList(other.adjacencyList), visited(other.visited), cachedFloyd(other.cachedFloyd) {
	if constexpr (!Directing) {
		for (auto iterList = adjacencyList.begin(); iterList != adjacencyList.end(); ++iterList)
			for (auto iterVertex = iterList->second.begin(); iterVertex != iterList->second.end(); ++iterVertex) {
				if constexpr (Weighting) {
					if (!adjacencyList[iterVertex->first].contains(iterList->first))
						AddA(iterVertex->first, iterList->first, iterVertex->second);
				}
				else if (!adjacencyList[*iterVertex].contains(iterList->first))
					AddA(*iterVertex, iterList->first);
			}
	}
}

template <bool Directing, bool Weighting>
inline void Graph<Directing, Weighting>::AddVertex(const std::string& vertex) {
	if (adjacencyList.contains(vertex))
		throw "Vertex " + vertex + " already exists";
	adjacencyList[vertex] = WeightingType<Weighting>();
	visited[vertex] = false;
	cachedFloyd.clear();
}

template <bool Directing, bool Weighting>
inline void Graph<Directing, Weighting>::AddA(const std::string& firstVertex, const std::string& secondVertex, double weightForWeightedGraph) {
	if (!adjacencyList.contains(firstVertex))
		throw "Vertex " + firstVertex + " doesn't exist";
	else if (!adjacencyList.contains(secondVertex))
		throw "Vertex " + secondVertex + " doesn't exist";
	else if (adjacencyList[firstVertex].contains(secondVertex))
		throw (Directing ? "Arc (" + firstVertex + ", " + secondVertex + ")" : "Edge {" + firstVertex + ", " + secondVertex + "}") + " already exists";
	if constexpr (Weighting)
		adjacencyList[firstVertex][secondVertex] = weightForWeightedGraph;
	else adjacencyList[firstVertex].insert(secondVertex);
}

template <bool Directing, bool Weighting>
inline void Graph<Directing, Weighting>::AddArc(const std::string& firstVertex, const std::string& secondVertex, double weightForWeightedGraph) {
	AddA(firstVertex, secondVertex, weightForWeightedGraph);
	if constexpr (!Directing)
		AddA(secondVertex, firstVertex, weightForWeightedGraph);
	cachedFloyd.clear();
}

template <bool Directing, bool Weighting>
inline void Graph<Directing, Weighting>::RemoveArc(const std::string& firstVertex, const std::string& secondVertex) {
	if (!adjacencyList.contains(firstVertex))
		throw "Vertex " + firstVertex + " doesn't exist";
	else if (!adjacencyList.contains(secondVertex))
		throw "Vertex " + secondVertex + " doesn't exist";
	else if (!adjacencyList[firstVertex].contains(secondVertex))
		throw (Directing ? "Arc (" + firstVertex + ", " + secondVertex + ")" : "Edge {" + firstVertex + ", " + secondVertex + "}") + " doesn't exists";
	adjacencyList[firstVertex].erase(secondVertex);
	if constexpr (!Directing)
		adjacencyList[secondVertex].erase(firstVertex);
	cachedFloyd.clear();
}

template <bool Directing, bool Weighting>
inline void Graph<Directing, Weighting>::RemoveVertex(const std::string& vertex) {
	if (!adjacencyList.contains(vertex))
		throw "Vertex " + vertex + " doesn't exist";
	for (auto iter = adjacencyList.begin(); iter != adjacencyList.end(); ++iter)
		if (iter->second.contains(vertex))
			RemoveArc(iter->first, vertex);
	adjacencyList.erase(vertex);
	visited.erase(vertex);
	cachedFloyd.clear();
}

template <bool Directing, bool Weighting>
inline void Graph<Directing, Weighting>::DFS(const std::string& vertex) const noexcept {
	std::stack<std::string> stack;
	std::string curVertex;
	visited[vertex] = true;
	stack.push(vertex);
	while (!stack.empty()) {
		curVertex = stack.top();
		stack.pop();
		for (auto item : adjacencyList.at(curVertex)) {
			if constexpr (Weighting) {
				if (!visited[item.first]) {
					stack.push(item.first);
					visited[item.first] = true;
				}
			}
			else if (!visited[item]) {
				stack.push(item);
				visited[item] = true;
			}
		}
	}
}

template <bool Directing, bool Weighting>
inline void Graph<Directing, Weighting>::BFS(const std::string& vertex) const noexcept {
	std::queue<std::string> queue;
	std::string curVertex;
	queue.push(vertex);
	visited[vertex] = true;
	while (!queue.empty()) {
		curVertex = queue.front();
		queue.pop();
		for (auto item : adjacencyList.at(curVertex)) {
			if constexpr (Weighting) {
				if (!visited[item.first]) {
					queue.push(item.first);
					visited[item.first] = true;
				}
			}
			else if (!visited[item]) {
				queue.push(item);
				visited[item] = true;
			}
		}
	}
}

template <bool Directing, bool Weighting>
inline void Graph<Directing, Weighting>::ResetVisited() const noexcept {
	for (auto iter = visited.begin(); iter != visited.end(); ++iter)
		iter->second = false;
}

template <bool Directing, bool Weighting>
inline int Graph<Directing, Weighting>::IsLinked() const noexcept {
	if constexpr (!Directing) {
		int result = 1;
		DFS(adjacencyList.begin()->first);
		for (auto iter = visited.begin(); iter != visited.end(); ++iter) {
			if (!iter->second)
				result = 0;
			else iter->second = false;
		}
		return result;
	}
	else {
		bool strongConnect;
		for (auto list : adjacencyList) {
			strongConnect = true;
			DFS(list.first);
			for (auto iter = visited.begin(); iter != visited.end(); ++iter) {
				if (!iter->second)
					strongConnect = false;
				else iter->second = false;
			}
			if (strongConnect) // если это сильно-св€зный орграф
				return 1;
		}
		return -Graph<0, Weighting>(*this).IsLinked(); // проверка на слабую св€зность (0 или -1)
	}
}

template <bool Directing, bool Weighting>
inline void GraphIO::Input(Graph<Directing, Weighting>& graph, std::ifstream& input) {
	bool d, w;
	input >> d >> w;
	if (d != Directing || w != Weighting)
		throw "Input graph doesn't match the declared type";
	std::string firstStr, secondStr;
	input.get();
	while (input.get() != '\n') {
		input >> firstStr;
		graph.AddVertex(firstStr);
	}
	double weight = 1;
	for (input >> firstStr; !input.eof(); input >> firstStr)
		while (input.get() != '\n') {
			input >> secondStr;
			if constexpr (Weighting)
				input >> weight;
			try {
				graph.AddArc(firstStr, secondStr, weight);
			}
			catch (...) {}
		}
}

template <bool Directing, bool Weighting>
inline void GraphIO::Output(const Graph<Directing, Weighting>& graph, std::ostream& output) {
	output << Directing << ' ' << Weighting << std::endl;
	for (auto list : graph.adjacencyList)
		output << ' ' << list.first;
	output << '\n';
	for (auto list : graph.adjacencyList) {
		output << ' ' << list.first;
		for (auto item : list.second) {
			if constexpr (Weighting)
				output << ' ' << item.first << ' ' << item.second;
			else output << ' ' << item;
		}
		output << '\n';
	}
}

template <bool Directing, bool Weighting>
inline std::unordered_map<std::string, std::string> Graph<Directing, Weighting>::Dijkstra(const std::string& fromWhere) const {
	if constexpr (Weighting) {
		std::unordered_map<std::string, double> lengthsOfShortestWaysToAll; // D
		std::unordered_map<std::string, std::string> shortestWaysToAll;		// P
		std::string* currentVertex = const_cast<std::string*>(&fromWhere);  // w
		std::string* nextVertex = nullptr;
		double minFromSWTWT;
		bool endingFlag = false;
		while (!endingFlag) {
			endingFlag = true;
			if (currentVertex != nullptr)
				visited[*currentVertex] = true;
			else break;
			minFromSWTWT = DBL_MAX;
			for (auto iter = adjacencyList.begin(); iter != adjacencyList.end(); ++iter) { // v
				if (!visited[iter->first]) {
					if (adjacencyList.at(*currentVertex).contains(iter->first)) {
						// D[v] = min(D[v], D[w] + C(w, v)
						if (lengthsOfShortestWaysToAll.contains(iter->first)) {
							if (lengthsOfShortestWaysToAll[*currentVertex] + adjacencyList.at(*currentVertex).at(iter->first) < lengthsOfShortestWaysToAll[iter->first]) {
								lengthsOfShortestWaysToAll[iter->first] = lengthsOfShortestWaysToAll[*currentVertex] + adjacencyList.at(*currentVertex).at(iter->first);
								shortestWaysToAll[iter->first] = *currentVertex; // добавл€ем ребро (w, v)
							}
						}
						else {
							lengthsOfShortestWaysToAll[iter->first] = lengthsOfShortestWaysToAll[*currentVertex] + adjacencyList.at(*currentVertex).at(iter->first);
							shortestWaysToAll[iter->first] = *currentVertex;
						}
					}
					// ищем минимальный элемент из D из оставшихс€
					if (lengthsOfShortestWaysToAll.contains(iter->first) && lengthsOfShortestWaysToAll[iter->first] < minFromSWTWT) {
						nextVertex = const_cast<std::string*>(&iter->first);
						minFromSWTWT = lengthsOfShortestWaysToAll[iter->first];
					}
					endingFlag = false;
				}
			}
			if (nextVertex == nullptr || *nextVertex == *currentVertex)
				break;
			else currentVertex = nextVertex;
		}
		ResetVisited();
		return shortestWaysToAll;
	}
	else throw "This graph is unweighted";
}

template <bool Directing, bool Weighting>
inline std::unordered_map<std::string, std::unordered_map<std::string, double>> Graph<Directing, Weighting>::GetAdjacencyMatrix() const noexcept {
	if constexpr (Weighting) {
		std::unordered_map<std::string, std::unordered_map<std::string, double>> adjacencyMatrix = adjacencyList;
		for (auto iter = adjacencyMatrix.begin(); iter != adjacencyMatrix.end(); ++iter)
			for (auto iter2 = adjacencyMatrix.begin(); iter2 != adjacencyMatrix.end(); ++iter2)
				if (!iter->second.contains(iter2->first)) {
					if (iter->first == iter2->first)
						iter->second[iter2->first] = 0;
					else iter->second[iter2->first] = DBL_MAX;
				}
		return adjacencyMatrix;
	}
	else {
		std::unordered_map<std::string, std::unordered_map<std::string, double>> adjacencyMatrix;
		for (auto iter = adjacencyList.begin(); iter != adjacencyList.end(); ++iter)
			for (auto iter2 = adjacencyList.begin(); iter2 != adjacencyList.end(); ++iter2) {
				if (!iter->second.contains(iter2->first)) {
					if (iter->first == iter2->first)
						adjacencyMatrix[iter->first][iter2->first] = 0;
					else adjacencyMatrix[iter->first][iter2->first] = DBL_MAX;
				}
				else adjacencyMatrix[iter->first][iter2->first] = 1;
			}
		return adjacencyMatrix;
	}
}

template <bool Directing, bool Weighting>
inline std::unordered_map<std::string, std::unordered_map<std::string, std::string>>& Graph<Directing, Weighting>::Floyd() const {
	if (cachedFloyd.empty()) {
		std::unordered_map<std::string, std::unordered_map<std::string, double>> adjacencyMatrix = GetAdjacencyMatrix();
		cachedFloyd = std::unordered_map<std::string, std::unordered_map<std::string, std::string>>();
		for (auto k_iter = adjacencyMatrix.begin(); k_iter != adjacencyMatrix.end(); ++k_iter)
			for (auto i_iter = adjacencyMatrix.begin(); i_iter != adjacencyMatrix.end(); ++i_iter)
				for (auto j_iter = i_iter->second.begin(); j_iter != i_iter->second.end(); ++j_iter)
					// A[i, j] = min(A[i, j], A[i, k] + A[k, j])
					if (i_iter->second[k_iter->first] + k_iter->second[j_iter->first] < j_iter->second) {
						j_iter->second = i_iter->second[k_iter->first] + k_iter->second[j_iter->first];
						cachedFloyd[i_iter->first][j_iter->first] = k_iter->first;
					}
	}
	return cachedFloyd;
}

template<bool Directing, bool Weighting>
inline std::unordered_map<std::string, std::unordered_map<std::string, std::string>> Graph<Directing, Weighting>::Johnson()
{
	std::unordered_map<std::string, std::unordered_map<std::string, std::string>> dists;
	AddVertex("s");
	for (auto it = adjacencyList.begin(); it != adjacencyList.end(); ++it)
	{
		AddArc("s", it->first, 0.f);
	}

	auto tempBellmanDists = FordBellman("s");

	if constexpr (Weighting)
	{
		for (auto u_iter = adjacencyList.begin(); u_iter != adjacencyList.end(); ++u_iter)
		{
			for (auto v_iter = u_iter->second.begin(); v_iter != u_iter->second.end(); ++v_iter)
			{
				v_iter->second = v_iter->second +
					tempBellmanDists[u_iter->first] - tempBellmanDists[v_iter->first];
			}
		}
	}
	else
	{
		throw std::exception("Graph must be oriented");
	}
	

	RemoveVertex("s");

	for (auto it = adjacencyList.begin(); it != adjacencyList.end(); ++it)
	{
		dists[it->first] = Dijkstra(it->first);
	}

	return dists;
}

template <bool Directing, bool Weighting>
inline std::vector<std::string> Graph<Directing, Weighting>::ShortestPath(const std::string& fromWhere, const std::string& whereTo, ShortestPathAlgorithm algorithmType) const {
	if (!adjacencyList.contains(fromWhere))
		throw "Vertex " + fromWhere + " doesn't exist";
	else if (!adjacencyList.contains(whereTo))
		throw "Vertex " + whereTo + " doesn't exist";
	if constexpr (Weighting) {
		if (algorithmType == ShortestPathAlgorithm::Default && cachedFloyd.empty() || algorithmType == ShortestPathAlgorithm::Dijkstra) {
			std::unordered_map<std::string, std::string> result = Dijkstra(fromWhere);
			if (!result.contains(whereTo))
				throw "Path from " + fromWhere + " to " + whereTo + " doesn't exist";
			std::vector<std::string> path;
			std::stack<std::string> stack;
			// поскольку нас интересует заданна€ вершина, до которой есть путь, то мы от этой вершины и отталкиваемс€,
			// перечисл€€ обратные дуги, то есть вершины в обратном пор€дке
			std::string* cur = const_cast<std::string*>(&whereTo);
			while (true) {
				stack.push(*cur);
				if (*cur == fromWhere)
					break;
				cur = &result[*cur];
			}
			while (!stack.empty()) {
				path.push_back(stack.top());
				stack.pop();
			}
			return path;
		}
		else if (algorithmType == ShortestPathAlgorithm::Default || algorithmType == ShortestPathAlgorithm::Floyd) {
			std::unordered_map<std::string, std::unordered_map<std::string, std::string>> result = Floyd();
			if (!result[fromWhere].contains(whereTo)) {
				if (!adjacencyList.at(fromWhere).contains(whereTo))
					throw "Path from " + fromWhere + " to " + whereTo + " doesn't exist";
				else {
					// јлгоритм не учитывает рЄбра, соедин€ющие вершины напр€мую, поэтому это проверка на наличие такого ребра
					std::vector<std::string> path(2);
					path[0] = fromWhere;
					path[1] = whereTo;
					return path;
				}
			}
			std::vector<std::string> path;
			std::function<void(const std::string&, const std::string&)> floydPath = [&floydPath, &result, &path, &fromWhere, &whereTo](const std::string& fw, const std::string& wt) {
				if (!result[fw].contains(wt))
					return;
				floydPath(fw, result[fw][wt]);
				path.push_back(result[fw][wt]);
				floydPath(result[fw][wt], wt);
			};
			path.push_back(fromWhere);
			floydPath(fromWhere, whereTo);
			path.push_back(whereTo);
			return path;
		}
		else {
			//std::unordered_map<std::string, std::string> result = FordBellman(fromWhere);
			//if (result[whereTo] == "")
				//throw "Path from " + fromWhere + " to " + whereTo + " doesn't exist";
			std::vector<std::string> path;
			//for (std::string* currentVertex = const_cast<std::string*>(&whereTo); *currentVertex != ""; currentVertex = &result[*currentVertex])
				//path.push_back(*currentVertex);
			//std::reverse(path.begin(), path.end());
			return path;
		}
	}
	else throw "This graph is unweighted";
}


template <bool Directing, bool Weighting>
inline std::unordered_map<std::string, double> Graph<Directing, Weighting>::FordBellman(const std::string& fromWhere) const {
	if constexpr (Weighting) {
		std::unordered_map<std::string, double> dist;
		std::unordered_map<std::string, std::string> path;
		for (auto iter = adjacencyList.begin(); iter != adjacencyList.end(); ++iter) {
			dist[iter->first] = DBL_MAX;
			path[iter->first] = "";
		}
		dist[fromWhere] = 0;
		bool negativeCycle = false;
		for (size_t i = 0; i < Size(); ++i) {
			negativeCycle = false;
			for (auto u_iter = adjacencyList.begin(); u_iter != adjacencyList.end(); ++u_iter)
				for (auto v_iter = u_iter->second.begin(); v_iter != u_iter->second.end(); ++v_iter)
					if (dist[u_iter->first] < DBL_MAX && dist[u_iter->first] + v_iter->second < dist[v_iter->first]) {
						dist[v_iter->first] = dist[u_iter->first] + v_iter->second;
						path[v_iter->first] = u_iter->first;
						negativeCycle = true;
					}
		}
		if (!negativeCycle)
			return dist;
		else throw std::string("This graph having negative cycle");
	}
	else throw std::string("This graph is unweighted");
}

template <bool Directing, bool Weighting>
inline double Graph<Directing, Weighting>::Distance(const std::string& fromWhere, const std::string& whereTo, ShortestPathAlgorithm algorithmType) const {
	std::vector<std::string> path = ShortestPath(fromWhere, whereTo, algorithmType);
	if constexpr (Weighting) {
		double length = 0;
		for (int i = 1; i < path.size(); ++i)
			length += adjacencyList.at(path[i - 1]).at(path[i]);
		return length;
	}
	else return path.size();
}

template <bool Directing, bool Weighting>
inline std::vector<std::string> Graph<Directing, Weighting>::GetVertices() const noexcept {
	std::vector<std::string> v;
	for (auto iter = adjacencyList.begin(); iter != adjacencyList.end(); ++iter)
		v.push_back(iter->first);
	return v;
}

template <bool Directing, bool Weighting>
inline std::unordered_map<std::pair<std::string, std::string>, double> Graph<Directing, Weighting>::GetArcs() const {
	std::unordered_map<std::pair<std::string, std::string>, double> edges;
	for (auto u_iter = adjacencyList.begin(); u_iter != adjacencyList.end(); ++u_iter)
		for (auto v_iter = u_iter->second.begin(); v_iter != u_iter->second.end(); ++v_iter) {
			if constexpr (Weighting) {
				if (!edges.contains({ u_iter->first, v_iter->first }))
					edges[std::pair(u_iter->first, v_iter->first)] = v_iter->second;
			}
			else if (!edges.contains({ u_iter->first, *v_iter }))
				edges[std::pair(u_iter->first, *v_iter)] = 1;
		}
	return edges;
}

template<bool Directing, bool Weighting>
inline std::unordered_map<std::string, bool> Graph<Directing, Weighting>::GetVisited() const noexcept {
	return visited;
}

template<bool Directing, bool Weighting>
inline std::vector<std::string> Graph<Directing, Weighting>::GetAdjacentVertices(const std::string& vertex) const {
	if (adjacencyList.contains(vertex)) {
		std::vector<std::string> result;
		if constexpr (Weighting)
			for (auto iter = adjacencyList.at(vertex).begin(); iter != adjacencyList.at(vertex).end(); ++iter)
				result.push_back(iter->first);
		else std::copy(adjacencyList.at(vertex).begin(), adjacencyList.at(vertex).end(), std::back_inserter(result));
	}
	else throw "Vertex doesn't exist";
}