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
inline Graph<Directing, Weighting>::Graph(const Graph<false, Weighting>& other) noexcept : adjacencyList(other.adjacencyList), visited(other.visited) {}

template <bool Directing, bool Weighting>
inline Graph<Directing, Weighting>::Graph() noexcept {}


template <bool Directing, bool Weighting>
inline Graph<Directing, Weighting>::Graph(const Graph<true, Weighting>& other) noexcept : adjacencyList(other.adjacencyList), visited(other.visited) {
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
}

template <bool Directing, bool Weighting>
inline void Graph<Directing, Weighting>::ResetVisited() const noexcept {
	for (auto iter = visited.begin(); iter != visited.end(); ++iter)
		iter->second = false;
}

template <bool Directing, bool Weighting>
inline void GraphIO::Input(Graph<Directing, Weighting>& graph, std::ifstream& input) {
	bool d, w;
	input >> d >> w;
	if (d != Directing || w != Weighting)
		throw "Input graph doesn't match the declared type";
	std::string firstStr, secondStr;
	double weight = 1;
	for (input >> firstStr; !input.eof(); input >> firstStr) {
		if (!graph.adjacencyList.contains(firstStr))
			graph.AddVertex(firstStr);
		while (input.get() != '\n') {
			input >> secondStr;
			if constexpr (Weighting)
				input >> weight;
			try {
				if (!graph.adjacencyList.contains(secondStr))
					graph.AddVertex(secondStr);
				graph.AddArc(firstStr, secondStr, weight);
			}
			catch (...) {}
		}
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
		//vertex, distance
		std::unordered_map<std::string, int64_t> d;
		//vertex, it's parent
		std::unordered_map<std::string, std::string> parents;
		using Pair = std::pair<int64_t, std::string>;

		/*for (const auto pair : adjacencyList)
		{
			if (pair.first != fromWhere)
			{
				d.insert(std::make_pair(pair.first, INT64_MAX));
				parents.insert(std::make_pair(pair.first, ""));
			}

			else
			{
				d.insert(std::make_pair(pair.first, 0));
				parents.insert(std::make_pair(pair.first, ""));
			}

		}*/

		for (auto it = adjacencyList.begin(); it != adjacencyList.end(); ++it)
		{
			if (it->first != fromWhere)
			{
				d.insert(std::make_pair(it->first, INT64_MAX));
				parents.insert(std::make_pair(it->first, ""));
			}
			else
			{
				d.insert(std::make_pair(it->first, 0));
				parents.insert(std::make_pair(it->first, ""));
			}
		}

		std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> pq;
		pq.push(std::make_pair(0, fromWhere));

		while (!pq.empty())
		{
			auto cur_pair = pq.top();
			auto cur_v = cur_pair.second;
			auto cur_d = cur_pair.first;
			pq.pop();

			if (cur_d > d[cur_v])
			{
				continue;
			}

			/*for (const auto pair : adjacencyList[cur_pair.second])
			{
				auto cur_u = pair.first;
				auto w = pair.second;
				if (d[cur_u] > d[cur_v] + w)
				{
					d[cur_u] = d[cur_v] + w;
					parents[cur_u] = cur_v;
					pq.push(std::make_pair(d[cur_u], cur_u));
				}
			}*/
			auto tempAdjList = adjacencyList.at(cur_pair.second);
			for (auto pair = tempAdjList.begin(); pair != tempAdjList.end(); ++pair)
			{
				auto cur_u = pair->first;
				auto w = pair->second;
				if (d[cur_u] > d[cur_v] + w)
				{
					d[cur_u] = d[cur_v] + w;
					parents[cur_u] = cur_v;
					pq.push(std::make_pair(d[cur_u], cur_u));
				}
			}
		}

		return parents;
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
	std::unordered_map<std::string, std::unordered_map<std::string, double>> adjacencyMatrix = GetAdjacencyMatrix();
	std::unordered_map<std::string, std::unordered_map<std::string, std::string>> floydMatrix;
	for (auto k_iter = adjacencyMatrix.begin(); k_iter != adjacencyMatrix.end(); ++k_iter)
		for (auto i_iter = adjacencyMatrix.begin(); i_iter != adjacencyMatrix.end(); ++i_iter)
			for (auto j_iter = i_iter->second.begin(); j_iter != i_iter->second.end(); ++j_iter)
				// A[i, j] = min(A[i, j], A[i, k] + A[k, j])
				if (i_iter->second[k_iter->first] + k_iter->second[j_iter->first] < j_iter->second) {
					j_iter->second = i_iter->second[k_iter->first] + k_iter->second[j_iter->first];
					floydMatrix[i_iter->first][j_iter->first] = k_iter->first;
				}
	return floydMatrix;
}

template<bool Directing, bool Weighting>
inline std::unordered_map<std::string, std::unordered_map<std::string, std::string>> Graph<Directing, Weighting>::Johnson() {
	std::unordered_map<std::string, std::unordered_map<std::string, std::string>> dists;
	AddVertex("s");
	for (auto it = adjacencyList.begin(); it != adjacencyList.end(); ++it)
		AddArc("s", it->first, 0.f);
	auto tempBellmanDists = FordBellman("s");
	if constexpr (Weighting) {
		for (auto u_iter = adjacencyList.begin(); u_iter != adjacencyList.end(); ++u_iter)
			for (auto v_iter = u_iter->second.begin(); v_iter != u_iter->second.end(); ++v_iter)
				v_iter->second = v_iter->second +
					tempBellmanDists[u_iter->first] - tempBellmanDists[v_iter->first];
	}
	else throw std::exception("Graph must be oriented");
	RemoveVertex("s");
	for (auto it = adjacencyList.begin(); it != adjacencyList.end(); ++it)
		dists[it->first] = Dijkstra(it->first);
	return dists;
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
