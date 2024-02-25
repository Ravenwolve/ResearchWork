#pragma once
#include <string>
#include <iostream>
#include <unordered_map>
#include <unordered_set>

enum class ShortestPathAlgorithm {
	Dijkstra,
	Floyd,
	Bellman_Ford,
	Default
};

template <bool Directing, bool Weighting>
class Graph;

template <bool W>
struct ConditionalGraphType {
	using ListType = std::unordered_set<std::string>;
};

template <>
struct ConditionalGraphType<true> {
	using ListType = std::unordered_map<std::string, double>;
};

template <bool W>
using WeightingType = typename ConditionalGraphType<W>::ListType;

template<>
struct std::hash<std::pair<std::string, std::string>> {
	std::size_t operator()(std::pair<std::string, std::string> const& s) const noexcept {
		return std::hash<std::string>{}(s.first + '\n' + s.second);
	}
};

namespace GraphIO {
	template <bool Directing, bool Weighting>
	inline void Input(Graph<Directing, Weighting>&, std::ifstream&);

	template <bool Directing, bool Weighting>
	inline void Output(const Graph<Directing, Weighting>&, std::ostream&);
}

template <bool Directing, bool Weighting = 0>
class Graph {
	friend class Graph<false, Weighting>;
	friend class Graph<true, Weighting>;
	friend void GraphIO::Output(const Graph&, std::ostream&);
protected:
	mutable std::unordered_map<std::string, bool> visited;
	mutable std::unordered_map<std::string, std::unordered_map<std::string, std::string>> cachedFloyd;
	std::unordered_map<std::string, WeightingType<Weighting>> adjacencyList;
	inline void AddA(const std::string&, const std::string&, double = 1);
	inline void DFS(const std::string&) const noexcept;
	inline void BFS(const std::string&) const noexcept;
	inline void ResetVisited() const noexcept;
	// ���������� ���� �� ������� fromWhere �� ���� ������ �� ���������� ����� ��� ������������� ����� ���������� ��������
	inline std::unordered_map<std::string, std::string> Dijkstra(const std::string& fromWhere) const;
	inline std::unordered_map<std::string, std::unordered_map<std::string, std::string>>& Floyd() const;
	inline std::unordered_map<std::string, std::unordered_map<std::string, std::string>> Johnson();
	inline std::unordered_map<std::string, std::unordered_map<std::string, double>> GetAdjacencyMatrix() const noexcept;
public:
	inline Graph() noexcept;
	inline Graph(const Graph<false, Weighting>&) noexcept;
	inline Graph(const Graph<true, Weighting>&) noexcept;
	inline ~Graph() noexcept = default;
	// ���������� ���������� ������ � �����.
	inline size_t Size() const noexcept;
	// ��������� ������� � �������� ������
	inline void AddVertex(const std::string&);
	// ��� �������: ��������� ����, ������� �� ������ �������� ������� �� ������;
	// ��� ������������������ �����: ��������� �����, ����������� �������� ���� ������.
	inline void AddArc(const std::string&, const std::string&, double = 1);
	// ������� ������� � �������� ������.
	inline void RemoveVertex(const std::string&);
	// ��� �������: ������� ����, ������� �� ������ �������� ������� �� ������;
	// ��� ������������������ �����: ������� �����, ����������� �������� ���� ������.
	inline void RemoveArc(const std::string&, const std::string&);
	inline std::unordered_map<std::string, bool> GetVisited() const noexcept;
	// ��� �������:
	// - ���������� 1, ���� ���� ������ �������;
	// - ���������� -1, ���� ���� ����� �������.
	// ��� ������������������ �����:
	// - ���������� 1, ���� ���� �������.
	// ��� ����� ������ ������:
	// - ���������� 0, ���� ���� ���������.
	inline int IsLinked() const noexcept;
	// ���������� ���� ����� ��������� ���������
	inline std::unordered_map<std::string, double> FordBellman(const std::string& fromWhere) const;
	inline std::vector<std::string> ShortestPath(const std::string&, const std::string&, ShortestPathAlgorithm = ShortestPathAlgorithm::Dijkstra) const;
	// ���������� ����� ��������� ���������
	inline double Distance(const std::string&, const std::string&, ShortestPathAlgorithm = ShortestPathAlgorithm::Dijkstra) const;
	inline std::vector<std::string> GetVertices() const noexcept;
	inline std::unordered_map<std::pair<std::string, std::string>, double> GetArcs() const;
	inline std::vector<std::string> GetAdjacentVertices(const std::string&) const;
};