#pragma once
#include <stack>
#include "Shared/Graph/Graph.h"

namespace GameAI
{
	enum class Eulerianity
	{
		notEulerian,
		semiEulerian,
		eulerian,
	};

	class EulerianPath final
	{
	public:
		EulerianPath(Graph* const pGraph);

		Eulerianity IsEulerian() const;
		std::vector<Node*> FindPath(Eulerianity& eulerianity) const;

	private:
		void VisitAllNodesDFS(const std::vector<Node*>& pNodes, std::vector<bool>& visited, int startIndex) const;
		bool IsConnected() const;

		Graph* m_pGraph;
	};

	inline EulerianPath::EulerianPath(Graph* const pGraph)
		: m_pGraph(pGraph)
	{
	}

	inline Eulerianity EulerianPath::IsEulerian() const
	{
		// TODO If the graph is not connected, there can be no Eulerian Trail
		if (!IsConnected()) return Eulerianity::notEulerian;

		// TODO Count nodes with odd degree 
		int oddDegreeCount = 0;
		std::vector<Node*> nodes = m_pGraph->GetActiveNodes();
		for (const auto& node : nodes)
		{
			int degree = static_cast<int>(m_pGraph->FindConnectionsWith(node->GetId()).size());

			if (degree % 2 != 0)
			{
				++oddDegreeCount;
			}
		}

		// TODO A connected graph with more than 2 nodes with an odd degree (an odd amount of connections) is not Eulerian
		if (oddDegreeCount > 2) return Eulerianity::notEulerian;

		// TODO A connected graph with exactly 2 nodes with an odd degree is Semi-Eulerian (unless there are only 2 nodes)
		// TODO An Euler trail can be made, but only starting and ending in these 2 nodes
		if (nodes.size() > 2 && oddDegreeCount == 2) return Eulerianity::semiEulerian;

		// TODO A connected graph with no odd nodes is Eulerian
		if (oddDegreeCount == 0) return Eulerianity::eulerian;

		return Eulerianity::notEulerian;
	}

	inline std::vector<Node*> EulerianPath::FindPath(Eulerianity& eulerianity) const
	{
		// Get a copy of the graph because this algorithm involves removing edges
		Graph graphCopy = m_pGraph->Clone();
		std::vector<Node*> Path = {};
		std::vector<Node*> Nodes = graphCopy.GetActiveNodes();
		int currentNodeId{ Graphs::InvalidNodeId };
		
		// TODO Check if there can be an Euler path
		// TODO If this graph is not eulerian, return the empty path
		eulerianity = IsEulerian();
		if (eulerianity == Eulerianity::notEulerian) return Path;

		// TODO Start algorithm loop
		std::stack<int> nodeStack;

		// Choose starting node
		if (eulerianity == Eulerianity::semiEulerian)
		{
			for (auto* node : Nodes)
			{
				if (m_pGraph->FindConnectionsWith(node->GetId()).size() % 2 != 0)
				{
					currentNodeId = node->GetId();
					break;
				}
			}
		}
		else
		{
			for (auto* node : Nodes)
			{
				if (!m_pGraph->FindConnectionsWith(node->GetId()).empty())
				{
					currentNodeId = node->GetId();
					break;
				}
			}
		}

		if (currentNodeId == Graphs::InvalidNodeId) return Path;

		while (!nodeStack.empty() || !graphCopy.FindConnectionsFrom(currentNodeId).empty())
		{
			auto connections = graphCopy.FindConnectionsFrom(currentNodeId);

			if (!connections.empty())
			{
				nodeStack.push(currentNodeId);

				int neighborId = connections[0]->GetToId();

				graphCopy.RemoveConnection(currentNodeId, neighborId);
				graphCopy.RemoveConnection(neighborId, currentNodeId);

				currentNodeId = neighborId;
			}
			else
			{
				Path.push_back(m_pGraph->GetNode(currentNodeId).get());

				if (!nodeStack.empty())
				{
					currentNodeId = nodeStack.top();
					nodeStack.pop();
				}
			}
		}
		Path.push_back(m_pGraph->GetNode(currentNodeId).get());

		std::reverse(Path.begin(), Path.end());
		return Path;
	}

	inline void EulerianPath::VisitAllNodesDFS(const std::vector<Node*>& Nodes, std::vector<bool>& visited, int startIndex ) const
	{
		// TODO Mark the visited node
		visited[startIndex] = true;

		// TODO Ask the graph for the connections from that node
		int currentId = Nodes[startIndex]->GetId();
		std::vector<Connection*> pConnections = m_pGraph->FindConnectionsFrom(currentId);

		// TODO recursively visit any valid connected nodes that were not visited before
		// TODO Tip: use an index-based for-loop to find the correct index
		for (auto pConnection : pConnections)
		{
			int targetId = pConnection->GetToId();
			int targetIndex = -1;

			for (int i = 0; i < static_cast<int>(Nodes.size()); ++i)
			{
				if (Nodes[i]->GetId() == targetId)
				{
					targetIndex = i;
					break;
				}
			}

			if (targetIndex != -1 && !visited[targetIndex])
			{
				VisitAllNodesDFS(Nodes, visited, targetIndex);
			}
		}
	}

	inline bool EulerianPath::IsConnected() const
	{
		std::vector<Node*> Nodes = m_pGraph->GetActiveNodes();
		if (Nodes.size() == 0) return false;

		// TODO choose a starting node
		int startIndex = -1;
		for (int i = 0; i < static_cast<int>(Nodes.size()); ++i)
		{
			if (!m_pGraph->FindConnectionsFrom(Nodes[i]->GetId()).empty())
			{
				startIndex = i;
				break;
			}
		}

		if (startIndex == -1) return true;

		// TODO start a depth-first-search traversal from the node that has at least one connection
		std::vector<bool> visitedNodes(Nodes.size(), false);
		VisitAllNodesDFS(Nodes, visitedNodes, startIndex);

		// TODO if a node was never visited, this graph is not connected
		for (int i = 0; i < static_cast<int>(Nodes.size()); ++i)
		{
			bool hasConnections = !m_pGraph->FindConnectionsFrom(Nodes[i]->GetId()).empty();
			if (hasConnections && !visitedNodes[i])
			{
				return false;
			}
		}

		return true;
	}
}