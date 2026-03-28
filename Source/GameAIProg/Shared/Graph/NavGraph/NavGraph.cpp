#include "NavGraph.h"

#include "NavGraphNode.h"

GameAI::NavGraph::NavGraph(std::unique_ptr<TriPolygon> && NavPoly)
	: Graph{false}
	, pNavPoly{std::move(NavPoly)}
{
	CreateNavigationGraph();
}

GameAI::NavGraph::NavGraph(const NavGraph& Other)
	: Graph(false)
{
	Nodes.reserve(Other.Nodes.size());
	for (std::unique_ptr<Node> const & OtherNode : Other.Nodes)
	{
		Nodes.push_back(std::make_unique<NavGraphNode>(*dynamic_cast<NavGraphNode*>(OtherNode.get())));
	}
        
	Connections.reserve(Other.Connections.size());
	for (std::unique_ptr<Connection> const & OtherConnection : Other.Connections)
	{
		Connections.push_back(std::make_unique<Connection>(*OtherConnection.get()));
	}
}

std::unique_ptr<GameAI::NavGraph> GameAI::NavGraph::Clone() const
{
	return std::make_unique<NavGraph>(*this);
}

int GameAI::NavGraph::GetNodeIdFromEdgeIndex(int EdgeIdx) const
{
	if (EdgeIdx >= 0)
	{
		for (auto const & pNode : Nodes)
		{
			if (reinterpret_cast<NavGraphNode*>(pNode.get())->GetEdgeIdx() == EdgeIdx)
			{
				return pNode->GetId();
			}
		}
	}
	
	return Graphs::InvalidNodeId;
}

std::vector<TriPolygon::Triangle> GameAI::NavGraph::GetTrianglesFromEdge(const TriPolygon::Edge& edge) const
{
	std::vector<TriPolygon::Triangle> triangles{};

	for (auto const& triangle : pNavPoly->GetTriangles())
	{
		if (triangle.HasEdge(edge))
		{
			triangles.push_back(triangle);
		}
	}

	return triangles;
}

void GameAI::NavGraph::CreateNavigationGraph()
{
	//1. Go over all the edges of the navigation mesh and create nodes
			// Create node here
	auto const& edges = pNavPoly->GetEdges();

	for (int i = 0; i < edges.size(); ++i)
	{
		const auto& connectedTriangles{ GetTrianglesFromEdge(edges[i]) };
		if (connectedTriangles.size() <= 1) continue;

		FVector p1 = edges[i].GetP1(*pNavPoly);
		FVector p2 = edges[i].GetP2(*pNavPoly);

		FVector2D midpoint{
			(p1.X + p2.X) / 2.0f,
			(p1.Y + p2.Y) / 2.0f
		};

		auto node = std::make_unique<NavGraphNode>(midpoint, i);
		AddNode(std::move(node));
	}

	//2. Create connections now that every node is created	
		//2 valid nodes -> 1 connection
		//3 valid nodes -> 3 connections
	for (auto const& triangle : pNavPoly->GetTriangles())
	{
		std::vector<int> nodeIds;

		for (const auto& edge : triangle.GetEdges())
		{
			auto const& edgeIdx = pNavPoly->FindEdgeIndex(edge);

			if (edgeIdx.has_value())
			{
				int nodeId = GetNodeIdFromEdgeIndex(edgeIdx.value());
				if (nodeId != Graphs::InvalidNodeId)
				{
					nodeIds.push_back(nodeId);
				}
			}
		}

		auto const nrOfValidNodes = nodeIds.size();
		if (nrOfValidNodes == 2)
		{
			AddConnection(nodeIds[0], nodeIds[1]);
		}
		else if (nrOfValidNodes == 3)
		{
			AddConnection(nodeIds[0], nodeIds[1]);
			AddConnection(nodeIds[1], nodeIds[2]);
			AddConnection(nodeIds[2], nodeIds[0]);
		}
	}

	//3. Set the connections cost to the actual distance
	SetConnectionCostsToDistances();
}
