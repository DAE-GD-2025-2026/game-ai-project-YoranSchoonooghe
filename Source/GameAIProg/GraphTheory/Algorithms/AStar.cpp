#include "AStar.h"

using namespace GameAI;

AStar::AStar(Graph* const pGraph, HeuristicFunctions::Heuristic hFunction)
	: pGraph(pGraph)
	, HeuristicFunction(hFunction)
{
}

std::vector<Node*>AStar::FindPath(Node* const pStartNode, Node* const pGoalNode)
{
	std::vector<Node*> path{};

	std::vector<NodeRecord> openList;
	std::vector<NodeRecord> closedList;
	NodeRecord currentNodeRecord;

	NodeRecord startRecord;
	startRecord.pNode = pStartNode;
	startRecord.pConnection = nullptr;
	startRecord.estimatedTotalCost = GetHeuristicCost(pStartNode, pGoalNode);

	openList.push_back(startRecord);
	while (!openList.empty())
	{
		currentNodeRecord = *std::min_element(openList.begin(), openList.end());
		if (currentNodeRecord.pNode == pGoalNode)
		{
			break;
		}

		auto pConnections = pGraph->FindConnectionsFrom(currentNodeRecord.pNode->GetId());
		for (auto pConnection : pConnections)
		{
			Node* pNextNode = pGraph->GetNode(pConnection->GetToId()).get();
			
			float totalGCost = currentNodeRecord.costSoFar + pConnection->GetWeight();

			auto closedIt = std::find_if(closedList.begin(), closedList.end(),
				[pNextNode](const NodeRecord& r) { return r.pNode == pNextNode; });

			if (closedIt != closedList.end())
			{
				if (closedIt->costSoFar <= totalGCost)
				{
					continue;
				}
				else
				{
					closedList.erase(closedIt);
				}
			}

			auto openIt = std::find_if(openList.begin(), openList.end(),
				[pNextNode](const NodeRecord& r) { return r.pNode == pNextNode; });

			if (openIt != openList.end())
			{
				if (openIt->costSoFar <= totalGCost)
				{
					continue;
				}
				else
				{
					openList.erase(openIt);
				}
			}

			NodeRecord nextRecord;
			nextRecord.pNode = pNextNode;
			nextRecord.pConnection = pConnection;
			nextRecord.costSoFar = totalGCost;
			nextRecord.estimatedTotalCost = totalGCost + GetHeuristicCost(pNextNode, pGoalNode);
			openList.push_back(nextRecord);
		}

		auto currentIt = std::find(openList.begin(), openList.end(), currentNodeRecord);
		if (currentIt != openList.end())
		{
			openList.erase(currentIt);
		}
		closedList.push_back(currentNodeRecord);
	}

	if (currentNodeRecord.pNode == pGoalNode)
	{
		while (currentNodeRecord.pNode != pStartNode)
		{
			path.push_back(currentNodeRecord.pNode);

			int fromNodeId = currentNodeRecord.pConnection->GetFromId();

			auto it = std::find_if(closedList.begin(), closedList.end(),
				[fromNodeId](const NodeRecord& r) { return r.pNode->GetId() == fromNodeId; });

			if (it != closedList.end())
			{
				currentNodeRecord = *it;
			}
			else
			{
				break;
			}
		}

		path.push_back(pStartNode);
		std::reverse(path.begin(), path.end());
	}

	return path;
}

float AStar::GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const
{
	FVector2D toDestination = pGraph->GetNode(pEndNode->GetId())->GetPosition() - pGraph->GetNode(pStartNode->GetId())->GetPosition();
	return HeuristicFunction(abs(toDestination.X), abs(toDestination.Y));
}