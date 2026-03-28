#include "NavGraphPathfinding.h"

#include "AStar.h"
#include "PathSmoothing.h"
#include "VectorTypes.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

using namespace GameAI;

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos,
	NavGraph* const pNavGraph, std::vector<FVector2D>& debugNodePositions, std::vector<NavLine>& debugPortals) 
{
	//Create the path to return
	std::vector<FVector2D> finalPath{};

	//Get the start and endTriangle
	auto const& startTriangle = pNavGraph->GetNavPolygon()->GetTriangleAtPosition(startPos, true);
	auto const& endTriangle = pNavGraph->GetNavPolygon()->GetTriangleAtPosition(endPos, true);

	if (!startTriangle || !endTriangle) return finalPath;

	if (startTriangle == endTriangle)
	{
		finalPath.push_back(startPos);
		finalPath.push_back(endPos);

		return finalPath;
	}

	//We have valid start/end triangles and they are not the same
	//=> Start looking for a path
	//Copy the graph
	auto pNavGraphClone = pNavGraph->Clone();

	//Create Extra node for the Start Node (Agent's position
	int const lineIdx{ -1 };

	auto startNode = std::make_unique<NavGraphNode>(startPos, lineIdx);
	int const startNodeId = pNavGraphClone->AddNode(std::move(startNode));

	for (auto const& edge : startTriangle->GetEdges())
	{
		std::optional<int> edgeIdx = pNavGraph->GetNavPolygon()->FindEdgeIndex(edge);
		if (edgeIdx.has_value())
		{
			int nodeId = pNavGraphClone->GetNodeIdFromEdgeIndex(edgeIdx.value());
			if (nodeId != Graphs::InvalidNodeId)
			{
				pNavGraphClone->AddConnection(startNodeId, nodeId);
			}
		}
	}

	//Create extra node for the endNode
	auto endNode = std::make_unique<NavGraphNode>(endPos, lineIdx);
	int const endNodeId = pNavGraphClone->AddNode(std::move(endNode));

	for (auto const& edge : endTriangle->GetEdges())
	{
		std::optional<int> edgeIdx = pNavGraph->GetNavPolygon()->FindEdgeIndex(edge);
		if (edgeIdx.has_value())
		{
			int nodeId = pNavGraphClone->GetNodeIdFromEdgeIndex(edgeIdx.value());
			if (nodeId != Graphs::InvalidNodeId)
			{
				pNavGraphClone->AddConnection(endNodeId, nodeId);
			}
		}
	}

	pNavGraphClone->SetConnectionCostsToDistances();

	//Run A star on new graph
	Node* pStartNode = pNavGraphClone->GetNode(startNodeId).get();
	Node* pEndNode = pNavGraphClone->GetNode(endNodeId).get();

	AStar pathFinder(pNavGraphClone.get(), HeuristicFunctions::Euclidean);
	std::vector<Node*> pPathNodes = pathFinder.FindPath(pStartNode, pEndNode);

	finalPath.reserve(pPathNodes.size());
	debugNodePositions.reserve(pPathNodes.size());
	for (const auto pPathNode : pPathNodes)
	{
		const auto nodePosition = pPathNode->GetPosition();

		finalPath.emplace_back(nodePosition);

		//Debug Visualisation
		debugNodePositions.emplace_back(nodePosition);
	}

	// Extra: Run optimiser on new graph (First check if everything works without SSFA!)
	// debugPortals = SSFA::FindPortals(nodes, *pNavGraph->GetNavPolygon());
	// finalPath = SSFA::OptimizePortals(debugPortals, *pNavGraph->GetNavPolygon());
	
	return finalPath;
}

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos, NavGraph* const pNavGraph)
{
	std::vector<FVector2D> debugNodePositions{};
	std::vector<NavLine> debugPortals{};

	return FindPath(startPos, endPos, pNavGraph, debugNodePositions, debugPortals);
}