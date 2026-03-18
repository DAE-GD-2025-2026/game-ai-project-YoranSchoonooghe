#include "BFS.h"

#include <map>
#include <queue>

#include "Shared/Graph/Graph.h"

using namespace GameAI;

BFS::BFS(Graph* const pGraph)
	: pGraph(pGraph)
{
}

// TODO Breath First Search Algorithm searches for a path from the startNode to the destinationNode
std::vector<Node*> BFS::FindPath(Node* const pStartNode, Node* const pDestinationNode) const
{
	std::vector<Node*> path;

    std::queue<Node*> openList;
    std::map<Node*, Node*> parent;
    std::vector<Node*> visited;

    openList.push(pStartNode);
    visited.push_back(pStartNode);

    Node* pCurrentNode = nullptr;
    bool foundGoal = false;

    while (!openList.empty())
    {
        pCurrentNode = openList.front();
        openList.pop();

        if (pCurrentNode == pDestinationNode)
        {
            foundGoal = true;
            break;
        }

        auto connections = pGraph->FindConnectionsFrom(pCurrentNode->GetId());
        for (auto const& pConnection : connections)
        {
            Node* pNextNode = pGraph->GetNode(pConnection->GetToId()).get();

            if (std::find(visited.begin(), visited.end(), pNextNode) == visited.end())
            {
                visited.push_back(pNextNode);
                parent[pNextNode] = pCurrentNode;
                openList.push(pNextNode);
            }
        }
    }

    if (foundGoal)
    {
        Node* pPathNode = pDestinationNode;
        while (pPathNode != pStartNode)
        {
            path.push_back(pPathNode);
            pPathNode = parent[pPathNode];
        }
        path.push_back(pStartNode);

        std::reverse(path.begin(), path.end());
    }

	return path;
}
