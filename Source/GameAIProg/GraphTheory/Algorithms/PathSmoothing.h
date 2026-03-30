#pragma once
#include <vector>

#include "NavGraphPathfinding.h"
#include "Movement/Pathfinding/Navmesh/TriPolygon.h"
#include "Shared/Graph/Graph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

namespace GameAI
{
	class SSFA final
{
public:
	//=== SSFA Functions ===
	//--- References ---
	//http://digestingduck.blogspot.be/2010/03/simple-stupid-funnel-algorithm.html
	//https://gamedev.stackexchange.com/questions/68302/how-does-the-simple-stupid-funnel-algorithm-work
	static std::vector<NavLine> FindPortals(std::vector<Node*> const & Path, TriPolygon const & NavPoly)
	{
		//Container
		std::vector<NavLine> Portals = {};
		
		if (Path.empty()) return Portals;

		NavLine startPortal{};
		startPortal.P1 = Path.front()->GetPosition();
		startPortal.P2 = Path.front()->GetPosition();
		Portals.push_back(startPortal);

		//For each node received, get it's corresponding line
		for (size_t i = 0; i < Path.size(); ++i)
		{
			const auto* navNode = dynamic_cast<const NavGraphNode*>(Path[i]);
			if (!navNode) continue;

			int edgeIdx = navNode->GetEdgeIdx();
			if (edgeIdx == -1) continue;

			const auto& edges = NavPoly.GetEdges();
			const auto& edge = edges[edgeIdx];

			//Redetermine it's "orientation" based on the required path (left-right vs right-left) - p1 should be right point
			FVector2D p1{ edge.GetP1(NavPoly).X, edge.GetP1(NavPoly).Y };
			FVector2D p2{ edge.GetP2(NavPoly).X, edge.GetP2(NavPoly).Y };

			FVector2D toNextNode;
			if (i + 1 < Path.size())
			{
				toNextNode = Path[i + 1]->GetPosition() - Path[i]->GetPosition();
			}
			else if (i > 0)
			{
				toNextNode = Path[i]->GetPosition() - Path[i - 1]->GetPosition();
			}
			else
			{
				continue;
			}

			FVector2D toP1 = p1 - Path[i]->GetPosition();

			float const crossProduct = FVector2D::CrossProduct(toNextNode, toP1);

			NavLine portal{};
			if (crossProduct < 0.0f)
			{
				portal.P1 = p1;
				portal.P2 = p2;
			}
			else
			{
				portal.P1 = p2;
				portal.P2 = p1;
			}

			//Store portal
			Portals.push_back(portal);
		}

		//Add degenerate portal to force end evaluation
		if (!Path.empty())
		{
			FVector2D endPos = Path.back()->GetPosition();
			Portals.push_back(NavLine{ endPos, endPos });
		}

		return Portals;
	}

	static std::vector<FVector2D> OptimizePortals(std::vector<NavLine> const & Portals, TriPolygon const & NavPoly)
	{
		std::vector<FVector2D> Path{};

		if (Portals.empty()) return Path;

		//P1 == right point of portal, P2 == left point of portal
		
		int apexIndex{ 0 };
		int leftLegIndex{ 1 };
		int rightLegIndex{ 1 };

		FVector2D apexPoint{ Portals[apexIndex].P1 };
		Path.push_back(apexPoint);

		FVector2D rightLeg{ Portals[rightLegIndex].P1 - apexPoint };
		FVector2D leftLeg{ Portals[leftLegIndex].P2 - apexPoint };

		for (size_t i = 1; i < Portals.size(); ++i)
		{
			// --- RIGHT CHECK ---
			// 1. See if moving funnel inwards - RIGHT
			FVector2D newRightLeg{ Portals[i].P1 - apexPoint };
			if (FVector2D::CrossProduct(rightLeg, newRightLeg) >= 0.0f)
			{
				// 2. See if new line degenerates a line segment - RIGHT
				if (FVector2D::CrossProduct(leftLeg, newRightLeg) > 0.0f)
				{
					// Leftleg becomes new apex point
					apexPoint = apexPoint + leftLeg;
					apexIndex = leftLegIndex;
					Path.push_back(apexPoint);

					// Calculate new legs (if not the end)
					i = apexIndex;
					rightLeg = Portals[i].P1 - apexPoint;
					leftLeg = Portals[i].P2 - apexPoint;
					leftLegIndex = i;
					rightLegIndex = i;
					continue;
				}
				else
				{
					rightLeg = newRightLeg;
					rightLegIndex = i;
				}
			}

			// --- LEFT CHECK ---
			// 1. See if moving funnel inwards - LEFT
			FVector2D newLeftLeg{ Portals[i].P2 - apexPoint };
			if (FVector2D::CrossProduct(leftLeg, newLeftLeg) <= 0.0f)
			{
				// 2. See if new line degenerates a line segment - LEFT
				if (FVector2D::CrossProduct(rightLeg, newLeftLeg) < 0.0f)
				{
					// Rightleg becomes new apex point
					apexPoint = apexPoint + rightLeg;
					apexIndex = rightLegIndex;
					Path.push_back(apexPoint);

					// Calculate new legs (if not the end)
					i = apexIndex;
					rightLeg = Portals[i].P1 - apexPoint;
					leftLeg = Portals[i].P2 - apexPoint;
					leftLegIndex = i;
					rightLegIndex = i;
					continue;
				}
				else
				{
					leftLeg = newLeftLeg;
					leftLegIndex = i;
				}
			}
		}

		// Add last path point
		Path.push_back(Portals.back().P1);

		return Path;
	}
private:
	SSFA() {};
	~SSFA() {};
};
}
