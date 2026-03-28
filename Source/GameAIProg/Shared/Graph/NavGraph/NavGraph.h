#pragma once
#include "Movement/Pathfinding/Navmesh/TriPolygon.h"
#include "Shared/Graph/Graph.h"

namespace GameAI
{
	class NavGraph : public Graph
	{
	public:
		explicit NavGraph(std::unique_ptr<TriPolygon> && NavPoly);
		NavGraph(const NavGraph& Other);
		
		std::unique_ptr<NavGraph> Clone() const;
		
		TriPolygon const * GetNavPolygon() const {return pNavPoly.get();}
		int GetNodeIdFromEdgeIndex(int EdgeIdx) const;
		std::vector<TriPolygon::Triangle> GetTrianglesFromEdge(const TriPolygon::Edge& edge) const;
		
	private:
		std::unique_ptr<TriPolygon> pNavPoly;

		void CreateNavigationGraph();
	};
}
