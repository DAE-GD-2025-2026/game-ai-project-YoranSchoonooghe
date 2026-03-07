#include "SpacePartitioning.h"
#include <string>

// --- Cell ---
// ------------
Cell::Cell(float Left, float Bottom, float Width, float Height)
{
	BoundingBox.Min = { Left, Bottom };
	BoundingBox.Max = { BoundingBox.Min.X + Width, BoundingBox.Min.Y + Height };
}

std::vector<FVector2D> Cell::GetRectPoints() const
{
	const float left = BoundingBox.Min.X;
	const float bottom = BoundingBox.Min.Y;
	const float width = BoundingBox.Max.X - BoundingBox.Min.X;
	const float height = BoundingBox.Max.Y - BoundingBox.Min.Y;

	std::vector<FVector2D> rectPoints =
	{
		{ left , bottom  },
		{ left , bottom + height  },
		{ left + width , bottom + height },
		{ left + width , bottom  },
	};

	return rectPoints;
}

// --- Partitioned Space ---
// -------------------------
CellSpace::CellSpace(UWorld* pWorld, float Width, float Height, int Rows, int Cols, int MaxEntities)
	: pWorld{pWorld}
	, SpaceWidth{Width}
	, SpaceHeight{Height}
	, NrOfRows{Rows}
	, NrOfCols{Cols}
	, NrOfNeighbors{0}
{
	Neighbors.SetNum(MaxEntities);
	
	//calculate bounds of a cell
	CellWidth = Width / Cols;
	CellHeight = Height / Rows;

	// TODO create the cells
	Cells.reserve(Rows * Cols);

	for (int row = 0; row < Rows; ++row)
	{
		for (int col = 0; col < Cols; ++col)
		{
			int index = row * Cols + col;

			const float widthOffset = -Width / 2;
			const float heightOffset = -Height / 2;

			Cells.emplace_back(
				row * CellHeight + heightOffset,
				col * CellWidth + widthOffset,
				CellWidth,
				CellHeight
			);
		}
	}
}

void CellSpace::AddAgent(ASteeringAgent& Agent)
{
	// TODO Add the agent to the correct cell
	int cellIndex = PositionToIndex(Agent.GetPosition());

	if (cellIndex == -1) return;

	Cells[cellIndex].Agents.push_back(&Agent);
}

void CellSpace::UpdateAgentCell(ASteeringAgent& Agent, const FVector2D& OldPos)
{
	//TODO Check if the agent needs to be moved to another cell.
	//TODO Use the calculated index for oldPos and currentPos for this
	FVector2D newPos = Agent.GetPosition();

	int oldIndex = PositionToIndex(OldPos);
	int newIndex = PositionToIndex(newPos);

	if (oldIndex != newIndex)
	{
		if (oldIndex != -1)
		{
			Cells[oldIndex].Agents.remove(&Agent);
		}

		if (newIndex != -1)
		{
			Cells[newIndex].Agents.push_back(&Agent);
		}
	}
}

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius)
{
	// TODO Register the neighbors for the provided agent
	// TODO Only check the cells that are within the radius of the neighborhood

	NrOfNeighbors = 0;

	const FVector2D agentPos = Agent.GetPosition();

	FRect neighborRect;
	neighborRect.Min = FVector2D(
		agentPos.X - QueryRadius,
		agentPos.Y - QueryRadius
	);
	neighborRect.Max = FVector2D(
		agentPos.X + QueryRadius,
		agentPos.Y + QueryRadius
	);

	for (const auto& cell : Cells)
	{
		if (!DoRectsOverlap(neighborRect, cell.BoundingBox)) continue;

		for (const auto& pAgent : cell.Agents)
		{
			if (pAgent == &Agent) continue;

			const float distance = FVector2D::Distance(
				agentPos,
				pAgent->GetPosition()
			);
			
			if (distance <= QueryRadius)
			{
				Neighbors[NrOfNeighbors] = pAgent;
				NrOfNeighbors++;
			}
		}
	}
}

void CellSpace::EmptyCells()
{
	for (Cell& c : Cells)
		c.Agents.clear();
}

void CellSpace::RenderCells(bool showCount) const
{
	// TODO Render the cells with the number of agents inside of it
	for (int index = 0; index < Cells.size(); ++index)
	{
		const auto& cell = Cells[index];

		const float CENTER_X = cell.BoundingBox.Min.X + CellWidth / 2;
		const float CENTER_Y = cell.BoundingBox.Min.Y + CellHeight / 2;

		DrawDebugBox(
			pWorld,
			FVector(CENTER_X, CENTER_Y, 20.f),
			FVector(CellWidth / 2, CellHeight / 2, 0.f),
			FColor::Blue
		);
		
		if (showCount)
		{
			FString AgentCountText = FString::Printf(TEXT("%lld"), (int64)cell.Agents.size());

			DrawDebugString(
				pWorld,
				FVector(CENTER_X, CENTER_Y, 20.f),
				AgentCountText,
				nullptr,
				FColor::Red,
				0.01f,
				false,
				1.5f
			);
		}
	}
}

void CellSpace::RenderOverlappingCells(const ASteeringAgent& Agent, float QueryRadius) const
{
	const FVector2D agentPos = Agent.GetPosition();

	FRect neighborRect;
	neighborRect.Min = FVector2D(
		agentPos.X - QueryRadius,
		agentPos.Y - QueryRadius
	);
	neighborRect.Max = FVector2D(
		agentPos.X + QueryRadius,
		agentPos.Y + QueryRadius
	);

	for (const auto& cell : Cells)
	{
		if (DoRectsOverlap(neighborRect, cell.BoundingBox))
		{
			const float CENTER_X = cell.BoundingBox.Min.X + CellWidth / 2;
			const float CENTER_Y = cell.BoundingBox.Min.Y + CellHeight / 2;

			DrawDebugSolidBox(
				pWorld,
				FVector(CENTER_X, CENTER_Y, 1.f),
				FVector(CellWidth / 2, CellHeight / 2, 0.f),
				FColor::Emerald
			);
		}
	}
}

int CellSpace::PositionToIndex(FVector2D const & Pos) const
{
	// TODO Calculate the index of the cell based on the position
	for (int index = 0; index < Cells.size(); ++index)
	{
		const auto& cell = Cells[index];

		if (Pos.X < cell.BoundingBox.Min.X || Pos.X > cell.BoundingBox.Max.X)
			continue;
		if (Pos.Y < cell.BoundingBox.Min.Y || Pos.Y > cell.BoundingBox.Max.Y)
			continue;

		return index;
	}

	return -1;
}

bool CellSpace::DoRectsOverlap(FRect const & RectA, FRect const & RectB) const
{
	// Check if the rectangles are separated on either axis
	if (RectA.Max.X < RectB.Min.X || RectA.Min.X > RectB.Max.X) return false;
	if (RectA.Max.Y < RectB.Min.Y || RectA.Min.Y > RectB.Max.Y) return false;
    
	// If they are not separated, they must overlap
	return true;
}