#include "SpacePartitioning.h"

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
				col * CellWidth + widthOffset,
				row * CellHeight + heightOffset,
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

	Cells[cellIndex].Agents.push_back(&Agent);
}

void CellSpace::UpdateAgentCell(ASteeringAgent& Agent, const FVector2D& OldPos)
{
	//TODO Check if the agent needs to be moved to another cell.
	//TODO Use the calculated index for oldPos and currentPos for this
	int oldIndex = PositionToIndex(OldPos);
	int newIndex = PositionToIndex(Agent.GetPosition());

	if (oldIndex != newIndex)
	{

	}
}

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius)
{
	// TODO Register the neighbors for the provided agent
	// TODO Only check the cells that are within the radius of the neighborhood
}

void CellSpace::EmptyCells()
{
	for (Cell& c : Cells)
		c.Agents.clear();
}

void CellSpace::RenderCells() const
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

bool CellSpace::DoRectsOverlap(FRect const & RectA, FRect const & RectB)
{
	// Check if the rectangles are separated on either axis
	if (RectA.Max.X < RectB.Min.X || RectA.Min.X > RectB.Max.X) return false;
	if (RectA.Max.Y < RectB.Min.Y || RectA.Min.Y > RectB.Max.Y) return false;
    
	// If they are not separated, they must overlap
	return true;
}