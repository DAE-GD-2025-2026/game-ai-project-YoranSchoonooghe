#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"


//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput steering{};

	if (pFlock->GetNrOfNeighbors() == 0)
	{
		steering.IsValid = false;
		return steering;
	}

	steering.LinearVelocity = pFlock->GetAverageNeighborPos() - pAgent.GetPosition();
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent.GetMaxLinearSpeed();

	return steering;
}

//*********************
//SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput steering{};

	const int neighborCount = pFlock->GetNrOfNeighbors();

	if (neighborCount == 0)
	{
		steering.IsValid = false;
		return steering;
	}

	FVector2D separationForce = FVector2D::ZeroVector;
	const FVector2D agentPos = pAgent.GetPosition();

	const TArray<ASteeringAgent*>& neighbors = pFlock->GetNeighbors();

	for (int i = 0; i < neighborCount; ++i)
	{
		const FVector2D toAgent = agentPos - neighbors[i]->GetPosition();
		const float distance = toAgent.Length();

		if (distance > FLT_EPSILON)
		{
			separationForce += toAgent / distance;
		}
	}

	separationForce.Normalize();
	separationForce *= pAgent.GetOriginalMaxSpeed();

	steering.LinearVelocity = separationForce;

	return SteeringOutput();
}

//*************************
//VELOCITY MATCH (FLOCKING)
SteeringOutput VelocityMatch::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput steering{};

	if (pFlock->GetNrOfNeighbors() == 0)
	{
		steering.IsValid = false;
		return steering;
	}

	FVector2D avgVelocity = pFlock->GetAverageNeighborVelocity();

	avgVelocity.Normalize();
	avgVelocity *= pAgent.GetOriginalMaxSpeed();

	steering.LinearVelocity = avgVelocity;

	return steering;
}
