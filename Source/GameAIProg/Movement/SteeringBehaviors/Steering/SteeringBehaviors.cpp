#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//SEEK
//*******
// TODO: Do the Week01 assignment :^)

SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	steering.LinearVelocity = Target.Position - Agent.GetPosition();

	return steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	steering.LinearVelocity = Agent.GetPosition() - Target.Position;

	return steering;
}

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	steering.LinearVelocity = Target.Position - Agent.GetPosition();
	
	const float SLOW_RADIUS{ 500.f };
	const float TARGET_RADIUS{ 100.f };
	float distanceToTarget{};
	
	distanceToTarget = (Target.Position - Agent.GetPosition()).Length();

	if (distanceToTarget > SLOW_RADIUS)
	{
		Agent.SetMaxLinearSpeed(Agent.GetOriginalMaxSpeed());
	}
	else if (distanceToTarget < TARGET_RADIUS)
	{
		Agent.SetMaxLinearSpeed(0);
	}
	else
	{
		Agent.SetMaxLinearSpeed((distanceToTarget - TARGET_RADIUS) / (SLOW_RADIUS - TARGET_RADIUS) * Agent.GetOriginalMaxSpeed());
	}
	//DrawDebugDirectionalArrow()
	DrawDebugCircle(
		Agent.GetWorld(),
		FVector3d(Agent.GetPosition().X, Agent.GetPosition().Y, 0),
		TARGET_RADIUS,
		12,
		FColor::Orange,
		false,
		-1.0f,
		0U,
		0.0f,
		FVector(1, 0, 0),
		FVector(0, 1, 0),
		false
	);

	DrawDebugCircle(
		Agent.GetWorld(),
		FVector3d(Agent.GetPosition().X, Agent.GetPosition().Y, 0),
		SLOW_RADIUS,
		12,
		FColor::Blue,
		false,
		-1.0f,
		0U,
		0.0f,
		FVector(1, 0, 0),
		FVector(0, 1, 0),
		false
	);

	return steering;
}
