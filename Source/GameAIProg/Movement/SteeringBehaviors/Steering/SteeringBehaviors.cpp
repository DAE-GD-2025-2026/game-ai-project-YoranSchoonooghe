#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//SEEK
//*******
// TODO: Do the Week01 assignment :^)

SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	steering.LinearVelocity = Target.Position - Agent.GetPosition();

	DrawDebugDirectionalArrow(
		Agent.GetWorld(),
		FVector3d(Agent.GetPosition(), 0),
		FVector3d(Agent.GetPosition(), 0) + Agent.GetMaxLinearSpeed() * Agent.GetActorForwardVector(),
		//FVector3d(Target.Position, 0),
		0,
		FColor::Magenta
	);

	DrawTarget(Agent);

	return steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	steering.LinearVelocity = Agent.GetPosition() - Target.Position;

	DrawTarget(Agent);

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

	DrawTarget(Agent);

	return steering;
}

SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	FVector forward = Agent.GetActorForwardVector();
	FVector toTarget = FVector(Target.Position - Agent.GetPosition(), 0);
	toTarget.Normalize();

	const float DOT_PRODUCT = FVector::DotProduct(forward, toTarget);
	const float MARGIN = 0.001f;

	if (DOT_PRODUCT < (1 - MARGIN))
	{
		const int DIRECTION = static_cast<int>(FVector::CrossProduct(forward, toTarget).Z / FVector::CrossProduct(forward, toTarget).Length());
		const float ANGULAR_VELOCITY = 5.0f;

		steering.AngularVelocity = DIRECTION * ANGULAR_VELOCITY;
	}
	else
	{
		steering.AngularVelocity = 0.0f;
	}

	DrawTarget(Agent);

	return steering;
}

SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	FVector2D predictedPosition;

	Agent.SetMaxLinearSpeed(Agent.GetOriginalMaxSpeed() / 2.f);

	const float timeToReachTarget = (Target.Position - Agent.GetPosition()).Length() / Agent.GetMaxLinearSpeed();

	predictedPosition = Target.Position + Target.LinearVelocity * timeToReachTarget;

	steering.LinearVelocity = predictedPosition - Agent.GetPosition();

	DrawTarget(Agent);
	DrawDebugPoint(Agent.GetWorld(), FVector(predictedPosition, 0), 5, FColor::Magenta);

	return steering;
}

void ISteeringBehavior::DrawTarget(ASteeringAgent& Agent)
{
	DrawDebugPoint(Agent.GetWorld(), FVector(Target.Position, 0), 5, FColor::Red);
}
