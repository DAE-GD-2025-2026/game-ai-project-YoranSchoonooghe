#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//SEEK
//*******
// TODO: Do the Week01 assignment :^)

SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	steering.LinearVelocity = Target.Position - Agent.GetPosition();

	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugDirectionalArrow(
			Agent.GetWorld(),
			FVector3d(Agent.GetPosition(), 0),
			FVector3d(Agent.GetPosition(), 0) + Agent.GetMaxLinearSpeed() * Agent.GetActorForwardVector(),
			//FVector3d(Target.Position, 0),
			0,
			FColor::Magenta
		);

		DrawTarget(Agent);
	}

	return steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	steering.LinearVelocity = Agent.GetPosition() - Target.Position;

	if (Agent.GetDebugRenderingEnabled())
	{
		DrawTarget(Agent);
	}

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
	
	if (Agent.GetDebugRenderingEnabled())
	{
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
	}

	return steering;
}

SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	Agent.SetIsAutoOrienting(false);

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

	if (Agent.GetDebugRenderingEnabled())
	{
		DrawTarget(Agent);
	}

	return steering;
}

SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	FVector2D predictedPosition = CalculatePredictedPosition(Agent);

	steering.LinearVelocity = predictedPosition - Agent.GetPosition();

	if (Agent.GetDebugRenderingEnabled())
	{
		DrawTarget(Agent);
		DrawDebugPoint(Agent.GetWorld(), FVector(predictedPosition, 0), 5, FColor::Magenta);
	}

	return steering;
}

SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	FVector2D predictedPosition = CalculatePredictedPosition(Agent);

	const float EVADE_RADIUS{ 300.f };

	if ((predictedPosition - Agent.GetPosition()).Length() < EVADE_RADIUS)
	{
		steering.LinearVelocity = Agent.GetPosition() - predictedPosition;
	}
	else
	{
		steering.IsValid = false;
	}

	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugCircle(
			Agent.GetWorld(),
			FVector3d(Agent.GetPosition().X, Agent.GetPosition().Y, 0),
			EVADE_RADIUS,
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

		DrawDebugPoint(Agent.GetWorld(), FVector(predictedPosition, 0), 5, FColor::Magenta);
	}

	return steering;
}

SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	
	FVector2D circleCenter;

	circleCenter = Agent.GetPosition() + FVector2D(Agent.GetActorForwardVector().X, Agent.GetActorForwardVector().Y) * m_OffsetDistance;

	int angleChange = rand() % (2 * m_MaxAngleChange + 1) - m_MaxAngleChange;
	m_Angle = m_Angle + angleChange * PI / 180.0f;

	const FVector2D pointOnCircle = FVector2D(
		circleCenter.X + m_Radius * cosf(m_Angle),
		circleCenter.Y + m_Radius * sinf(m_Angle)
	);

	steering.LinearVelocity = pointOnCircle - Agent.GetPosition();

	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugCircle(
			Agent.GetWorld(),
			FVector(circleCenter, 0),
			m_Radius,
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

		DrawDebugPoint(Agent.GetWorld(), FVector(pointOnCircle, 0), 5, FColor::Red);
	}

	return steering;
}

FVector2D ISteeringBehavior::CalculatePredictedPosition(ASteeringAgent& Agent)
{
	FVector2D predictedPosition;

	const float MAX_TIME = { 0.7f };
	float timeToReachTarget = (Target.Position - Agent.GetPosition()).Length() / Agent.GetMaxLinearSpeed();
	timeToReachTarget = std::min(timeToReachTarget, MAX_TIME);

	predictedPosition = Target.Position + Target.LinearVelocity * timeToReachTarget;

	return predictedPosition;
}

void ISteeringBehavior::DrawTarget(ASteeringAgent& Agent)
{
	DrawDebugPoint(Agent.GetWorld(), FVector(Target.Position, 0), 5, FColor::Red);
}
