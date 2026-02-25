#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"


Flock::Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize,
	float WorldSize,
	ASteeringAgent* const pAgentToEvade,
	bool bTrimWorld)
	: pWorld{pWorld}
	, FlockSize{ FlockSize }
	, pAgentToEvade{pAgentToEvade}
{
	Agents.SetNum(FlockSize);

 // TODO: initialize the flock and the memory pool
	pNeighbors.SetNum(FlockSize);

	FActorSpawnParameters spawnParams;
	spawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

	for (int i = 0; i < FlockSize; ++i)
	{
		FVector spawnPos = FVector(
			FMath::RandRange(-WorldSize, WorldSize),
			FMath::RandRange(-WorldSize, WorldSize),
			90.f
		);

		Agents[i] = pWorld->SpawnActor<ASteeringAgent>(
			AgentClass,
			spawnPos,
			FRotator::ZeroRotator,
			spawnParams
		);
	}

	std::vector<BlendedSteering::WeightedBehavior> weightedBehaviors;

	pSeparationBehavior = std::make_unique<Separation>(this);
	pCohesionBehavior = std::make_unique<Cohesion>(this);
	pVelMatchBehavior = std::make_unique<VelocityMatch>(this);
	pWanderBehavior = std::make_unique<Wander>();

	weightedBehaviors.emplace_back(pSeparationBehavior.get(), 1.5f);
	weightedBehaviors.emplace_back(pCohesionBehavior.get(), 1.0f);
	weightedBehaviors.emplace_back(pVelMatchBehavior.get(), 1.0f);
	weightedBehaviors.emplace_back(pWanderBehavior.get(), 0.3f);

	pBlendedSteering = std::make_unique<BlendedSteering>(weightedBehaviors);

	pEvadeBehavior = std::make_unique<Evade>();

	std::vector<ISteeringBehavior*> priorityBehaviors;

	pPrioritySteering = std::make_unique<PrioritySteering>(priorityBehaviors);
	pPrioritySteering->AddBehaviour(pEvadeBehavior.get());
	pPrioritySteering->AddBehaviour(pBlendedSteering.get());
}

Flock::~Flock()
{
 // TODO: Cleanup any additional data
}

void Flock::Tick(float DeltaTime)
{
 // TODO: update the flock
 // TODO: for every agent:
  // TODO: register the neighbors for this agent (-> fill the memory pool with the neighbors for the currently evaluated agent)
  // TODO: update the agent (-> the steeringbehaviors use the neighbors in the memory pool)
  // TODO: trim the agent to the world
	for (int i = 0; i < FlockSize; ++i)
	{
		ASteeringAgent* pAgent = Agents[i];

		RegisterNeighbors(pAgent);

		Agents[i]->SetSteeringBehavior(pPrioritySteering.get());

		pAgent->Tick(DeltaTime);
	}

	if (pEvadeBehavior && pAgentToEvade)
	{
		FTargetData Target;
		Target.Position = pAgentToEvade->GetPosition();
		Target.Orientation = pAgentToEvade->GetRotation();
		Target.LinearVelocity = pAgentToEvade->GetLinearVelocity();
		Target.AngularVelocity = pAgentToEvade->GetAngularVelocity();

		pEvadeBehavior->SetTarget(Target);
	}
}

void Flock::RenderDebug()
{
 // TODO: Render all the agents in the flock
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	//UI
	{
		//Setup
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &bWindowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target");
		ImGui::Text("RMB: move cam.");
		ImGui::Text("Scrollwheel: zoom cam.");
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();

  // TODO: implement ImGUI checkboxes for debug rendering here

		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

  // TODO: implement ImGUI sliders for steering behavior weights here
	
		ImGuiHelpers::ImGuiSliderFloatWithSetter("Separation",
			pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight, 0.f, 1.f,
			[this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight = InVal; }, "%.2f");

		ImGuiHelpers::ImGuiSliderFloatWithSetter("Cohesion",
			pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight, 0.f, 1.f,
			[this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight = InVal; }, "%.2f");

		ImGuiHelpers::ImGuiSliderFloatWithSetter("Velocity Match",
			pBlendedSteering->GetWeightedBehaviorsRef()[2].Weight, 0.f, 1.f,
			[this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[2].Weight = InVal; }, "%.2f");
		
		ImGuiHelpers::ImGuiSliderFloatWithSetter("Wander",
			pBlendedSteering->GetWeightedBehaviorsRef()[3].Weight, 0.f, 1.f,
			[this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[3].Weight = InVal; }, "%.2f");

  //End
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
 // TODO: Debugrender the neighbors for the first agent in the flock
}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
 // TODO: Implement
	NrOfNeighbors = 0;

	const FVector2D agentPos = pAgent->GetPosition();

	for (int i = 0; i < FlockSize; ++i)
	{
		ASteeringAgent* pOther = Agents[i];

		if (pOther == pAgent) continue;

		const float distance = FVector2D::Distance(
			agentPos,
			pOther->GetPosition()
		);

		if (distance <= NeighborhoodRadius)
		{
			pNeighbors[NrOfNeighbors] = pOther;
			NrOfNeighbors++;
		}
	}
}
#endif

FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D avgPosition = FVector2D::ZeroVector;

 // TODO: Implement
	if (NrOfNeighbors == 0)
	{
		return avgPosition;
	}

	for (int i = 0; i < NrOfNeighbors; ++i)
	{
		avgPosition += pNeighbors[i]->GetPosition();
	}

	avgPosition /= static_cast<float>(NrOfNeighbors);

	return avgPosition;
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D avgVelocity = FVector2D::ZeroVector;

 // TODO: Implement
	if (NrOfNeighbors == 0)
	{
		return avgVelocity;
	}

	for (int i = 0; i < NrOfNeighbors; ++i)
	{
		FVector neighborVelocity = pNeighbors[i]->GetVelocity();
		avgVelocity += FVector2D(neighborVelocity.X, neighborVelocity.Y);
	}

	avgVelocity /= static_cast<float>(NrOfNeighbors);

	return avgVelocity;
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
 // TODO: Implement
}

