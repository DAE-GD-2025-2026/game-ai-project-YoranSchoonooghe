#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"
#include "../SpacePartitioning/SpacePartitioning.h"


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
#ifdef GAMEAI_USE_SPACE_PARTITIONING
	pPartitionedSpace = std::make_unique<CellSpace>(
		pWorld,
		2 * WorldSize,
		2 * WorldSize,
		NrOfCellsX,
		NrOfCellsX,
		FlockSize
	);

	OldPositions.SetNum(FlockSize);

#else
	pNeighbors.SetNum(FlockSize);

#endif

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

		Agents[i]->SetDebugRenderingEnabled(false);

#ifdef GAMEAI_USE_SPACE_PARTITIONING
		pPartitionedSpace->AddAgent(*Agents[i]);
		OldPositions[i] = Agents[i]->GetPosition();
#endif
	}

	InitializeSteering();
}

Flock::~Flock()
{
 // TODO: Cleanup any additional data
	delete pCellSpace;
	pCellSpace = nullptr;
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

#ifdef GAMEAI_USE_SPACE_PARTITIONING
		
		pPartitionedSpace->RegisterNeighbors(*pAgent, NeighborhoodRadius);
		
		pPartitionedSpace->UpdateAgentCell(*pAgent, OldPositions[i]);
		OldPositions[i] = pAgent->GetPosition();

		if (i == 0)
		{
			pFirstAgentNeighbors = pPartitionedSpace->GetNeighbors();
			NrOfFirstAgentNeighbors = pPartitionedSpace->GetNrOfNeighbors();
		}
#else
		RegisterNeighbors(pAgent);
		
		if (i == 0)
		{
			pFirstAgentNeighbors = GetNeighbors();
			NrOfFirstAgentNeighbors = GetNrOfNeighbors();
		}
#endif


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

	DrawDebugPoint(pWorld, FVector(pAgentToEvade->GetPosition(), 20), 10, FColor::Cyan);

#ifdef GAMEAI_USE_SPACE_PARTITIONING
	if (DebugRenderPartitions)
	{
		pPartitionedSpace->RenderCells();
	}
#endif

	if (DebugRenderNeighborhood)
	{
		RenderNeighborhood();
	}
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
		if (ImGui::Checkbox("Render Agent Steering", &DebugRenderSteering))
		{
			Agents[0]->SetDebugRenderingEnabled(DebugRenderSteering);
		}
		
		ImGui::Checkbox("Render Agent Neighborhood", &DebugRenderNeighborhood);

#ifdef GAMEAI_USE_SPACE_PARTITIONING
		ImGui::Checkbox("Render Cell Space", &DebugRenderPartitions);
#endif

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
		
		ImGuiHelpers::ImGuiSliderFloatWithSetter("Seek",
			pBlendedSteering->GetWeightedBehaviorsRef()[4].Weight, 0.f, 1.f,
			[this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[4].Weight = InVal; }, "%.2f");

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Agent to Evade");
		ImGui::Spacing();
		
		if (ImGui::Checkbox("Render Steering", &DebugRenderEvadeSteering))
		{
			pAgentToEvade->SetDebugRenderingEnabled(DebugRenderEvadeSteering);
		}

		float v = pAgentToEvade->GetMaxLinearSpeed();
		if (ImGui::SliderFloat("Lin", &v, 0.f, 1000.f, "%.2f"))
			pAgentToEvade->SetMaxLinearSpeed(v);

  //End
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
 // TODO: Debugrender the neighbors for the first agent in the flock
	const auto& firstAgent = Agents[0];
	const auto& firstAgentPos = firstAgent->GetPosition();

	DrawDebugPoint(pWorld, FVector(firstAgent->GetPosition(), 20), 10, FColor::Yellow);

	DrawDebugCircle(
		pWorld,
		FVector(firstAgent->GetPosition(), 20),
		NeighborhoodRadius,
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

#ifdef GAMEAI_USE_SPACE_PARTITIONING
	DrawDebugBox(
		pWorld,
		FVector(firstAgentPos.X, firstAgentPos.Y, 20.f),
		FVector(NeighborhoodRadius, NeighborhoodRadius, 0.f),
		FColor::Blue
	);
#endif

	for (int i = 0; i < NrOfFirstAgentNeighbors; ++i)
	{
		const auto& pNeighbor = pFirstAgentNeighbors[i];
		if (pNeighbor != nullptr && pNeighbor != firstAgent)
		{
			DrawDebugPoint(pWorld, FVector(pNeighbor->GetPosition(), 20), 10, FColor::Green);
		}
	}
}

int Flock::GetNrOfNeighbors() const
{
#ifdef GAMEAI_USE_SPACE_PARTITIONING
	return pPartitionedSpace->GetNrOfNeighbors();
#else
	return NrOfNeighbors;
#endif
}

const TArray<ASteeringAgent*>& Flock::GetNeighbors() const
{
#ifdef GAMEAI_USE_SPACE_PARTITIONING
	return pPartitionedSpace->GetNeighbors();;
#else
	return pNeighbors;
#endif
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
	if (GetNrOfNeighbors() == 0)
	{
		return avgPosition;
	}

	for (int i = 0; i < GetNrOfNeighbors(); ++i)
	{
		avgPosition += GetNeighbors()[i]->GetPosition();
	}

	avgPosition /= static_cast<float>(GetNrOfNeighbors());

	return avgPosition;
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D avgVelocity = FVector2D::ZeroVector;

 // TODO: Implement
	if (GetNrOfNeighbors() == 0)
	{
		return avgVelocity;
	}

	for (int i = 0; i < GetNrOfNeighbors(); ++i)
	{
		FVector neighborVelocity = GetNeighbors()[i]->GetVelocity();
		avgVelocity += FVector2D(neighborVelocity.X, neighborVelocity.Y);
	}

	avgVelocity /= static_cast<float>(GetNrOfNeighbors());

	return avgVelocity;
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
 // TODO: Implement
	pSeekBehavior->SetTarget(Target);
}

void Flock::InitializeSteering()
{
	std::vector<BlendedSteering::WeightedBehavior> weightedBehaviors;

	pSeparationBehavior = std::make_unique<Separation>(this);
	pCohesionBehavior = std::make_unique<Cohesion>(this);
	pVelMatchBehavior = std::make_unique<VelocityMatch>(this);
	pWanderBehavior = std::make_unique<Wander>();
	pSeekBehavior = std::make_unique<Seek>();

	weightedBehaviors.emplace_back(pSeparationBehavior.get(), 1.0f);
	weightedBehaviors.emplace_back(pCohesionBehavior.get(), 1.0f);
	weightedBehaviors.emplace_back(pVelMatchBehavior.get(), 1.0f);
	weightedBehaviors.emplace_back(pWanderBehavior.get(), 0.3f);
	weightedBehaviors.emplace_back(pSeekBehavior.get(), 0.7f);

	pBlendedSteering = std::make_unique<BlendedSteering>(weightedBehaviors);

	pEvadeBehavior = std::make_unique<Evade>();

	std::vector<ISteeringBehavior*> priorityBehaviors;

	pPrioritySteering = std::make_unique<PrioritySteering>(priorityBehaviors);
	pPrioritySteering->AddBehaviour(pEvadeBehavior.get());
	pPrioritySteering->AddBehaviour(pBlendedSteering.get());
}

