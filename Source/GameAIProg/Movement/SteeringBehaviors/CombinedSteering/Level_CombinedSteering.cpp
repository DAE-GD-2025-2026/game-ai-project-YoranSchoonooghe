#include "Level_CombinedSteering.h"

#include "imgui.h"


// Sets default values
ALevel_CombinedSteering::ALevel_CombinedSteering()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_CombinedSteering::BeginPlay()
{
	Super::BeginPlay();

	AddDrunkAgent();
	AddEvadingAgent();
}

void ALevel_CombinedSteering::BeginDestroy()
{
	Super::BeginDestroy();

}

void ALevel_CombinedSteering::AddDrunkAgent()
{
	m_pDrunkAgent = GetWorld()->SpawnActor<ASteeringAgent>(
		SteeringAgentClass,
		FVector{ 0,0,90 },
		FRotator::ZeroRotator
	);

	if (!IsValid(m_pDrunkAgent)) return;

	m_pDrunkSeekBehavior = new Seek();
	Wander* pWander = new Wander();

	std::vector<BlendedSteering::WeightedBehavior> weightedBehaviors;
	weightedBehaviors.emplace_back(m_pDrunkSeekBehavior, 0.5f);
	weightedBehaviors.emplace_back(pWander, 0.5f);

	m_pBlendedSteering = new BlendedSteering(weightedBehaviors);

	m_pDrunkAgent->SetSteeringBehavior(m_pBlendedSteering);
}

void ALevel_CombinedSteering::AddEvadingAgent()
{
	m_pEvadingAgent = GetWorld()->SpawnActor<ASteeringAgent>(
		SteeringAgentClass,
		FVector{ 500,0,90 },
		FRotator::ZeroRotator
	);

	if (!IsValid(m_pEvadingAgent)) return;

	Wander* pWander = new Wander();
	m_pEvadeBehavior = new Evade();

	std::vector<ISteeringBehavior*> priorityBehaviors;
	priorityBehaviors.emplace_back(m_pEvadeBehavior);
	priorityBehaviors.emplace_back(pWander);

	PrioritySteering* pPrioritySteering = new PrioritySteering(priorityBehaviors);

	m_pEvadingAgent->SetSteeringBehavior(pPrioritySteering);
}

// Called every frame
void ALevel_CombinedSteering::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
#pragma region UI
	//UI
	{
		//Setup
		bool windowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Game AI", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
	
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
		ImGui::Spacing();
	
		ImGui::Text("Flocking");
		ImGui::Spacing();
		ImGui::Spacing();
	
		if (ImGui::Checkbox("Debug Rendering", &CanDebugRender))
		{
   // TODO: Handle the debug rendering of your agents here :)
		}
		ImGui::Checkbox("Trim World", &TrimWorld->bShouldTrimWorld);
		if (TrimWorld->bShouldTrimWorld)
		{
			ImGuiHelpers::ImGuiSliderFloatWithSetter("Trim Size",
				TrimWorld->GetTrimWorldSize(), 1000.f, 3000.f,
				[this](float InVal) { TrimWorld->SetTrimWorldSize(InVal); });
		}
		
		ImGui::Spacing();
		ImGui::Spacing();
		ImGui::Spacing();
	
		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

		ImGuiHelpers::ImGuiSliderFloatWithSetter("Seek",
			m_pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight, 0.f, 1.f,
			[this](float InVal) { m_pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight = InVal; }, "%.2f");
		
		ImGuiHelpers::ImGuiSliderFloatWithSetter("Wander",
			m_pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight, 0.f, 1.f,
			[this](float InVal) { m_pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight = InVal; }, "%.2f");
	
		//End
		ImGui::End();
	}
#pragma endregion
	
	// Combined Steering Update
 // TODO: implement handling mouse click input for seek
	if (!m_pDrunkSeekBehavior) return;

	m_pDrunkSeekBehavior->SetTarget(MouseTarget);

 // TODO: implement Make sure to also evade the wanderer
	if (m_pEvadeBehavior && m_pDrunkAgent)
	{
		FTargetData Target;
		Target.Position = m_pDrunkAgent->GetPosition();
		Target.Orientation = m_pDrunkAgent->GetRotation();
		Target.LinearVelocity = m_pDrunkAgent->GetLinearVelocity();
		Target.AngularVelocity = m_pDrunkAgent->GetAngularVelocity();

		m_pEvadeBehavior->SetTarget(Target);
	}
}
