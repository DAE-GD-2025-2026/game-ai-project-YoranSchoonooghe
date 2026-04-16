// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_DirectorAI.h"

ALevel_DirectorAI::ALevel_DirectorAI()
{

}

void ALevel_DirectorAI::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	UpdateIntensity();
	UpdateDirectorState(DeltaTime);
	UpdateImGui();
}

void ALevel_DirectorAI::BeginPlay()
{
	Super::BeginPlay();

	Agent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{ 2100.0, 2100.0, 90 }, FRotator::ZeroRotator);
	Agent->SetDebugRenderingEnabled(false);
}

void ALevel_DirectorAI::UpdateIntensity()
{
	float intensity{ 0.0f };

	intensity += 0.4f * (100 - PlayerHealth);
	intensity += 0.3f * (std::clamp(RecentDamageTaken, 0, 100));
	intensity += 0.3f * (std::clamp(NearbyEnemies, 0, 100));

	IntensityLevel = std::clamp(intensity, 0.0f, 100.0f);
}

void ALevel_DirectorAI::UpdateDirectorState(float DeltaTime)
{
	TimeInCurrentState += DeltaTime;

	if (TimeInCurrentState < MinTimeInState) return;

	if (TimeInCurrentState >= MaxTimeInState)
	{
		MoveToNextState();

		return;
	}

	switch (DirectorState)
	{
	case ALevel_DirectorAI::DirectorAIState::Relax:
		if (IntensityLevel < 20.0f)
			MoveToNextState();
		break;
	case ALevel_DirectorAI::DirectorAIState::BuildUp:
		if (IntensityLevel > 50.0f)
			MoveToNextState();
		break;
	case ALevel_DirectorAI::DirectorAIState::Peak:
		if (IntensityLevel > 80.0f)
			MoveToNextState();
		break;
	}

}

void ALevel_DirectorAI::MoveToNextState()
{
	int currentStateId = static_cast<int>(DirectorState);

	++currentStateId;
	currentStateId %= 3;

	DirectorState = static_cast<DirectorAIState>(currentStateId);

	TimeInCurrentState = 0.0f;
}

const char* ALevel_DirectorAI::ToString(DirectorAIState state)
{
	switch (state)
	{
	case ALevel_DirectorAI::DirectorAIState::Relax:
		return "Relax";
		break;
	case ALevel_DirectorAI::DirectorAIState::BuildUp:
		return "Build-Up";
		break;
	case ALevel_DirectorAI::DirectorAIState::Peak:
		return "Peak";
		break;
	default:
		return "Invalid State";
		break;
	}
}

void ALevel_DirectorAI::UpdateImGui()
{
#pragma region UI
	//UI
	{
		//Setup
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: Set Target");
		ImGui::Unindent();

		/*Spacing*/ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing(); ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		/*Spacing*/ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing(); ImGui::Spacing();

		ImGui::Text("Director AI");
		ImGui::Spacing();

		ImGui::Text("Current state: %s", ToString(DirectorState));
		ImGui::Spacing();
		ImGui::Text("Time in current state: %.1f", TimeInCurrentState);
		ImGui::Spacing();

		ImGui::Text("Intensity: %.1f", IntensityLevel);
		ImGui::Spacing();

		ImGui::Text("Parameters");

		ImGui::SliderInt("Health", &PlayerHealth, 0, 100);
		ImGui::SliderInt("Damage", &RecentDamageTaken, 0, 100);
		if (ImGui::IsItemHovered())
		{
			ImGui::SetTooltip("Recent Damage Taken by Player");
		}
		ImGui::SliderInt("Nearby Enemies", &NearbyEnemies, 0, 100);

		//End
		ImGui::End();
	}
#pragma endregion
}