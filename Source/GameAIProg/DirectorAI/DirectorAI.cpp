// Fill out your copyright notice in the Description page of Project Settings.


#include "DirectorAI/DirectorAI.h"

// Sets default values
ADirectorAI::ADirectorAI()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ADirectorAI::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void ADirectorAI::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	UpdateIntensity();
	UpdateState(DeltaTime);
}

void ADirectorAI::UpdateIntensity()
{
	float intensity{ 0.0f };

	intensity += 0.4f * (100 - PlayerHealth);
	intensity += 0.3f * (std::clamp(RecentDamageTaken, 0, 100));
	intensity += 0.3f * (std::clamp(NearbyEnemies, 0, 100));

	IntensityLevel = std::clamp(intensity, 0.0f, 100.0f);
}

void ADirectorAI::UpdateState(float DeltaTime)
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
	case ADirectorAI::State::Relax:
		if (IntensityLevel < 20.0f)
			MoveToNextState();
		break;
	case ADirectorAI::State::BuildUp:
		if (IntensityLevel > 50.0f)
			MoveToNextState();
		break;
	case ADirectorAI::State::Peak:
		if (IntensityLevel > 80.0f)
			MoveToNextState();
		break;
	}

}

void ADirectorAI::MoveToNextState()
{
	int currentStateId = static_cast<int>(DirectorState);

	++currentStateId;
	currentStateId %= 3;

	DirectorState = static_cast<State>(currentStateId);

	TimeInCurrentState = 0.0f;
}