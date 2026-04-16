// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Shared/Level_Base.h"
#include "Level_DirectorAI.generated.h"

UCLASS()
class GAMEAIPROG_API ALevel_DirectorAI : public ALevel_Base
{
	GENERATED_BODY()

public:

	ALevel_DirectorAI();

	virtual void Tick(float DeltaTime) override;

protected:
	virtual void BeginPlay() override;

private:
	void UpdateIntensity();
	void UpdateDirectorState(float DeltaTime);
	void MoveToNextState();

	UPROPERTY()
	ASteeringAgent* Agent{ nullptr };

	enum class DirectorAIState
	{
		Relax,
		BuildUp,
		Peak
	};
	const char* ToString(DirectorAIState state);

	DirectorAIState DirectorState{ DirectorAIState::Relax };
	float MaxTimeInState{ 50.0f };
	float MinTimeInState{ 10.0f };
	float TimeInCurrentState{ 0.0f };

	float IntensityLevel{ 0 };
	
	int PlayerHealth{ 100 };
	int RecentDamageTaken{ 0 };
	int NearbyEnemies{ 0 };

	void UpdateImGui();

};
