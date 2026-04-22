// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "DirectorAI.generated.h"

UCLASS()
class GAMEAIPROG_API ADirectorAI : public AActor
{
	GENERATED_BODY()
	
public:	
	ADirectorAI();

	virtual void Tick(float DeltaTime) override;

	enum class State
	{
		Relax,
		BuildUp,
		Peak
	};

	State GetState() const { return DirectorState; };
	float GetTimeInCurrentState() const { return TimeInCurrentState; };
	float GetIntensity() const { return IntensityLevel; };
	int* GetPlayerHealth() { return &PlayerHealth; };
	int* GetRecentDamageTaken() { return &RecentDamageTaken; };
	int* GetNearbyEnemies() { return &NearbyEnemies; };

protected:
	virtual void BeginPlay() override;

private:
	void UpdateIntensity();
	void UpdateState(float DeltaTime);
	void MoveToNextState();

	State DirectorState{ State::Relax };
	float MaxTimeInState{ 50.0f };
	float MinTimeInState{ 10.0f };
	float TimeInCurrentState{ 0.0f };

	float IntensityLevel{ 0 };

	int PlayerHealth{ 100 };
	int RecentDamageTaken{ 0 };
	int NearbyEnemies{ 0 };
};
