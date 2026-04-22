// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "DirectorAI.h"
#include "GraphTheory/Level_GraphTheory.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "GraphTheory/Algorithms/NavGraphPathfinding.h"
#include "Shared/Level_Base.h"
#include "Level_DirectorAI.generated.h"

UCLASS()
class GAMEAIPROG_API ALevel_DirectorAI : public ALevel_Base
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "NavmeshLevel|Input")
	UInputAction* SetTargetAction{};

	ALevel_DirectorAI();

	virtual void Tick(float DeltaTime) override;

	virtual void BindLevelInputActions() override;

protected:
	virtual void BeginPlay() override;

private:
	std::unique_ptr<GameAI::NavGraph> NavigationGraph;
	std::unique_ptr<GameAI::GraphRenderer> Renderer;

	UPROPERTY()
	ADirectorAI* DirectorAI{ nullptr };

	UPROPERTY()
	ASteeringAgent* Agent{ nullptr };
	PathFollow PathFollow{};

	const char* ToString(ADirectorAI::State state);

	void UpdateImGui();

	TArray<TArray<FVector>> ExtractNavMeshTris() const;

	void SetTarget();
};
