// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_DirectorAI.h"

#include "NavigationSystem.h"
#include "AI/NavigationSystemBase.h"
#include "GraphTheory/Algorithms/AStar.h"
#include "GraphTheory/Algorithms/NavGraphPathfinding.h"
#include "NavMesh/RecastNavMesh.h"
#include "Runtime/Navmesh/Public/Detour/DetourNavMesh.h"
#include "Shared/GameAISpectator.h"

FORCEINLINE FVector RecastToUnreal(const double* RecastVertex)
{
	return FVector(
		static_cast<float>(-RecastVertex[0]),
		static_cast<float>(-RecastVertex[2]),
		static_cast<float>(RecastVertex[1])
	);
}

ALevel_DirectorAI::ALevel_DirectorAI()
{
	PrimaryActorTick.bCanEverTick = true;
}

void ALevel_DirectorAI::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	NavigationGraph->GetNavPolygon()->DrawDebug(GetWorld(), FColor::Yellow);

	for (const FVector& Vertex : NavigationGraph->GetNavPolygon()->GetVertices())
	{
		DrawDebugPoint(GetWorld(), Vertex, 10.0f, FColor::Cyan);
	}

	Renderer->RenderGraph(*NavigationGraph.get());

	UpdateImGui();
}

void ALevel_DirectorAI::BindLevelInputActions()
{
	Super::BindLevelInputActions();

	PlayerEnhancedInputComponent->BindAction(SetTargetAction, ETriggerEvent::Triggered,
		this, &ALevel_DirectorAI::SetTarget);
}

void ALevel_DirectorAI::BeginPlay()
{
	Super::BeginPlay();
	TrimWorld->bShouldTrimWorld = false;

	DirectorAI = GetWorld()->SpawnActor<ADirectorAI>(ADirectorAI::StaticClass());

	if (AGameAISpectator* Player = Cast<AGameAISpectator>(PlayerController->GetPawnOrSpectator()); Player)
	{
		Player->SetCameraProjection(ECameraProjectionMode::Orthographic);
	}

	Agent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{ 2100.0, 2100.0, 90 }, FRotator::ZeroRotator);
	Agent->SetDebugRenderingEnabled(false);
	Agent->SetSteeringBehavior(&PathFollow);

	auto NavPoly{ std::make_unique<TriPolygon>() };
	for (TArray<FVector> const& Tri : ExtractNavMeshTris())
	{
		NavPoly->AddTriangle(Tri);
	}

	NavigationGraph = std::make_unique<GameAI::NavGraph>(std::move(NavPoly));
	Renderer = std::make_unique<GameAI::GraphRenderer>(GetWorld());
	Renderer->SetRenderOptions(GameAI::GraphRenderOptions{
		true,
		false,
		false,
		true,
		false
		});
}

const char* ALevel_DirectorAI::ToString(ADirectorAI::State state)
{
	switch (state)
	{
	case ADirectorAI::State::Relax:
		return "Relax";
		break;
	case ADirectorAI::State::BuildUp:
		return "Build-Up";
		break;
	case ADirectorAI::State::Peak:
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

		ImGui::Text("Current state: %s", ToString(DirectorAI->GetState()));
		ImGui::Spacing();
		ImGui::Text("Time in current state: %.1f", DirectorAI->GetTimeInCurrentState());
		ImGui::Spacing();

		ImGui::Text("Intensity: %.1f", DirectorAI->GetIntensity());
		ImGui::Spacing();

		ImGui::Text("Parameters");

		ImGui::SliderInt("Health", DirectorAI->GetPlayerHealth(), 0, 100);
		ImGui::SliderInt("Damage", DirectorAI->GetRecentDamageTaken(), 0, 100);
		if (ImGui::IsItemHovered())
		{
			ImGui::SetTooltip("Recent Damage Taken by Player");
		}
		ImGui::SliderInt("Nearby Enemies", DirectorAI->GetNearbyEnemies(), 0, 100);

		//End
		ImGui::End();
	}
#pragma endregion
}

TArray<TArray<FVector>> ALevel_DirectorAI::ExtractNavMeshTris() const
{
	TArray<TArray<FVector>> Polys{};

	ANavigationData* NavData = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld())->GetDefaultNavDataInstance();
	if (dtNavMesh const* NavMesh = Cast<ARecastNavMesh>(NavData)->GetRecastMesh())
	{
		for (int TileIdx{ 0 }; TileIdx < NavMesh->getMaxTiles(); ++TileIdx)
		{
			dtMeshTile const* Tile{ NavMesh->getTile(TileIdx) };
			if (!Tile || !Tile->header || !Tile->polys) continue;

			for (int i = 0; i < Tile->header->detailMeshCount; ++i)
			{
				const dtPolyDetail* DetailMesh = &Tile->detailMeshes[i];
				const dtPoly* Poly = &Tile->polys[i];

				for (int triIdx = 0; triIdx < DetailMesh->triCount; ++triIdx)
				{
					const unsigned char* TriData = &Tile->detailTris[(DetailMesh->triBase + triIdx) * 4];

					TArray<FVector> TriVerts{};
					for (int corner = 0; corner < 3; ++corner)
					{
						unsigned char idx = TriData[corner];
						const double* Vert;

						if (idx < Poly->vertCount)
						{
							Vert = &Tile->verts[Poly->verts[idx] * 3];
						}
						else
						{
							int detailVertIdx = DetailMesh->vertBase + (idx - Poly->vertCount);
							Vert = &Tile->detailVerts[detailVertIdx * 3];
						}

						TriVerts.Add(RecastToUnreal(Vert));
					}
					Polys.Add(TriVerts);
				}
			}
		}
	}

	return Polys;
}

void ALevel_DirectorAI::SetTarget()
{
	GameAI::NavMeshPathfinding Pathfinder{};

	std::vector<FVector2D> debugNodePositions{};
	std::vector<GameAI::NavLine> debugPortals{};

	std::vector<FVector2D> Path = Pathfinder.FindPath(
		Agent->GetPosition(),
		FVector2D{ LatestMouseWorldPos },
		NavigationGraph.get(),
		debugNodePositions,
		debugPortals
	);

	PathFollow.SetPath(Path);
	if (Path.size() > 0)
	{
		Agent->SetPosition(Path[0]);
	}
}