// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ProceduralMeshComponent.h"
#include "Grid_Region.h"
#include "room_description_builder.generated.h"

//Used to track regions within a DCEL and categorize them

struct Region_Suggestion {
	FLL<Pgrd> centroids;
	FLL<FLL<Pgrd> *> boundaries;

	bool contains(Pgrd const &test);
};

USTRUCT()
struct FBuild_Line {
	GENERATED_BODY()

	UPROPERTY(EditAnyWhere)
	FVector2D start;
	UPROPERTY(EditAnyWhere)
	FVector2D end;

	UPROPERTY(EditAnyWhere)
	bool start_row;
	UPROPERTY(EditAnyWhere)
	bool end_row;
};

struct rigid_line {
	Pgrd start;
	Pgrd end;

	bool start_row;
	bool end_row;

	rigid_line() {

	}
	rigid_line(FBuild_Line const &ref) {
		start = Pgrd(ref.start.X, ref.start.Y);
		end = Pgrd(ref.end.X, ref.end.Y);

		start_row = ref.start_row;
		end_row = ref.end_row;
	}
};

struct Type_Tracker {
	DCEL<Pgrd> * system;

	float min_room_width;
	float min_hall_width;

	FLL<Region<Pgrd> *> Exteriors;
	FLL<Region<Pgrd> *> Nulls;
	FLL<Region<Pgrd> *> Rooms;
	FLL<Region<Pgrd> *> Halls;
	FLL<Region<Pgrd> *> Smalls;

	bool isRoom(Region<Pgrd> const * target) {
		for (auto room : Rooms) {
			if (room == target) {
				return true;
			}
		}
		return false;
	}

	Type_Tracker(DCEL<Pgrd> * sys, float room, float hall) {
		system = sys;

		Exteriors.append(sys->region());

		min_room_width = room;
		min_hall_width = hall;
	}

	FLL<Region<Pgrd> *> createRoom(Region_Suggestion const &suggested);
	FLL<Region<Pgrd> *> createHall(Region_Suggestion const &suggested);
	FLL<Region<Pgrd> *> createNull(Region_Suggestion const &suggested);
};


UCLASS()
class ROOM_BUILDER_API Aroom_description_builder : public AActor
{
	GENERATED_BODY()

	UPROPERTY(EditAnyWhere, Category = "gen_config")
	bool use_static_seed;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
	int32 random_seed;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
	UMaterial* Wall_Material;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
	UMaterial* Floor_Material;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
	UMaterial* Ceiling_Material;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
	USceneComponent* root;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
	int32 rooms_per_segment;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
	int32 closets_per_segment;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
	double wall_thickness;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
	double room_height;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
	double door_height;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
	double door_width;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
	double min_room_width;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
	double min_hall_width;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
	double room_width;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
	double room_depth;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
	double hall_width;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
	TArray<FBuild_Line> Lines;

	UProceduralMeshComponent* CollisionMesh;

public:
	// Sets default values for this actor's properties
	Aroom_description_builder();

	UProceduralMeshComponent * CreateMeshComponent();
	void ActivateMeshComponent(UProceduralMeshComponent * component);

	void CreateWallSegment(Edge<Pgrd> const * target, float bottom, float top, UProceduralMeshComponent * component, int section_id);
	void CreateDoorSegment(Edge<Pgrd> const * target, float bottom, float top, UProceduralMeshComponent * component, int section_id);
	void CreateWindowSegment(Edge<Pgrd> const * target, float bottom, float top, UProceduralMeshComponent * component, int section_id);

	void CreateFloorAndCeiling(Region<Pgrd> * source, float bottom, float top);
	void CreateWallSections(Region<Pgrd> * source, float bottom, float top, Type_Tracker & tracker);

	void Create_System(Type_Tracker & tracker);

	void buldingFromBlock(Type_Tracker &frame, FLL<rigid_line> &list);

	void Main_Generation_Loop();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;
};
