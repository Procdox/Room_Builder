// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ProceduralMeshComponent.h"
#include "../ThirdParty/ClipperLib/Includes/ClipperLib.h"
#include "../ThirdParty/DCEL_CUTS/Includes/DCEL_Region.h"
//#include "DCEL.h"
#include "room_description_builder.generated.h"

USTRUCT()
struct FRoom_Prototype {
	GENERATED_BODY()

		UPROPERTY(EditAnyWhere)
		TArray<FVector2D> Border;
	UPROPERTY(EditAnyWhere)
		float Bottom_Height;
	UPROPERTY(EditAnyWhere)
		float Top_Height;
	UPROPERTY(EditAnyWhere)
		TArray<FVector2D> Wall_Sections;
	UPROPERTY(EditAnyWhere)
		TArray<FVector2D> Door_Sections;
};

UCLASS()
class ROOM_BUILDER_API Aroom_description_builder : public AActor
{
	GENERATED_BODY()

	UPROPERTY(EditAnyWhere, Category = "gen_config")
		USceneComponent* root;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
		int32 room_count;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
		int32 area_factor;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
		int32 area_scale;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
		int32 room_factor;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
		int32 room_scale;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
		float min_ratio;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
		int32 min_area;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
		int32 min_width;

	UProceduralMeshComponent* CollisionMesh;

public:
	// Sets default values for this actor's properties
	Aroom_description_builder();

	void CreateDoor(Pint const & wall_left, Pint const & wall_right, float bottom, float top);

	void Create_Floor_Ceiling_New(Region const & source, float bottom, float top);
	void Create_Wall_Sections_New(Region const & source, float bottom, float top);

	void Main_Generation_Loop();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;



};
