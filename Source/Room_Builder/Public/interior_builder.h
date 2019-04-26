

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ProceduralMeshComponent.h"
#include "Grid_Region.h"
#include "interior_builder.generated.h"

USTRUCT()
struct FRoom_Details
{
	GENERATED_BODY()

	FRoom_Details();

	UPROPERTY(EditAnyWhere, Category = "gen_config")
	UMaterial* Wall_Material;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
	UMaterial* Floor_Material;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
	UMaterial* Ceiling_Material;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
	double wall_thickness;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
	double door_height;

	UPROPERTY(EditAnyWhere, Category = "gen_config")
	double door_width;
};

enum Wall_Type
{
	wall,
	door,
	window,
	open
};

struct Room_Boundary
{
	Room_Boundary(Face<Pgrd> * reference);

	bool walled;
	FLL<Pgrd> Points;
	FLL<Pgrd> Offsets;
	FLL<Wall_Type> Segment_Types;
};

struct Room_Layout
{
	Room_Layout(Region<Pgrd> * reference, double _bottom, double _top);

	FLL<Room_Boundary *> Boundaries;

	double bottom;
	double top;

	bool ceiling;
	bool floor;
};

UCLASS()
class ROOM_BUILDER_API Ainterior_builder : public AActor
{
	GENERATED_BODY()

	UPROPERTY(EditAnyWhere)
	USceneComponent* root;

	UProceduralMeshComponent* CollisionMesh;

public:
	Ainterior_builder();

	UProceduralMeshComponent * CreateMeshComponent();
	void ActivateMeshComponent(UProceduralMeshComponent * component);
	
	FRoom_Details const * details;
	Room_Layout const * layout;

	void CreateWallSegment(FLL<Pgrd>::FLL_iterator_c point, FLL<Pgrd>::FLL_iterator_c offset, UProceduralMeshComponent * component, int section_id);
	//void CreateDoorSegment(FLL<Pgrd>::FLL_iterator_c point, FLL<Pgrd>::FLL_iterator_c offset, UProceduralMeshComponent * component, int section_id);
	//void CreateWindowSegment(FLL<Pgrd>::FLL_iterator_c point, FLL<Pgrd>::FLL_iterator_c offset, UProceduralMeshComponent * component, int section_id);

	void CreateFloorAndCeiling();
	void CreateWallSections(Room_Boundary const * boundary);

	void Create(Room_Layout const * _layout, FRoom_Details const * _details);
};