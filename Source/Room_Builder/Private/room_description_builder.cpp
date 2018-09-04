// Fill out your copyright notice in the Description page of Project Settings.

#include "room_description_builder.h"
#include "Algo/Reverse.h"
#include "DrawDebugHelpers.h"


enum edge_correction { null_static, null_intersect, null_atvertex, connected_static, connected_intersect, connected_atvertex };


//find the order of the set of edges st:
//	for an edge
//		base_vertex.y != end_vertex.y
//		contain a point [x,y] 
//			in their [base_vertex, end_vertex) range where 
//				X is greater than test_point.x
//				y = test_point.y
//an odd parity implies the point is interior
//an even parity implites

//==========================================================================================================
//========================================= utilities ======================================================
//==========================================================================================================


void Draw_Border(const TArray<FVector2D> &border, float height, const UWorld *ref) {
	float offset = 0;
	for (int index = 0; index < border.Num(); index++) {
		int next = (index + 1) % border.Num();

		DrawDebugLine(
			ref,
			FVector(border[index], height + offset * 5),
			FVector(border[next], height + offset * 5),
			//FColor(FMath::RandRange(0,255), FMath::RandRange(0, 255), FMath::RandRange(0, 255)),
			FColor(255, 0, 100),
			true,
			-1,
			0,
			5
		);
		//offset++;
	}
}


//==========================================================================================================
//======================================= array transforms =================================================
//==========================================================================================================


ClipperLib::Path to_Path(const TArray<FVector2D> &source) {
	ClipperLib::Path result;

	for (auto& vector : source) {
		result.push_back(ClipperLib::IntPoint(vector.X, vector.Y));
	}

	return result;
};
ClipperLib::Path to_Path(const TArray<_P> &source) {
	ClipperLib::Path result;

	for (auto& vector : source) {
		result.push_back(ClipperLib::IntPoint(vector.X, vector.Y));
	}

	return result;
};

ClipperLib::Paths to_Paths(const F_DCEL::Face* source) {
	ClipperLib::Paths result;
	result.push_back(to_Path(source->getRootEdge()->listPoints()));

	int holes = source->getHoleCount();

	for (int kk = 0; kk < holes; kk++) {
		result.push_back(to_Path(source->getHole(kk)->listPoints()));
	}

	return result;
};

TArray<FVector2D> to_FVector(const ClipperLib::Path &source) {
	TArray<FVector2D> result;

	for (auto& point : source) {
		result.Push(FVector2D(point.X, point.Y));
	}

	return result;
};
TArray<FVector2D> to_FVector(const TArray<_P> &source) {
	TArray<FVector2D> result;

	for (auto& vector : source) {
		result.Push(FVector2D(vector.X, vector.Y));
	}

	return result;
}

TArray<_P> to_Primitive(const ClipperLib::Path &source) {
	TArray<_P> result;

	for (auto& vector : source) {
		result.Push(_P(vector.X, vector.Y));
	}

	return result;
}


//==========================================================================================================
//==================================== sets default values =================================================
//==========================================================================================================

Aroom_description_builder::Aroom_description_builder()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	root = CreateDefaultSubobject<USceneComponent>(TEXT("GeneratedRoot"));
	CollisionMesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("CollisionMesh"));
	CollisionMesh->AttachToComponent(root, FAttachmentTransformRules::KeepRelativeTransform);
	CollisionMesh->ContainsPhysicsTriMeshData(true);
	CollisionMesh->bUseAsyncCooking = true;

	RootComponent = root;

	room_count = 40;

	area_factor = 4;
	area_scale = 300;

	room_factor = 2;
	room_scale = 200;

	min_ratio = .5;
	min_area = 300;

	min_width = 150;
}


//==========================================================================================================
//=================================== triangulate utilities ================================================
//==========================================================================================================

bool TryIntersect(const FVector2D &A_S, const FVector2D &A_E, const FVector2D &B_S, const FVector2D &B_E) {
	double ua, ub, denom;
	denom = (B_E.Y - B_S.Y)*(A_E.X - A_S.X) - (B_E.X - B_S.X)*(A_E.Y - A_S.Y);
	if (denom == 0) {
		return false;
	}
	ua = ((B_E.X - B_S.X)*(A_S.Y - B_S.Y) - (B_E.Y - B_S.Y)*(A_S.X - B_S.X)) / denom;
	ub = ((A_E.X - A_S.X)*(A_S.Y - B_S.Y) - (A_E.Y - A_S.Y)*(A_S.X - B_S.X)) / denom;

	return (ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1);
}
bool IsConcave(const FVector2D &A, const FVector2D &B, const FVector2D &C) {
	FVector2D In = B - A;
	FVector2D Out = C - B;
	FVector2D OutCCW(-Out.Y, Out.X);

	return (FVector2D::DotProduct(In, OutCCW)<0);
}
template <class T>
TArray<T> Reverse_Copy_Buffer(const TArray<T> &Buffer) {
	const int32 size = Buffer.Num();

	TArray<T> product;
	product.SetNum(size);

	for (int32 ii = 0; ii < size; ii++) {
		product[size - ii - 1] = Buffer[ii];
	}

	return product;
}

TArray<int32> Triangulate_Sequence(const TArray<FVector2D> &VectorBuffer, TArray<int32> &IndexBuffer) {
	/*  triangulate via ear clipping  */
	/*  requires intersect and concavity check  */
	TArray<int32> Triangles;
	int32 frozen = 0;
	{
		int32 tri_A = 0;
		int32 tri_B, tri_C, segment_start_index, segment_end_index;
		bool success = false;

		while (IndexBuffer.Num() > 2) {
			tri_B = (tri_A + 1) % IndexBuffer.Num();
			tri_C = (tri_A + 2) % IndexBuffer.Num();
			segment_start_index = IndexBuffer[tri_A];
			segment_end_index = IndexBuffer[tri_C];
			success = true;

			//is an ear?
			if (!IsConcave(VectorBuffer[segment_start_index], VectorBuffer[IndexBuffer[tri_B]], VectorBuffer[segment_end_index])) {
				success = false;
			}
			else {
				//preserves planarity?
				for (int32 index_ex = 0; index_ex < IndexBuffer.Num(); index_ex++) {
					int32 test_start_index = IndexBuffer[index_ex];
					int32 test_end_index = IndexBuffer[(index_ex + 1) % (IndexBuffer.Num())];

					if (test_start_index == segment_start_index ||
						test_end_index == segment_start_index ||
						test_end_index == segment_end_index ||
						test_start_index == segment_end_index) {
						continue;
					}

					if (TryIntersect(VectorBuffer[segment_start_index], VectorBuffer[segment_end_index],
						VectorBuffer[test_start_index], VectorBuffer[test_end_index])) {

						success = false;
						break;
					}

				}
			}

			if (success) {

				//remove point
				Triangles.Add(IndexBuffer[tri_A]);
				Triangles.Add(IndexBuffer[tri_B]);
				Triangles.Add(IndexBuffer[tri_C]);

				IndexBuffer.RemoveAt(tri_B);

				tri_A = tri_A - 1;
				if (tri_A < 0) {
					tri_A = IndexBuffer.Num() - 1;
				}
				frozen = 0;
			}
			else {
				tri_A = (tri_A + 1) % IndexBuffer.Num();
				frozen++;
				if (frozen > IndexBuffer.Num()) {
					//if (GEngine)
					//	GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, TEXT("Failed to Triangulate destructible mesh!"));
					break;
				}
			}
		}
	}
	return Triangles;
}

//==========================================================================================================
//======================================== creation ========================================================
//==========================================================================================================

void Aroom_description_builder::CreateDoor(const FVector2D &wall_left, const FVector2D &wall_right, float Bottom, float Top) {
	UProceduralMeshComponent* Wall_Mesh = NewObject<UProceduralMeshComponent>();
	Wall_Mesh->AttachToComponent(root, FAttachmentTransformRules::KeepRelativeTransform);
	Wall_Mesh->ContainsPhysicsTriMeshData(true);
	Wall_Mesh->bUseAsyncCooking = true;

	TArray<FVector> Vertices;
	TArray<int32> Triangles;
	TArray<FVector> Normals;
	TArray<FVector2D> UV0;
	TArray<FProcMeshTangent> Tangents;
	TArray<FLinearColor> VertexColors;

	FVector2D Mid = (wall_left + wall_right) / 2;
	FVector2D dir = wall_left - wall_right;
	dir.Normalize();
	dir *= 30;//door radial width
	FVector2D doorframe_left = Mid + dir;
	FVector2D doorframe_right = Mid - dir;

	Vertices.Push(FVector(wall_left, Bottom));
	Vertices.Push(FVector(wall_left, Top));
	Vertices.Push(FVector(doorframe_left, Bottom));
	Vertices.Push(FVector(doorframe_left, Top));
	Vertices.Push(FVector(doorframe_right, Bottom));
	Vertices.Push(FVector(doorframe_right, Top));
	Vertices.Push(FVector(wall_right, Bottom));
	Vertices.Push(FVector(wall_right, Top));

	Triangles.Push(7);
	Triangles.Push(5);
	Triangles.Push(6);
	Triangles.Push(6);
	Triangles.Push(5);
	Triangles.Push(4);

	Triangles.Push(3);
	Triangles.Push(1);
	Triangles.Push(2);
	Triangles.Push(2);
	Triangles.Push(1);
	Triangles.Push(0);

	FVector2D normal(dir.Y, -dir.X);
	Normals.Push(FVector(normal, 0));
	Normals.Push(FVector(normal, 0));
	Normals.Push(FVector(normal, 0));
	Normals.Push(FVector(normal, 0));
	Normals.Push(FVector(normal, 0));
	Normals.Push(FVector(normal, 0));
	Normals.Push(FVector(normal, 0));
	Normals.Push(FVector(normal, 0));

	UV0.Push(FVector2D(0, Bottom));
	UV0.Push(FVector2D(0, Top));
	UV0.Push(FVector2D(1, Bottom));
	UV0.Push(FVector2D(1, Top));
	UV0.Push(FVector2D(0, Bottom));
	UV0.Push(FVector2D(0, Top));
	UV0.Push(FVector2D(1, Bottom));
	UV0.Push(FVector2D(1, Top));

	Tangents.Push(FProcMeshTangent(0, 0, 1));
	Tangents.Push(FProcMeshTangent(0, 0, 1));
	Tangents.Push(FProcMeshTangent(0, 0, 1));
	Tangents.Push(FProcMeshTangent(0, 0, 1));
	Tangents.Push(FProcMeshTangent(0, 0, 1));
	Tangents.Push(FProcMeshTangent(0, 0, 1));
	Tangents.Push(FProcMeshTangent(0, 0, 1));
	Tangents.Push(FProcMeshTangent(0, 0, 1));

	auto color = FLinearColor();
	color.MakeRandomColor();
	color.A = 1.0;
	VertexColors.Push(color);
	VertexColors.Push(color);
	VertexColors.Push(color);
	VertexColors.Push(color);
	VertexColors.Push(color);
	VertexColors.Push(color);
	VertexColors.Push(color);
	VertexColors.Push(color);


	const float door_height = 110;
	if (Top - Bottom >= door_height) {
		Vertices.Push(FVector(doorframe_left, Bottom + door_height));
		Vertices.Push(FVector(doorframe_right, Bottom + door_height));

		Triangles.Push(5);
		Triangles.Push(3);
		Triangles.Push(9);
		Triangles.Push(9);
		Triangles.Push(3);
		Triangles.Push(8);

		Normals.Push(FVector(normal, 0));
		Normals.Push(FVector(normal, 0));

		float uv_ratio = door_height / (Top - Bottom);
		UV0.Push(FVector2D(0, uv_ratio));
		UV0.Push(FVector2D(1, uv_ratio));

		Tangents.Push(FProcMeshTangent(0, 0, 1));
		Tangents.Push(FProcMeshTangent(0, 0, 1));

		VertexColors.Push(color);
		VertexColors.Push(color);
	}

	Wall_Mesh->CreateMeshSection_LinearColor(0, Vertices, Triangles, Normals, UV0, VertexColors, Tangents, true);
	Wall_Mesh->ContainsPhysicsTriMeshData(true);

	Wall_Mesh->Activate();

	Wall_Mesh->RegisterComponentWithWorld(GetWorld());
}
void Aroom_description_builder::Create_Floor_Ceiling_New(const F_DCEL::Face* source, float Bottom, float Top) {
	auto Border = to_FVector(source->getRootEdge()->listPoints());
	TArray<int32> Index_Faked;
	Index_Faked.SetNum(Border.Num());
	for (int32 ii = 0; ii < Border.Num(); ii++) {
		Index_Faked[ii] = ii;
	}
	TArray<int32> Triangles_top = Triangulate_Sequence(Border, Index_Faked);
	TArray<int32> Triangles_bottom = Reverse_Copy_Buffer(Triangles_top);

	UProceduralMeshComponent* Floor_Mesh = NewObject<UProceduralMeshComponent>();
	Floor_Mesh->AttachToComponent(root, FAttachmentTransformRules::KeepRelativeTransform);
	Floor_Mesh->ContainsPhysicsTriMeshData(true);
	Floor_Mesh->bUseAsyncCooking = true;

	TArray<FVector> vertices_bottom;
	TArray<FVector> vertices_top;
	TArray<FVector> normals_bottom;
	TArray<FVector> normals_top;
	TArray<FVector2D> UV0_bottom;
	TArray<FVector2D> UV0_top;
	TArray<FProcMeshTangent> tangents_bottom;
	TArray<FProcMeshTangent> tangents_top;
	TArray<FLinearColor> vertexColors_bottom;
	TArray<FLinearColor> vertexColors_top;

	for (auto& vector : Border) {
		vertices_bottom.Add(FVector(vector.X, vector.Y, Bottom));
		vertices_top.Add(FVector(vector.X, vector.Y, Top));
		normals_bottom.Add(FVector(0, 0, 1));
		normals_top.Add(FVector(0, 0, -1));
		UV0_bottom.Add(vector / 10);
		UV0_top.Add(vector / 10);
		tangents_bottom.Add(FProcMeshTangent(0, 1, 0));
		tangents_top.Add(FProcMeshTangent(0, 1, 0));
		vertexColors_bottom.Add(FLinearColor(0.75, 0.75, 0.75, 1.0));
		vertexColors_top.Add(FLinearColor(0.75, 0.75, 0.75, 1.0));
	}

	Floor_Mesh->CreateMeshSection_LinearColor(0, vertices_bottom, Triangles_bottom, normals_bottom, UV0_bottom, vertexColors_bottom, tangents_bottom, true);
	Floor_Mesh->CreateMeshSection_LinearColor(1, vertices_top, Triangles_top, normals_top, UV0_top, vertexColors_top, tangents_top, true);

	Floor_Mesh->RegisterComponentWithWorld(GetWorld());

}
void Aroom_description_builder::Create_Wall_Sections_New(const F_DCEL::Face* source, float Bottom, float Top, int &h) {
	{
		auto Border = to_FVector(source->getRootEdge()->listPoints());
		Draw_Border(Border, Top, GetWorld());
		TArray<FVector2D> Wall_Sections;
		Wall_Sections.Push(Border[0]);
		for (int ii = 1; ii < Border.Num(); ii++) {
			Wall_Sections.Push(Border[ii]);
			Wall_Sections.Push(Border[ii]);
		}
		Wall_Sections.Push(Border[0]);

		const int32 size = Wall_Sections.Num() / 2;
		for (int32 ii = 0; ii < size; ii++) {
			int32 a = ii * 2;

			CreateDoor(Wall_Sections[a + 1], Wall_Sections[a], Bottom, Top);
		}
	}
	for(int ii = 0; ii < source->getHoleCount(); ii++){

		auto Border = to_FVector(source->getHole(ii)->listPoints());
		Draw_Border(Border, Top, GetWorld());
		TArray<FVector2D> Wall_Sections;
		Wall_Sections.Push(Border[0]);
		for (int ii = 1; ii < Border.Num(); ii++) {
			Wall_Sections.Push(Border[ii]);
			Wall_Sections.Push(Border[ii]);
		}
		Wall_Sections.Push(Border[0]);

		const int32 size = Wall_Sections.Num() / 2;
		for (int32 ii = 0; ii < size; ii++) {
			int32 a = ii * 2;

			CreateDoor(Wall_Sections[a + 1], Wall_Sections[a], Bottom, Top);
		}
	}
}

/*
bool Are_Parrallel(const room_model::wall_segment &A, const room_model::wall_segment &B) {
FVector2D A_normal = A.a - A.b;
A_normal.Normalize();
FVector2D B_normal = B.a - B.b;
B_normal.Normalize();

return (A_normal.Equals(B_normal) || A_normal.Equals(-B_normal));
}

bool Are_Touching(const room_model::wall_segment &A, const room_model::wall_segment &B) {
if (A.elevation == B.elevation) {
if (A.a.Equals(B.a) || A.a.Equals(B.b) || A.b.Equals(B.a) || A.b.Equals(B.b)) {
return true;
}
}
return false;
//TODO deal with multi-floor touching
}

struct true_wall_group {
TArray<room_model::wall_segment*> walls;
};

struct possible_wall_groups {
TArray<true_wall_group*> groups;
};
*/

TArray<_P> Generate_Boundary(float x, float y, const _P &center, int bevel_level) {
	TArray<_P> boundary;

	int bevel_bl = FMath::FRandRange(0, bevel_level);
	int bevel_tl = FMath::FRandRange(0, bevel_level);
	int bevel_tr = FMath::FRandRange(0, bevel_level);
	int bevel_br = FMath::FRandRange(0, bevel_level);

	if (bevel_bl > 0) {
		boundary.Push(_P(bevel_bl * 100 - x, -y) + center);
		boundary.Push(_P(-x, bevel_bl * 100 - y) + center);
	}
	else {
		boundary.Push(_P(-x, -y) + center);
	}

	if (bevel_tl > 0) {
		boundary.Push(_P(-x, y - bevel_tl * 100) + center);
		boundary.Push(_P(bevel_tl * 100 - x, y) + center);
	}
	else {
		boundary.Push(_P(-x, y) + center);
	}

	if (bevel_tr > 0) {
		boundary.Push(_P(x - bevel_tr * 100, y) + center);
		boundary.Push(_P(x, y - bevel_tr * 100) + center);
	}
	else {
		boundary.Push(_P(x, y) + center);
	}

	if (bevel_br > 0) {
		boundary.Push(_P(x, bevel_br * 100 - y) + center);
		boundary.Push(_P(x - bevel_br * 100, -y) + center);
	}
	else {
		boundary.Push(_P(x, -y) + center);
	}

	return boundary;
}

bool Analyze_Suggested(const ClipperLib::Paths &source) {

	//small inset
	//this prevents the area from being seperable by a small region, prevents against chokes
	//later, this should attempt to decompose such an area into smaller suggestions
	{
		#define test_width 50

		ClipperLib::ClipperOffset clipper;

		ClipperLib::Paths result;

		clipper.AddPaths(source, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
		clipper.Execute(result, -test_width / 2);

		if (result.size() != 1) {
			return false;
		}
	}
	//large inset
	//this prevents the area from being to narrow entirely, this also guarantees the [  area > 4*(test_radius)^2  ]
	//this should be changed for hallways
	{
		#define test_radius 50

		ClipperLib::ClipperOffset clipper;

		ClipperLib::Paths result;

		clipper.AddPaths(source, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
		clipper.Execute(result, -test_radius);

		if (result.size() < 1) {
			return false;
		}
	}

	return true;
}

bool Cull_Suggested(F_DCEL::Face* target, TArray<F_DCEL::Face*> &results, TArray<F_DCEL::Face*> &nulls) {
	
	//cull regions that are to narrow by restricting the min width
	ClipperLib::Paths passable;
	ClipperLib::Paths negatives;
	{
#define min_width 100

		ClipperLib::Paths source = to_Paths(target);

		ClipperLib::ClipperOffset clipper_reducer;
		ClipperLib::ClipperOffset clipper_expander;
		ClipperLib::Clipper clipper_subtract;
		
		ClipperLib::Paths reduced;

		clipper_reducer.AddPaths(source, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
		clipper_reducer.Execute(reduced, -min_width / 2);
		
		clipper_expander.AddPaths(reduced, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
		clipper_expander.Execute(passable, min_width / 2);

		clipper_subtract.AddPaths(source, ClipperLib::ptSubject, ClipperLib::etClosedPolygon);
		clipper_subtract.AddPaths(passable, ClipperLib::ptClip, ClipperLib::etClosedPolygon);
		clipper_subtract.Execute(ClipperLib::ctDifference, negatives, ClipperLib::pftNonZero);

		if (passable.size() < 1) {
			return false;
		}
	}

	//determine rooms by restricting the min width
	ClipperLib::PolyTree rooms;
	{
#define room_min_width 200

		ClipperLib::ClipperOffset clipper_reducer;
		ClipperLib::ClipperOffset clipper_expander;

		ClipperLib::Paths reduced;

		clipper_reducer.AddPaths(passable, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
		clipper_reducer.Execute(reduced, -room_min_width / 2);

		clipper_expander.AddPaths(reduced, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
		clipper_expander.Execute(rooms, room_min_width / 2);

		if (rooms.ChildCount() < 1) {
			return false;
		}
	}

	//allocate rooms from main region, 

	return true;
}

void Aroom_description_builder::Main_Generation_Loop() {

	//generate enclosure (null space)
	float system_x = FMath::RandRange(2, 6) * 5;
	float system_y = FMath::RandRange(2, 6) * 5;

	F_DCEL system_new;
	TArray<F_DCEL::Face*> Null_Regions;
	auto system_bounds_new = Generate_Boundary(system_x * 100, system_y * 100, _P(0, 0), 9);
	Null_Regions.Push(system_new.createFace(system_bounds_new));

	int h = 1;
	Draw_Border(to_FVector(system_bounds_new), 0, GetWorld());
	
	TArray<F_DCEL::Face*> rooms;

	for (int index = 0; index < room_count; index++)
	{

		//generate room shape
		float local_x = FMath::RandRange(2, 6);
		float local_y = FMath::RandRange(2, 6);
		FVector2D jump_old;

		jump_old.X = FMath::RandRange(-(int)system_x, (int)system_x);
		jump_old.Y = FMath::RandRange(-(int)system_y, (int)system_y);

		_P jump_new(jump_old.X, jump_old.Y);
		int bevel_level = FMath::RandRange(0, FMath::Min<int>(local_x, local_y));

		auto bounds_new = Generate_Boundary(local_x * 100, local_y * 100, jump_new * 100, bevel_level);

		//cull to null region, add pieces to options
		TArray<TArray<F_DCEL::Face*>> options;
		TArray<TArray<F_DCEL::Face*>> new_nulls;
		int ii = 0;
		for (auto null : Null_Regions) {
			options.Push(TArray<F_DCEL::Face*>());
			new_nulls.Push(TArray<F_DCEL::Face*>());
			null->subAllocateFace(bounds_new, options[ii], new_nulls[ii++]);
		}

		Null_Regions.Empty();
		//for each section, clean its boundary of narrows and analyse the shape, if rejected, merge back into all nulls
		for (int ii = 0; ii < options.Num(); ii++) {
			for (int jj = 0; jj < options[ii].Num(); jj++) {

				if (!Analyze_Suggested(to_Paths(options[ii][jj]))) {
					//delete this option

					int fused = -1;
					for (int kk = 0; kk < new_nulls[ii].Num(); kk++) {
						if (fused < 0) {
							if (new_nulls[ii][kk]->mergeWithFace(options[ii][jj])) {
								fused = kk;
							}
						}
						else {
							if (new_nulls[ii][fused]->mergeWithFace(new_nulls[ii][kk])) {
								new_nulls[ii].RemoveAt(kk--);
							}
						}
					}

					options[ii].RemoveAt(jj--);

				}
				else {
					rooms.Push(options[ii][jj]);
				}
			}
			for (auto null : new_nulls[ii]) {
				Null_Regions.Push(null);
			}
		}
	}
	for (auto room : rooms) {
		Create_Floor_Ceiling_New(room, 0, 200);
		Create_Wall_Sections_New(room, 0, 200, h);
		h++;
	}

	//system_new.CLEAN();
}

// Called when the game starts or when spawned
void Aroom_description_builder::BeginPlay()
{
	Super::BeginPlay();

	//is the orientation of room creation correct?

	Main_Generation_Loop();

}

// Called every frame
void Aroom_description_builder::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}


//style guide
//objects are full cap underscored
//variables are non-cap underscored
//functions are camel case