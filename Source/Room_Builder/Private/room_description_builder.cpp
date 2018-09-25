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

#define color_pink FColor(200,0,200)
#define color_red FColor(255,0,0)
#define color_green FColor(0,255,0)
#define color_blue FColor(0,0,255)

void Draw_Border(const TArray<FVector2D> &border, float height, const UWorld *ref, FColor color = color_pink) {
	float offset = 0;
	for (int index = 0; index < border.Num(); index++) {
		int next = (index + 1) % border.Num();

		DrawDebugLine(
			ref,
			FVector(border[index], height + offset * 5),
			FVector(border[next], height + offset * 5),
			//FColor(FMath::RandRange(0,255), FMath::RandRange(0, 255), FMath::RandRange(0, 255)),
			color,
			true,
			-1,
			0,
			10
		);
		//offset++;
	}
}

_P circularUniformPoint(float radius = 1.f) {
	float t = 2 * PI*FMath::RandRange(0.f, 1.f);
	float u = FMath::RandRange(0.f, 1.f) + FMath::RandRange(0.f, 1.f);
	float r = u;
	if (u > 1)
		r = 2 - u;
	r *= radius;
	return _P(r*cos(t), r*sin(t));
}
_P boxUniformPoint(float width = 1.f, float height = 1.f) {
	return _P(FMath::RandRange(0.f, width), FMath::RandRange(0.f, height));
}
_P boxUniformPoint(const FBox2D &box) {
	return _P(FMath::RandRange(box.Min.X, box.Max.X), FMath::RandRange(box.Min.Y, box.Min.Y));
}

FBox2D getBounds(const F_DCEL::Face* target) {
	FBox2D result;
	auto boundary = target->getRootEdge()->listPoints();
	result.Max.X = boundary[0].X;
	result.Max.Y = boundary[0].Y;
	result.Min = result.Max;

	for (auto point : boundary) {
		result.Min.X = FMath::Min(result.Min.X, point.X);
		result.Min.Y = FMath::Min(result.Min.Y, point.Y);
		result.Max.X = FMath::Max(result.Max.X, point.X);
		result.Max.Y = FMath::Max(result.Max.Y, point.Y);
	}

	return result;
}


//==========================================================================================================
//======================================= array transforms =================================================
//==========================================================================================================

#define Clipper_Scale grid_coef
#define room_min_width 280 * Clipper_Scale
#define hall_min_width 180 * Clipper_Scale
#define Fcc TArray<F_DCEL::Face*>



ClipperLib::Path to_Path(const TArray<FVector2D> &source) {
	ClipperLib::Path result;

	for (auto& vector : source) {
		result.push_back(ClipperLib::IntPoint(vector.X * Clipper_Scale, vector.Y * Clipper_Scale));
	}

	return result;
};
ClipperLib::Path to_Path(const TArray<_P> &source) {
	ClipperLib::Path result;

	for (auto& vector : source) {
		result.push_back(ClipperLib::IntPoint(vector.X * Clipper_Scale, vector.Y * Clipper_Scale));
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
		result.Push(FVector2D(point.X / Clipper_Scale, point.Y / Clipper_Scale));
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
		result.Push(_P(vector.X / Clipper_Scale, vector.Y / Clipper_Scale));
	}

	return result;
}

//methods for transforming a PolyTree into a DCEL
void AllocateNode(const ClipperLib::PolyNode &ref, F_DCEL::Face* target, Fcc &ins, Fcc &outs) {
	//the poly tree structure guarantees non first order children won't touch edges, making recursive calculation simple
	Fcc created_ins;
	Fcc created_outs;

	if (target->subAllocateFace(to_Primitive(ref.Contour), created_ins, created_outs)) {
		for (auto node : ref.Childs) {
			AllocateNode(*node, created_ins[0], ins, outs);
		}

		if (ref.IsHole()) {
			outs.Append(created_ins);
		}
		else {
			ins.Append(created_ins);
		}
	}
}
void AllocateTree(ClipperLib::PolyTree &ref, F_DCEL::Face* target, Fcc &ins, Fcc &outs) {
	//first order children may touch edges, meaning reconsideration of target is nessecary
	Fcc relevants;

	relevants.Push(target);

	for (auto node : ref.Childs) {
		UE_LOG(LogTemp, Warning, TEXT("---NODE---"));
		Fcc created_ins;
		Fcc created_outs;
		for (int ii = 0; ii < relevants.Num(); ii++) {
			UE_LOG(LogTemp, Warning, TEXT("----RELEVANT----"));
			if (relevants[ii]->subAllocateFace(to_Primitive(node->Contour), created_ins, created_outs)) {
				relevants.RemoveAt(ii--);
				relevants.Append(created_outs);
				ins.Append(created_ins);

				for (auto child : node->Childs) {
					AllocateNode(*child, created_ins[0], ins, outs);
				}

				break;
			}
		}
	}

	outs.Append(relevants);
}
void AllocateTree(ClipperLib::PolyTree &ref, Fcc &targets, Fcc &ins, Fcc &outs) {
	for (auto target : targets) {
		UE_LOG(LogTemp, Warning, TEXT("--TARGET--"));
		AllocateTree(ref, target, ins, outs);
	}
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
		//Draw_Border(Border, Top, GetWorld());
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
		//Draw_Border(Border, Top, GetWorld());
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


//==========================================================================================================
//================================== generation utilities ==================================================
//==========================================================================================================

struct Type_Tracker {
	Fcc Nulls;
	Fcc Rooms;
	Fcc Halls;
	Fcc Smalls;
	bool isRoom(const F_DCEL::Face* target) {
		for (auto room : Rooms) {
			if (room == target) {
				return true;
			}
		}
		return false;
	}
	void display(const UWorld* world) {
		for (auto small : Smalls) {
			small->cleanBorder();
			Draw_Border(to_FVector(small->getRootEdge()->listPoints()), 70, world, color_pink);
		}

		for (auto room : Rooms) {
			room->cleanBorder();
			//Create_Floor_Ceiling_New(room, 0, 200);
			//Create_Wall_Sections_New(room, 0, 200, h);
			Draw_Border(to_FVector(room->getRootEdge()->listPoints()), 80, world, color_blue);
		}

		for (auto null : Nulls) {
			null->cleanBorder();
			Draw_Border(to_FVector(null->getRootEdge()->listPoints()), 65, world, color_red);
		}

		for (auto hall : Halls) {
			hall->cleanBorder();
			Draw_Border(to_FVector(hall->getRootEdge()->listPoints()), 75, world, color_green);
		}
	}
	//void Safety() {
	//	for (auto room : Rooms) {
	//		check(room->getDad() != NULL);
	//	}
	//}
};

bool Cull_Suggested(F_DCEL::Face* target, Fcc &results, Fcc &nulls) {

	ClipperLib::Paths reduced;
	Fcc rooms;
	Fcc outers;

	{

		ClipperLib::Paths source = to_Paths(target);
		ClipperLib::ClipperOffset clipper_reducer;


		clipper_reducer.AddPaths(source, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
		clipper_reducer.Execute(reduced, -room_min_width / 2);
	}

	if (reduced.size() < 1) {
		nulls.Push(target);
		return false;
	}

	outers.Push(target);

	//IF THE ROOM HAS ANY HOLES WE'RE FUCKED
	for (auto section : reduced) {
		Fcc created_outers;

		ClipperLib::PolyTree rooms_tree;
		ClipperLib::ClipperOffset clipper_expander;
		clipper_expander.MiterLimit = 100;

		clipper_expander.AddPath(section, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
		clipper_expander.Execute(rooms_tree, room_min_width / 2);

		AllocateTree(rooms_tree, outers, rooms, created_outers);

		outers = created_outers;
	}

	for (auto null : outers) {
		nulls.Push(null);
	}
	for (auto room : rooms) {
		results.Push(room);
	}

	return true;
}

void mergeGroup(Fcc &nulls) {
	for (int ii = 0; ii < nulls.Num(); ii++) {
		for (int jj = ii + 1; jj < nulls.Num(); jj++) {
			if (nulls[ii]->mergeWithFace(nulls[jj])) {
				nulls.RemoveAt(jj);
				jj = ii;
			}
		}
	}
}

void cleanNulls(Type_Tracker &target) {
	Fcc input_nulls;
	input_nulls.Append(target.Nulls);
	input_nulls.Append(target.Halls);
	//input_nulls.Append(target.Smalls);
	mergeGroup(input_nulls);

	Fcc cleaned_nulls;
	Fcc pruned_halls;
	Fcc pruned_smalls;

	for (int ii = 0; ii < input_nulls.Num(); ii++) {
		ClipperLib::Paths source = to_Paths(input_nulls[ii]);
		ClipperLib::Paths room_ready;
		{
			ClipperLib::Paths reduced;
			ClipperLib::Paths expanded;

			ClipperLib::ClipperOffset clipper_reducer;
			ClipperLib::ClipperOffset clipper_expander;

			clipper_expander.MiterLimit = 100;

			clipper_reducer.AddPaths(source, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
			clipper_reducer.Execute(reduced, -room_min_width / 2);

			clipper_expander.AddPaths(reduced, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
			clipper_expander.Execute(expanded, room_min_width / 2);

			ClipperLib::Clipper clipper_restriction;

			clipper_restriction.AddPaths(expanded, ClipperLib::ptSubject, true);
			clipper_restriction.AddPaths(source, ClipperLib::ptClip, true);
			clipper_restriction.Execute(ClipperLib::ctIntersection, room_ready);
		}

		ClipperLib::Paths hall_ready;
		{
			ClipperLib::Paths reduced;
			ClipperLib::Paths halls_cleared;
			ClipperLib::Paths rooms_cleared;

			ClipperLib::ClipperOffset clipper_reducer;
			ClipperLib::ClipperOffset clipper_expander;

			clipper_expander.MiterLimit = 100;

			clipper_reducer.AddPaths(source, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
			clipper_reducer.Execute(reduced, -hall_min_width / 2);

			clipper_expander.AddPaths(reduced, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
			clipper_expander.Execute(halls_cleared, hall_min_width / 2);

			ClipperLib::Clipper clipper_subtract;

			clipper_subtract.AddPaths(halls_cleared, ClipperLib::ptSubject, true);
			clipper_subtract.AddPaths(room_ready, ClipperLib::ptClip, true);
			clipper_subtract.Execute(ClipperLib::ctDifference, rooms_cleared);

			ClipperLib::Clipper clipper_restriction;

			clipper_restriction.AddPaths(rooms_cleared, ClipperLib::ptSubject, true);
			clipper_restriction.AddPaths(source, ClipperLib::ptClip, true);
			clipper_restriction.Execute(ClipperLib::ctIntersection, hall_ready);
		}

		ClipperLib::PolyTree room_tree;
		ClipperLib::PolyTree hall_tree;
		{
			ClipperLib::Clipper clipper_room;
			ClipperLib::Clipper clipper_hall;

			clipper_room.AddPaths(room_ready, ClipperLib::ptSubject, true);
			clipper_hall.AddPaths(hall_ready, ClipperLib::ptSubject, true);

			clipper_room.Execute(ClipperLib::ctUnion, room_tree);
			clipper_hall.Execute(ClipperLib::ctUnion, hall_tree);
		}

		Fcc room_ins;
		Fcc room_outs;
		Fcc hall_ins;
		Fcc hall_outs;
		Fcc debug_ins;
		Fcc debug_outs;

		TArray<_P> debug_input = input_nulls[ii]->getRootEdge()->listPoints();
		TArray<TArray<_P>> debug_room_ins;
		TArray<TArray<_P>> debug_room_outs;
		TArray<TArray<_P>> debug_hall_ins;
		TArray<TArray<_P>> debug_hall_outs;
		TArray<TArray<_P>> debug_debug_ins;
		TArray<TArray<_P>> debug_debug_outs;

		UE_LOG(LogTemp, Warning, TEXT(""));
		UE_LOG(LogTemp, Warning, TEXT("ALLOCATE ROOMS"));
		UE_LOG(LogTemp, Warning, TEXT(""));


		AllocateTree(room_tree, input_nulls[ii], room_ins, room_outs);

		for (auto room : room_ins) {
			debug_room_ins.Push(room->getRootEdge()->listPoints());
		}
		for (auto room : room_outs) {
			debug_room_outs.Push(room->getRootEdge()->listPoints());
		}

		UE_LOG(LogTemp, Warning, TEXT(""));
		UE_LOG(LogTemp, Warning, TEXT("ALLOCATE HALLS"));
		UE_LOG(LogTemp, Warning, TEXT(""));

		AllocateTree(hall_tree, room_outs, hall_ins, hall_outs);
		
		for (auto room : hall_ins) {
			debug_hall_ins.Push(room->getRootEdge()->listPoints());
		}
		for (auto room : hall_outs) {
			debug_hall_outs.Push(room->getRootEdge()->listPoints());
		}
		
		if (hall_ins.Num() != hall_tree.ChildCount()) {
			F_DCEL system_debug;
			F_DCEL::Face* inner = system_debug.createFace(debug_input);

			UE_LOG(LogTemp, Warning, TEXT(""));
			UE_LOG(LogTemp, Warning, TEXT("ALLOCATE DEBUG"));
			UE_LOG(LogTemp, Warning, TEXT(""));

			AllocateTree(room_tree, inner, debug_ins, debug_outs);
			for (auto room : debug_ins) {
				debug_debug_ins.Push(room->getRootEdge()->listPoints());
			}
			for (auto room : debug_outs) {
				debug_debug_outs.Push(room->getRootEdge()->listPoints());
			}
			//check(hall_ins.Num() == hall_tree.ChildCount());
		}

		//check(room_ins.Num() == room_tree.ChildCount());
		//check(hall_ins.Num() == hall_tree.ChildCount());

		for (auto null : room_ins) {
			cleaned_nulls.Push(null);
		}
		for (auto null : hall_ins) {
			pruned_halls.Push(null);
		}
		for (auto null : hall_outs) {
			pruned_smalls.Push(null);
		}
	}

	target.Nulls = cleaned_nulls;
	target.Halls = pruned_halls;
	target.Smalls.Append(pruned_smalls);
	mergeGroup(target.Smalls);

	//mergeGroup(target.Nulls);
	//mergeGroup(target.Halls);
	//mergeGroup(target.Smalls);
}
//allocates a region randomly from the provided nulls, listing interior faces

void mergeSmalls(Type_Tracker &target) {
	//for each small
	//for connected room
	//union the borders, reduce, and intersect with small
	//choose room with greatest area 
	for (int ii = 0; ii < target.Smalls.Num(); ii++) {
		auto local_boundary = to_Path(target.Smalls[ii]->getRootEdge()->listPoints());
		
		auto neighbors = target.Smalls[ii]->getNeighbors();

		F_DCEL::Face* best_neighbor = nullptr;
		float best_area_score = 0;
		ClipperLib::PolyTree best_tree;

		for (auto neighbor : neighbors) {
			if (!target.isRoom(neighbor)) {
				continue;
			}


			auto other_boundary = to_Path(neighbor->getRootEdge()->listPoints());
			ClipperLib::Paths grouping;
			ClipperLib::Paths reduced;
			ClipperLib::Paths expanded;
			ClipperLib::Paths result;

			ClipperLib::Clipper clipper_additive;
			ClipperLib::ClipperOffset clipper_reducer;
			ClipperLib::ClipperOffset clipper_expander;
			ClipperLib::Clipper clipper_intersect;

			clipper_expander.MiterLimit = 100;

			clipper_additive.AddPath(other_boundary, ClipperLib::ptSubject, true);
			clipper_additive.AddPath(local_boundary, ClipperLib::ptSubject, true);
			clipper_additive.Execute(ClipperLib::ctUnion, grouping);

			clipper_reducer.AddPaths(grouping, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
			clipper_reducer.Execute(reduced, -room_min_width / 2);

			clipper_expander.AddPaths(reduced, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
			clipper_expander.Execute(expanded, room_min_width / 2);

			clipper_intersect.AddPath(local_boundary, ClipperLib::ptSubject, true);
			clipper_intersect.AddPaths(expanded, ClipperLib::ptClip, true);
			clipper_intersect.Execute(ClipperLib::ctIntersection, result);

			if (result.size() > 0) {
				float area_score = ClipperLib::Area(result[0]);
				if (area_score > best_area_score) {
					best_area_score = area_score;
					best_neighbor = neighbor;
					best_tree.Clear();
					clipper_intersect.Execute(ClipperLib::ctIntersection, best_tree);
				}
			}
		}

		if (best_neighbor != nullptr) {
			Fcc ins;
			Fcc outs;

			AllocateTree(best_tree, target.Smalls[ii], ins, outs);

			check(ins.Num() > 0);

			for (auto in : ins) {
				best_neighbor->mergeWithFace(in);
			}

			target.Smalls.RemoveAt(ii--);
			target.Smalls.Append(outs);
		}
	}
}

TArray<_P> choosePointsNear(const F_DCEL::Face &target, int count){
	//pick n points AWAY from the polygon
	//get center offsets of each face
	auto bounds = getBounds(&target);
	auto extent = bounds.GetExtent();
	auto center = bounds.GetCenter();
	auto size = 2 * FMath::Max(extent.X, extent.Y);

	auto segments = target.getRootEdge()->listPoints();
	auto segment_size = segments.Num();

	TArray<_P> seeds;
	for (int ii = 0; ii < count; ii++) {
		auto direction = circularUniformPoint();
		direction.Normalize();
		auto seed = direction * size;
		seed.X += center.X;
		seed.Y += center.Y;
		//seeds.Push(seed);

		_P best(0,0);
		float best_score = FLT_MAX;

		for (int jj = 0; jj < segment_size; jj++) {
			auto A = segments[jj];
			auto B = segments[(jj + 1) % segment_size];

			auto AB = B - A;
			auto AP = seed - A;
			float lengthSqrAB = AB.X * AB.X + AB.Y * AB.Y;
			float t = (AP.X * AB.X + AP.Y * AB.Y) / lengthSqrAB;

			if (t < 0) {
				t = 0;
			}
			if (t > 1) {
				t = 1;
			}

			_P point = A + (AB * t);
			point.toGrid(P_micro_grid);

			float score = (point - seed).SizeSquared();
			if (score < best_score) {
				best_score = score;
				best = point;
			}
		}

		seeds.Push(best);
	}

	//for each edge, offset it and get the nearest point, tracking the best fit
	return seeds;
}
/*
struct HallwayPrototype {
	HallwayPrototype* parent;
	int parent_generation;

	TArray<HallwayPrototype*> children;

	TArray<_P> path;
	HallwayPrototype(HallwayPrototype* p, int g) {
		parent = p;
		parent_generation = g;
	}
	//returns true if it found a child to merge with
	bool mergeUp() {

		auto grand = parent->parent;
		/*
		grand
		^ parent, p_g -> NULL
		v children -[parent]> this
		parent (tbdeleted) {path} -prefix-clone> this
		^ parent, p_g -> grand
		v children -prefix-clone> this
		this

		children parents need to be re set
		children generations need to be recalculated
		*./
		
		
		//find parent 
		int p_id = grand->children.Find(parent);
		grand->children[p_id] = this;

		//these change OUR references, so they should be last

		for (auto child : parent->children) {
			child->parent = this;
		}
		int parent_lifetime = parent->children.Num();
		for (auto child : children) {
			child->parent_generation += parent_lifetime;
		}

		children.Insert(parent->children, 0);
		path.Insert(parent->path, 0);

		parent_generation = parent->parent_generation;
		
		delete parent;

		parent = grand;

	}
	void retreat() {
		if (children.Num() == 0) {
			parent->children.Remove(this);
			delete this;
		}
		else {
			auto end = children[children.Num() - 1];
			children.RemoveAt(children.Num() - 1);

			end->mergeUp();
		}
	}

	
};
/*
void prototyper() {
	
	//has a set of open line paths for clipper that are expanded
	//use open square for now, this will create artifacts at
	
	//create a set of lines that greedily explore from one room to another
	//make several attempts
	//each attempt uses a set of tree like hall ways which explore a frontier greedily
	//after delete insignificant hallways, (consider length, objective reached, room touched)

	//if a hallway -becomes- insignificant, merge its last child into it, if no children, delete
	//F_DCEL::Face* start;
	//F_DCEL::Face* end;

	TArray<HallwayPrototype*> frontier;

	//pick initial points that are on the boundary of the start
	//pick a side that isn't to short
	TArray<_P> start_canidates;

	TArray<F_DCEL::Edge*> canidates;
	start->getRootEdge()->listFaceLoop(canidates);

	for (int ii = 0; ii < canidates.Num(); ii++) {
		auto start = canidates[ii]->getStartPoint()->getPosition();
		auto end = canidates[ii]->getEndPoint()->getPosition();
		if ((end - start).Size() < 150) {
			canidates.RemoveAt(ii--);
		}
	}

	for(int ii = 0; ii < 3; ii++){
		//auto target = canidates/
	}

	//pick a canidate based on... 
	//best fitness
	//shortest length
	//closest to goal


	do {
		//pick a canidate

		//suggest some segments

		//choose a segment and add it

		//no segments?
		//retreat
	} while (true);
}
//==========================================================================================================
//======================================= generation =======================================================
//==========================================================================================================
*/

TArray<_P> Square_Generator(int x, int y, _P center) {
	TArray<_P> boundary;
	boundary.Push(_P(-x, -y) + center);
	boundary.Push(_P(-x, y) + center);
	boundary.Push(_P(x, y) + center);
	boundary.Push(_P(x, -y) + center);

	return boundary;
}
TArray<_P> Diamond_Generator(int x, int y, _P center) {
	TArray<_P> boundary;
	if (x < y) {
		int d = y - x;
		boundary.Push(_P(-x, -d) + center);
		boundary.Push(_P(-x, d) + center);
		boundary.Push(_P(0, y) + center);
		boundary.Push(_P(x, d) + center);
		boundary.Push(_P(x, -d) + center);
		boundary.Push(_P(0, -y) + center);
	}
	else if (x > y) {
		int d = x - y;
		boundary.Push(_P(-x, 0) + center);
		boundary.Push(_P(-d, y) + center);
		boundary.Push(_P(d, y) + center);
		boundary.Push(_P(x, 0) + center);
		boundary.Push(_P(d, -y) + center);
		boundary.Push(_P(-d, -y) + center);
	}
	else {
		boundary.Push(_P(-x, 0) + center);
		boundary.Push(_P(0, x) + center);
		boundary.Push(_P(x, 0) + center);
		boundary.Push(_P(0, -x) + center);
	}
	

	return boundary;
}
TArray<_P> Bevel_Generator(int x, int y, _P center) {
	int level = FMath::Min(x/3, y/3);

	TArray<_P> boundary;

	boundary.Push(_P(-x, level - y) + center);
	boundary.Push(_P(-x, y - level) + center);
	boundary.Push(_P(level - x, y) + center);
	boundary.Push(_P(x - level, y) + center);
	boundary.Push(_P(x, y - level) + center);
	boundary.Push(_P(x, level - y) + center);
	boundary.Push(_P(x - level, -y) + center);
	boundary.Push(_P(level - x, -y) + center);

	return boundary;
}

typedef TArray<_P> (*generatorFunc)(int, int, _P);

TArray<_P> Pick_Generator(int x, int y, _P center) {
	generatorFunc generator = Square_Generator;
	float gen_choice = FMath::RandRange(0, 1);
	if (gen_choice > .4) {
		if (gen_choice > .7) {
			generator = Bevel_Generator;
		}
		else {
			generator = Diamond_Generator;
		}
	}

	return generator(x, y, center);
}

//void Generate_Building_Shape(F_DCEL::Face* Available, Type_Tracker &system_types) {
	//generates a set of trackers for each building region
	//these come with predefined nulls and infrastructure halls

//}

bool createRoomAtPoint(Type_Tracker &system_types, const _P &point, int scale = 1, Fcc *created = nullptr) {
	Fcc created_rooms;
	Fcc created_nulls;
	Fcc raw_faces;
	
	int null_index = -1;

	//find containing null
	for (int ii = 0; ii < system_types.Nulls.Num(); ii++) {
		if (system_types.Nulls[ii]->contains(point)) {
			null_index = ii;
			break;
		}
	}

	if (null_index < 0) {
		return false;
	}

	auto chosen_null = system_types.Nulls[null_index];
	system_types.Nulls.RemoveAt(null_index);

	int area = FMath::RandRange(6, 14);
	int width = FMath::RandRange(2, area / 2);
	int length = area / width;

	auto bounds = Pick_Generator(width * 100 * scale, length * 100 * scale, point);

	//cull to null region
	if (!chosen_null->subAllocateFace(bounds, raw_faces, created_nulls)) {
		system_types.Nulls.Push(chosen_null);
	}
	for (auto null : raw_faces) {
		check(null->getDad() != NULL);
	}
	for (auto null : created_nulls) {
		check(null->getDad() != NULL);
	}

	//remerge rejected regions with nulls
	for (int jj = 0; jj < raw_faces.Num(); jj++) {
		Cull_Suggested(raw_faces[jj], created_rooms, created_nulls);

		for (auto null : created_nulls) {
			check(null->getRootEdge() != NULL);
		}

		mergeGroup(created_nulls);
	}

	for (auto null : created_nulls) {
		check(null->getRootEdge() != NULL);
		system_types.Nulls.Push(null);
	}

	cleanNulls(system_types);
	
	system_types.Rooms.Append(created_rooms);
	if (created != nullptr) {
		created->Append(created_rooms);
	}
	return created_rooms.Num() > 0;
}

//attempts to create rooms distributed evenly across the space
bool createDistributedRooms(Type_Tracker &system_types, const int attempts_per_cell = 5, int scale = 1, Fcc *created = nullptr) {
	//GRID METHOD
	//from a grid, for each grid point we select a point within a tolerance radius
	//choose a grid
	//skew, stretch, rotation, offset
	//offset.X = (FMath::RandRange((int)null_bounds.Min.X / 100, (int)null_bounds.Max.X) / 100) * 100.f;
	//offset.Y = (FMath::RandRange((int)null_bounds.Min.Y / 100, (int)null_bounds.Max.Y) / 100) * 100.f;
	
	//offset only for now
#define grid_width 10
#define grid_height 10
#define tolerance 4

	_P offset = boxUniformPoint(grid_width, grid_height);
	_P skew = boxUniformPoint(grid_width, grid_height);

	for (int xx = -5; xx <= 5; xx++) {
		for (int yy = -5; yy <= 5; yy++) {
			//test a couple points until we find one in the space
			int safety = attempts_per_cell;
			_P grid = _P(xx * grid_width, yy * grid_height) + offset + _P(skew.X*yy, skew.Y*xx);

			do {
				_P choice = circularUniformPoint(tolerance) + grid;
				choice *= 100.f;
				choice.toGrid(100.f);

				//choice.X = (int)(choice.X) * 100.f;
				//choice.Y = (int)(choice.Y) * 100.f;

				if (createRoomAtPoint(system_types, choice, scale, created)) {
					break;
				}

			}while(safety-- > 0);
		}
	}
	return true;

	//CHOICE METHOD
	//_P choice = circularUniformPoint();
}

bool fillNullSpace(Type_Tracker &system_types, int safety = 100, int scale = 1, Fcc *created = nullptr) {
	while (system_types.Nulls.Num() > 0 && (safety--) > 0) {
		auto bounds = getBounds(system_types.Nulls[0]);
		auto choice = boxUniformPoint(bounds);
		choice.toGrid(100.f);
		createRoomAtPoint(system_types, choice, scale, created);
	}
	return true;
}

bool createNearRooms(Type_Tracker &system_types, F_DCEL::Face &target, int attempts = 10, int scale = 1, Fcc *created = nullptr) {
	auto seeds = choosePointsNear(target, attempts);
	for (auto choice : seeds) {
		createRoomAtPoint(system_types, choice, scale, created);
	}
	return true;
}

bool createLinkingHalls(F_DCEL::Face* target, Fcc &nulls, Fcc &created_faces) {
	//tries to generate a set of halls linking two target faces

	return true;
}

struct building_region {
	int size;
	F_DCEL::Face* region;
	building_region* parent;
	building_region(int s, F_DCEL::Face* r) {
		size = s;
		region = r;
		parent = nullptr;
	}
	building_region(int s, F_DCEL::Face* r, building_region* p) {
		size = s;
		region = r;
		parent = p;
	}
};

void create_Layout(Type_Tracker &system_types, int large, int medium, int small) {
	//large is 10-14
	//med is 7-11
	//small is 4-8
	TArray<building_region*> larges;
	TArray<building_region*> mediums;
	TArray<building_region*> smalls;

	for (int ii = 0; ii < large; ii++) {
		//pick point
		auto point = circularUniformPoint(800);
		point.toGrid(P_micro_grid);
		Fcc temp_created;
		if (createRoomAtPoint(system_types, point, FMath::RandRange(10, 14), &temp_created)) {
			for (auto room : temp_created) {
				larges.Push(new building_region(0, room));
			}
		}
	}
	for (auto region : larges) {
		//creates some MEDIUMS
		Fcc temp_created;
		createNearRooms(system_types, *region->region, 5, FMath::RandRange(7, 11), &temp_created);
		for (auto room : temp_created) {
			mediums.Push(new building_region(1, room, region));
		}
	}
	for (auto region : mediums) {
		//creates some MEDIUMS
		Fcc temp_created;
		createNearRooms(system_types, *region->region, 5, FMath::RandRange(4, 8), &temp_created);
		for (auto room : temp_created) {
			smalls.Push(new building_region(2, room, region));
		}
	}

	for (auto region : larges) {
		delete region;
	}
	for (auto region : mediums) {
		delete region;
	}
	for (auto region : smalls) {
		delete region;
	}
}

void Aroom_description_builder::Main_Generation_Loop() {
	F_DCEL system_new;
	Type_Tracker system_types;

	//generate enclosure (null space)
	//float system_x = FMath::RandRange(5, 9) * 4;
	//float system_y = FMath::RandRange(2, 4) * 4;

	auto system_bounds = Square_Generator(25000,25000,_P(0,0));
	Draw_Border(to_FVector(system_bounds), 0, GetWorld());

	system_types.Nulls.Push(system_new.createFace(system_bounds));
	//system_types.Nulls.Push(system_new.createUniverse());

	create_Layout(system_types, 2, 2, 2);

	//generate MAIN region
	//createRoomAtPoint(system_types, _P(0, 0), 4);

	//createNearRooms(system_types, *system_types.Rooms[0], 5, 2);

	//generate offset regions

	//createDistributedRooms(system_types);

	//fillNullSpace(system_types);

	//mergeSmalls(system_types);

	system_types.display(GetWorld());
}

//==========================================================================================================
//====================================== member specific ===================================================
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