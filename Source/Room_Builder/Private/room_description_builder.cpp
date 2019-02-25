// Fill out your copyright notice in the Description page of Project Settings.

#include "room_description_builder.h"
#include "Grid_Tools.h"
#include "Algo/Reverse.h"
#include "DrawDebugHelpers.h"
#include "ConstructorHelpers.h"

//==========================================================================================================
//========================================== transforms ====================================================
//==========================================================================================================



FVector2D convert(Pgrd const &target) {
	return FVector2D(target.X.n * 10, target.Y.n * 10);
}

TArray<FVector2D> convert(FLL<Pgrd> const &target) {
	TArray<FVector2D> result;

	for (auto x : target)
		result.Push(convert(x));

	return result;
}


TArray<FVector2D> toFVector(FLL<Pgrd> const &target) {
	TArray<FVector2D> product;

	for (auto point : target) {
		product.Push(convert(point));
	}

	return product;
}

//==========================================================================================================
//========================================= utilities ======================================================
//==========================================================================================================

#define color_Pnk FColor(200,0,200)
#define color_red FColor(255,0,0)
#define color_green FColor(0,255,0)
#define color_blue FColor(0,0,255)

void Draw_Border(const TArray<FVector2D> &border, float height, const UWorld *ref, FColor color = color_Pnk) {
	float offset = 0;
	for (int64 index = 0; index < border.Num(); index++) {
		int64 next = (index + 1) % border.Num();

		DrawDebugLine(
			ref,
			FVector(border[index], height + offset * 5),
			FVector(border[next], height + offset * 5),
			//FColor(FMath::RandRange(0,255), FMath::RandRange(0, 255), FMath::RandRange(0, 255)),
			color,
			true,
			-1,
			0,
			3
		);
		//offset++;
	}
}


Pgrd circularUniformPoint(grd radius = 1, int64 divisions = 100) {
	float t = 2 * PI*FMath::RandRange(0.f, 1.f);
	float u = FMath::RandRange(0.f, 1.f) + FMath::RandRange(0.f, 1.f);
	float r = u;
	if (u > 1)
		r = 2 - u;
	r *= divisions;
	return Pgrd(r*cos(t),r*sin(t)) * radius / divisions;

}
Pgrd boxUniformPoint(grd width = 10, grd height = 10, int64 divisions = 100) {
	return Pgrd(width * (int64)FMath::RandRange(0, divisions), height * (int64)FMath::RandRange(0, divisions)) / divisions;
}
Pgrd boxUniformPoint(PBox const & box, int64 divisions = 100) {
	int64 X = (int64)FMath::RandRange(-(float)divisions, (float)divisions);
	int64 Y = (int64)FMath::RandRange(-(float)divisions, (float)divisions);

	return Pgrd(X, Y) * box.getExtent() + box.getCenter();
}

PBox getBounds(Region<Pgrd> * target) {

	PBox result;

	auto init = target->getBounds().last()->getRoot()->getStart()->getPosition();

	result.Max.X = init.X;
	result.Max.Y = init.Y;

	result.Min = result.Max;

	for(auto boundary : target->getBounds()){
		auto points = boundary->getLoopPoints();

		for (auto point : points) {
			result.Min.X = FMath::Min(result.Min.X, point.X);
			result.Min.Y = FMath::Min(result.Min.Y, point.Y);
			result.Max.X = FMath::Max(result.Max.X, point.X);
			result.Max.Y = FMath::Max(result.Max.Y, point.Y);
		}
	}

	return result;
}

//==========================================================================================================
//=================================== chord split utilities ================================================
//==========================================================================================================



//==========================================================================================================
//=================================== triangulate utilities ================================================
//==========================================================================================================

namespace tri_utils
{
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
	TArray<T> Reverse(const TArray<T> &Buffer) {
		const int32 size = Buffer.Num();

		TArray<T> product;
		product.SetNum(size);

		for (int32 ii = 0; ii < size; ii++) {
			product[size - ii - 1] = Buffer[ii];
		}

		return product;
	}

	TArray<int32> Triangulate(const TArray<FVector2D> &VectorBuffer, TArray<int32> &IndexBuffer) {
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
						UE_LOG(LogTemp, Warning, TEXT("TRIANGULATION FROZEN"));
						break;
					}
				}
			}
		}
		return Triangles;
	}

}

//==========================================================================================================
//======================================== creation ========================================================
//==========================================================================================================

UProceduralMeshComponent * Aroom_description_builder::CreateMeshComponent() {
	UProceduralMeshComponent* component = NewObject<UProceduralMeshComponent>();

	component->AttachToComponent(root, FAttachmentTransformRules::KeepRelativeTransform);
	component->ContainsPhysicsTriMeshData(true);
	component->bUseAsyncCooking = true;

	return component;
}

void Aroom_description_builder::ActivateMeshComponent(UProceduralMeshComponent * component) {
	component->ContainsPhysicsTriMeshData(true);

	component->Activate();

	component->RegisterComponentWithWorld(GetWorld());
}

void generateInsetPoints(Edge<Pgrd> const * target, grd const & distance, 
	Pgrd & result_A, Pgrd & result_B) {

	Pgrd const previous = target->getLast()->getStart()->getPosition();
	Pgrd const A = target->getStart()->getPosition();
	Pgrd const B = target->getEnd()->getPosition();
	Pgrd const next = target->getNext()->getEnd()->getPosition();

	Pgrd A_last = previous - A;
	Pgrd A_next = B - A;
	Pgrd B_last = A - B;
	Pgrd B_next = next - B;

	A_last.Normalize();
	A_next.Normalize();
	B_last.Normalize();
	B_next.Normalize();
	
	{
		result_A = A_last + A_next;

		if (result_A == Pgrd(0, 0)) {
			result_A.X = A_next.Y;
			result_A.Y = -A_next.X;

			result_A *= distance;
		}
		else {
			result_A.Normalize();
			Pgrd rot(A_next.Y, -A_next.X);

			result_A *= distance / result_A.Dot(rot);
		}

		result_A += A;
	}

	{
		result_B = B_last + B_next;

		if (result_B == Pgrd(0, 0)) {
			result_B.X = B_next.Y;
			result_B.Y = -B_next.X;

			result_B *= distance;
		}
		else {
			result_B.Normalize();
			Pgrd rot(B_next.Y, -B_next.X);

			result_B *= distance / result_B.Dot(rot);
		}

		result_B += B;
	}

}

FLL<Pgrd> generateInsetPoints(Face<Pgrd> * target, grd const & distance) {

	FLL<Pgrd> result;

	for (auto edge : target->getLoopEdges()) {
		Pgrd const previous = edge->getLast()->getStart()->getPosition();
		Pgrd const A = edge->getStart()->getPosition();
		Pgrd const B = edge->getEnd()->getPosition();

		Pgrd A_last = previous - A;
		Pgrd A_next = B - A;

		A_last.Normalize();
		A_next.Normalize();

		Pgrd inset = A_last + A_next;

		if (inset == Pgrd(0, 0)) {
			inset.X = A_next.Y;
			inset.Y = -A_next.X;

			inset *= distance;
		}
		else {
			inset.Normalize();
			Pgrd rot(A_next.Y, -A_next.X);

			inset *= distance / inset.Dot(rot);
		}

		inset += A;

		result.append(inset);
	}

	return result;
}

void Aroom_description_builder::CreateWallSegment(Edge<Pgrd> const * target, float bottom, float top,
	UProceduralMeshComponent * component, int section_id) {

	TArray<FVector> Vertices;
	TArray<int32> Triangles;
	TArray<FVector> Normals;
	TArray<FVector2D> UV0;
	TArray<FProcMeshTangent> Tangents;
	TArray<FLinearColor> VertexColors;

	Pgrd wall_left, wall_right;
	generateInsetPoints(target, grd(wall_thickness / 2), wall_left, wall_right);

	Pgrd dir = wall_left - wall_right;
	dir.Normalize();

	Pgrd normal(dir.Y, -dir.X);

	FVector2D f_wall_left = convert(wall_left);
	FVector2D f_wall_right = convert(wall_right);

	FVector f_normal(convert(normal), 0);

	Vertices.Push(FVector(f_wall_left, bottom));
	Vertices.Push(FVector(f_wall_left, top));
	Vertices.Push(FVector(f_wall_right, bottom));
	Vertices.Push(FVector(f_wall_right, top));

	Triangles.Push(0);
	Triangles.Push(1);
	Triangles.Push(2);
	Triangles.Push(2);
	Triangles.Push(1);
	Triangles.Push(3);

	
	Normals.Push(f_normal);
	Normals.Push(f_normal);
	Normals.Push(f_normal);
	Normals.Push(f_normal);

	UV0.Push(FVector2D(0, bottom));
	UV0.Push(FVector2D(0, top));
	UV0.Push(FVector2D(1, bottom));
	UV0.Push(FVector2D(1, top));

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

	component->CreateMeshSection_LinearColor(section_id, Vertices, Triangles, Normals, UV0, VertexColors, Tangents, true);
}

void Aroom_description_builder::CreateDoorSegment(Edge<Pgrd> const * target, float bottom, float top,
	UProceduralMeshComponent * component, int section_id) {

	TArray<FVector> Vertices;
	TArray<int32> Triangles;
	TArray<FVector> Normals;
	TArray<FVector2D> UV0;
	TArray<FProcMeshTangent> Tangents;
	TArray<FLinearColor> VertexColors;

	Pgrd wall_left, wall_right;
	generateInsetPoints(target, grd(wall_thickness / 2), wall_left, wall_right);

	Pgrd dir = wall_left - wall_right;
	dir.Normalize();

	Pgrd normal(dir.Y, -dir.X);

	FVector2D f_wall_left = convert(wall_left);
	FVector2D f_wall_right = convert(wall_right);

	FVector f_normal(convert(normal), 0);

	FVector2D f_inset_left = convert(target->getStart()->getPosition());
	FVector2D f_inset_right = convert(target->getEnd()->getPosition());

	if (top - bottom > door_height) {
		Vertices.Push(FVector(f_wall_left, bottom));
		Vertices.Push(FVector(f_wall_left, bottom + door_height));
		
		Vertices.Push(FVector(f_inset_left, bottom));
		Vertices.Push(FVector(f_inset_left, bottom + door_height));

		Vertices.Push(FVector(f_wall_right, bottom));
		Vertices.Push(FVector(f_wall_right, bottom + door_height));

		Vertices.Push(FVector(f_inset_right, bottom));
		Vertices.Push(FVector(f_inset_right, bottom + door_height));

		Vertices.Push(FVector(f_wall_left, top));
		Vertices.Push(FVector(f_wall_right, top));
	}
	else {
		Vertices.Push(FVector(convert(wall_left), bottom));
		Vertices.Push(FVector(convert(wall_left), top));

		Vertices.Push(FVector(f_inset_right, bottom));
		Vertices.Push(FVector(f_inset_right, top));

		Vertices.Push(FVector(convert(wall_right), bottom));
		Vertices.Push(FVector(convert(wall_right), top));

		Vertices.Push(FVector(f_inset_right, bottom));
		Vertices.Push(FVector(f_inset_right, top));
	}

	//left inset
	Triangles.Push(0);
	Triangles.Push(1);
	Triangles.Push(2);
	Triangles.Push(2);
	Triangles.Push(1);
	Triangles.Push(3);

	//right inset
	Triangles.Push(7);
	Triangles.Push(5);
	Triangles.Push(6);
	Triangles.Push(6);
	Triangles.Push(5);
	Triangles.Push(4);

	//bottom inset
	Triangles.Push(0);
	Triangles.Push(2);
	Triangles.Push(4);
	Triangles.Push(4);
	Triangles.Push(2);
	Triangles.Push(6);

	//top inset
	Triangles.Push(1);
	Triangles.Push(5);
	Triangles.Push(3);
	Triangles.Push(3);
	Triangles.Push(5);
	Triangles.Push(7);

	//top panel
	if (top - bottom > door_height) {
		Triangles.Push(1);
		Triangles.Push(8);
		Triangles.Push(5);
		Triangles.Push(5);
		Triangles.Push(8);
		Triangles.Push(9);
	}

	Normals.Push(f_normal);
	Normals.Push(f_normal);
	Normals.Push(f_normal);
	Normals.Push(f_normal);
	Normals.Push(f_normal);
	Normals.Push(f_normal);
	Normals.Push(f_normal);
	Normals.Push(f_normal);
	Normals.Push(f_normal);
	Normals.Push(f_normal);

	UV0.Push(FVector2D(0, bottom));
	UV0.Push(FVector2D(0, top));
	UV0.Push(FVector2D(1, bottom));
	UV0.Push(FVector2D(1, top));
	UV0.Push(FVector2D(0, bottom));
	UV0.Push(FVector2D(0, top));
	UV0.Push(FVector2D(1, bottom));
	UV0.Push(FVector2D(1, top));
	UV0.Push(FVector2D(1, bottom));
	UV0.Push(FVector2D(1, top));

	Tangents.Push(FProcMeshTangent(0, 0, 1));
	Tangents.Push(FProcMeshTangent(0, 0, 1));
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
	VertexColors.Push(color);
	VertexColors.Push(color);

	component->CreateMeshSection_LinearColor(section_id, Vertices, Triangles, Normals, UV0, VertexColors, Tangents, false);
}
void Aroom_description_builder::CreateWindowSegment(Edge<Pgrd> const * target, float bottom, float top,
	UProceduralMeshComponent * component, int section_id) {

}

void Aroom_description_builder::CreateFloorAndCeiling(Region<Pgrd> * source, float bottom, float top) {
	auto Border = toFVector(generateInsetPoints(source->getBounds().last(), grd(wall_thickness/2)));

	Border = tri_utils::Reverse(Border);

	TArray<int32> Index_Faked;
	Index_Faked.SetNum(Border.Num());
	for (int32 ii = 0; ii < Border.Num(); ii++) {
		Index_Faked[ii] = ii;
	}
	TArray<int32> Triangles_top = tri_utils::Triangulate(Border, Index_Faked);
	TArray<int32> Triangles_bottom = tri_utils::Reverse(Triangles_top);

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
		vertices_bottom.Add(FVector(vector.X, vector.Y, bottom));
		vertices_top.Add(FVector(vector.X, vector.Y, top));
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

	Floor_Mesh->SetMaterial(0, Floor_Material);
	Floor_Mesh->SetMaterial(1, Ceiling_Material);

	Floor_Mesh->RegisterComponentWithWorld(GetWorld());

}
void Aroom_description_builder::CreateWallSections(Region<Pgrd> * source, float bottom, float top, Type_Tracker & tracker) {

	float door_tolerance = door_width + wall_thickness;

	for (auto border : source->getBounds()) {
		auto border_points = border->getLoopEdges();

		for (auto edge : border_points) {

			Pgrd const A = edge->getStart()->getPosition();
			Pgrd const B = edge->getEnd()->getPosition();

			auto segment = B - A;
			grd size = segment.Size();

			Region<Pgrd> * op = edge->getInv()->getFace()->getGroup();
			auto component = CreateMeshComponent();

			if (edge->mark == 0) {
				if (size <= door_tolerance || op == nullptr || op->mark == 0) {
					CreateWallSegment(edge, bottom, top, component, 0);
					component->SetMaterial(0, Wall_Material);
					edge->mark = 1;
					edge->getInv()->mark = 1;
				}
				else {
					auto mid_point = (segment / 2) + A;
					segment.Normalize();
					segment *= door_width / 2;

					edge->subdivide(mid_point + segment);
					edge->subdivide(mid_point - segment);
					
					auto middle = edge->getNext();
					auto opposite = middle->getNext();

					CreateWallSegment(edge, bottom, top, component, 0);
					CreateDoorSegment(middle, bottom, top, component, 1);
					CreateWallSegment(opposite, bottom, top, component, 2);
					component->SetMaterial(0, Wall_Material);
					component->SetMaterial(1, Wall_Material);
					component->SetMaterial(2, Wall_Material);

					edge->mark = 1;
					edge->getInv()->mark = 1;

					middle->mark = 2;
					middle->getInv()->mark = 2;

					opposite->mark = 1;
					opposite->getInv()->mark = 1;
				}
			}
			else if (edge->mark == 1) {
				CreateWallSegment(edge, bottom, top, component, 0);
				component->SetMaterial(0, Wall_Material);
			}
			else {
				CreateDoorSegment(edge, bottom, top, component, 0);
				component->SetMaterial(0, Wall_Material);
			}

			ActivateMeshComponent(component);
		}
	}
}

void Aroom_description_builder::Create_System(Type_Tracker & tracker) {

	tracker.system->resetEdgeMarks();
	tracker.system->resetRegionMarks();

	for (auto ext : tracker.Exteriors) {
		cleanRegion(ext);
	}

	for (auto room : tracker.Rooms){
		room->mark = 1;
		cleanRegion(room);
	}

	for (auto hall : tracker.Halls) {
		hall->mark = 1;
		cleanRegion(hall);
	}

	for (auto ext : tracker.Exteriors) {
		CreateWallSections(ext, 0, room_height, tracker);
	}

	for (auto room : tracker.Rooms) {
		CreateFloorAndCeiling(room, 0, room_height);
		CreateWallSections(room, 0, room_height, tracker);
	}

	for (auto hall : tracker.Halls) {
		CreateFloorAndCeiling(hall, 0, room_height);
		CreateWallSections(hall, 0, room_height, tracker);
	}
}

//==========================================================================================================
//======================================= generation =======================================================
//==========================================================================================================

namespace shape_generators
{
	FLL<Pgrd> Square_Generator(grd x, grd y, Pgrd center) {
		FLL<Pgrd> boundary;
		boundary.append(Pgrd(-x, -y) + center);
		boundary.append(Pgrd(-x, y) + center);
		boundary.append(Pgrd(x, y) + center);
		boundary.append(Pgrd(x, -y) + center);

		return boundary;
	}
	FLL<Pgrd> Diamond_Generator(grd x, grd y, Pgrd center) {
		FLL<Pgrd> boundary;
		if (x < y) {
			grd d = y - x;
			boundary.append(Pgrd(-x, -d) + center);
			boundary.append(Pgrd(-x, d) + center);
			boundary.append(Pgrd(0, y) + center);
			boundary.append(Pgrd(x, d) + center);
			boundary.append(Pgrd(x, -d) + center);
			boundary.append(Pgrd(0, -y) + center);
		}
		else if (x > y) {
			grd d = x - y;
			boundary.append(Pgrd(-x, 0) + center);
			boundary.append(Pgrd(-d, y) + center);
			boundary.append(Pgrd(d, y) + center);
			boundary.append(Pgrd(x, 0) + center);
			boundary.append(Pgrd(d, -y) + center);
			boundary.append(Pgrd(-d, -y) + center);
		}
		else {
			boundary.append(Pgrd(-x, 0) + center);
			boundary.append(Pgrd(0, x) + center);
			boundary.append(Pgrd(x, 0) + center);
			boundary.append(Pgrd(0, -x) + center);
		}


		return boundary;
	}
	FLL<Pgrd> Bevel_Generator(grd x, grd y, Pgrd center) {
		grd level;
		if (x < y) {
			level = x / 4;
		}
		else {
			level = y / 4;
		}

		FLL<Pgrd> boundary;

		boundary.append(Pgrd(-x, level - y) + center);
		boundary.append(Pgrd(-x, y - level) + center);
		boundary.append(Pgrd(level - x, y) + center);
		boundary.append(Pgrd(x - level, y) + center);
		boundary.append(Pgrd(x, y - level) + center);
		boundary.append(Pgrd(x, level - y) + center);
		boundary.append(Pgrd(x - level, -y) + center);
		boundary.append(Pgrd(level - x, -y) + center);

		return boundary;
	}
	FLL<Pgrd> Rectangle_Generator(grd x, grd y, Pgrd center) {
		grd skew = (int64)FMath::RandRange((float)0, (float)10);
		Pgrd angle(skew / 10, (grd(10) - skew) / 10);
		Pgrd perp((skew - 10) / 10, skew / 10);

		FLL<Pgrd> boundary;

		boundary.append(center - angle * x - perp * y);
		boundary.append(center - angle * x + perp * y);
		boundary.append(center + angle * x + perp * y);
		boundary.append(center + angle * x - perp * y);

		return boundary;
	}

	typedef FLL<Pgrd>(*generatorFunc)(grd, grd, Pgrd);
}

FLL<Pgrd> Pick_Generator(grd x, grd y, Pgrd center) {
	using namespace shape_generators;

	generatorFunc generator = Square_Generator;
	float gen_choice = FMath::RandRange(0, 1);
	if (gen_choice > .4) {
		if (gen_choice > .7) {
			generator = Bevel_Generator;
			UE_LOG(LogTemp, Warning, TEXT("style: bevel\n"));
		}
		else {
			generator = Diamond_Generator;
			UE_LOG(LogTemp, Warning, TEXT("style: diamond\n"));
		}
	}
	else {
		UE_LOG(LogTemp, Warning, TEXT("style: square\n"));
	}

	return generator(x, y, center);
}



Region_List Type_Tracker::createRoom(Region_Suggestion const &suggested) {
	Region_List final_room_set;

	for (auto boundary : suggested.boundaries) {
		allocateBoundaryFromInto(*boundary, Nulls, final_room_set);
	}

	UE_LOG(LogTemp, Warning, TEXT("room smalls"));
	removeSmallSections(final_room_set, min_room_width, Nulls);

	UE_LOG(LogTemp, Warning, TEXT("hall smalls"));
	removeSmallSections(Nulls, min_hall_width, Smalls);

	Rooms.append(final_room_set);

	return final_room_set;
}

Region_List Type_Tracker::createHall(Region_Suggestion const &suggested) {
	Region_List final_hall_set;

	for (auto boundary : suggested.boundaries) {
		allocateBoundaryFromInto(*boundary, Nulls, final_hall_set);
	}

	removeSmallSections(final_hall_set, min_hall_width, Nulls);
	removeSmallSections(Nulls, min_hall_width, Smalls);

	Halls.append(final_hall_set);

	return final_hall_set;
}

Region_List Type_Tracker::createNull(Region_Suggestion const &suggested) {
	Region_List final_null_set;

	for (auto boundary : suggested.boundaries) {
		allocateBoundaryFromInto(*boundary, Exteriors, final_null_set);
	}

	removeSmallSections(final_null_set, min_room_width, Exteriors);

	Nulls.append(final_null_set);

	return final_null_set;
}

bool Region_Suggestion::contains(Pgrd const &test) {
	for (auto region : boundaries)
		if (getPointRelation(*region, test) != point_exterior)
			return true;
	return false;
}

void clusterSuggestions(FLL<Region_Suggestion*> &suggested, grd const &tolerance) {
	auto x = suggested.begin();
	for (auto x = suggested.begin(); x != suggested.end();++x) {
		for (auto y = x.next(); y != suggested.end();) {
			//try and find a pair of centroids within tolerance
			bool seperate = true;
			for (auto x_p : x->centroids) {
				for (auto y_p : y->centroids) {
					//if ((x_p - y_p).Size() < tolerance)
					if(x->contains(y_p) && y->contains(x_p)) {
						seperate = false;
						break;
					}
				}
				if (!seperate)
					break;
			}


			//if found, merge all of y into x
			if (seperate)
				++y;
			else {
				x->centroids.absorb(y->centroids);
				x->boundaries.absorb(y->boundaries);

				suggested.remove(*y);

				y = x.next();
			}
		}
	}
}

FLL<Region_Suggestion*> suggestDistribution(Pgrd const &A, Pgrd const &B, grd const &room_width, grd const &room_depth, grd const &min_hall_width, bool start_row = true, bool end_row = true) {
	FLL<Region_Suggestion*> result;

	Pgrd dir = B - A;
	dir.Normalize();
	Pgrd par(dir.Y, -dir.X);
	par *= (room_depth + min_hall_width / 2);

	grd full_segment = (B - A).Size() + (room_width * 2) - min_hall_width;
	int rooms = (full_segment / room_width).n;
	grd segment = full_segment / rooms;


	int i = 0;
	if (!start_row)
		i++;

	if (!end_row)
		rooms--;

	for (; i < rooms; i++) {
		grd offset = (segment * i) - room_width + min_hall_width / 2;
		Pgrd root = A + (dir * offset);

		{

			Region_Suggestion * suggest = new Region_Suggestion();

			FLL<Pgrd> * bounds = new FLL<Pgrd>();
			bounds->append(root);
			bounds->append(root + (dir * segment));
			bounds->append(root + par + (dir * segment));
			bounds->append(root + par);

			suggest->boundaries.append(bounds);
			suggest->centroids.append(root + (par / 2) + (dir * segment / 2));

			result.append(suggest);
		}

		{
			Region_Suggestion * suggest = new Region_Suggestion();

			FLL<Pgrd> * bounds = new FLL<Pgrd>();
			bounds->append(root - par);
			bounds->append(root - par + (dir * segment));
			bounds->append(root + (dir * segment));
			bounds->append(root);

			suggest->boundaries.append(bounds);
			suggest->centroids.append(root - (par / 2) + (dir * segment / 2));

			result.append(suggest);
		}
	}

	return result;
}

FLL<Pgrd> * wrapSegment(Pgrd const &A, Pgrd const &B, grd const &extent_perp, grd const &extent_ends) {
	FLL<Pgrd> * result = new FLL<Pgrd>();

	Pgrd dir = B - A;
	dir.Normalize();

	Pgrd par(-dir.Y, dir.X);

	dir *= extent_ends;
	par *= extent_perp;

	result->append(A - dir - par);
	result->append(A - dir + par);
	result->append(B + dir + par);
	result->append(B + dir - par);

	return result;
}

void Aroom_description_builder::buldingFromBlock(Type_Tracker &frame, FLL<rigid_line> &list) {
	
	UE_LOG(LogTemp, Warning, TEXT("Building Generation\n\n"));
	Region_Suggestion null_suggestion;

	for (auto x : list) {

		FLL<Pgrd> * null_boundary = wrapSegment(x.start, x.end, room_depth + min_hall_width / 2, room_depth - min_hall_width / 2);

		null_suggestion.boundaries.append(null_boundary);
	}

	frame.createNull(null_suggestion);



	UE_LOG(LogTemp, Warning, TEXT("Hall Generation\n\n\n"));
	Region_Suggestion hall_suggestion;

	for (auto x : list) {

		FLL<Pgrd> * hall_boundary = wrapSegment(x.start, x.end, hall_width / 2, hall_width / 2);

		hall_suggestion.boundaries.append(hall_boundary);
	}

	frame.createHall(hall_suggestion);



	UE_LOG(LogTemp, Warning, TEXT("Stuff Generation\n\n\n"));

	FLL<Region_Suggestion *> room_list;

	for (auto x : list) {
		auto p = suggestDistribution(x.start, x.end, room_width, room_depth, min_hall_width, x.start_row, x.end_row);
		room_list.absorb(p);
	}

	for (auto room_suggestion : room_list) {
		for (auto p : room_suggestion->centroids)
			DrawDebugLine(
				GetWorld(),
				FVector(convert(p), 10),
				FVector(convert(p), 20),
				color_blue,
				true,
				-1,
				0,
				5
			);
	}

	clusterSuggestions(room_list, room_width);


	UE_LOG(LogTemp, Warning, TEXT("ROOMS\n"));
	for (auto room_suggestion : room_list) {
		FColor color(FMath::RandRange(0, 255), FMath::RandRange(0, 255), FMath::RandRange(0, 255));
		for (auto p : room_suggestion->centroids) {
			UE_LOG(LogTemp, Warning, TEXT("room at : %f, %f"), p.X.n, p.Y.n);
			DrawDebugLine(
				GetWorld(),
				FVector(convert(p), 20),
				FVector(convert(p), 30),
				color,
				true,
				-1,
				0,
				5
			);
		}

		//frame.createRoom(*room_suggestion);

		//for (auto p : room_suggestion->boundaries)
		//	Draw_Border(convert(*p), 30, GetWorld(), color);

		for(auto r : frame.createRoom(*room_suggestion))
			for (auto p : r->getBounds())
				Draw_Border(convert(p->getLoopPoints()), 50, GetWorld(), color);
	}
	UE_LOG(LogTemp, Warning, TEXT("SMALLS\n"));

	frame.Smalls.absorb(frame.Nulls);

	//display leftovers
	{
		for (auto n : frame.Smalls)
			for (auto p : n->getBounds())
				Draw_Border(convert(p->getLoopPoints()), 60, GetWorld(), FColor(0, 200, 0));
	}

	Region_List all_smalls;
	Region_List smalls;

	for (auto small : frame.Smalls) {

		//filters for neighboring rooms, removes them from frame consideration for potential edits
		Region_List neighbors = small->getNeighbors();
		Region_List room_neighbors;
		for (auto neighbor : neighbors)
			if (frame.Rooms.remove(neighbor))
				room_neighbors.append(neighbor);

		Region_List rooms;
		

		smalls.append(small);

		Region_List novel_smalls;
		Region_List relevant;

		for (auto potential : room_neighbors) {
			relevant.append(potential);

			for (auto part : smalls) {
				if (!merge(potential, part)) {
					novel_smalls.append(part);
				}
			}

			removeSmallSections(relevant, min_room_width, novel_smalls);

			smalls.clear();

			rooms.absorb(relevant);
			smalls.absorb(novel_smalls);
		}

		frame.Rooms.absorb(rooms);
		all_smalls.absorb(smalls);
	}

	frame.Smalls.clear();
	frame.Smalls.absorb(all_smalls);
}

//==========================================================================================================
//====================================== member specific ===================================================
//==========================================================================================================

void Aroom_description_builder::Main_Generation_Loop() {
	UE_LOG(LogTemp, Warning, TEXT("Main Generation"));
	
	if (use_static_seed)
		FMath::RandInit(random_seed);

	/*DCEL<Pgrd> system_new;
	Type_Tracker system_types(&system_new);

	

	UE_LOG(LogTemp, Warning, TEXT("SRand Seed %d\n"), random_seed);

	auto system_bounds = shape_generators::Square_Generator(100, 100, Pgrd(0, 0));

	system_types.Nulls.append(system_new.region(system_bounds));

	create_Layout(system_types, unalligned_count, alligned_count, GetWorld());*/

	DCEL<Pgrd> * system = new DCEL<Pgrd>();
	Type_Tracker frame(system, min_room_width, min_hall_width);

	FLL<rigid_line> list;
	for (auto p : Lines)
		list.append(rigid_line(p));

	buldingFromBlock(frame, list);

	Create_System(frame);

	delete system;
}

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

	use_static_seed = false;
	random_seed = 0;

	rooms_per_segment = 10;
	closets_per_segment = 6;

	wall_thickness = 10;
	room_height = 100;
	door_height = 80;

	min_room_width = 18;
	min_hall_width = 12;

	room_width = 54;
	room_depth = 54;
	hall_width = 12;

	door_width = 3;

	Wall_Material = nullptr;
	Floor_Material = nullptr;
	Ceiling_Material = nullptr;
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