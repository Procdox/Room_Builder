// Fill out your copyright notice in the Description page of Project Settings.

#include "room_description_builder.h"
#include "Algo/Reverse.h"
#include "DrawDebugHelpers.h"

//==========================================================================================================
//========================================== transforms ====================================================
//==========================================================================================================

FVector2D convert(Pgrd const &target) {
	return FVector2D(target.X.n * 10, target.Y.n * 10);
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
			7
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

namespace chord_splits
{

	struct split_canidate 
	{
		Edge<Pgrd> * A_edge;
		Edge<Pgrd> * B_edge;
		Pgrd A;
		Pgrd B;
		grd distance;
	};

	//returns if test is between A and B clockwise (right about the origin from A, left about from B)
	bool betweenVectors(Pgrd const &A, Pgrd const &B, Pgrd const &test) {

		Pgrd A_inward(A.Y, -A.X);
		Pgrd B_inward(-B.Y, B.X);

		grd bounds_relation = A_inward.Dot(B);

		if (bounds_relation > 0) {
			//the angle between bounds is in (0,180)
			return A_inward.Dot(test) > 0 && B_inward.Dot(test) > 0;
		}
		else if (bounds_relation == 0) {
			//the angle between bounds is 180 or 0, or one bound is length 0
			//any case other than 180 is due to an error as used for determine interiors

			return A_inward.Dot(test) > 0;
		}
		else {
			//the angle between bounds is in (180,360)
			return A_inward.Dot(test) > 0 || B_inward.Dot(test) > 0;
		}
	}

	struct bvs {
		Pgrd A_inward;
		Pgrd B_inward;
		grd bounds_relation;

		bool test(Pgrd const &test) const {
			if (bounds_relation > 0) {
				//the angle between bounds is in (0,180)
				return A_inward.Dot(test) > 0 && B_inward.Dot(test) > 0;
			}
			else if (bounds_relation == 0) {
				//the angle between bounds is 180 or 0, or one bound is length 0
				//any case other than 180 is due to an error as used for determine interiors

				return A_inward.Dot(test) > 0;
			}
			else {
				//the angle between bounds is in (180,360)
				return A_inward.Dot(test) > 0 || B_inward.Dot(test) > 0;
			}
		}

		bvs(Pgrd const &A, Pgrd const &B) {
			A_inward.X = A.Y;
			A_inward.Y = -A.X;

			B_inward.X = -B.Y;
			B_inward.Y = B.X;

			bounds_relation = A_inward.Dot(B);
		}
	};

//#define debug_chords

	void chord_clean(Region<Pgrd> * target, grd const & thresh, FLL<Region<Pgrd> *> & ins, FLL<Region<Pgrd> *> & outs) {
		//if no pair is small enough to split, add to ins and return
		split_canidate result;
		grd diameter = thresh+1;
		bool found_option = false;

		FLL<Edge<Pgrd> *> relevants;

		for (auto border : target->getBounds()) {
			border->getLoopEdges(relevants);
		}
#ifdef debug_chords
		UE_LOG(LogTemp, Warning, TEXT("cleaning..."));
#endif


		grd min_offset, max_offset;

		for (auto edge : relevants) {

			Pgrd const A_start = edge->getEnd()->getPosition();
			Pgrd const A_before = edge->getStart()->getPosition();
			Pgrd const A_after = edge->getNext()->getEnd()->getPosition();

			Pgrd const A = A_start - A_before;

			bvs const A_angle(A_after - A_start, A_before - A_start);

			min_offset = linear_offset(A, A_start);
			max_offset = min_offset;

			//ear clip attempt
			{
				Pgrd const B_start = edge->getNext()->getNext()->getEnd()->getPosition();
				Pgrd const B_end = edge->getNext()->getNext()->getNext()->getEnd()->getPosition();

				Pgrd A_normal = A;
				A_normal.Normalize();

				Pgrd B_normal = B_end - B_start;
				B_normal.Normalize();

				Pgrd const segment = B_start - A_start;

				if (A_normal == B_normal && A_angle.test(segment)) {

					grd const distance = segment.Size();

					if (distance < thresh)
						if (!found_option || distance < result.distance) {
							found_option = true;
							result.distance = distance;

							result.A = A_start;
							result.B = B_start;

#ifdef debug_chords
							UE_LOG(LogTemp, Warning, TEXT("split: (%f,%f) to (%f,%f), distance %f"), result.A.X.n,
								result.A.Y.n, result.B.X.n, result.B.Y.n, result.distance.n);
#endif

							result.A_edge = edge;
							result.B_edge = edge->getNext()->getNext();
						}

				}
			}

			for (auto compare : relevants) {
				Pgrd const B_start = compare->getStart()->getPosition();

				grd const raw = linear_offset(A, B_start);

				if (min_offset > raw)
					min_offset = raw;

				if (max_offset < raw)
					max_offset = raw;

				if (compare == edge || compare == edge->getNext()) {
					continue;
				}

				//get smallest points
				
				Pgrd const B_end = compare->getEnd()->getPosition();

				Pgrd const B_segment = B_end - B_start;
				Pgrd const B_perp(-B_segment.Y, B_segment.X);

				if (!A_angle.test(B_perp))
					continue;

				Pgrd const offset = A_start - B_start;

				if (B_perp.Dot(offset) >= 0)
					continue;

				Pgrd intersect;

				if (offset.Dot(B_end - B_start) <= 0) {
					intersect = B_start;
					Pgrd const B_last = compare->getLast()->getStart()->getPosition();

					if (!betweenVectors(B_end - B_start, B_last - B_start, offset))
						continue;
				}
				else if ((A_start - B_end).Dot(B_start - B_end) <= 0){
					intersect = B_end;

					Pgrd const B_next = compare->getNext()->getEnd()->getPosition();

					if (!betweenVectors(B_next - B_end, B_start - B_end, A_start - B_end))
						continue;
				}
				else
					Pgrd::getIntersect(B_start, B_end, A_start, A_start + B_perp, intersect);

				Pgrd const segment = intersect - A_start;

				if (!A_angle.test(segment))
					continue;

				grd const distance = segment.Size();

				if(distance < thresh)
					if (!found_option || distance < result.distance) {						
						found_option = true;
						result.distance = distance;
						
						result.A = A_start;
						result.B = intersect;

#ifdef debug_chords
						UE_LOG(LogTemp, Warning, TEXT("split: (%f,%f) to (%f,%f), distance %f"), result.A.X.n,
							result.A.Y.n, result.B.X.n, result.B.Y.n, result.distance.n);
#endif

						result.A_edge = edge;
						result.B_edge = compare;
					}
			}

			grd const t = max_offset - min_offset;
			grd const offset = t * t * (A.X * A.X + A.Y * A.Y);
#ifdef debug_chords
			UE_LOG(LogTemp, Warning, TEXT("offset: %f"), offset.n);
#endif
			if (offset < diameter)
				diameter = offset;
		}
		if (diameter <= thresh) {
#ifdef debug_chords
			UE_LOG(LogTemp, Warning, TEXT("removed... \n"));
#endif
			outs.push(target);
		}
		else if (found_option) {
			if (result.B == result.B_edge->getStart()->getPosition()) {
				result.B_edge = result.B_edge->getLast();
			}else if (result.B != result.B_edge->getEnd()->getPosition()) {
				//in-line, subdivide
				result.B_edge->subdivide(result.B);
			}
			//split now occurs at end of A_edge and B_edge

#ifdef debug_chords
			UE_LOG(LogTemp, Warning, TEXT("subdividing \n"));
			//split
			UE_LOG(LogTemp, Warning, TEXT("Orig"));
			for (auto face : target->getBounds()) {
				UE_LOG(LogTemp, Warning, TEXT("Face >k-"));
				for (auto point : face->getLoopPoints()) {
					UE_LOG(LogTemp, Warning, TEXT("(%f,%f)"), point.X.n, point.Y.n);
				}
			}
#endif
			auto P = RegionAdd(target, result.A_edge, result.B_edge);

#ifdef debug_chords
			UE_LOG(LogTemp, Warning, TEXT("Result target"));
			for (auto face : target->getBounds()) {
				UE_LOG(LogTemp, Warning, TEXT("Face >r:"));
				for (auto point : face->getLoopPoints()) {
					UE_LOG(LogTemp, Warning, TEXT("(%f,%f)"), point.X.n, point.Y.n);
				}
			}
			if (P != nullptr) {
				UE_LOG(LogTemp, Warning, TEXT("Result P"));
				for (auto face : P->getBounds()) {
					UE_LOG(LogTemp, Warning, TEXT("Face >b:"));
					for (auto point : face->getLoopPoints()) {
						UE_LOG(LogTemp, Warning, TEXT("(%f,%f)"), point.X.n, point.Y.n);
					}
				}
			}

			UE_LOG(LogTemp, Warning, TEXT(" \n"));
#endif
			

			chord_clean(target, thresh, ins, outs);

			if (P != nullptr) {
				chord_clean(P, thresh, ins, outs);
			}
		}
		else {
#ifdef debug_chords
			UE_LOG(LogTemp, Warning, TEXT("added... \n"));
#endif
			ins.push(target);
		}
	}
}

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
//================================== generation utilities ==================================================
//==========================================================================================================

void mergeGroup(FLL<Region<Pgrd> *> & nulls) {
	UE_LOG(LogTemp, Warning, TEXT("Merging Group\n"));
	for (auto focus = nulls.begin(); focus != nulls.end(); ++focus) {
		merge(*focus, *focus);

		for (auto compare = focus.next(); compare != nulls.end();) {
			auto v = *compare;

			++compare;

			if (merge(*focus, v))
				nulls.remove(v);
		}
	}
}

void Cull(Region<Pgrd> * target, grd const &width, FLL<Region<Pgrd> *> &ins, FLL<Region<Pgrd> *> &outs) {
	UE_LOG(LogTemp, Warning, TEXT("Culling\n"));

	FLL<Region<Pgrd> *> _ins;
	FLL<Region<Pgrd>*> _outs;

	cleanRegion(target);

	chord_splits::chord_clean(target, width, _ins, _outs);

	outs.absorb(_outs);
	ins.absorb(_ins);

	mergeGroup(outs);
}

void Cull(FLL < Region<Pgrd> *> &targets, grd const &width, FLL<Region<Pgrd> *> &outs) {
	UE_LOG(LogTemp, Warning, TEXT("Culling\n"));

	FLL<Region<Pgrd> *> results;

	for (auto target : targets) {
		Cull(target, width, results, outs);
	}

	targets.clear();
	targets.absorb(results);
}

FLL<Region<Pgrd> *> Type_Tracker::cleanNulls(FLL<Region<Pgrd> *> &input_nulls) {
	UE_LOG(LogTemp, Warning, TEXT("Clean Nulls\n"));

	FLL<Region<Pgrd> *> result_nulls;
	FLL<Region<Pgrd> *> input_halls;

	//input_nulls.absorb(target.Nulls);
	mergeGroup(input_nulls);

	for (auto focus : input_nulls) {

		FLL<Region<Pgrd> *> room_ins;
		FLL<Region<Pgrd> *> room_outs;

		cleanRegion(focus);

		chord_splits::chord_clean(focus, min_room_width, room_ins, room_outs);

		for (auto null : room_ins) {
			merge(null, null);
		}

		result_nulls.absorb(room_ins);

		input_halls.absorb(room_outs);
	}

	UE_LOG(LogTemp, Warning, TEXT("Clean Halls\n"));

	for (auto focus : input_halls) {

		FLL<Region<Pgrd> *> hall_ins;
		FLL<Region<Pgrd> *> hall_outs;

		cleanRegion(focus);

		chord_splits::chord_clean(focus, min_hall_width, hall_ins, hall_outs);

		Halls.absorb(hall_ins);
		Smalls.absorb(hall_outs);
	}

	mergeGroup(Halls);
	mergeGroup(Smalls);

	for (auto region : Rooms)
		cleanRegion(region);

	for (auto region : Halls)
		cleanRegion(region);

	for (auto region : Smalls)
		cleanRegion(region);

	return result_nulls;
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

	component->CreateMeshSection_LinearColor(section_id, Vertices, Triangles, Normals, UV0, VertexColors, Tangents, true);
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

	Floor_Mesh->RegisterComponentWithWorld(GetWorld());

}
void Aroom_description_builder::CreateWallSections(Region<Pgrd> * source, float bottom, float top, Type_Tracker & tracker) {

	float door_tolerance = door_width + wall_thickness;
	UE_LOG(LogTemp, Warning, TEXT("allowed width: %f"), door_tolerance);

	for (auto border : source->getBounds()) {
		auto border_points = border->getLoopEdges();

		for (auto edge : border_points) {

			Pgrd const A = edge->getStart()->getPosition();
			Pgrd const B = edge->getEnd()->getPosition();

			auto segment = B - A;
			float size = segment.Size();

			Region<Pgrd> * op = edge->getInv()->getFace()->getGroup();
			auto component = CreateMeshComponent();

			if (edge->mark == 0) {
				if (size <= door_tolerance || op == nullptr || op->mark == 0) {
					CreateWallSegment(edge, bottom, top, component, 0);
					edge->mark = 1;
					edge->getInv()->mark = 1;
				}
				else {
					UE_LOG(LogTemp, Warning, TEXT("door width: %f"), size);
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
			}
			else {
				CreateDoorSegment(edge, bottom, top, component, 0);
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

FLL<Region<Pgrd> *> allocateBoundaryFrom(FLL<Pgrd> const &boundary, FLL<Region<Pgrd> *> &set) {
	FLL<Region<Pgrd> *> final_ins;
	FLL<Region<Pgrd> *> final_outs;

	for (auto member : set) {
		FLL<Region<Pgrd> *> created_ins;
		FLL<Region<Pgrd> *> created_outs;

		subAllocate(member, boundary, created_outs, created_ins);

		final_ins.absorb(created_ins);
		final_outs.absorb(created_outs);
	}

	set.clear();

	set.absorb(final_outs);

	return final_ins;
}

FLL<Region<Pgrd> *> allocateCleanedBoundaryFrom(FLL<Pgrd> const &boundary, double min_width, FLL<Region<Pgrd> *> &set) {
	FLL<Region<Pgrd> *> final_ins;
	FLL<Region<Pgrd> *> final_outs;

	for (auto member : set) {
		FLL<Region<Pgrd> *> created_ins;
		FLL<Region<Pgrd> *> created_outs;

		FLL<Region<Pgrd> *> raw_ins;

		subAllocate(member, boundary, created_outs, raw_ins);

		for (auto in : raw_ins) {
			Cull(in, min_width, created_ins, created_outs);
		}

		final_ins.absorb(created_ins);
		final_outs.absorb(created_outs);
	}

	set.clear();

	set.absorb(final_outs);

	return final_ins;
}

FLL<Region<Pgrd> *> Type_Tracker::createRoomFromBoundary(FLL<Pgrd> const &boundary) {

	FLL<Region<Pgrd> *> final_room_set = allocateCleanedBoundaryFrom(boundary, min_room_width, Nulls);

	Nulls = cleanNulls(Nulls);

	Rooms.append(final_room_set);

	return final_room_set;
}

FLL<Region<Pgrd> *> Type_Tracker::createHallFromBoundary(FLL<Pgrd> const &boundary) {

	FLL<Region<Pgrd> *> final_hall_set = allocateCleanedBoundaryFrom(boundary, min_hall_width, Nulls);

	Nulls = cleanNulls(Nulls);

	Halls.append(final_hall_set);

	return final_hall_set;
}

FLL<Region<Pgrd> *> Type_Tracker::createNullFromBoundary(FLL<Pgrd> const &boundary) {

	FLL<Region<Pgrd> *> final_null_set = allocateBoundaryFrom(boundary, Exteriors);

	Nulls.append(final_null_set);

	return final_null_set;
}

FLL<Region<Pgrd> *> createRoomAtPoint(Type_Tracker &system_types, Pgrd const &point, int64 scale = 1) {
	UE_LOG(LogTemp, Warning, TEXT("Create Room At Point %f,%f"), point.X.n, point.Y.n);

	int64 area = FMath::RandRange(6, 14);
	int64 width = FMath::RandRange(2, area / 2);
	int64 length = area / width;

	UE_LOG(LogTemp, Warning, TEXT("size: %d %d"), width, length);

	auto bounds = Pick_Generator(width * scale, length * scale, point);

	return system_types.createRoomFromBoundary(bounds);
}

FLL<Region<Pgrd> *> createRectangleAtPoint(Type_Tracker &system_types, Pgrd const &point, int64 scale = 1) {
	UE_LOG(LogTemp, Warning, TEXT("Create Rectangle At Point %f,%f"),point.X.n, point.Y.n);

	int64 area = FMath::RandRange(6, 14);
	int64 width = FMath::RandRange(2, area / 2);
	int64 length = area / width;

	UE_LOG(LogTemp, Warning, TEXT("size: %d %d"), width, length);

	auto bounds = shape_generators::Rectangle_Generator(width * scale, length * scale, point);

	return system_types.createRoomFromBoundary(bounds);
}

FLL<Pgrd> wrapSegment(Pgrd const &A, Pgrd const &B, grd const &radius) {
	FLL<Pgrd> result;
	Pgrd dir = B - A;
	dir.Normalize();
	dir *= radius;
	Pgrd par(-dir.Y, dir.X);

	result.append(A - dir - par);
	result.append(A - dir + par);
	result.append(B + dir + par);
	result.append(B + dir - par);

	return result;
}

FLL<Region<Pgrd> *> createRectangleNearSegment(Type_Tracker &system_types, Pgrd const &A, Pgrd const &B, grd const &radius) {

	Pgrd dir = B - A;
	dir.Normalize();
	Pgrd par(dir.Y, -dir.X);

	grd offset = FMath::RandRange(0, (B-A).Size());
	Pgrd root = dir * offset + A;

	int64 area = FMath::RandRange(6, 14);
	int64 width = FMath::RandRange(2, area / 2);
	int64 length = area / width;

	grd d_width = (radius / 4) * width;
	grd d_length = (radius / 4) * length;

	FLL<Pgrd> bounds;
	if (FMath::RandBool()) {
		bounds.append(root - (dir * d_width));
		bounds.append(root + (dir * d_width));
		bounds.append(root + (dir * d_width) + (par * d_length));
		bounds.append(root - (dir * d_width) + (par * d_length));
	}
	else {
		bounds.append(root - (dir * d_width) - (par * d_length));
		bounds.append(root + (dir * d_width) - (par * d_length));
		bounds.append(root + (dir * d_width));
		bounds.append(root - (dir * d_width));
	}

	UE_LOG(LogTemp, Warning, TEXT("Create"));
	for(auto p : bounds)
		UE_LOG(LogTemp, Warning, TEXT("%f,%f"), p.X.n, p.Y.n);

	return system_types.createRoomFromBoundary(bounds);
}

FLL<Region<Pgrd> *> createRoomNearSegment(Type_Tracker &system_types, Pgrd const &A, Pgrd const &B, grd const &radius) {

	Pgrd dir = B - A;
	dir.Normalize();
	Pgrd par(dir.Y, -dir.X);

	grd offset = FMath::RandRange(0, (B - A).Size());
	Pgrd root = dir * offset + A;

	int64 width = FMath::RandRange(2, 8);

	grd d_width = (radius / 8) * width;

	FLL<Pgrd> bounds;
	if (FMath::RandBool()) {
		bounds.append(root - (dir * d_width));
		bounds.append(root + (dir * d_width));
		bounds.append(root + (dir * d_width) + (par * radius));
		bounds.append(root - (dir * d_width) + (par * radius));
	}
	else {
		bounds.append(root - (dir * d_width) - (par * radius));
		bounds.append(root + (dir * d_width) - (par * radius));
		bounds.append(root + (dir * d_width));
		bounds.append(root - (dir * d_width));
	}

	UE_LOG(LogTemp, Warning, TEXT("Create"));
	for (auto p : bounds)
		UE_LOG(LogTemp, Warning, TEXT("%f,%f"), p.X.n, p.Y.n);

	return system_types.createRoomFromBoundary(bounds);
}

FLL<Region<Pgrd> *> createClosetNearSegment(Type_Tracker &system_types, Pgrd const &A, Pgrd const &B, grd const &hall_width, grd const &radius) {

	Pgrd dir = B - A;
	dir.Normalize();
	Pgrd par(dir.Y, -dir.X);

	grd offset = FMath::RandRange(0, (B - A).Size());
	Pgrd root = dir * offset + A;

	int64 width = FMath::RandRange(1, 2);
	int64 length = FMath::RandRange(2, 4);

	grd d_width = (radius / 12) * width;
	grd d_length = (radius / 12) * length;

	FLL<Pgrd> bounds;
	//left or right
	if (FMath::RandBool()) {
		//near or far
		if (FMath::RandBool()) {
			bounds.append(root - (dir * d_width) + (par * hall_width));
			bounds.append(root + (dir * d_width) + (par * hall_width));
			bounds.append(root + (dir * d_width) + (par * (d_length + hall_width)));
			bounds.append(root - (dir * d_width) + (par * (d_length + hall_width)));
		}
		else {
			bounds.append(root - (dir * d_width) + (par * (radius - d_length)));
			bounds.append(root + (dir * d_width) + (par * (radius - d_length)));
			bounds.append(root + (dir * d_width) + (par * radius));
			bounds.append(root - (dir * d_width) + (par * radius));
		}
	}
	else {
		//near or far
		if (FMath::RandBool()) {
			bounds.append(root - (dir * d_width) - (par * (d_length + hall_width)));
			bounds.append(root + (dir * d_width) - (par * (d_length + hall_width)));
			bounds.append(root + (dir * d_width) - (par * hall_width));
			bounds.append(root - (dir * d_width) - (par * hall_width));
		}
		else {
			bounds.append(root - (dir * d_width) - (par * radius));
			bounds.append(root + (dir * d_width) - (par * radius));
			bounds.append(root + (dir * d_width) - (par * (radius - d_length)));
			bounds.append(root - (dir * d_width) - (par * (radius - d_length)));
		}
	}

	UE_LOG(LogTemp, Warning, TEXT("Create"));
	for (auto p : bounds)
		UE_LOG(LogTemp, Warning, TEXT("%f,%f"), p.X.n, p.Y.n);

	return system_types.createHallFromBoundary(bounds);
}

//returns a type structure with halls and nulls representing 

Type_Tracker buldingFromBlock(Type_Tracker &frame, FLL<Pgrd> &A_list, FLL<Pgrd> &B_list, UWorld * ref) {
	
	auto y = B_list.begin();
	for (auto x = A_list.begin(); x != A_list.end();) {

		frame.createNullFromBoundary( wrapSegment(*x, *y, grd(frame.min_hall_width * 3)) );

		++x;
		++y;
	}

	mergeGroup(frame.Nulls);

	Cull(frame.Nulls, grd(frame.min_hall_width * 6), frame.Exteriors);

	mergeGroup(frame.Exteriors);

	y = B_list.begin();
	for (auto x = A_list.begin(); x != A_list.end();) {

		DrawDebugLine(
			ref,
			FVector(convert(*x), 15),
			FVector(convert(*y), 15),
			FColor(FMath::RandRange(0,255), FMath::RandRange(0, 255), FMath::RandRange(0, 255)),
			true,
			-1,
			0,
			7
		);
		auto halls = allocateBoundaryFrom(wrapSegment(*x, *y, grd(frame.min_hall_width / 2)), frame.Nulls);
		frame.Halls.absorb(halls);

		++x;
		++y;
	}

	mergeGroup(frame.Halls);

	Cull(frame.Halls, grd(frame.min_hall_width), frame.Nulls);

	y = B_list.begin();
	for (auto x = A_list.begin(); x != A_list.end();) {

		for (int ii = 0; ii < 0; ii++) {
			createClosetNearSegment(frame, *x, *y, grd(frame.min_hall_width / 2), grd(frame.min_hall_width * 3));
		}

		for (int ii = 0; ii < 3; ii++) {
			createRoomNearSegment(frame, *x, *y, grd(frame.min_hall_width * 3));
		}

		++x;
		++y;
	}

	/*Pgrd A = boxUniformPoint(140, 140) - Pgrd(70, 70);
	Pgrd B;
	do {
		B = boxUniformPoint(140, 140) - Pgrd(70, 70);
	} while ((A - B).Size() < 50);

	frame.Nulls.append(frame.system->region(wrapSegment(A, B, grd(30))));

	frame.createHallFromBoundary(wrapSegment(A, B, grd(7)));

	for (int ii = 0; ii < 10; ii++) {
		createClosetNearSegment(frame, A, B, grd(7), grd(30));
	}

	for (int ii = 0; ii < 20; ii++) {
		createRoomNearSegment(frame, A, B, grd(30));
	}*/

	return frame;
}

void create_Layout(Type_Tracker &system_types, int64 large, int64 medium, UWorld* ref = nullptr) {
	//large is 10-14
	//med is 7-11
	//small is 4-8
	UE_LOG(LogTemp, Warning, TEXT("Create Layout\n"));

	for (int64 ii = 0; ii < large; ii++) {
		//pick point
		auto point = boxUniformPoint(140,140) - Pgrd(70, 70);

		auto temp = createRectangleAtPoint(system_types, point, 7);

		UE_LOG(LogTemp, Warning, TEXT("Created %d sub-rooms \n"), temp.size());
	}

	for (int64 ii = 0; ii < medium; ii++) {
		//pick point
		auto point = boxUniformPoint(140, 140) - Pgrd(70, 70);

		auto temp = createRoomAtPoint(system_types, point, 7);

		UE_LOG(LogTemp, Warning, TEXT("Created %d sub-rooms \n"), temp.size());
	}
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

	FLL<Pgrd> As;
	FLL<Pgrd> Bs;
	for (auto p : A_list)
		As.append(Pgrd(p.X, p.Y));
	for (auto p : B_list)
		Bs.append(Pgrd(p.X, p.Y));

	auto system_types = buldingFromBlock(frame, As, Bs, GetWorld());

	Create_System(system_types);

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

	alligned_count = 5;
	unalligned_count = 30;

	wall_thickness = 10;
	room_height = 100;
	door_height = 80;

	min_room_width = 18;
	min_hall_width = 12;

	door_width = 3;
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