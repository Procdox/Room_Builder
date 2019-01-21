// Fill out your copyright notice in the Description page of Project Settings.

#include "room_description_builder.h"
#include "Algo/Reverse.h"
#include "DrawDebugHelpers.h"

#define room_min_width 300
#define hall_min_width 200

//==========================================================================================================
//========================================== transforms ====================================================
//==========================================================================================================

FVector2D convert(Pint const &target) {
	return FVector2D(target.X.toFloat(), target.Y.toFloat());
}

ClipperLib::Path toPath(FLL<Pint> const &target) {
	ClipperLib::Path product;
	for (auto point : target) {
		auto temp = point * 100;
		product.insert(product.begin(), ClipperLib::IntPoint(temp.X.toFloat(), temp.Y.toFloat()));
	}
	return product;
}

ClipperLib::Paths toPaths(Region<Pint> * target) {
	ClipperLib::Paths product;

	for (auto border : target->getBounds()) {
		product.push_back(toPath(border->getLoopPoints()));
	}

	return product;
}

FLL<Pint> fromPath(ClipperLib::Path const &target, FLL<Pint> const &suggestions) {
	FLL<Pint> product;

	FLL<FLL<Pint>::FLL_iterator_c> leads;

	for (auto point : target) {

		Pint raw(point.X, point.Y);
		raw /= 100;

		rto best_distance = 1;
		best_distance /= 10;

		//Pint choice = raw;

		FLL<Pint>::FLL_iterator_c choice = suggestions.end();

		for (auto compare = suggestions.begin(); compare != suggestions.end(); ++compare) {

			Pint target = *(compare.cyclic_next());

			Pint offset = (raw - target);

			if (offset.Y < 0)
				offset.Y *= -1;
			if (offset.X < 0)
				offset.X *= -1;

			rto distance = offset.Y;

			if (offset.X > offset.Y) {
				distance = offset.X;
			}

			if (distance < best_distance) {
				choice = compare;

				best_distance = distance;

				if (distance == 0) break;
			}
		}

		leads.append(choice);
		//if(choice != suggestions.end())
		//	product.append(*(choice.cyclic_next()));
		//else
		product.append(raw);
	}

	FLL<Pint> result;

	auto focus = product.begin().cyclic_next();
	auto kill = suggestions.end();

	for (auto lead = leads.begin(); lead != leads.end(); ++lead) {

		auto next = lead.cyclic_next();
		auto after = lead.cyclic_next().cyclic_next();

		if (*next != kill) {

			Pint project = *((*next).cyclic_next());
			//*(compare.cyclic_next());
			//*(choice.cyclic_next())
			UE_LOG(LogTemp, Warning, TEXT("point is stable: (%f,%f) => (%f, %f)"),(*focus).X.toFloat(), (*focus).Y.toFloat(), project.X.toFloat(), project.Y.toFloat());
			result.push(project);
			//result.push(*focus);
		}
		else {
			if (*lead != kill) {
				if (*after != kill) {
					
					Pint project;

					Pint A_S = *((*lead).cyclic_next());
					Pint A_E = *((*lead));
					Pint B_S = *((*after).cyclic_next());
					Pint B_E = *((*after).cyclic_next().cyclic_next());

					Pint::getIntersect(A_S, A_E, B_S, B_E, project);

					UE_LOG(LogTemp, Warning, TEXT("projecting to solve for next with both: (%f,%f) => (%f,%f)"), (*focus).X.toFloat(), (*focus).Y.toFloat(), project.X.toFloat(), project.Y.toFloat());

					result.push(project);
				}
				else {
					UE_LOG(LogTemp, Warning, TEXT("HMM solve for next with just this: (%f,%f)"), (*focus).X.toFloat(), (*focus).Y.toFloat());
					result.push(*focus);
				}
			}
			else {
				if (*(lead.cyclic_next().cyclic_next()) != kill) {
					UE_LOG(LogTemp, Warning, TEXT("HMM solve for next with just after: (%f,%f)"), (*focus).X.toFloat(), (*focus).Y.toFloat());
					result.push(*focus);
				}
				else {
					UE_LOG(LogTemp, Warning, TEXT("FUCK solve for next with NOTHING: (%f,%f)"), (*focus).X.toFloat(), (*focus).Y.toFloat());
					result.push(*focus);
				}
			}
		}

		focus = focus.cyclic_next();
	}

	//UE_LOG(LogTemp, Warning, TEXT("unlocked"));
	//for(auto point : product)
	//	UE_LOG(LogTemp, Warning, TEXT("(%f, %f)"), point.X.toFloat(), point.Y.toFloat());
	//
	//UE_LOG(LogTemp, Warning, TEXT("locked"));
	//for (auto point : result)
	//	UE_LOG(LogTemp, Warning, TEXT("(%f, %f)"), point.X.toFloat(), point.Y.toFloat());

	return result;
}

TArray<FVector2D> toFVector(FLL<Pint> const &target) {
	TArray<FVector2D> product;

	for (auto point : target) {
		product.Push(FVector2D(point.X.toFloat(), point.Y.toFloat()));
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
			FVector(border[index]*10, height + offset * 5),
			FVector(border[next]*10, height + offset * 5),
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


Pint circularUniformPoint(rto radius = 1, int64 divisions = 100) {
	float t = 2 * PI*FMath::RandRange(0.f, 1.f);
	float u = FMath::RandRange(0.f, 1.f) + FMath::RandRange(0.f, 1.f);
	float r = u;
	if (u > 1)
		r = 2 - u;
	r *= divisions;
	return Pint(r*cos(t),r*sin(t)) * radius / divisions;

}
Pint boxUniformPoint(rto width = 10, rto height = 10, int64 divisions = 100) {
	return Pint(width * (int64)FMath::RandRange(0, divisions), height * (int64)FMath::RandRange(0, divisions)) / divisions;
}
Pint boxUniformPoint(PBox const & box, int64 divisions = 100) {
	int64 X = (int64)FMath::RandRange(-(float)divisions, (float)divisions);
	int64 Y = (int64)FMath::RandRange(-(float)divisions, (float)divisions);

	return Pint(X, Y) * box.getExtent() + box.getCenter();
}

PBox getBounds(Region<Pint> * target) {

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
		Edge<Pint> * A_edge;
		Edge<Pint> * B_edge;
		Pint A;
		Pint B;
		rto distance;
	};

	//returns if test is between A and B clockwise (right about the origin from A, left about from B)
	bool betweenVectors(Pint const &A, Pint const &B, Pint const &test) {

		Pint A_inward(A.Y, -A.X);
		Pint B_inward(-B.Y, B.X);

		rto bounds_relation = A_inward.Dot(B);

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

#define debug_chords

	void chord_clean(Region<Pint> * target, rto const & thresh, FLL<Region<Pint> *> & ins, FLL<Region<Pint> *> & outs) {
		//if no pair is small enough to split, add to ins and return
		split_canidate result;
		rto diameter = thresh + 1;
		bool found_option = false;

		FLL<Edge<Pint> *> relevants;

		for (auto border : target->getBounds()) {
			auto r = border->getLoopEdges();
			relevants.absorb(r);
		}
#ifdef debug_chords
		UE_LOG(LogTemp, Warning, TEXT("cleaning..."));
#endif
		for (auto edge : relevants) {
			Pint A_start = edge->getEnd()->getPosition();
			Pint A_before = edge->getStart()->getPosition();
			Pint A_after = edge->getNext()->getEnd()->getPosition();

			Pint A = A_start - A_before;

			Pint A_left = A_before - A_start;
			Pint A_right = A_after - A_start;

			rto min_offset = linear_offset(A, A_start);
			rto max_offset = linear_offset(A, A_start);

			for (auto compare : relevants) {
				Pint B_start = compare->getStart()->getPosition();

				rto raw = linear_offset(A, B_start);

				if (min_offset > raw)
					min_offset = raw;

				if (max_offset < raw)
					max_offset = raw;

				if (compare == edge || compare == edge->getNext() || compare == edge->getLast()) {
					continue;
				}

				//get smallest points
				
				Pint B_end = compare->getEnd()->getPosition();

				Pint B_segment = B_end - B_start;
				Pint B_perp(-B_segment.Y, B_segment.X);

				if (!betweenVectors(A_right, A_left, B_perp))
					continue;

				Pint offset = A_start - B_start;

				if (B_perp.Dot(offset) >= 0)
					continue;

				Pint intersect;

				if (offset.Dot(B_end - B_start) <= 0) {
					intersect = B_start;
					Pint B_last = compare->getLast()->getStart()->getPosition();

					if (!betweenVectors(B_end - B_start, B_last - B_start, offset))
						continue;
				}
				else if ((A_start - B_end).Dot(B_start - B_end) <= 0){
					intersect = B_end;

					Pint B_next = compare->getNext()->getEnd()->getPosition();

					if (!betweenVectors(B_start - B_end, B_next - B_end, A_start - B_end))
						continue;
				}
				else
					Pint::getIntersect(B_start, B_end, A_start, A_start + B_perp, intersect);

				rto distance = (intersect - A_start).SizeSquared();

				if(distance < thresh)
					if (!found_option || distance < result.distance) {						
						found_option = true;
						result.distance = distance;
						
						result.A = A_start;
						result.B = intersect;

#ifdef debug_chords
						UE_LOG(LogTemp, Warning, TEXT("split: (%f,%f) to (%f,%f), distance %f"), result.A.X.toFloat(),
							result.A.Y.toFloat(), result.B.X.toFloat(), result.B.Y.toFloat(), result.distance.toFloat());
#endif

						result.A_edge = edge;
						result.B_edge = compare;
					}
			}

			rto t = max_offset - min_offset;

			rto offset = t * t * (A.X * A.X + A.Y * A.Y);
#ifdef debug_chords
			UE_LOG(LogTemp, Warning, TEXT("offset: %f"), offset.toFloat());
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
					UE_LOG(LogTemp, Warning, TEXT("(%f,%f)"), point.X.toFloat(), point.Y.toFloat());
				}
			}
#endif
			auto P = RegionAdd(target, result.A_edge, result.B_edge);

#ifdef debug_chords
			UE_LOG(LogTemp, Warning, TEXT("Result target"));
			for (auto face : target->getBounds()) {
				UE_LOG(LogTemp, Warning, TEXT("Face >r:"));
				for (auto point : face->getLoopPoints()) {
					UE_LOG(LogTemp, Warning, TEXT("(%f,%f)"), point.X.toFloat(), point.Y.toFloat());
				}
			}
			if (P != nullptr) {
				UE_LOG(LogTemp, Warning, TEXT("Result P"));
				for (auto face : P->getBounds()) {
					UE_LOG(LogTemp, Warning, TEXT("Face >b:"));
					for (auto point : face->getLoopPoints()) {
						UE_LOG(LogTemp, Warning, TEXT("(%f,%f)"), point.X.toFloat(), point.Y.toFloat());
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
						//if (GEngine)
						//	GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, TEXT("Failed to Triangulate destructible mesh!"));
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

void Aroom_description_builder::CreateDoor(Pint const & wall_left, Pint const & wall_right, float bottom, float top) {
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

	FVector2D Mid = convert((wall_left + wall_right) / 2);
	FVector2D dir = convert(wall_left - wall_right);
	dir.Normalize();
	dir *= 30;//door radial width
	FVector2D doorframe_left = Mid + dir;
	FVector2D doorframe_right = Mid - dir;

	Vertices.Push(FVector(convert(wall_left), bottom));
	Vertices.Push(FVector(convert(wall_left), top));
	Vertices.Push(FVector(doorframe_left, bottom));
	Vertices.Push(FVector(doorframe_left, top));
	Vertices.Push(FVector(doorframe_right, bottom));
	Vertices.Push(FVector(doorframe_right, top));
	Vertices.Push(FVector(convert(wall_right), bottom));
	Vertices.Push(FVector(convert(wall_right), top));

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

	UV0.Push(FVector2D(0, bottom));
	UV0.Push(FVector2D(0, top));
	UV0.Push(FVector2D(1, bottom));
	UV0.Push(FVector2D(1, top));
	UV0.Push(FVector2D(0, bottom));
	UV0.Push(FVector2D(0, top));
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
	if (top - bottom >= door_height) {
		Vertices.Push(FVector(doorframe_left, bottom + door_height));
		Vertices.Push(FVector(doorframe_right, bottom + door_height));

		Triangles.Push(5);
		Triangles.Push(3);
		Triangles.Push(9);
		Triangles.Push(9);
		Triangles.Push(3);
		Triangles.Push(8);

		Normals.Push(FVector(normal, 0));
		Normals.Push(FVector(normal, 0));

		float uv_ratio = door_height / (top - bottom);
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
void Aroom_description_builder::Create_Floor_Ceiling_New(Region<Pint> * source, float bottom, float top) {
	auto Border = toFVector(source->getBounds().last()->getLoopPoints());

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
void Aroom_description_builder::Create_Wall_Sections_New(Region<Pint> * source, float bottom, float top) {

	for(auto border : source->getBounds()) {
		auto border_points = border->getLoopPoints();

		Draw_Border(toFVector(border_points), 50, GetWorld());


		auto last = border_points.last();

		for (auto next : border_points) {

			CreateDoor(last, next, bottom, top);

			last = next;
		}
	}
}


//==========================================================================================================
//================================== generation utilities ==================================================
//==========================================================================================================

struct Type_Tracker {
	FLL<Region<Pint> *> Nulls;
	FLL<Region<Pint> *> Rooms;
	FLL<Region<Pint> *> Halls;
	FLL<Region<Pint> *> Smalls;
	bool isRoom(Region<Pint> const * target) {
		for (auto room : Rooms) {
			if (room == target) {
				return true;
			}
		}
		return false;
	}
	void display(const UWorld* world) {
		int64 p = 0;
		for (auto small : Smalls) {
			cleanRegion(small);
			for(auto border : small->getBounds())
				Draw_Border(toFVector(border->getLoopPoints()), 72, world, color_Pnk);
		}

		for (auto room : Rooms) {
			cleanRegion(room);
			//Create_Floor_Ceiling_New(room, 0, 200);
			//Create_Wall_Sections_New(room, 0, 200, h);
			for (auto border : room->getBounds())
				Draw_Border(toFVector(border->getLoopPoints()), 70, world, color_blue);
		}

		int i = 0;
		for (auto null : Nulls) {
			cleanRegion(null);
			for (auto border : null->getBounds())
				Draw_Border(toFVector(border->getLoopPoints()), 30 + (i++)*5, world, color_red);
		}

		for (auto hall : Halls) {
			cleanRegion(hall);
			for (auto border : hall->getBounds())
				Draw_Border(toFVector(border->getLoopPoints()), 74, world, color_green);
		}
	}
};

bool Cull_Suggested(Region<Pint> * target, FLL<Region<Pint> *> &results, FLL<Region<Pint> *> &nulls) {
	UE_LOG(LogTemp, Warning, TEXT("Culling\n"));

	FLL<Region<Pint> *> rooms;
	FLL<Region<Pint>*> outers;

	chord_splits::chord_clean(target, room_min_width, rooms, outers);

	nulls.append(outers);
	results.append(rooms);

	return true;
}

void mergeGroup(FLL<Region<Pint> *> & nulls) {
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

void cleanNulls(Type_Tracker &target) {
	UE_LOG(LogTemp, Warning, TEXT("Clean Nulls\n"));

	FLL<Region<Pint> *> input_nulls;
	FLL<Region<Pint> *> input_halls;
	input_nulls.absorb(target.Nulls);
	mergeGroup(input_nulls);

	for (auto focus : input_nulls) {

		FLL<Region<Pint> *> room_ins;
		FLL<Region<Pint> *> room_outs;

		chord_splits::chord_clean(focus, room_min_width, room_ins, room_outs);

		target.Nulls.absorb(room_ins);
		input_halls.absorb(room_outs);
	}

	for (auto focus : input_halls) {

		FLL<Region<Pint> *> hall_ins;
		FLL<Region<Pint> *> hall_outs;

		chord_splits::chord_clean(focus, hall_min_width, hall_ins, hall_outs);

		target.Halls.absorb(hall_ins);
		target.Smalls.absorb(hall_outs);
	}

	mergeGroup(target.Nulls);
	mergeGroup(target.Halls);
	mergeGroup(target.Smalls);

	for (auto region : target.Rooms)
		cleanRegion(region);

	for(auto region : target.Nulls)
		cleanRegion(region);

	for (auto region : target.Halls)
		cleanRegion(region);

	for (auto region : target.Smalls)
		cleanRegion(region);
}

/*void mergeSmalls(Type_Tracker &target) {
	UE_LOG(LogTemp, Warning, TEXT("Merge Smalls\n"));
	//for each small
	//for connected room
	//union the borders, reduce, and intersect with small
	//choose room with greatest area 
	for (auto small : target.Smalls) {
		auto local_boundary = toPaths(small);
		
		FLL<Region<Pint> *> neighbors = small->getNeighbors();

		Region<Pint> * best_neighbor = nullptr;
		float best_area_score = 0;
		ClipperLib::PolyTree best_tree;

		for (auto neighbor : neighbors) {
			if (!target.isRoom(neighbor)) {
				continue;
			}

			//chord_splits::chord_clean(neighbor, room_min_width, hall_ins, hall_outs);
			//auto other_boundary = toPaths(neighbor);

			//auto grouping = addPaths(other_boundary, local_boundary);
			//auto result = sizeRestrictPaths(grouping, room_min_width / 2);

			//if (result.size() > 0) {
			//	float area_score = ClipperLib::Area(result[0]);
			//	if (area_score > best_area_score) {
			//		best_area_score = area_score;
			//		best_neighbor = neighbor;
			//		makeTree(result, best_tree);
			//	}
			//}
		}

		if (best_neighbor != nullptr) {
			FLL<Region<Pint> *> ins;
			FLL<Region<Pint> *> outs;

			
			//polytree_utils::AllocateTree(best_tree, small, ins, outs);

			for (auto in : ins) {
				merge(best_neighbor, in);
			}

			target.Smalls.remove(small);
			target.Smalls.append(outs);
		}
	}
}*/


/*
FLL<Pint> choosePointsNear(Region<Pint> const & target, int64 count, rto offset){
	//pick n points AWAY from the polygon
	//get center offsets of each face
	
	auto bounds = getBounds(&target);
	auto extent = bounds.getExtent();
	auto center = bounds.getCenter();
	
	rto size = 2;

	if (extent.X > extent.Y) {
		size *= extent.X;
	}
	else {
		size *= extent.Y;
	}

	auto segments = target.getLoopPoints();

	FLL<Pint> seeds;
	for (int64 ii = 0; ii < count; ii++) {
		Pint seed = circularUniformPoint(size);
		seed.X += center.X;
		seed.Y += center.Y;

		//seeds.Push(seed);

		Pint best(0,0);
		float best_score = FLT_MAX;

		for (auto focus = segments.begin(); focus != segments.end(); ++focus) {
			Pint A = *focus;
			Pint B = *(focus.cyclic_next());
			
			Pint AB = B - A;
			Pint AP = seed - A;

			rto length = AB.SizeSquared();
			rto dot = (AP.X * AB.X + AP.Y * AB.Y);

			rto t = dot / length;

			Pint point = AB * t + A;
		}

		seeds.append(best);
	}

	//for each edge, offset it and get the nearest point, tracking the best fit
	return seeds;
}

struct HallwayPrototype {
	HallwayPrototype* parent;
	int64 parent_generation;

	TArray<HallwayPrototype*> children;

	TArray<_P> path;
	HallwayPrototype(HallwayPrototype* p, int64 g) {
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
		int64 p_id = grand->children.Find(parent);
		grand->children[p_id] = this;

		//these change OUR references, so they should be last

		for (auto child : parent->children) {
			child->parent = this;
		}
		int64 parent_lifetime = parent->children.Num();
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
	//Face<Pint>* start;
	//Face<Pint>* end;

	TArray<HallwayPrototype*> frontier;

	//pick initial points that are on the boundary of the start
	//pick a side that isn't to short
	TArray<_P> start_canidates;

	TArray<F_DCEL::Edge*> canidates;
	start->getRootEdge()->listFaceLoop(canidates);

	for (int64 ii = 0; ii < canidates.Num(); ii++) {
		auto start = canidates[ii]->getStartPoint()->getPosition();
		auto end = canidates[ii]->getEndPoint()->getPosition();
		if ((end - start).Size() < 150) {
			canidates.RemoveAt(ii--);
		}
	}

	for(int64 ii = 0; ii < 3; ii++){
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

FLL<Pint> Square_Generator(rto x, rto y, Pint center) {
	FLL<Pint> boundary;
	boundary.append(Pint(-x, -y) + center);
	boundary.append(Pint(-x, y) + center);
	boundary.append(Pint(x, y) + center);
	boundary.append(Pint(x, -y) + center);

	return boundary;
}
FLL<Pint> Diamond_Generator(rto x, rto y, Pint center) {
	FLL<Pint> boundary;
	if (x < y) {
		rto d = y - x;
		boundary.append(Pint(-x, -d) + center);
		boundary.append(Pint(-x, d) + center);
		boundary.append(Pint(0, y) + center);
		boundary.append(Pint(x, d) + center);
		boundary.append(Pint(x, -d) + center);
		boundary.append(Pint(0, -y) + center);
	}
	else if (x > y) {
		rto d = x - y;
		boundary.append(Pint(-x, 0) + center);
		boundary.append(Pint(-d, y) + center);
		boundary.append(Pint(d, y) + center);
		boundary.append(Pint(x, 0) + center);
		boundary.append(Pint(d, -y) + center);
		boundary.append(Pint(-d, -y) + center);
	}
	else {
		boundary.append(Pint(-x, 0) + center);
		boundary.append(Pint(0, x) + center);
		boundary.append(Pint(x, 0) + center);
		boundary.append(Pint(0, -x) + center);
	}
	

	return boundary;
}
FLL<Pint> Bevel_Generator(rto x, rto y, Pint center) {
	rto level;
	if (x < y) {
		level = x / 4;
	}
	else {
		level = y / 4;
	}

	FLL<Pint> boundary;

	boundary.append(Pint(-x, level - y) + center);
	boundary.append(Pint(-x, y - level) + center);
	boundary.append(Pint(level - x, y) + center);
	boundary.append(Pint(x - level, y) + center);
	boundary.append(Pint(x, y - level) + center);
	boundary.append(Pint(x, level - y) + center);
	boundary.append(Pint(x - level, -y) + center);
	boundary.append(Pint(level - x, -y) + center);

	return boundary;
}
FLL<Pint> Rectangle_Generator(rto x, rto y, Pint center) {
	rto skew = (int64)FMath::RandRange((float)0, (float)10);
	Pint angle(skew / 10, (rto(10) - skew) / 10);
	Pint perp((skew - 10) / 10, skew / 10);

	FLL<Pint> boundary;

	boundary.append(center - angle * x - perp * y);
	boundary.append(center - angle * x + perp * y);
	boundary.append(center + angle * x + perp * y);
	boundary.append(center + angle * x - perp * y);

	return boundary;
}

typedef FLL<Pint> (*generatorFunc)(rto, rto, Pint);

FLL<Pint> Pick_Generator(rto x, rto y, Pint center) {
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

//void Generate_Building_Shape(Face<Pint>* Available, Type_Tracker &system_types) {
	//generates a set of trackers for each building Region<Pint>
	//these come with predefined nulls and infrastructure halls
//}

bool createRoomAtPoint(Type_Tracker &system_types, Pint const &point, int64 scale = 1, FLL<Region<Pint> *> * created = nullptr) {
	UE_LOG(LogTemp, Warning, TEXT("Create Room At Point %f,%f"),point.X.toFloat(), point.Y.toFloat());
	FLL<Region<Pint> *> created_rooms;
	FLL<Region<Pint> *> created_nulls;
	FLL<Region<Pint> *> raw_faces;
	
	Region<Pint> * choice = nullptr;

	//find containing null
	for (auto null : system_types.Nulls) {
		if (contains(null, point).type != FaceRelationType::point_exterior) {
			choice = null;
			break;
		}
	}

	if (choice == nullptr) {
		UE_LOG(LogTemp, Warning, TEXT("Selected point is not in a null region\n"));
		return false;
	}

	system_types.Nulls.remove(choice);

	int64 area = FMath::RandRange(6, 14);
	int64 width = FMath::RandRange(2, area / 2);
	int64 length = area / width;

	UE_LOG(LogTemp, Warning, TEXT("size: %d %d"), width, length);

	auto bounds = Rectangle_Generator(width * scale, length * scale, point);

	

	//cull to null Region<Pint>
	UE_LOG(LogTemp, Warning, TEXT("CR - allocate node\n"));
	subAllocate(choice, bounds, created_nulls, raw_faces);

	//remerge rejected regions with nulls
	for (auto face : raw_faces) {
		Cull_Suggested(face, created_rooms, created_nulls);
	}

	system_types.Nulls.absorb(created_nulls);

	cleanNulls(system_types);

	if (created != nullptr) {
		created->append(created_rooms);
	}

	bool success = !created_rooms.empty();

	system_types.Rooms.absorb(created_rooms);

	return success;
}
//attempts to create rooms distributed evenly across the space
bool createDistributedRooms(Type_Tracker &system_types, const int64 attempts_per_cell = 5, int64 scale = 1, FLL<Region<Pint> *> *created = nullptr) {
	//GRID METHOD
	//from a grid, for each grid point we select a point within a tolerance radius
	//choose a grid
	//skew, stretch, rotation, offset
	//offset.X = (FMath::RandRange((int64)null_bounds.Min.X / 100, (int64)null_bounds.Max.X) / 100) * 100.f;
	//offset.Y = (FMath::RandRange((int64)null_bounds.Min.Y / 100, (int64)null_bounds.Max.Y) / 100) * 100.f;
	
	//offset only for now
#define grid_width 10
#define grid_height 10
#define tolerance 4

	Pint offset = boxUniformPoint(grid_width, grid_height);
	Pint skew = boxUniformPoint(grid_width, grid_height);

	for (int64 xx = -5; xx <= 5; xx++) {
		for (int64 yy = -5; yy <= 5; yy++) {
			//test a couple points until we find one in the space
			int64 safety = attempts_per_cell;
			Pint grid = Pint(xx * grid_width, yy * grid_height) + offset + Pint(skew.X*yy, skew.Y*xx);

			do {
				Pint choice = circularUniformPoint(tolerance) + grid;
				choice *= 100.f;
				//choice.toGrid(100.f);

				//choice.X = (int64)(choice.X) * 100.f;
				//choice.Y = (int64)(choice.Y) * 100.f;

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

bool fillNullSpace(Type_Tracker &system_types, int64 safety = 100, int64 scale = 1, FLL<Region<Pint> *> *created = nullptr) {
	while (!system_types.Nulls.empty() && (safety--) > 0) {

		auto bounds = getBounds(system_types.Nulls.last());

		auto choice = boxUniformPoint(bounds);

		createRoomAtPoint(system_types, choice, scale, created);
	}
	return true;
}

bool createNearRooms(Type_Tracker &system_types, Region<Pint> * target, int64 attempts = 10, int64 scale = 1, FLL<Region<Pint> *> *created = nullptr, const UWorld* ref = nullptr) {
	
	//auto seeds = choosePointsNear(target, 3 * attempts, 4);

	auto bounds = getBounds(target);

	attempts = attempts * 3;
	while (attempts > 0) {

		auto seed = boxUniformPoint(4, 4);
		if (seed.X > 0) {
			seed.X += bounds.Max.X;
		}
		else {
			seed.X += bounds.Min.X;
		}
		if (seed.Y > 0) {
			seed.Y += bounds.Max.Y;
		}
		else {
			seed.Y += bounds.Min.Y;
		}

		if (ref) {
			DrawDebugLine(
				ref,
				FVector(convert(seed) * 10, 70),
				FVector(convert(seed) * 10, 80),
				//FColor(FMath::RandRange(0,255), FMath::RandRange(0, 255), FMath::RandRange(0, 255)),
				color_Pnk,
				true,
				-1,
				0,
				6
			);
		}

		if (createRoomAtPoint(system_types, seed, scale, created)) {
			attempts -= 3;
		}
		else {
			attempts--;
		}
	}

	//auto seeds = choosePointsNear(target, attempts);
	//for (auto choice : seeds) {
	//	createRoomAtPoint(system_types, choice, scale, created);
	//}

	return true;
}

bool createLinkingHalls(Region<Pint> * target, FLL<Region<Pint> *> &nulls, FLL<Region<Pint> *> &created_faces) {
	//tries to generate a set of halls linking two target faces

	return true;
}

struct building_region {
	int64 size;
	Region<Pint> * region;
	building_region * parent;
	building_region(int64 s, Region<Pint> * r) {
		size = s;
		region = r;
		parent = nullptr;
	}
	building_region(int64 s, Region<Pint> * r, building_region * p) {
		size = s;
		region = r;
		parent = p;
	}
};

void create_Layout(Type_Tracker &system_types, int64 large, int64 medium, int64 small, UWorld* ref = nullptr) {
	//large is 10-14
	//med is 7-11
	//small is 4-8
	UE_LOG(LogTemp, Warning, TEXT("Create Layout\n"));
	TArray<building_region*> larges;
	TArray<building_region*> mediums;
	TArray<building_region*> smalls;

	//FLL<Region<Pint> *> temp_created;

	//createRoomAtPoint(system_types, Pint(0,0), 7, &temp_created);

	//for (auto room : temp_created) {
	//	larges.Push(new building_region(0, room));
	//}

	for (int64 ii = 0; ii < large; ii++) {
		//pick point
		auto point = boxUniformPoint(140,140) - Pint(70, 70);
		//point.toGrid(P_micro_grid);
		FLL<Region<Pint> *> temp_created;

		if (createRoomAtPoint(system_types, point, 7, &temp_created))
			UE_LOG(LogTemp, Warning, TEXT("Created %d sub-rooms \n"), temp_created.size());

		//for (auto room : temp_created) {
		//	larges.Push(new building_region(0, room));
		//}
	}

	/*{
		FLL<Region<Pint> *> temp_created;
		fakeCreate(system_types, Pint(rto(28), rto(-68) / 5), Bevel_Generator, 3, 2, &temp_created);
		for (auto room : temp_created) {
			larges.Push(new building_region(0, room));
		}
	}
	{
		FLL<Region<Pint> *> temp_created;
		fakeCreate(system_types, Pint(rto(28), rto(152) / 5), Bevel_Generator, 3, 3, &temp_created);
		for (auto room : temp_created) {
			larges.Push(new building_region(0, room));
		}
	}
	{
		FLL<Region<Pint> *> temp_created;
		fakeCreate(system_types, Pint(rto(28)/5, rto(-151) / 5), Square_Generator, 5, 2, &temp_created);
		for (auto room : temp_created) {
			larges.Push(new building_region(0, room));
		}
	}*/


	//UE_LOG(LogTemp, Warning, TEXT("Larges created: %d\n"), larges.Num());

	/*FLL<Face<Pint> *> temp_created;
	createRoomAtPoint(system_types, _P(10 * grid_coef, -10 * grid_coef), 1, &temp_created);
	createRoomAtPoint(system_types, _P(-5 * grid_coef, -8 * grid_coef), 1, &temp_created);
	for (auto room : temp_created) {
		larges.Push(new building_region(0, room));
	}*/


	/*for (auto large : larges) {
		//creates some MEDIUMS
		FLL<Region<Pint> *> temp_created;
		createNearRooms(system_types, large->region, medium, 5, &temp_created, ref);
		for (auto room : temp_created) {
			mediums.Push(new building_region(1, room, large));
		}
	}
	for (auto medium : mediums) {
		//creates some MEDIUMS
		FLL<Region<Pint> *> temp_created;
		createNearRooms(system_types, medium->region, small, 4, &temp_created, ref);
		for (auto room : temp_created) {
			smalls.Push(new building_region(2, room, medium));
		}
	}*/

	for (auto large : larges) {
		delete large;
	}
	for (auto medium : mediums) {
		delete medium;
	}
	for (auto small : smalls) {
		delete small;
	}
}

void Aroom_description_builder::Main_Generation_Loop() {
	UE_LOG(LogTemp, Warning, TEXT("Main Generation\n"));
	DCEL<Pint> system_new;
	Type_Tracker system_types;

	FMath::SRandInit(0);
	FMath::RandInit(0);

	auto system_bounds = Square_Generator(100, 100, Pint(0,0));

	Draw_Border(toFVector(system_bounds), 0, GetWorld());
	
	system_types.Nulls.append(system_new.region(system_bounds));

	create_Layout(system_types, room_count, 3, 2, GetWorld());

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