// Fill out your copyright notice in the Description page of Project Settings.

#include "room_description_builder.h"
#include "Algo/Reverse.h"
#include "DrawDebugHelpers.h"

#define room_min_width 300
#define hall_min_width 200

//==========================================================================================================
//========================================== transforms ====================================================
//==========================================================================================================

FVector2D convert(Pgrd const &target) {
	return FVector2D(target.X.toFloat() * 10, target.Y.toFloat() * 10);
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

#define debug_chords

	void chord_clean(Region<Pgrd> * target, grd const & thresh, FLL<Region<Pgrd> *> & ins, FLL<Region<Pgrd> *> & outs) {
		//if no pair is small enough to split, add to ins and return
		split_canidate result;
		grd diameter = thresh + 1;
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

				grd const distance = segment.SizeSquared();

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

			grd const t = max_offset - min_offset;
			grd const offset = t * t * (A.X * A.X + A.Y * A.Y);
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
//================================== generation utilities ==================================================
//==========================================================================================================

struct Type_Tracker {
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
	void display(Aroom_description_builder &builder) {
		int64 p = 0;

		UWorld * world = builder.GetWorld();
		for (auto small : Smalls) {
			cleanRegion(small);
			for(auto border : small->getBounds())
				Draw_Border(toFVector(border->getLoopPoints()), 72, world, color_Pnk);
		}

		for (auto room : Rooms) {
			cleanRegion(room);
			builder.Create_Floor_Ceiling_New(room, 0, 200);
			builder.Create_Wall_Sections_New(room, 0, 200, Rooms);
			for (auto border : room->getBounds())
				Draw_Border(toFVector(border->getLoopPoints()), 80, world, color_blue);
		}

		int i = 0;
		for (auto null : Nulls) {
			cleanRegion(null);
			for (auto border : null->getBounds())
				Draw_Border(toFVector(border->getLoopPoints()), 0 + i*15, world, color_red);
			i++;
		}

		for (auto hall : Halls) {
			cleanRegion(hall);
			for (auto border : hall->getBounds())
				Draw_Border(toFVector(border->getLoopPoints()), 74, world, color_green);
		}
	}
};

bool Cull_Suggested(Region<Pgrd> * target, FLL<Region<Pgrd> *> &results, FLL<Region<Pgrd> *> &nulls) {
	UE_LOG(LogTemp, Warning, TEXT("Culling\n"));

	FLL<Region<Pgrd> *> rooms;
	FLL<Region<Pgrd>*> outers;

	cleanRegion(target);

	chord_splits::chord_clean(target, room_min_width, rooms, outers);

	nulls.absorb(outers);
	results.absorb(rooms);

	return true;
}

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

void cleanNulls(Type_Tracker &target, FLL<Region<Pgrd> *> &input_nulls) {
	UE_LOG(LogTemp, Warning, TEXT("Clean Nulls\n"));

	//FLL<Region<Pgrd> *> input_nulls;
	FLL<Region<Pgrd> *> input_halls;
	//input_nulls.absorb(target.Nulls);
	mergeGroup(input_nulls);

	for (auto focus : input_nulls) {

		FLL<Region<Pgrd> *> room_ins;
		FLL<Region<Pgrd> *> room_outs;

		cleanRegion(focus);

		chord_splits::chord_clean(focus, room_min_width, room_ins, room_outs);

		for (auto null : room_ins) {
			merge(null, null);
		}

		target.Nulls.absorb(room_ins);

		input_halls.absorb(room_outs);
	}

	UE_LOG(LogTemp, Warning, TEXT("Clean Halls\n"));

	for (auto focus : input_halls) {

		FLL<Region<Pgrd> *> hall_ins;
		FLL<Region<Pgrd> *> hall_outs;

		cleanRegion(focus);

		chord_splits::chord_clean(focus, hall_min_width, hall_ins, hall_outs);

		target.Halls.absorb(hall_ins);
		target.Smalls.absorb(hall_outs);
	}

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

/*
FLL<Pgrd> choosePointsNear(Region<Pgrd> const & target, int64 count, grd offset){
	//pick n points AWAY from the polygon
	//get center offsets of each face
	
	auto bounds = getBounds(&target);
	auto extent = bounds.getExtent();
	auto center = bounds.getCenter();
	
	grd size = 2;

	if (extent.X > extent.Y) {
		size *= extent.X;
	}
	else {
		size *= extent.Y;
	}

	auto segments = target.getLoopPoints();

	FLL<Pgrd> seeds;
	for (int64 ii = 0; ii < count; ii++) {
		Pgrd seed = circularUniformPoint(size);
		seed.X += center.X;
		seed.Y += center.Y;

		//seeds.Push(seed);

		Pgrd best(0,0);
		float best_score = FLT_MAX;

		for (auto focus = segments.begin(); focus != segments.end(); ++focus) {
			Pgrd A = *focus;
			Pgrd B = *(focus.cyclic_next());
			
			Pgrd AB = B - A;
			Pgrd AP = seed - A;

			grd length = AB.SizeSquared();
			grd dot = (AP.X * AB.X + AP.Y * AB.Y);

			grd t = dot / length;

			Pgrd point = AB * t + A;
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
	//Face<Pgrd>* start;
	//Face<Pgrd>* end;

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
}*/

//==========================================================================================================
//======================================== creation ========================================================
//==========================================================================================================

void Aroom_description_builder::CreateWall(Pgrd const & wall_left, Pgrd const & wall_right, float bottom, float top) {

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

	FVector2D dir = convert(wall_left - wall_right);

	dir.Normalize();
	dir *= 30;//door radial width

	Vertices.Push(FVector(convert(wall_left), bottom));
	Vertices.Push(FVector(convert(wall_left), top));
	Vertices.Push(FVector(convert(wall_right), bottom));
	Vertices.Push(FVector(convert(wall_right), top));

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

	Wall_Mesh->CreateMeshSection_LinearColor(0, Vertices, Triangles, Normals, UV0, VertexColors, Tangents, true);
	Wall_Mesh->ContainsPhysicsTriMeshData(true);

	Wall_Mesh->Activate();

	Wall_Mesh->RegisterComponentWithWorld(GetWorld());
}
void Aroom_description_builder::CreateDoor(Pgrd const & wall_left, Pgrd const & wall_right, float bottom, float top) {
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

void Aroom_description_builder::Create_Floor_Ceiling_New(Region<Pgrd> * source, float bottom, float top) {
	auto Border = toFVector(source->getBounds().last()->getLoopPoints());
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
void Aroom_description_builder::Create_Wall_Sections_New(Region<Pgrd> * source, float bottom, float top, FLL<Region<Pgrd> *> &Rooms) {

	for (auto border : source->getBounds()) {
		auto border_points = border->getLoopEdges();

		for (auto edge : border_points) {

			Pgrd const A = edge->getStart()->getPosition();
			Pgrd const B = edge->getEnd()->getPosition();

			Region<Pgrd> * op = edge->getInv()->getFace()->getGroup();

			if ((B - A).SizeSquared() < 40 || !Rooms.contains(op)) {
				CreateWall(B, A, bottom, top);
			}
			else {
				CreateDoor(B, A, bottom, top);
			}
		}
	}
}


//==========================================================================================================
//======================================= generation =======================================================
//==========================================================================================================


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

typedef FLL<Pgrd> (*generatorFunc)(grd, grd, Pgrd);

FLL<Pgrd> Pick_Generator(grd x, grd y, Pgrd center) {
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

//void Generate_Building_Shape(Face<Pgrd>* Available, Type_Tracker &system_types) {
	//generates a set of trackers for each building Region<Pgrd>
	//these come with predefined nulls and infrastructure halls
//}
bool createRoomAtPoint(Type_Tracker &system_types, Pgrd const &point, int64 scale = 1, FLL<Region<Pgrd> *> * created = nullptr) {
	UE_LOG(LogTemp, Warning, TEXT("Create Room At Point %f,%f"), point.X.toFloat(), point.Y.toFloat());
	FLL<Region<Pgrd> *> created_rooms;
	FLL<Region<Pgrd> *> created_nulls;
	FLL<Region<Pgrd> *> raw_faces;

	Region<Pgrd> * choice = nullptr;

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

	auto bounds = Pick_Generator(width * scale, length * scale, point);



	//cull to null Region<Pgrd>
	UE_LOG(LogTemp, Warning, TEXT("CR - allocate node\n"));
	subAllocate(choice, bounds, created_nulls, raw_faces);

	//remerge rejected regions with nulls
	for (auto face : raw_faces) {
		Cull_Suggested(face, created_rooms, created_nulls);
	}

	//system_types.Nulls.absorb(created_nulls);

	cleanNulls(system_types, created_nulls);

	if (created != nullptr) {
		created->append(created_rooms);
	}

	bool success = !created_rooms.empty();

	system_types.Rooms.absorb(created_rooms);

	return success;
}
bool createRectangleAtPoint(Type_Tracker &system_types, Pgrd const &point, int64 scale = 1, FLL<Region<Pgrd> *> * created = nullptr) {
	UE_LOG(LogTemp, Warning, TEXT("Create Room At Point %f,%f"),point.X.toFloat(), point.Y.toFloat());
	FLL<Region<Pgrd> *> created_rooms;
	FLL<Region<Pgrd> *> created_nulls;
	FLL<Region<Pgrd> *> raw_faces;
	
	Region<Pgrd> * choice = nullptr;

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

	

	//cull to null Region<Pgrd>
	UE_LOG(LogTemp, Warning, TEXT("CR - allocate node\n"));
	subAllocate(choice, bounds, created_nulls, raw_faces);

	//remerge rejected regions with nulls
	for (auto face : raw_faces) {
		Cull_Suggested(face, created_rooms, created_nulls);
	}

	//system_types.Nulls.absorb(created_nulls);

	cleanNulls(system_types, created_nulls);

	if (created != nullptr) {
		created->append(created_rooms);
	}

	bool success = !created_rooms.empty();

	system_types.Rooms.absorb(created_rooms);

	return success;
}
//attempts to create rooms distributed evenly across the space
bool createDistributedRooms(Type_Tracker &system_types, const int64 attempts_per_cell = 5, int64 scale = 1, FLL<Region<Pgrd> *> *created = nullptr) {
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

	Pgrd offset = boxUniformPoint(grid_width, grid_height);
	Pgrd skew = boxUniformPoint(grid_width, grid_height);

	for (int64 xx = -5; xx <= 5; xx++) {
		for (int64 yy = -5; yy <= 5; yy++) {
			//test a couple points until we find one in the space
			int64 safety = attempts_per_cell;
			Pgrd grid = Pgrd(xx * grid_width, yy * grid_height) + offset + Pgrd(skew.X*yy, skew.Y*xx);

			do {
				Pgrd choice = circularUniformPoint(tolerance) + grid;
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

bool fillNullSpace(Type_Tracker &system_types, int64 safety = 100, int64 scale = 1, FLL<Region<Pgrd> *> *created = nullptr) {
	while (!system_types.Nulls.empty() && (safety--) > 0) {

		auto bounds = getBounds(system_types.Nulls.last());

		auto choice = boxUniformPoint(bounds);

		createRoomAtPoint(system_types, choice, scale, created);
	}
	return true;
}

bool createNearRooms(Type_Tracker &system_types, Region<Pgrd> * target, int64 attempts = 10, int64 scale = 1, FLL<Region<Pgrd> *> *created = nullptr, const UWorld* ref = nullptr) {
	
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

bool createLinkingHalls(Region<Pgrd> * target, FLL<Region<Pgrd> *> &nulls, FLL<Region<Pgrd> *> &created_faces) {
	//tries to generate a set of halls linking two target faces

	return true;
}

struct building_region {
	int64 size;
	Region<Pgrd> * region;
	building_region * parent;
	building_region(int64 s, Region<Pgrd> * r) {
		size = s;
		region = r;
		parent = nullptr;
	}
	building_region(int64 s, Region<Pgrd> * r, building_region * p) {
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

	//FLL<Region<Pgrd> *> temp_created;

	//createRoomAtPoint(system_types, Pgrd(0,0), 7, &temp_created);

	//for (auto room : temp_created) {
	//	larges.Push(new building_region(0, room));
	//}

	for (int64 ii = 0; ii < large; ii++) {
		//pick point
		auto point = boxUniformPoint(140,140) - Pgrd(70, 70);
		//point.toGrid(P_micro_grid);
		FLL<Region<Pgrd> *> temp_created;

		if (createRectangleAtPoint(system_types, point, 7, &temp_created))
			UE_LOG(LogTemp, Warning, TEXT("Created %d sub-rooms \n"), temp_created.size());

		//for (auto room : temp_created) {
		//	larges.Push(new building_region(0, room));
		//}
	}

	for (int64 ii = 0; ii < medium; ii++) {
		//pick point
		auto point = boxUniformPoint(140, 140, 27) - Pgrd(70, 70);
		//point.toGrid(P_micro_grid);
		FLL<Region<Pgrd> *> temp_created;

		if (createRoomAtPoint(system_types, point, 5, &temp_created))
			UE_LOG(LogTemp, Warning, TEXT("Created %d sub-rooms \n"), temp_created.size());

		//for (auto room : temp_created) {
		//	larges.Push(new building_region(0, room));
		//}
	}

	/*{
		FLL<Region<Pgrd> *> temp_created;
		fakeCreate(system_types, Pgrd(grd(28), grd(-68) / 5), Bevel_Generator, 3, 2, &temp_created);
		for (auto room : temp_created) {
			larges.Push(new building_region(0, room));
		}
	}
	{
		FLL<Region<Pgrd> *> temp_created;
		fakeCreate(system_types, Pgrd(grd(28), grd(152) / 5), Bevel_Generator, 3, 3, &temp_created);
		for (auto room : temp_created) {
			larges.Push(new building_region(0, room));
		}
	}
	{
		FLL<Region<Pgrd> *> temp_created;
		fakeCreate(system_types, Pgrd(grd(28)/5, grd(-151) / 5), Square_Generator, 5, 2, &temp_created);
		for (auto room : temp_created) {
			larges.Push(new building_region(0, room));
		}
	}*/


	//UE_LOG(LogTemp, Warning, TEXT("Larges created: %d\n"), larges.Num());

	/*FLL<Face<Pgrd> *> temp_created;
	createRoomAtPoint(system_types, _P(10 * grid_coef, -10 * grid_coef), 1, &temp_created);
	createRoomAtPoint(system_types, _P(-5 * grid_coef, -8 * grid_coef), 1, &temp_created);
	for (auto room : temp_created) {
		larges.Push(new building_region(0, room));
	}*/


	/*for (auto large : larges) {
		//creates some MEDIUMS
		FLL<Region<Pgrd> *> temp_created;
		createNearRooms(system_types, large->region, medium, 5, &temp_created, ref);
		for (auto room : temp_created) {
			mediums.Push(new building_region(1, room, large));
		}
	}
	for (auto medium : mediums) {
		//creates some MEDIUMS
		FLL<Region<Pgrd> *> temp_created;
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
	UE_LOG(LogTemp, Warning, TEXT("Main Generation"));
	DCEL<Pgrd> system_new;
	Type_Tracker system_types;

	int seed = 45732;// FMath::RandRange(0, 100000);
	FMath::RandInit(seed);
	UE_LOG(LogTemp, Warning, TEXT("SRand Seed %d\n"), seed);

	auto system_bounds = Square_Generator(100, 100, Pgrd(0,0));

	Draw_Border(toFVector(system_bounds), 0, GetWorld());
	
	system_types.Nulls.append(system_new.region(system_bounds));

	create_Layout(system_types, large_count, medium_count, 2, GetWorld());

	system_types.display(*this);
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

	large_count = 4;
	medium_count = 10;

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