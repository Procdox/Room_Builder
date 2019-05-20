#include "interior_builder.h"
#include "DrawDebugHelpers.h"
#include "Grid_Tools.h"

Ainterior_builder::Ainterior_builder()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	root = CreateDefaultSubobject<USceneComponent>(TEXT("GeneratedRoot"));
	CollisionMesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("CollisionMesh"));
	CollisionMesh->AttachToComponent(root, FAttachmentTransformRules::KeepRelativeTransform);
	CollisionMesh->ContainsPhysicsTriMeshData(true);
	CollisionMesh->bUseAsyncCooking = true;

	RootComponent = root;
}

FRoom_Details::FRoom_Details()
{
	wall_thickness = 10;
	door_height = 80;
	door_width = 3;

	Wall_Material = nullptr;
	Floor_Material = nullptr;
	Ceiling_Material = nullptr;
}

//==========================================================================================================
//=================================== inset utilities ================================================
//==========================================================================================================

/*void generateInsetPoints(Edge<Pgrd> const * target, grd const & distance,
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
}*/

Room_Boundary::Room_Boundary(Face<Pgrd> * reference)
{
	for (auto edge : reference->getLoopEdges()) {
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
		}
		else {
			inset.Normalize();
			Pgrd rot(A_next.Y, -A_next.X);

			inset /= inset.Dot(rot);
		}

		inset;

		Points.append(A);
		Offsets.append(inset);
		walled = true;
	}
}

//only detects local intersections
FLL<Pgrd> Room_Boundary::Inset(grd const distance) const
{
	//account for local intersections

	//project insets from roots
	//if you find an intersection
		//reverse project a new root and project to the new inset
		//replace the intersecting roots with the novel
		//replace the intersecting inset with the novel
		//rescan for intersections

	FLL<Pgrd> roots(Points);
	FLL<Pgrd> insets;
	FLL<Pgrd>::FLL_iterator_c point = roots.begin();
	for (auto source : Offsets) {
		insets.append(source * distance + *point);
		++point;
	}
	
	FLL<Pgrd>::FLL_iterator A_roots = roots.begin_unsafe();
	FLL<Pgrd>::FLL_iterator A_insets = insets.begin_unsafe();

	do {
		auto B_roots = A_roots.cyclic_next();
		auto C_roots = B_roots.cyclic_next();
		auto D_roots = C_roots.cyclic_next();

		auto B_insets = A_insets.cyclic_next();
		auto C_insets = B_insets.cyclic_next();
		auto D_insets = C_insets.cyclic_next();

		Pgrd Br = *B_roots;
		Pgrd Bi = *B_insets;

		Pgrd Cr = *C_roots;
		Pgrd Ci = *C_insets;

		Pgrd I;

		if (Pgrd::getIntersect(Br, Bi, Cr, Ci, I)) {

			Pgrd Ar = *A_roots;
			Pgrd Dr = *D_roots;

			grd r = (Br - I).Size() / (Br - Bi).Size();
			Pgrd before = Ar - Br;
			Pgrd after = Dr - Cr;
			before.Normalize();
			after.Normalize();

			Pgrd dir = before + after;

			if (dir == Pgrd(0, 0)) {
				dir.X = after.Y;
				dir.Y = -after.X;
			}
			else {
				dir.Normalize();
				Pgrd rot(after.Y, -after.X);

				dir /= dir.Dot(rot);
			}

			Pgrd root = I - (dir * r * distance);
			Pgrd inset = I + (dir * (grd(1) - r) * distance);

			A_roots.remove_next();
			A_roots.remove_next();

			A_roots.insert_after(root);

			A_insets.remove_next();
			A_insets.remove_next();

			A_insets.insert_after(inset);
		}
		else {
			++A_roots;
			++A_insets;
		}
	} while (A_roots != roots.end_unsafe() && roots.size() > 3);

	return insets;
}

Room_Layout::Room_Layout(Region<Pgrd> * reference, double _bottom, double _top)
{
	for (auto face : reference->getBounds())
	{
		Boundaries.append(new Room_Boundary(face));
	}

	bottom = _bottom;
	top = _top;

	ceiling = true;
	floor = true;
}

//==========================================================================================================
//=================================== triangulate utilities ================================================
//==========================================================================================================

namespace tri_utils
{
	bool tryIntersect(const Pgrd &A_S, const Pgrd &A_E, const Pgrd &B_S, const Pgrd &B_E) {
		const auto A = A_E - A_S;
		const auto B = B_E - B_S;

		const auto D = A_S - B_S;

		const auto denom = A.X * B.Y - A.Y * B.X;

		if (denom == 0) { //REFACTOR
			return false;
		}

		const auto s = (A.X * D.Y - A.Y * D.X) / denom;
		const auto t = (B.X * D.Y - B.Y * D.X) / denom;

		return (s >= 0 && s <= 1 && t >= 0 && t <= 1);
	}
	bool tryInteriorIntersect(const Pgrd &A_S, const Pgrd &A_E, const Pgrd &B_S, const Pgrd &B_E) {
		const auto A = A_E - A_S;
		const auto B = B_E - B_S;

		const auto D = A_S - B_S;

		const auto denom = A.X * B.Y - A.Y * B.X;

		if (denom == 0) { //REFACTOR
			return false;
		}

		const auto s = (A.X * D.Y - A.Y * D.X) / denom;
		const auto t = (B.X * D.Y - B.Y * D.X) / denom;

		return (s > 0 && s < 1 && t >= 0 && t <= 1);
	}
	bool IsConcave(const Pgrd &A, const Pgrd &B, const Pgrd &C) {
		Pgrd In = B - A;
		Pgrd Out = C - B;
		Pgrd OutCCW(-Out.Y, Out.X);

		return (In.Dot(OutCCW) < 0);
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
	TArray<Pgrd> MergeBorders(TArray<TArray<Pgrd>> &Borders) {
		while (Borders.Num() > 1)
		{
			const auto size = Borders.Num();
			bool success = false;

			int poly_A, poly_B, idx_A, idx_B;

			for (poly_A = 0; poly_A < size - 1; poly_A++) {
				const int32 poly_A_size = Borders[poly_A].Num();

				for (poly_B = poly_A + 1; poly_B < size; poly_B++) {
					const int32 poly_B_size = Borders[poly_B].Num();

					for (idx_A = 0; idx_A < poly_A_size; idx_A++) {
						for (idx_B = 0; idx_B < poly_B_size; idx_B++) {

							Pgrd A = Borders[poly_A][idx_A];
							Pgrd B = Borders[poly_B][idx_B];

							{
								Pgrd In_A = Borders[poly_A][(idx_A + poly_A_size - 1) % poly_A_size];
								Pgrd Out_A = Borders[poly_A][(idx_A + 1) % poly_A_size];

								Pgrd In_B = Borders[poly_B][(idx_B + poly_B_size - 1) % poly_B_size];
								Pgrd Out_B = Borders[poly_B][(idx_B + 1) % poly_B_size];

								success = angledBetween(Out_A - A, In_A - A,  B - A) && angledBetween(Out_B - B, In_B - B,  A - B);
							}

							if (!success)
								continue;

							for (int poly_X = 0; poly_X < size; poly_X++) {
								const int32 poly_X_size = Borders[poly_X].Num();

								for (int idx_X = 0; idx_X < poly_X_size; idx_X++) {
									int idx_Y = (idx_X + 1) % poly_X_size;

									if (poly_X == poly_A)
										if (idx_X == idx_A || idx_Y == idx_A)
											continue;

									if (poly_X == poly_B)
										if (idx_X == idx_B || idx_Y == idx_B)
											continue;

									Pgrd X = Borders[poly_X][idx_X];
									Pgrd Y = Borders[poly_X][idx_Y];

									if (tryInteriorIntersect(A, B, X, Y)) {
										success = false;
										break;
									}

								}
								if (!success)
									break;
							}
							if (success)
								goto AFTER_SEARCH;
						}
					}
				}
			}

			AFTER_SEARCH:

			if (success) {
				/*  migrate poly b to poly a, linking between index_a and index_a  */
				/*  this operation decreases the size of UnlinkedIndexBuffer by 1  */
				int32 tail_length = Borders[poly_A].Num() - idx_A;
				Borders[poly_A].AddUninitialized(Borders[poly_B].Num() + 2);

				//now copy the tail of poly_a to the end of poly_a
				for (int32 offset = tail_length - 1; offset >= 0; offset--) {
					int32 source = idx_A + offset;
					int32 destination = idx_A + Borders[poly_B].Num() + 2 + offset;
					Borders[poly_A][destination] = Borders[poly_A][source];
				}

				//now copy in poly_b
				int32 position = 1;
				for (int32 offset = idx_B; offset < Borders[poly_B].Num(); offset++) {
					Borders[poly_A][idx_A + position++] = Borders[poly_B][offset];
				}
				for (int32 offset = 0; offset <= idx_B; offset++) {
					Borders[poly_A][idx_A + position++] = Borders[poly_B][offset];
				}

				Borders.RemoveAt(poly_B);
			}
			else {
				//no valid merge was found
				throw;
			}

		}
		return Borders[0];
	}
	//TArray<int32> MergeBuffers(TArray<Pgrd> &VectorBuffer, TArray<TArray<int32>> &UnlinkedIndexBuffer)
	//{
	//	TArray<int32> IndexBuffer;
	//	{
	//		int32 poly_a, poly_b, index_a, index_b, segment_start_index, segment_end_index;
	//		int32 poly_ex, index_ex, test_start_index, test_end_index;
	//		bool valid = false;
	//		poly_a = 0;
	//		poly_b = 0;
	//		index_a = 0;
	//		index_b = 0;
	//
	//		while (UnlinkedIndexBuffer.Num() > 1) {
	//			const int32 buffer_size = UnlinkedIndexBuffer.Num();
	//			for (poly_a = 0; poly_a < buffer_size - 1; poly_a++) {
	//				const int32 poly_a_size = UnlinkedIndexBuffer[poly_a].Num();
	//				for (poly_b = poly_a + 1; poly_b < buffer_size; poly_b++) {
	//					const int32 poly_b_size = UnlinkedIndexBuffer[poly_b].Num();
	//					for (index_a = 0; index_a < poly_a_size; index_a++) {
	//						for (index_b = 0; index_b < poly_b_size; index_b++) {
	//							valid = false;
	//							segment_start_index = UnlinkedIndexBuffer[poly_a][index_a];
	//							segment_end_index = UnlinkedIndexBuffer[poly_b][index_b];
	//
	//							/*  range check  */
	//							//if the difference from the inner bisector is greater than the leaving difference
	//							//find inner bisector
	//							{
	//								int32 coming_index = UnlinkedIndexBuffer[poly_a][(index_a + poly_a_size - 1) % poly_a_size];
	//								int32 going_index = UnlinkedIndexBuffer[poly_a][(index_a + 1) % poly_a_size];
	//								Pgrd coming_vector = VectorBuffer[segment_start_index] - VectorBuffer[coming_index];
	//								Pgrd going_vector = VectorBuffer[going_index] - VectorBuffer[segment_start_index];
	//								coming_vector.Normalize();
	//								going_vector.Normalize();
	//								Pgrd inner_bisector = going_vector - coming_vector;
	//								inner_bisector.Normalize();
	//								//flip if angle is concave
	//
	//								if(going_vector.Dot(Pgrd(-coming_vector.Y, coming_vector.X)) < 0)
	//								{
	//									inner_bisector *= -1;
	//								}
	//								Pgrd sweep = VectorBuffer[segment_end_index] - VectorBuffer[segment_start_index];
	//								sweep.Normalize();
	//								valid = (sweep.Dot(inner_bisector) > going_vector.Dot(inner_bisector));
	//							}
	//							if (!valid) { continue; }
	//							{
	//								int32 coming_index = UnlinkedIndexBuffer[poly_b][(index_b + poly_b_size - 1) % poly_b_size];
	//								int32 going_index = UnlinkedIndexBuffer[poly_b][(index_b + 1) % poly_b_size];
	//								Pgrd coming_vector = VectorBuffer[segment_start_index] - VectorBuffer[coming_index];
	//								Pgrd going_vector = VectorBuffer[going_index] - VectorBuffer[segment_start_index];
	//								coming_vector.Normalize();
	//								going_vector.Normalize();
	//								Pgrd inner_bisector = going_vector - coming_vector;
	//								inner_bisector.Normalize();
	//								//flip if angle is concave
	//								if (going_vector.Dot(Pgrd(-coming_vector.Y, coming_vector.X)) < 0)
	//								{
	//									inner_bisector *= -1;
	//								}
	//								Pgrd sweep = VectorBuffer[segment_start_index] - VectorBuffer[segment_end_index];
	//								sweep.Normalize();
	//								valid = (sweep.Dot(inner_bisector) > going_vector.Dot(inner_bisector));
	//							}
	//							if (!valid) { continue; }
	//
	//							/*  test if the proposed linking segment would preserve planarity  */
	//							for (poly_ex = 0; poly_ex < buffer_size; poly_ex++) {
	//								const int32 poly_ex_size = UnlinkedIndexBuffer[poly_ex].Num();
	//								for (index_ex = 0; index_ex < poly_ex_size; index_ex++) {
	//									test_start_index = UnlinkedIndexBuffer[poly_ex][index_ex];
	//									test_end_index = UnlinkedIndexBuffer[poly_ex][(index_ex + 1) % poly_ex_size];
	//
	//									if (test_start_index == segment_start_index ||
	//										test_end_index == segment_start_index ||
	//										test_end_index == segment_end_index ||
	//										test_start_index == segment_end_index) {
	//										continue;
	//									}
	//
	//									if (tryIntersect(VectorBuffer[segment_start_index], VectorBuffer[segment_end_index],
	//										VectorBuffer[test_start_index], VectorBuffer[test_end_index])) {
	//
	//										valid = false;
	//										break;
	//									}
	//
	//								}
	//								if (!valid) {
	//									break;
	//								}
	//							}
	//							if (valid) {
	//								break;
	//							}
	//						}
	//						if (valid) {
	//							break;
	//						}
	//					}
	//					if (valid) {
	//						break;
	//					}
	//				}
	//				if (valid) {
	//					break;
	//				}
	//			}
	//			if (valid) {
	//				/*  migrate poly b to poly a, linking between index_a and index_a  */
	//				/*  this operation decreases the size of UnlinkedIndexBuffer by 1  */
	//				int32 tail_length = UnlinkedIndexBuffer[poly_a].Num() - index_a;
	//				UnlinkedIndexBuffer[poly_a].AddUninitialized(UnlinkedIndexBuffer[poly_b].Num() + 2);
	//
	//				//now copy the tail of poly_a to the end of poly_a
	//				for (int32 offset = tail_length - 1; offset >= 0; offset--) {
	//					int32 source = index_a + offset;
	//					int32 destination = index_a + UnlinkedIndexBuffer[poly_b].Num() + 2 + offset;
	//					UnlinkedIndexBuffer[poly_a][destination] = UnlinkedIndexBuffer[poly_a][source];
	//				}
	//
	//				//now copy in poly_b
	//				int32 position = 1;
	//				for (int32 offset = index_b; offset < UnlinkedIndexBuffer[poly_b].Num(); offset++) {
	//					UnlinkedIndexBuffer[poly_a][index_a + position++] = UnlinkedIndexBuffer[poly_b][offset];
	//				}
	//				for (int32 offset = 0; offset <= index_b; offset++) {
	//					UnlinkedIndexBuffer[poly_a][index_a + position++] = UnlinkedIndexBuffer[poly_b][offset];
	//				}
	//
	//				UnlinkedIndexBuffer.RemoveAt(poly_b);
	//			}
	//			else {
	//				//no valid merge was found
	//				throw;
	//			}
	//		}
	//	}
	//	IndexBuffer = UnlinkedIndexBuffer[0];
	//	return IndexBuffer;
	//}

	TArray<int32> Triangulate(TArray<Pgrd> const &VectorBuffer, TArray<int32> &IndexBuffer) {
		/*  triangulate via ear clipping  */
		/*  requires intersect and concavity check  */
		TArray<int32> Triangles;
		int32 frozen = 0;
		{
			int32 abt_In, abt_A, abt_B, abt_C, abt_Out;
			abt_In = 0;

			int32 idx_In, idx_A, idx_B, idx_C, idx_Out;

			Pgrd pt_In, pt_A, pt_B, pt_C, pt_Out;

			bool success;

			while (IndexBuffer.Num() > 2) {
				int size = IndexBuffer.Num();

				//abt_Int
				abt_A = (abt_In + 1) % size;
				abt_B = (abt_In + 2) % size;
				abt_C = (abt_In + 3) % size;
				abt_Out = (abt_In + 4) % size;

				idx_In = IndexBuffer[abt_In];
				idx_A = IndexBuffer[abt_A];
				idx_B = IndexBuffer[abt_B];
				idx_C = IndexBuffer[abt_C];
				idx_Out = IndexBuffer[abt_Out];

				pt_In = VectorBuffer[idx_In];
				pt_A = VectorBuffer[idx_A];
				pt_B = VectorBuffer[idx_B];
				pt_C = VectorBuffer[idx_C];
				pt_Out = VectorBuffer[idx_Out];

				success = false;

				//is an ear?
				if (IsConcave(pt_C, pt_B, pt_A )) {

					//respects corners
					if (angledBetween(pt_B - pt_A, pt_In - pt_A, pt_C - pt_A) && angledBetween(pt_Out - pt_C, pt_B - pt_C, pt_A - pt_C)) {

						//preserves planarity?
						success = true;

						for (int32 index_ex = 0; index_ex < IndexBuffer.Num(); index_ex++) {
							int32 test_start_index = IndexBuffer[index_ex];
							int32 test_end_index = IndexBuffer[(index_ex + 1) % (IndexBuffer.Num())];

							if (test_start_index == idx_A ||
								test_end_index == idx_A ||
								test_end_index == idx_C ||
								test_start_index == idx_C ) {
								continue;
							}

							if (tryInteriorIntersect(pt_A, pt_C,
								VectorBuffer[test_start_index], VectorBuffer[test_end_index])) {

								success = false;
								break;
							}

						}
					}
				}

				if (success) {

					//remove point
					Triangles.Add(idx_C);
					Triangles.Add(idx_B);
					Triangles.Add(idx_A);

					IndexBuffer.RemoveAt(abt_B);

					abt_In = IndexBuffer.Num() <= abt_In ? IndexBuffer.Num() - 1 : abt_In;
					frozen = 0;
				}
				else
				{
					abt_In = (abt_In + 1) % IndexBuffer.Num();
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

UProceduralMeshComponent * Ainterior_builder::CreateMeshComponent() {
	UProceduralMeshComponent* component = NewObject<UProceduralMeshComponent>();

	component->AttachToComponent(root, FAttachmentTransformRules::KeepRelativeTransform);
	component->ContainsPhysicsTriMeshData(true);
	component->bUseAsyncCooking = true;

	return component;
}

void Ainterior_builder::ActivateMeshComponent(UProceduralMeshComponent * component) {
	component->ContainsPhysicsTriMeshData(true);

	component->Activate();

	component->RegisterComponentWithWorld(GetWorld());
}


void Ainterior_builder::CreateWallSegment(FLL<Pgrd>::FLL_iterator_c point,
	UProceduralMeshComponent * component, int section_id) {

	TArray<FVector> Vertices;
	TArray<int32> Triangles;
	TArray<FVector> Normals;
	TArray<FVector2D> UV0;
	TArray<FProcMeshTangent> Tangents;
	TArray<FLinearColor> VertexColors;

	Pgrd wall_left = *point;
	Pgrd wall_right = *(point.cyclic_next());

	Pgrd dir = wall_left - wall_right;
	dir.Normalize();

	Pgrd normal(dir.Y, -dir.X);

	FVector2D f_wall_left = convert(wall_left);
	FVector2D f_wall_right = convert(wall_right);

	FVector f_normal(convert(normal), 0);

	Vertices.Push(FVector(f_wall_left, layout->bottom));
	Vertices.Push(FVector(f_wall_left, layout->top));
	Vertices.Push(FVector(f_wall_right, layout->bottom));
	Vertices.Push(FVector(f_wall_right, layout->top));

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

	UV0.Push(FVector2D(0, layout->bottom));
	UV0.Push(FVector2D(0, layout->top));
	UV0.Push(FVector2D(1, layout->bottom));
	UV0.Push(FVector2D(1, layout->top));

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

/*void Ainterior_builder::CreateDoorSegment(FLL<Pgrd>::FLL_iterator_c point, FLL<Pgrd>::FLL_iterator_c offset,
	UProceduralMeshComponent * component, int section_id) {

	TArray<FVector> Vertices;
	TArray<int32> Triangles;
	TArray<FVector> Normals;
	TArray<FVector2D> UV0;
	TArray<FProcMeshTangent> Tangents;
	TArray<FLinearColor> VertexColors;

	Pgrd wall_left, wall_right;
	generateInsetPoints(target, grd(details->wall_thickness / 2), wall_left, wall_right);

	Pgrd dir = wall_left - wall_right;
	dir.Normalize();

	Pgrd normal(dir.Y, -dir.X);

	FVector2D f_wall_left = convert(wall_left);
	FVector2D f_wall_right = convert(wall_right);

	FVector f_normal(convert(normal), 0);

	FVector2D f_inset_left = convert(target->getStart()->getPosition());
	FVector2D f_inset_right = convert(target->getEnd()->getPosition());

	if (top - bottom > details->door_height) {
		Vertices.Push(FVector(f_wall_left, bottom));
		Vertices.Push(FVector(f_wall_left, bottom + details->door_height));

		Vertices.Push(FVector(f_inset_left, bottom));
		Vertices.Push(FVector(f_inset_left, bottom + details->door_height));

		Vertices.Push(FVector(f_wall_right, bottom));
		Vertices.Push(FVector(f_wall_right, bottom + details->door_height));

		Vertices.Push(FVector(f_inset_right, bottom));
		Vertices.Push(FVector(f_inset_right, bottom + details->door_height));

		Vertices.Push(FVector(f_wall_left, top));
		Vertices.Push(FVector(f_wall_right, top));
	}
	else {
		Vertices.Push(FVector(convert(wall_left), bottom));
		Vertices.Push(FVector(convert(wall_left), top));

		Vertices.Push(FVector(f_inset_left, bottom));
		Vertices.Push(FVector(f_inset_left, top));

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
	if (top - bottom > details->door_height) {
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
void Ainterior_builder::CreateWindowSegment(FLL<Pgrd>::FLL_iterator_c point, FLL<Pgrd>::FLL_iterator_c offset,
	UProceduralMeshComponent * component, int section_id) {

}*/

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
			FVector(border[index], height + offset * 2),
			FVector(border[next], height + offset * 2),
			//FColor(FMath::RandRange(0,255), FMath::RandRange(0, 255), FMath::RandRange(0, 255)),
			color,
			true,
			-1,
			0,
			3
		);
		offset++;
	}
}

void Ainterior_builder::CreateFloorAndCeiling() {
	//border linking

	TArray<TArray<Pgrd>> Borders;
	for (auto ref : layout->Boundaries)
	{
		TArray<Pgrd> border;
		FLL<Pgrd> points = ref->Inset(details->wall_thickness);

		auto next_point = points.begin();

		while (next_point != points.end())
		{
			border.Push(*next_point);

			++next_point;
		}
		Borders.Push(border);
	}

	TArray<Pgrd> VectorBuffer_Raw = tri_utils::MergeBorders(Borders);
	TArray<Pgrd> VectorBuffer_Compute;
	TArray<FVector2D> VectorBuffer_Display;
	TArray<int32> IndexBuffer;
	int i = 0;

	for (auto vector : VectorBuffer_Raw)
	{
		int idx = VectorBuffer_Compute.Find(vector);
		if (idx != INDEX_NONE)
			IndexBuffer.Push(idx);
		else
		{
			VectorBuffer_Compute.Push(vector);
			VectorBuffer_Display.Push(convert(vector));
			IndexBuffer.Push(i++);
		}
	}

	Draw_Border(VectorBuffer_Display, 100, GetWorld());
	
	/*TArray<Pgrd> VectorBuffer_Compute;
	TArray<FVector2D> VectorBuffer_Display;
	TArray<TArray<int32>> IndexBuffer_Unlinked;
	int ii = 0;

	for (auto ref : layout->Boundaries)
	{
		TArray<int32> index_segment;
		//auto next_offset = ref->Offsets.begin();
		auto next_point = ref->Points.begin();

		while (next_point != ref->Points.end())
		{
			Pgrd inset = *next_point + (*next_offset * details->wall_thickness);

			VectorBuffer_Compute.Push(inset);
			VectorBuffer_Display.Push(convert(inset));

			++next_offset;
			++next_point;

			index_segment.Push(ii++);
		}

		//border_segment = tri_utils::Reverse(border_segment);
		//index_segment = tri_utils::Reverse(index_segment);

		//Border.Append(border_segment);
		IndexBuffer_Unlinked.Push(index_segment);
	}

	auto IndexBuffer = tri_utils::MergeBuffers(VectorBuffer_Compute, IndexBuffer_Unlinked);*/

	TArray<int32> Triangles_top = tri_utils::Triangulate(VectorBuffer_Compute, IndexBuffer);
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

	for (auto& vector : VectorBuffer_Display) {
		vertices_bottom.Add(FVector(vector.X, vector.Y, layout->bottom));
		vertices_top.Add(FVector(vector.X, vector.Y, layout->top));
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

	Floor_Mesh->SetMaterial(0, details->Floor_Material);
	Floor_Mesh->SetMaterial(1, details->Ceiling_Material);

	Floor_Mesh->RegisterComponentWithWorld(GetWorld());

}
void Ainterior_builder::CreateWallSections(Room_Boundary const * source) {

	FLL<Pgrd> points = source->Inset(details->wall_thickness);

	//double door_tolerance = details->door_width + details->wall_thickness;

	auto next_point = points.begin();

	while (next_point != points.end())
	{
		auto component = CreateMeshComponent();
		CreateWallSegment(next_point, component, 0);
		component->SetMaterial(0, details->Wall_Material);

		++next_point;

		ActivateMeshComponent(component);
	}


	/*auto border_points = border->getLoopEdges();

	for (auto edge : border_points) {

		Pgrd const A = edge->getStart()->getPosition();
		Pgrd const B = edge->getEnd()->getPosition();

		auto segment = B - A;
		grd size = segment.Size();

		Region<Pgrd> * op = edge->getInv()->getFace()->getGroup();
		auto component = CreateMeshComponent();

		if (edge->mark == 0) {
			if (size <= door_tolerance || op == nullptr || op->mark == 0) {
				CreateWallSegment(edge, component, 0);
				component->SetMaterial(0, details->Wall_Material);
				edge->mark = 1;
				edge->getInv()->mark = 1;
			}
			else {
				auto mid_point = (segment / 2) + A;
				segment.Normalize();
				segment *= details->door_width / 2;

				edge->subdivide(mid_point + segment);
				edge->subdivide(mid_point - segment);

				auto middle = edge->getNext();
				auto opposite = middle->getNext();

				CreateWallSegment(edge, component, 0);
				CreateDoorSegment(middle, component, 1);
				CreateWallSegment(opposite, component, 2);
				component->SetMaterial(0, details->Wall_Material);
				component->SetMaterial(1, details->Wall_Material);
				component->SetMaterial(2, details->Wall_Material);

				edge->mark = 1;
				edge->getInv()->mark = 1;

				middle->mark = 2;
				middle->getInv()->mark = 2;

				opposite->mark = 1;
				opposite->getInv()->mark = 1;
			}
		}
		else if (edge->mark == 1) {
			CreateWallSegment(edge, component, 0);
			component->SetMaterial(0, details->Wall_Material);
		}
		else {
			CreateDoorSegment(edge, component, 0);
			component->SetMaterial(0, details->Wall_Material);
		}

		ActivateMeshComponent(component);
	}*/
}

void Ainterior_builder::Create(Room_Layout const * _layout, FRoom_Details const * _details)
{
	details = _details;
	layout = _layout;

	CreateFloorAndCeiling();

	for (auto boundary : layout->Boundaries)
		if(boundary->walled)
			CreateWallSections(boundary);
	

}
