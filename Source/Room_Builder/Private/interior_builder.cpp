#include "interior_builder.h"
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

		inset += A;

		Points.append(A);
		Offsets.append(inset);
		walled = true;
	}
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

		return (FVector2D::DotProduct(In, OutCCW) < 0);
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
	TArray<int32> MergeBuffers(TArray<FVector2D> &VectorBuffer, TArray<TArray<int32>> &UnlinkedIndexBuffer)
	{
		TArray<int32> IndexBuffer;
		{
			int32 poly_a, poly_b, index_a, index_b, segment_start_index, segment_end_index;
			int32 poly_ex, index_ex, test_start_index, test_end_index;
			bool valid = false;
			poly_a = 0;
			poly_b = 0;
			index_a = 0;
			index_b = 0;
			while (UnlinkedIndexBuffer.Num() > 1) {
				const int32 buffer_size = UnlinkedIndexBuffer.Num();
				for (poly_a = 0; poly_a < buffer_size - 1; poly_a++) {
					const int32 poly_a_size = UnlinkedIndexBuffer[poly_a].Num();
					for (poly_b = poly_a + 1; poly_b < buffer_size; poly_b++) {
						const int32 poly_b_size = UnlinkedIndexBuffer[poly_b].Num();
						for (index_a = 0; index_a < poly_a_size; index_a++) {
							for (index_b = 0; index_b < poly_b_size; index_b++) {
								valid = false;
								segment_start_index = UnlinkedIndexBuffer[poly_a][index_a];
								segment_end_index = UnlinkedIndexBuffer[poly_b][index_b];

								/*  range check  */
								//if the difference from the inner bisector is greater than the leaving difference
								//find inner bisector
								{
									int32 coming_index = UnlinkedIndexBuffer[poly_a][(index_a + poly_a_size - 1) % poly_a_size];
									int32 going_index = UnlinkedIndexBuffer[poly_a][(index_a + 1) % poly_a_size];
									FVector2D coming_vector = VectorBuffer[segment_start_index] - VectorBuffer[coming_index];
									FVector2D going_vector = VectorBuffer[going_index] - VectorBuffer[segment_start_index];
									coming_vector.Normalize();
									going_vector.Normalize();
									FVector2D inner_bisector = going_vector - coming_vector;
									inner_bisector.Normalize();
									//flip if angle is concave
									if (FVector2D::DotProduct(FVector2D(-coming_vector.Y, coming_vector.X), going_vector) < 0) {
										inner_bisector *= -1;
									}
									FVector2D sweep = VectorBuffer[segment_end_index] - VectorBuffer[segment_start_index];
									sweep.Normalize();
									valid = (FVector2D::DotProduct(sweep, inner_bisector) > FVector2D::DotProduct(going_vector, inner_bisector));
								}
								if (!valid) { continue; }
								{
									int32 coming_index = UnlinkedIndexBuffer[poly_b][(index_b + poly_b_size - 1) % poly_b_size];
									int32 going_index = UnlinkedIndexBuffer[poly_b][(index_b + 1) % poly_b_size];
									FVector2D coming_vector = VectorBuffer[segment_start_index] - VectorBuffer[coming_index];
									FVector2D going_vector = VectorBuffer[going_index] - VectorBuffer[segment_start_index];
									coming_vector.Normalize();
									going_vector.Normalize();
									FVector2D inner_bisector = going_vector - coming_vector;
									inner_bisector.Normalize();
									//flip if angle is concave
									if (FVector2D::DotProduct(FVector2D(-coming_vector.Y, coming_vector.X), going_vector) < 0) {
										inner_bisector *= -1;
									}
									FVector2D sweep = VectorBuffer[segment_start_index] - VectorBuffer[segment_end_index];
									sweep.Normalize();
									valid = (FVector2D::DotProduct(sweep, inner_bisector) > FVector2D::DotProduct(going_vector, inner_bisector));
								}
								if (!valid) { continue; }

								/*  test if the proposed linking segment would preserve planarity  */
								for (poly_ex = 0; poly_ex < buffer_size; poly_ex++) {
									const int32 poly_ex_size = UnlinkedIndexBuffer[poly_ex].Num();
									for (index_ex = 0; index_ex < poly_ex_size; index_ex++) {
										test_start_index = UnlinkedIndexBuffer[poly_ex][index_ex];
										test_end_index = UnlinkedIndexBuffer[poly_ex][(index_ex + 1) % poly_ex_size];

										if (test_start_index == segment_start_index ||
											test_end_index == segment_start_index ||
											test_end_index == segment_end_index ||
											test_start_index == segment_end_index) {
											continue;
										}

										if (TryIntersect(VectorBuffer[segment_start_index], VectorBuffer[segment_end_index],
											VectorBuffer[test_start_index], VectorBuffer[test_end_index])) {

											valid = false;
											break;
										}

									}
									if (!valid) {
										break;
									}
								}
								if (valid) {
									break;
								}
							}
							if (valid) {
								break;
							}
						}
						if (valid) {
							break;
						}
					}
					if (valid) {
						break;
					}
				}
				if (valid) {
					/*  migrate poly b to poly a, linking between index_a and index_a  */
					/*  this operation decreases the size of UnlinkedIndexBuffer by 1  */
					int32 tail_length = UnlinkedIndexBuffer[poly_a].Num() - index_a;
					UnlinkedIndexBuffer[poly_a].AddUninitialized(UnlinkedIndexBuffer[poly_b].Num() + 2);

					//now copy the tail of poly_a to the end of poly_a
					for (int32 offset = tail_length - 1; offset >= 0; offset--) {
						int32 source = index_a + offset;
						int32 destination = index_a + UnlinkedIndexBuffer[poly_b].Num() + 2 + offset;
						UnlinkedIndexBuffer[poly_a][destination] = UnlinkedIndexBuffer[poly_a][source];
					}

					//now copy in poly_b
					int32 position = 1;
					for (int32 offset = index_b; offset < UnlinkedIndexBuffer[poly_b].Num(); offset++) {
						UnlinkedIndexBuffer[poly_a][index_a + position++] = UnlinkedIndexBuffer[poly_b][offset];
					}
					for (int32 offset = 0; offset <= index_b; offset++) {
						UnlinkedIndexBuffer[poly_a][index_a + position++] = UnlinkedIndexBuffer[poly_b][offset];
					}

					UnlinkedIndexBuffer.RemoveAt(poly_b);
				}
				else {
					//no valid merge was found
					throw;
				}
			}
		}

		IndexBuffer = UnlinkedIndexBuffer[0];
		return IndexBuffer;
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


void Ainterior_builder::CreateWallSegment(FLL<Pgrd>::FLL_iterator_c point, FLL<Pgrd>::FLL_iterator_c offset,
	UProceduralMeshComponent * component, int section_id) {

	TArray<FVector> Vertices;
	TArray<int32> Triangles;
	TArray<FVector> Normals;
	TArray<FVector2D> UV0;
	TArray<FProcMeshTangent> Tangents;
	TArray<FLinearColor> VertexColors;

	Pgrd wall_left = *point + (*offset * details->wall_thickness);
	Pgrd wall_right = *(point.cyclic_next()) + (*(offset.cyclic_next()) * details->wall_thickness);

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

void Ainterior_builder::CreateFloorAndCeiling() {
	//border linking
	
	TArray<FVector2D> Border;
	TArray<TArray<int32>> indexBuffer;
	int ii = 0;

	for (auto ref : layout->Boundaries)
	{
		TArray<int32> index_segment;
		TArray<FVector2D> border_segment;
		auto next_offset = ref->Offsets.begin();
		auto next_point = ref->Points.begin();

		while (next_point != ref->Points.end())
		{
			border_segment.Push(convert(*next_point + (*next_offset * details->wall_thickness)));

			++next_offset;
			++next_point;

			index_segment.Push(ii++);
		}

		//border_segment = tri_utils::Reverse(border_segment);
		//index_segment = tri_utils::Reverse(index_segment);

		Border.Append(border_segment);
		indexBuffer.Push(index_segment);
	}

	auto Index = tri_utils::MergeBuffers(Border, indexBuffer);

	TArray<int32> Triangles_top = tri_utils::Triangulate(Border, Index);
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



	double door_tolerance = details->door_width + details->wall_thickness;

	auto next_offset = source->Offsets.begin();
	auto next_point = source->Points.begin();
	while (next_point != source->Points.end())
	{
		auto component = CreateMeshComponent();
		CreateWallSegment(next_point, next_offset, component, 0);
		component->SetMaterial(0, details->Wall_Material);

		++next_point;
		++next_offset;

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
