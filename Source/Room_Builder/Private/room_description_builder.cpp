// Fill out your copyright notice in the Description page of Project Settings.

#include "room_description_builder.h"
#include "Algo/Reverse.h"
#include "DrawDebugHelpers.h"

//namespace LINKED_BUTTS
struct edge;
struct face;

struct vertex {
	edge *base_edge;
	FVector2D position;
};

struct edge {
	vertex * base_vertex;
	vertex * end_vertex() {
		return inverse->base_vertex;
	}

	FVector2D & start_position() {
		return base_vertex->position;
	}

	FVector2D & end_position() {
		return end_vertex()->position;
	}

	void subdivide(const FVector2D &position) {
		vertex* mid = new vertex();
		mid->position = position;
		mid->base_edge = inverse;

		edge* my_n = new edge();
		edge* my_i = new edge();

		my_n->base_face = base_face;
		my_i->base_face = inverse->base_face;

		inverse->base_vertex->base_edge = my_i;

		my_i->base_vertex = inverse->base_vertex;
		my_n->base_vertex = mid;
		inverse->base_vertex = mid;

		my_n->inverse = my_i;
		my_i->inverse = my_n;

		my_n->next = next;
		my_i->previous = inverse->previous;

		next->previous = my_n;
		inverse->previous->next = my_i;

		my_n->previous = this;
		my_i->next = inverse;

		next = my_n;
		inverse->previous = my_i;
	}

	edge *previous;
	edge *next;
	edge *inverse;

	face *base_face;
};

struct face {
	TArray<edge*> base_edges;

	ClipperLib::Paths Export_Interior() const {
		ClipperLib::Paths result;

		for (auto component : base_edges) {
			edge* current_edge = component;
			ClipperLib::Path section;

			do {
				section.push_back(ClipperLib::IntPoint(current_edge->base_vertex->position.X, current_edge->base_vertex->position.Y));
				current_edge = current_edge->next;
			} while (current_edge != component);

			result.push_back(section);
		}

		return result;
	}
	bool isBaseEdge(edge* test) {
		for (auto compare : base_edges) {
			if (test == compare) {
				return true;
			}
		}
		return false;
	}
};

enum edge_correction { null_static, null_intersect, null_atvertex, connected_static, connected_intersect, connected_atvertex };

struct suggested_component {
	edge* initial_edge;
	TArray<edge_correction> edge_types;
};

struct suggested_region {
	TArray<suggested_component> components;
	face* region;
};

//find the order of the set of edges st:
//	for an edge
//		base_vertex.y != end_vertex.y
//		contain a point [x,y] 
//			in their [base_vertex, end_vertex) range where 
//				X is greater than test_point.x
//				y = test_point.y
//an odd parity implies the point is interior
//an even parity implites
bool isPointInterior(const FVector2D &test_point, const TArray<FVector2D> &region) {
	const int size = region.Num();
	int count = 0;
	for (int ii = 0; ii < size; ii++) {
		int next = (ii + 1) % size;

		//if the edge is parrallel to the x-axis, skip
		if (region[ii].Y == region[next].Y) {
			continue;
		}

		float y_offset = test_point.Y - region[ii].Y;
		float y_length = region[next].Y - region[ii].Y;
		float y_ratio = y_offset / y_length;

		if (y_ratio < 1.f && y_ratio >= 0.f) {

			float x_length = region[next].X - region[ii].X;
			float x = region[ii].X + x_length * y_ratio;

			if (x == test_point.X) {
				return true;
			}
			if (x > test_point.X) {
				count++;
			}
		}
	}

	return (count % 2 == 1);
}

bool isPointInterior(const FVector2D &test_point, const face &region) {
	for (auto &start : region.base_edges) {
		edge *focus = start;
		int count = 0;

		do {
			const FVector2D &start_vector = focus->start_position();
			const FVector2D &end_vector = focus->end_position();


			if (start_vector.Y == end_vector.Y) {
				focus = focus->next;
				continue;
			}

			float y_offset = test_point.Y - start_vector.Y;
			float y_length = end_vector.Y - start_vector.Y;
			float y_ratio = y_offset / y_length;

			if (y_ratio < 1.f && y_ratio >= 0.f) {

				float x_length = end_vector.X - start_vector.X;
				float x = start_vector.X + x_length * y_ratio;

				if (x == test_point.X) {
					return true;
				}
				if (x > test_point.X) {
					count++;
				}
			}


			focus = focus->next;
		} while (focus != start);

		if (count % 2 == 1) {
			return true;
		}
	}

	return false;
}

bool isPointStrictlyInterior(const FVector2D &test_point, const face &region) {
	for (auto &start : region.base_edges) {
		edge *focus = start;
		int count = 0;

		do {
			const FVector2D &start_vector = focus->start_position();
			const FVector2D &end_vector = focus->end_position();


			if (start_vector.Y == end_vector.Y) {
				focus = focus->next;
				continue;
			}

			float y_offset = test_point.Y - start_vector.Y;
			float y_length = end_vector.Y - start_vector.Y;
			float y_ratio = y_offset / y_length;

			if (y_ratio < 1.f && y_ratio >= 0.f) {

				float x_length = end_vector.X - start_vector.X;
				float x = start_vector.X + x_length * y_ratio;

				if (x == test_point.X) {
					return false;
				}
				if (x > test_point.X) {
					count++;
				}
			}


			focus = focus->next;
		} while (focus != start);

		if (count % 2 == 1) {
			return true;
		}
	}

	return false;
}

float intersectRatio(const FVector2D &A_S, const FVector2D &A_E, const FVector2D &B_S, const FVector2D &B_E) {
	double ua, ub, denom;

	denom = (B_E.Y - B_S.Y)*(A_E.X - A_S.X) - (B_E.X - B_S.X)*(A_E.Y - A_S.Y);
	if (denom == 0) {
		return false;
	}

	ua = ((B_E.X - B_S.X)*(A_S.Y - B_S.Y) - (B_E.Y - B_S.Y)*(A_S.X - B_S.X)) / denom;
	ub = ((A_E.X - A_S.X)*(A_S.Y - B_S.Y) - (A_E.Y - A_S.Y)*(A_S.X - B_S.X)) / denom;

	if (ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1) {
		return ua;
	}

	return -1.f;
}

struct RegionIntersectData {
	edge* target_edge;
	float ratio;
};

RegionIntersectData calculateFirstIntersect(const FVector2D &A_S, const FVector2D &A_E, const face &region) {
	float best_ratio = 1.1f;
	edge* best_edge = nullptr;

	for (auto &start : region.base_edges) {
		edge* focus = start;
		do {
			float ratio = intersectRatio(A_S, A_E, focus->start_position(), focus->end_position());

			if (ratio < best_ratio && ratio > 0.f) {
				best_ratio = ratio;
				best_edge = focus;
			}

			focus = focus->next;
		} while (focus != start);
	}

	RegionIntersectData result;
	result.ratio = best_ratio;
	result.target_edge = best_edge;

	return result;

}

//returns a pointer to an exterior edge
edge* vertexBufferToRegion(const TArray<FVector2D> &buffer, face *interior, face *exterior) {
	const int size = buffer.Num();

	//base_face, base_vertex, inverse, next, previous

	TArray<vertex*> points;

	for (auto &v : buffer) {
		vertex *point = new vertex();
		edge *inner = new edge();
		edge *outer = new edge();

		point->position = v;

		inner->inverse = outer;
		outer->inverse = inner;

		inner->base_face = interior;
		outer->base_face = exterior;

		outer->base_vertex = point;
		point->base_edge = outer;

		points.Push(point);
	}

	for (int index = 0; index < size; index++) {
		int next = (index + 1) % size;

		points[index]->base_edge->next = points[next]->base_edge;
		points[next]->base_edge->previous = points[index]->base_edge;

		points[index]->base_edge->inverse->previous = points[next]->base_edge->inverse;
		points[next]->base_edge->inverse->next = points[index]->base_edge->inverse;

		points[index]->base_edge->inverse->base_vertex = points[next];
	}


	//at this point, the structure is complete

	return points[0]->base_edge->inverse;
}

struct universe {
	face* null_region;
	face* exterior_region;
	TArray<face*> included_regions;

	universe(TArray<FVector2D> &buffer) {
		exterior_region = new face();
		null_region = new face();

		null_region->base_edges.Push(vertexBufferToRegion(buffer, null_region, exterior_region));
		exterior_region->base_edges.Push(null_region->base_edges[0]->inverse);

		included_regions.Push(exterior_region);
	}

	//takes in a buffer, intersects it with the null region, and rigs onto (but not into) universe
	suggested_region* rig_suggestion(TArray<FVector2D> &buffer) {
		//start by finding a point inside the null region, if one does not exist, this region is empty :(


		//find all exterior vectors and make note of them
		const int size = buffer.Num();
		TArray<int> exteriorPoints;

		for (int ii = 0; ii < size; ii++) {
			if (isPointStrictlyInterior(buffer[ii], *null_region)) {
				exteriorPoints.Push(ii);
			}
		}

		//is the region encapsulated?
		if (exteriorPoints.Num() < 1) {
			return nullptr;
		}


		suggested_region* suggestion = new suggested_region();
		suggestion->region = new face();

		while (exteriorPoints.Num() > 0) {
			suggestion->components.Push(suggested_component());
			suggested_component &component = suggestion->components.Last();

			int start_vector = exteriorPoints.Pop();
			int next_vector = start_vector;

			vertex* initial_vertex = new vertex();
			initial_vertex->position = buffer[start_vector];

			component.initial_edge = new edge();
			initial_vertex->base_edge = component.initial_edge;
			component.initial_edge->base_vertex = initial_vertex;
			component.initial_edge->base_face = suggestion->region;

			edge* current_edge = component.initial_edge;


			enum trace_state { tracking_source, tracking_ambient };

			trace_state current_state = tracking_source;

			while (true) {
				if (current_state == tracking_source) {
					next_vector = (next_vector + 1) % size;
					float best_ratio = 1.1f;
					edge* best_edge = nullptr;
					for (auto& region : included_regions) {
						RegionIntersectData result = calculateFirstIntersect(current_edge->start_position(), buffer[next_vector], *region);
						if (result.ratio < best_ratio) {
							best_ratio = result.ratio;
							best_edge = result.target_edge;
						}
					}

					if (best_edge == nullptr) {
						//no intersections

						if (next_vector == start_vector) {
							//link to start
							component.edge_types.Push(null_static);

							current_edge->next = component.initial_edge;
							component.initial_edge->previous = current_edge;

							edge* inverse_edge = new edge();
							current_edge->inverse = inverse_edge;
							inverse_edge->inverse = current_edge;

							inverse_edge->base_vertex = component.initial_edge->base_vertex;
							inverse_edge->base_face = null_region;

							//escape! the trace!
							break;
						}
						else {
							//create end vertex, inverse edge, and next edge
							component.edge_types.Push(null_static);

							//pop external vector from list
							exteriorPoints.Remove(next_vector);

							vertex* next_vertex = new vertex();
							edge* next_edge = new edge();
							edge* inverse_edge = new edge();

							current_edge->next = next_edge;


							next_vertex->position = buffer[next_vector];
							next_vertex->base_edge = next_edge;

							next_edge->base_face = suggestion->region;
							next_edge->base_vertex = next_vertex;
							next_edge->previous = current_edge;

							current_edge->inverse = inverse_edge;
							inverse_edge->inverse = current_edge;

							inverse_edge->base_vertex = next_vertex;
							inverse_edge->base_face = null_region;


							current_edge = next_edge;
						}
					}
					else {
						//intersection!

						//needs vertex intersect check
						//does this happen at the end of the checked segment, indicating this segment should be included
						//does this happen at the beginning of the checked segment, indicating the previous segment should be included
						FVector2D intersect = (buffer[next_vector] - current_edge->start_position()) * best_ratio + current_edge->start_position();

						vertex* next_vertex;
						edge* next_edge = new edge();
						edge* inverse_edge = new edge();

						if (intersect.Equals(best_edge->end_position())) {
							component.edge_types.Push(null_atvertex);

							next_vertex = best_edge->end_vertex();
						}
						else if (intersect.Equals(best_edge->start_position())) {
							component.edge_types.Push(null_atvertex);
							best_edge = best_edge->previous;

							next_vertex = best_edge->end_vertex();
						}
						else {
							component.edge_types.Push(null_intersect);
							next_vertex = new vertex();
							next_vertex->position = intersect;
							next_vertex->base_edge = next_edge;
						}

						current_edge->next = next_edge;

						next_edge->base_face = suggestion->region;
						next_edge->base_vertex = next_vertex;
						next_edge->previous = current_edge;

						next_edge->inverse = best_edge;

						current_edge->inverse = inverse_edge;
						inverse_edge->inverse = current_edge;

						inverse_edge->base_vertex = next_vertex;
						inverse_edge->base_face = null_region;

						current_edge = next_edge;
						current_state = tracking_ambient;
					}
				}
				else if (current_state == tracking_ambient) {
					//does this segment intersect anything?
					float best_ratio = 1.1f;
					int best_index = -1;

					for (int ii = 0; ii < size; ii++) {
						float ratio = intersectRatio(current_edge->start_position(), current_edge->end_position(), buffer[ii], buffer[(ii + 1) % size]);
						if (ratio < best_ratio && ratio > 0.f) {
							best_ratio = ratio;
							best_index = ii;
						}
					}
					if (best_index >= 0) {
						// intersect!

						//needs vertex intersect check
						//does this happen at the end of the checked segment, indicating this segment should be included
						//does this happen at the beginning of the checked segment, indicating the previous segment should be included
						FVector2D intersect = (current_edge->end_position() - current_edge->start_position()) * best_ratio + current_edge->start_position();

						vertex* next_vertex;
						edge* next_edge = new edge();

						if (intersect.Equals(current_edge->start_position())) {
							component.edge_types.Push(connected_atvertex);

							next_vertex = current_edge->base_vertex;
						}
						else {
							component.edge_types.Push(connected_intersect);
							next_vertex = new vertex();
							next_vertex->position = intersect;
							next_vertex->base_edge = next_edge;
						}

						current_edge->next = next_edge;

						next_edge->base_face = suggestion->region;
						next_edge->base_vertex = next_vertex;
						next_edge->previous = current_edge;

						current_edge = next_edge;
						current_state = tracking_source;

						next_vector = best_index;
					}
					else {
						// no intersect
						component.edge_types.Push(connected_static);

						edge* next_edge = new edge();

						current_edge->next = next_edge;

						next_edge->base_face = suggestion->region;
						next_edge->base_vertex = current_edge->inverse->base_vertex;
						next_edge->previous = current_edge;
						next_edge->inverse = current_edge->inverse->previous;

						edge* preview = current_edge->inverse->previous;
						if (preview->inverse->base_face == null_region) {
							next_edge->inverse = preview;
						}
						else {
							next_edge->inverse = preview->inverse->previous;
						}

						current_edge = next_edge;
					}
				}
			};




		}

		return suggestion;

		//problems: 
		//	vertex - vertex intersect duplication


		//trace clockwise around the border, when an intersect is found, subdivide at it, and mark the end
		//intersect tests are done against a subset of the included regions
		//find the EARLIEST intersect each time
		//types of intersect:
		//	normal, a crosses b
		//		subdivide at, and begin tracing b counter clockewise, 
		//		until face swap between included regions ( if next->inverse->base_face != null_region )
		//		or another intersect occurs
		//	parrallel, an end point of b lies on a
		//		equivilent to a normal
		//	parrallel, an end point of a lies on b
		//		use a region short tag, if 
		//	switch, an end point of a lies on an end point of b
	}

	//rigs an onto region into
	void enact_suggestion(suggested_region &target) {
		//see notebook



		//for each component, wrap around
		for (auto &component : target.components) {
			edge* last_intersect = nullptr;
			bool connected = false;

			edge* current_edge = component.initial_edge;

			for (int index = 0; index < component.edge_types.Num(); index++) {
				edge_correction current_type = component.edge_types[index];

				if (current_type == null_static) {
					//forward connect inverses

					current_edge->next->inverse->next = current_edge->inverse;
					current_edge->inverse->previous = current_edge->next->inverse;
				}
				else if (current_type == null_intersect) {
					last_intersect = current_edge;
					connected = true;

					//subdivide next->inverse
					//connect next->inverse->next->inverse to inverse or connect to appropriate edge

					current_edge->next->inverse->subdivide(current_edge->next->start_position());

					current_edge->next->inverse->inverse->previous->next = current_edge->inverse;
					current_edge->inverse->previous = current_edge->next->inverse->inverse->previous;
				}
				else if (current_type == null_atvertex) {
					last_intersect = current_edge;
					connected = true;

					//connect next->inverse->next->inverse to inverse or connect to appropriate edge

					current_edge->next->inverse->inverse->previous->next = current_edge->inverse;
					current_edge->inverse->previous = current_edge->next->inverse->inverse->previous;
				}
				else if (current_type == connected_static) {
					if (null_region->isBaseEdge(current_edge->inverse->inverse)) {
						null_region->base_edges.Remove(current_edge->inverse->inverse);
						null_region->base_edges.Push(last_intersect);
						last_intersect = nullptr;
					}

					//replace inverse inverse reference with self and DELETE

					delete current_edge->inverse->inverse;
					current_edge->inverse->inverse = current_edge;
				}
				else if (current_type == connected_intersect) {
					if (null_region->isBaseEdge(current_edge->inverse->inverse)) {
						null_region->base_edges.Remove(current_edge->inverse->inverse);
						null_region->base_edges.Push(last_intersect);
						last_intersect = nullptr;
					}
					else if (last_intersect != nullptr) {
						null_region->base_edges.Push(last_intersect);
						last_intersect = nullptr;
					}

					//subdivide inverse
					//connect inverse->inverse to next->inverse
					//set inverse to inverse->next
					//replace inverse inverse reference with self and DELETE

					current_edge->subdivide(current_edge->next->start_position());

					//current_edge now points to the one preceding the desired, config its inverse before correcting
					current_edge->inverse->inverse->previous = current_edge->next->inverse;
					current_edge->next->inverse->next = current_edge->inverse->inverse;

					current_edge->inverse = current_edge->inverse->next;

					delete current_edge->inverse->inverse;
					current_edge->inverse->inverse = current_edge;
				}
				else { //current_type == connected_atvertex
					if (null_region->isBaseEdge(current_edge->inverse->inverse)) {
						null_region->base_edges.Remove(current_edge->inverse->inverse);
						null_region->base_edges.Push(last_intersect);
						last_intersect = nullptr;
					}
					else if (last_intersect != nullptr) {
						null_region->base_edges.Push(last_intersect);
						last_intersect = nullptr;
					}

					//connect inverse->inverse->next to next->inverse
					//replace inverse inverse reference with self and DELETE

					current_edge->inverse->inverse->next->previous = current_edge->next->inverse;
					current_edge->next->inverse->next = current_edge->inverse->inverse;

					delete current_edge->inverse->inverse;
					current_edge->inverse->inverse = current_edge;
				}

				current_edge = current_edge->next;
			}

			if (!connected) {
				null_region->base_edges.Push(component.initial_edge);
			}

		}

		target.region->base_edges.Push(target.components[0].initial_edge);
	};

};


// Sets default values
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

float Calculate_Area(const TArray<FVector2D> &VectorBuffer) {
	const int32 sizem = VectorBuffer.Num() - 1;
	float Area = 0;

	for (int32 ii = 0; ii < sizem; ii++) {
		float Y = VectorBuffer[ii + 1].Y - VectorBuffer[ii].Y;
		float X = (VectorBuffer[ii + 1].X + VectorBuffer[ii].X) / 2;
		Area += X * Y;
	}

	float Y = VectorBuffer[0].Y - VectorBuffer[sizem].Y;
	float X = (VectorBuffer[0].X + VectorBuffer[sizem].X) / 2;
	Area += X * Y;

	UE_LOG(LogTemp, Warning, TEXT("Area is %f"), Area);

	return Area;
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
					if (GEngine)
						GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, TEXT("Failed to Triangulate destructible mesh!"));
					break;
				}
			}
		}
	}
	return Triangles;
}

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

void Aroom_description_builder::Create_Floor_Ceiling(const TArray<FVector2D> &Border, float Bottom, float Top) {
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

void Aroom_description_builder::Create_Wall_Sections(const TArray<FVector2D> &Wall_Sections, float Bottom, float Top) {
	const int32 size = Wall_Sections.Num() / 2;
	for (int32 ii = 0; ii < size; ii++) {
		int32 a = ii * 2;

		CreateDoor(Wall_Sections[a], Wall_Sections[a + 1], Bottom, Top);
		/*UProceduralMeshComponent* Wall_Mesh = NewObject<UProceduralMeshComponent>();
		Wall_Mesh->AttachToComponent(root, FAttachmentTransformRules::KeepRelativeTransform);
		Wall_Mesh->ContainsPhysicsTriMeshData(true);
		Wall_Mesh->bUseAsyncCooking = true;

		TArray<FVector> Vertices;
		TArray<int32> Triangles;
		TArray<FVector> Normals;
		TArray<FVector2D> UV0;
		TArray<FProcMeshTangent> Tangents;
		TArray<FLinearColor> VertexColors;

		Vertices.Push(FVector(Wall_Sections[a], Bottom));
		Vertices.Push(FVector(Wall_Sections[a], Top));
		Vertices.Push(FVector(Wall_Sections[a+1], Bottom));
		Vertices.Push(FVector(Wall_Sections[a+1], Top));

		Triangles.Push(3);
		Triangles.Push(1);
		Triangles.Push(2);
		Triangles.Push(2);
		Triangles.Push(1);
		Triangles.Push(0);

		//calculate right normal
		FVector2D normal = Wall_Sections[a + 1] - Wall_Sections[a];
		normal.Set(normal.Y, -normal.X);

		Normals.Push(FVector(normal,0));
		Normals.Push(FVector(normal, 0));
		Normals.Push(FVector(normal, 0));
		Normals.Push(FVector(normal, 0));

		UV0.Push(FVector2D(0, Bottom));
		UV0.Push(FVector2D(0, Top));
		UV0.Push(FVector2D(1, Bottom));
		UV0.Push(FVector2D(1, Top));
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

		Wall_Mesh->RegisterComponentWithWorld(GetWorld());*/
	}
}

ClipperLib::Paths Subtract_Region(const ClipperLib::Paths &origin, const ClipperLib::Path &next) {
	ClipperLib::Clipper Clipper;

	Clipper.AddPaths(origin, ClipperLib::PolyType::ptSubject, true);
	Clipper.AddPath(next, ClipperLib::PolyType::ptClip, true);

	ClipperLib::Paths result;

	Clipper.Execute(ClipperLib::ClipType::ctDifference, result, ClipperLib::PolyFillType::pftNonZero, ClipperLib::PolyFillType::pftNonZero);

	return result;
}

ClipperLib::Path Vector_to_Path(const TArray<FVector2D> &source) {
	ClipperLib::Path result;

	for (auto& vector : source) {
		result.push_back(ClipperLib::IntPoint(vector.X, vector.Y));
	}

	return result;
};

TArray<FVector2D> Path_to_Vector(const ClipperLib::Path &source) {
	TArray<FVector2D> result;

	for (auto& point : source) {
		result.Push(FVector2D(point.X, point.Y));
	}

	return result;
};

bool Point_in_Path(const ClipperLib::Path &region, const FVector2D &point) {
	return ClipperLib::PointInPolygon(ClipperLib::IntPoint(point.X, point.Y), region)>0;
}

ClipperLib::Path Return_Allowed_Linked_Region(const TArray<FVector2D> &Suggested, const ClipperLib::Paths &Available, const FVector2D &door) {
	ClipperLib::Paths origin;
	origin.push_back(Vector_to_Path(Suggested));

	ClipperLib::Clipper Clipper;

	Clipper.AddPaths(origin, ClipperLib::PolyType::ptSubject, true);
	Clipper.AddPaths(Available, ClipperLib::PolyType::ptClip, true);

	ClipperLib::Paths result;

	Clipper.Execute(ClipperLib::ClipType::ctIntersection, result, ClipperLib::PolyFillType::pftNonZero, ClipperLib::PolyFillType::pftNonZero);

	//find which has the door as a member
	for (int32 index = 0; index < result.size(); index++) {
		if (Point_in_Path(result[index], door)) {
			return result[index];
		}
	}

	return ClipperLib::Path();

};

ClipperLib::Path Return_Allowed_Largest_Region(const TArray<FVector2D> &Suggested, const ClipperLib::Paths &Available) {
	ClipperLib::Paths origin;
	origin.push_back(Vector_to_Path(Suggested));

	ClipperLib::Clipper Clipper;

	Clipper.AddPaths(origin, ClipperLib::PolyType::ptSubject, true);
	Clipper.AddPaths(Available, ClipperLib::PolyType::ptClip, true);

	ClipperLib::Paths result;

	Clipper.Execute(ClipperLib::ClipType::ctIntersection, result, ClipperLib::PolyFillType::pftNonZero, ClipperLib::PolyFillType::pftNonZero);

	if (result.size() > 0) {
		//find which has the door as a member
		int32 max_index = 0;
		int32 max_value = ClipperLib::Area(result[0]);

		for (int32 index = 1; index < result.size(); index++) {
			int32 value = ClipperLib::Area(result[index]);
			if (value > max_value) {
				max_index = index;
				max_value = value;
			}
		}

		return result[max_index];
	}

	return ClipperLib::Path();

};


struct room_model {
	bool empty;
	FBox2D Bounding_Box;
	TArray<FVector2D> Boundary;
	int32 elevation;

	room_model(int x, int y) {
		Boundary.Push(FVector2D(0, 0));
		Boundary.Push(FVector2D(0, y));
		Boundary.Push(FVector2D(x, y));
		Boundary.Push(FVector2D(x, 0));

		Bounding_Box.Min = FVector2D(0, 0);
		Bounding_Box.Max = FVector2D(x, y);
		empty = false;

		set_center(FVector2D(0, 0));

		elevation = 0;
	}

	struct wall_segment {
		wall_segment* Inverse;
		wall_segment* Next;
		wall_segment* Previous;
		room_model* Room;

		FVector2D a;
		FVector2D b;
		int32 elevation;

		wall_segment(room_model* owner) {
			elevation = 0;
			Room = owner;
		}
	};

	TArray<wall_segment> walls;

	void set_center(FVector2D suggested) {
		if (empty) {
			return;
		}

		for (int32 index = 0; index < Boundary.Num(); index++) {
			Boundary[index] += suggested - Bounding_Box.GetCenter();
		}

		Bounding_Box.ShiftBy(suggested - Bounding_Box.GetCenter());
	}

	void recalculate_bounds() {
		if (Boundary.Num() == 0) {
			empty = true;
			Bounding_Box.Min = FVector2D(0, 0);
			Bounding_Box.Max = FVector2D(0, 0);
			return;
		}
		empty = false;

		Bounding_Box.Min = Boundary[0];
		Bounding_Box.Max = Boundary[0];

		for (auto &vector : Boundary) {
			if (vector.X > Bounding_Box.Max.X) {
				Bounding_Box.Max.X = vector.X;
			}
			if (vector.Y > Bounding_Box.Max.Y) {
				Bounding_Box.Max.Y = vector.Y;
			}
			if (vector.X < Bounding_Box.Min.X) {
				Bounding_Box.Min.X = vector.X;
			}
			if (vector.Y < Bounding_Box.Min.Y) {
				Bounding_Box.Min.Y = vector.Y;
			}
		}
	}

	float filling_ratio() const {
		if (empty) {
			return 0;
		}

		return (float)ClipperLib::Area(Vector_to_Path(Boundary)) / Bounding_Box.GetArea();
	}

	void Intersect_Region(const ClipperLib::Paths &available) {
		if (empty) {
			return;
		}

		Boundary = Path_to_Vector(Return_Allowed_Largest_Region(Boundary, available));
		recalculate_bounds();
	}

	void Clean_Narrows(int32 width) {
		if (empty) {
			return;
		}

		ClipperLib::ClipperOffset clipper_inset;
		ClipperLib::ClipperOffset clipper_outset;

		ClipperLib::Paths inset_path;
		ClipperLib::Paths result;

		clipper_inset.AddPath(Vector_to_Path(Boundary), ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
		clipper_inset.Execute(inset_path, -width / 2);

		if (inset_path.size() > 0) {

			int32 max_index = 0;
			int32 max_value = ClipperLib::Area(inset_path[0]);

			for (int32 index = 1; index < inset_path.size(); index++) {
				int32 value = ClipperLib::Area(inset_path[index]);
				if (value > max_value) {
					max_index = index;
					max_value = value;
				}
			}

			clipper_outset.AddPath(inset_path[max_index], ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
			clipper_outset.Execute(result, width / 2);

			Boundary = Path_to_Vector(result[0]);
		}
		else {
			Boundary.Empty();
		}


		recalculate_bounds();
	}
};

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

room_model* Generate_Room(int factor, int scale) {
	float x, y;
	x = FMath::RandRange(factor, 3 * factor);
	y = FMath::RandRange(factor, 3 * factor);



	return new room_model(x*scale, y*scale);
};

TArray<FVector2D> Generate_Boundary(int factor, int scale, const FVector2D &center) {
	float x, y;

	x = FMath::RandRange(factor, 3 * factor);
	y = FMath::RandRange(factor, 3 * factor);

	TArray<FVector2D> boundary;

	boundary.Push(FVector2D(-x * scale, -y * scale) + center);
	boundary.Push(FVector2D(-x * scale, y * scale) + center);
	boundary.Push(FVector2D(x * scale, y * scale) + center);
	boundary.Push(FVector2D(x * scale, -y * scale) + center);

	return boundary;
}

TArray<FVector2D> gen_bounds(float x, float y, const FVector2D &center, int bevel_level) {
	TArray<FVector2D> boundary;

	int bevel_bl = FMath::FRandRange(0, bevel_level);
	int bevel_tl = FMath::FRandRange(0, bevel_level);
	int bevel_tr = FMath::FRandRange(0, bevel_level);
	int bevel_br = FMath::FRandRange(0, bevel_level);

	if (bevel_bl > 0) {
		boundary.Push(FVector2D(bevel_bl * 100 - x, -y) + center);
		boundary.Push(FVector2D(-x, bevel_bl * 100 - y) + center);
	}
	else {
		boundary.Push(FVector2D(-x, -y) + center);
	}

	if (bevel_tl > 0) {
		boundary.Push(FVector2D(-x, y - bevel_tl * 100) + center);
		boundary.Push(FVector2D(bevel_tl * 100 - x, y) + center);
	}
	else {
		boundary.Push(FVector2D(-x, y) + center);
	}

	if (bevel_tr > 0) {
		boundary.Push(FVector2D(x - bevel_tr * 100, y) + center);
		boundary.Push(FVector2D(x, y - bevel_tr * 100) + center);
	}
	else {
		boundary.Push(FVector2D(x, y) + center);
	}

	if (bevel_br > 0) {
		boundary.Push(FVector2D(x, bevel_br * 100 - y) + center);
		boundary.Push(FVector2D(x - bevel_br * 100, -y) + center);
	}
	else {
		boundary.Push(FVector2D(x, -y) + center);
	}

	return boundary;
}

TArray<FVector2D> Clean_Narrows(const TArray<FVector2D> &source, int32 width) {
	if (source.Num() < 1) {
		return TArray<FVector2D>();
	}

	ClipperLib::ClipperOffset clipper_inset;
	ClipperLib::ClipperOffset clipper_outset;

	ClipperLib::Paths inset_path;
	ClipperLib::Paths result;

	clipper_inset.AddPath(Vector_to_Path(source), ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
	clipper_inset.Execute(inset_path, -width / 2);

	if (inset_path.size() > 0) {

		int32 max_index = 0;
		int32 max_value = ClipperLib::Area(inset_path[0]);

		for (int32 index = 1; index < inset_path.size(); index++) {
			int32 value = ClipperLib::Area(inset_path[index]);
			if (value > max_value) {
				max_index = index;
				max_value = value;
			}
		}

		clipper_outset.AddPath(inset_path[max_index], ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
		clipper_outset.Execute(result, width / 2);

		return Path_to_Vector(result[0]);
	}
	else {
		return TArray<FVector2D>();
	}
}

void DrawBorder(const TArray<FVector2D> &border, float height, const UWorld *ref) {
	for (int index = 0; index < border.Num(); index++) {
		int next = (index + 1) % border.Num();

		DrawDebugLine(
			ref,
			FVector(border[index], height),
			FVector(border[next], height),
			FColor(0, 255, 0),
			true,
			-1,
			0,
			5
		);
	}
}

TArray<FVector2D> Inset(const TArray<FVector2D> &source, int32 width) {
	if (source.Num() < 1) {
		return TArray<FVector2D>();
	}

	ClipperLib::ClipperOffset clipper_inset;
	ClipperLib::ClipperOffset clipper_outset;

	ClipperLib::Paths inset_path;
	ClipperLib::Paths result;

	clipper_inset.AddPath(Vector_to_Path(source), ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
	clipper_inset.Execute(inset_path, -width);

	if (inset_path.size() > 0) {

		return Path_to_Vector(inset_path[0]);

	}
	else {
		return TArray<FVector2D>();
	}
}


void Aroom_description_builder::Main_Generation_Loop() {
	{
		auto test_bounds = gen_bounds(6, 6, FVector2D(0, 0), 0);
		universe test_system(test_bounds);

		auto bounds = gen_bounds(4, 4, FVector2D(-6, 0), 0);
		auto culled = Path_to_Vector(Return_Allowed_Largest_Region(bounds, test_system.null_region->Export_Interior()));
		auto suggested = test_system.rig_suggestion(culled);

		test_system.enact_suggestion(*suggested);

		DrawBorder(Path_to_Vector(suggested->region->Export_Interior()[0]), 0, GetWorld());
	}




	float system_x = FMath::RandRange(2, 6) * 5;
	float system_y = FMath::RandRange(2, 6) * 5;
	FVector2D center(0, 0);
	auto system_bounds = gen_bounds(system_x * 100, system_y * 100, center, 9);
	universe cut_system(system_bounds);

	int h = 0;
	DrawBorder(system_bounds, (h++) * 100, GetWorld());


	for (int index = 0; index < room_count; index++)
	{
		float local_x = FMath::RandRange(2, 6);
		float local_y = FMath::RandRange(2, 6);

		FVector2D jump;
		jump.X = FMath::RandRange(-(int)system_x, (int)system_x);
		jump.Y = FMath::RandRange(-(int)system_y, (int)system_y);
		int bevel_level = FMath::RandRange(0, FMath::Min<int>(local_x, local_y));

		auto bounds = gen_bounds(local_x * 100, local_y * 100, jump * 100, bevel_level);

		auto culled = Path_to_Vector(Return_Allowed_Largest_Region(bounds, cut_system.null_region->Export_Interior()));
		auto short_bounds = Clean_Narrows(culled, 100);

		auto suggested = cut_system.rig_suggestion(short_bounds);
		if (suggested != nullptr) {
			cut_system.enact_suggestion(*suggested);

			TArray<FVector2D> Border_Raw = Path_to_Vector(suggested->region->Export_Interior()[0]);

			auto Border = Inset(Border_Raw, 10);

			auto borders = cut_system.null_region->Export_Interior();
			for (auto path : borders) {
				DrawBorder(Path_to_Vector(path), (h) * 100, GetWorld());
			}
			h++;

			TArray<FVector2D> Walls;
			for (int index_border = 0; index_border < Border.Num(); index_border++) {
				int next = (index_border + 1) % Border.Num();
				Walls.Push(Border[index_border]);
				Walls.Push(Border[next]);
			}

			Create_Floor_Ceiling(Border, 0, 200);
			Create_Wall_Sections(Walls, 0, 200);
		}

		//draw system boundary

		//FOR EVERY REGION, DRAW ITS 


	}
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
