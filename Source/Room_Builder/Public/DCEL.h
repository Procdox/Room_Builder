#pragma once

#include "CoreMinimal.h"
#include "DCEL.generated.h"

//MARK FLOAT USED

//static bool compare_float(const float &a, const float &b) {
//	return (FMath::Abs(a - b) < (FLT_EPSILON * FMath::Max3(FMath::Abs(a), FMath::Abs(b), 1.f)));
//}

#define grid_coef 2.f

enum point_near_segment_state {left_of_segment, right_of_segment, before_segment, after_segment, on_start, on_end, on_segment};

struct _P {
	int X;
	int Y;
	_P() {
		X = 0.f;
		Y = 0.f;
	}
	_P(int m) {
		X = 0.f;
		Y = 0.f;
	}
	_P(int x, int y) {
		X = x;
		Y = y;
	}
	_P(int x, int y, int m) {
		X = x;
		Y = y;
	}

	_P operator+(const _P &add) const {
		return _P(X + add.X, Y + add.Y);
	}
	_P operator-(const _P &sub) const {
		return _P(X - sub.X, Y - sub.Y);
	}
	_P operator*(int mul) const {
		return _P(X * mul, Y * mul);
	}
	_P operator*(const _P &mul) const {
		return _P(X * mul.X, Y * mul.Y);
	}
	_P operator/(int div) const {
		check(X % div == 0);
		check(Y % div == 0);
		return _P(X / div, Y / div);
	}
	_P operator/(const _P &div) const {
		check(X % div.X == 0);
		check(Y % div.Y == 0);
		return _P(X / div.X, Y / div.Y);
	}

	_P& operator+=(const _P &add) {
		X += add.X;
		Y += add.Y;
	}
	_P& operator-=(const _P &sub) {
		X -= sub.X;
		Y -= sub.Y;
	}
	_P& operator*=(float mul) {
		X *= mul;
		Y *= mul;
	}
	_P& operator*=(const _P &mul) {
		X *= mul.X;
		Y *= mul.Y;
	}
	_P& operator/=(int div) {
		check(X % div == 0);
		check(Y % div == 0);
		X /= div;
		Y /= div;
	}
	_P& operator/=(const _P &div) {
		check(X % div.X == 0);
		check(Y % div.Y == 0);
		X /= div.X;
		Y /= div.Y;
	}

	bool operator==(const _P &test) const {
		return test.X == X && test.Y == Y;
	}
	bool operator!=(const _P &test) const {
		return test.X != X || test.Y != Y;
	}
	int SizeSquared() const {
		return X * X + Y * Y;
	}
	float Size() const {
		return FMath::Sqrt(SizeSquared()); //MARK FLOAT USED
	}
	//void Normalize() {
	//	const int sizeSq = SizeSquared();
	//	const int sizeSq = SizeSquared();
	//	if (sizeSq == 0) {
	//		return;
	//	}
	//	X /= size;
	//	Y /= size;
	//}

	int Dot(const _P &b) const {
		return X*b.X + Y*b.Y;
	}

	static int Area(const TArray<_P> &boundary) {
		float total = 0;

		const int Num = boundary.Num();

		for (int ii = 0; ii < Num; ii++) {
			const _P &current = boundary[ii];
			const _P &next = boundary[(ii + 1) % Num];
			const float width = next.X - current.X;
			const float avg_height = (next.Y + current.Y) / 2;

			total += width * avg_height;
		}

		return total;
	}

	point_near_segment_state getState(const _P &start, const _P &end) const {
		if (*this == start) {
			return on_start;
		}
		if (*this == end) {
			return on_start;
		}

		const int TWAT = (X * start.Y - Y * start.X) + (start.X * end.Y - start.Y * end.X) + (end.X * Y - end.Y * X);
		if (TWAT > 0) {
			return left_of_segment;
		}
		if (TWAT < 0) {
			return right_of_segment;
		}

		const int target_size = (start - end).SizeSquared();
		if ((*this - end).SizeSquared() > target_size) {
			return before_segment;
		}
		if ((*this - start).SizeSquared() > target_size) {
			return after_segment;
		}

		return on_segment;
	}

	static bool areParrallel(const _P &A_S, const _P &A_E, const _P &B_S, const _P &B_E) {
		const _P A = A_E - A_S;
		const _P A_R = A + B_S;

		//if A lies on B or after it they are parrallel
		auto state = A_R.getState(B_S, B_E);
		return (state == on_segment || state == on_end || state == after_segment);

	}

	static bool isOnSegment(const _P &test, const _P &a, const _P &b) {
		auto state = test.getState(a, b);
		return (state == on_segment || state == on_start);
	}

	static bool inRegionCW(const _P &test, const _P &before, const _P &corner, const _P &after) {
		return (test.getState(before, corner) == right_of_segment && test.getState(corner, after) == right_of_segment);
	}

	//static bool isOnSegment(const _P &test, const _P &a, const _P &b) {
	//	if (test == a) {
	//		return true;
	//	}
	//	_P A = test - a;
	//	_P B = b - a;
	//	if (A.SizeSquared() < B.SizeSquared()) {
	//		A.Normalize();
	//		B.Normalize();
	//		return compare_float(1.f, _P::Dot(A, B)); //MARK FLOAT USED
	//		//return 1 - _P::Dot(A, B) < FLT_EPSILON; //FLOAT_COMPARISON
	//	}
	//	return false;
	//}

	//static bool inRegionCW(const _P &left, const _P &right, const _P &test) {
	//	_P bisector = left + right;
	//	bisector.Normalize();
	//	//test and fix for concavity
	//	const _P test_concave(left.Y, -left.X);
	//	const float orientation = _P::Dot(right, test_concave); //MARK FLOAT USED
	//	if (orientation < 0) {
	//		bisector = bisector * -1;
	//	}
	//	else if (compare_float(orientation, 0)) { //MARK FLOAT USED
	//		//else if (FMath::Abs(orientation) < FLT_EPSILON) { //FLOAT_COMPARISON
	//		bisector = test_concave;
	//	}
	//	return _P::Dot(bisector, left) <= _P::Dot(bisector, test); //MARK FLOAT USED
	//}

};


#define interaction_state int
#define unknown_region -1
#define external_region 0
#define external_boundary 1
#define internal_region 2



USTRUCT()
struct ROOM_BUILDER_API F_DCEL {
	GENERATED_BODY()

	class Point;
	class Edge;
	class Face;
	class Space;

	class Point {
		friend F_DCEL;
		F_DCEL* dad;

		Edge* root_edge;
		_P position;
		Point(F_DCEL* creator) {
			dad = creator;
			root_edge = nullptr;
		}
		Point(F_DCEL* creator, const _P &p) {
			dad = creator;
			position = p;
		}

	public:
		Edge * getEdge() const {
			return root_edge;
		}

		_P getPosition() const {
			return position;
		}

		void addEdge(Edge* target) {
			if (root_edge == nullptr) {
				root_edge = target;
			}
		}

		void removeEdge(const Edge* target) {
			if (target == root_edge) {
				if (target->getCWOfRoot() == nullptr || target->getCWOfRoot() == target) {
					root_edge = nullptr;
				}
				else {
					root_edge = target->getCWOfRoot();
				}
			}

		}

		void setPosition(const _P &p) {
			position = p;
		}
	};

	class Edge {
		friend F_DCEL;
		F_DCEL* dad;

		Point* root_point;
		Edge* inverse_edge;
		Edge* next_edge;
		Edge* last_edge;
		Face* face;

		Edge(F_DCEL* creator) {
			dad = creator;
			root_point = nullptr;
			inverse_edge = nullptr;
			next_edge = nullptr;
			last_edge = nullptr;
		}

	public:
		int public_marker;

		void listFaceLoop(TArray<Edge*> &target) {
			Edge* focus = this;
			do {
				target.Push(focus);

				focus = focus->next_edge;
			} while (focus != this);
		}

		//re-assigns a loop of edges (representing the boundary of a face) to another face object
		bool reFaceLoop(Face* target) {
			//todo: safety escape with failure
			Edge* focus = this;
			do {
				focus->face = target;

				focus = focus->next_edge;
			} while (focus != this);

			return true;
		}

		Edge* getContainingSegment(const _P &test_point) {
			//is it on a hole border
			Edge* focus = this;
			do {
				if (focus->getStartPoint() && focus->getEndPoint()) {
					if (test_point == focus->getStartPoint()->getPosition() ||
						_P::isOnSegment(test_point, focus->getStartPoint()->getPosition(), focus->getEndPoint()->getPosition())) {

						return focus;
					}
				}
				focus = focus->next_edge;
			} while (focus != this);

			return nullptr;
		}

		double loopArea() const {
			const Edge* focus = this;
			double total = 0;

			do {
				auto A = focus->root_point->getPosition();
				auto B = focus->next_edge->root_point->getPosition();
				double width = B.X - A.X;
				double avg_height = (A.Y + B.Y) / 2;

				total += width * avg_height;

				focus = focus->next_edge;
			} while (focus != this);

			return total;
		}

		void setRoot(Point* new_home) {
			removeFromRoot();
			root_point = new_home;
		}

		void removeFromRoot() {
			if (root_point != nullptr) {
				root_point->removeEdge(this);
			}
			root_point = nullptr;

			Edge* next = inverse_edge->next_edge;
			Edge* last = last_edge;
			if (next != nullptr) {
				next->last_edge = last;
			}
			if (last != nullptr) {
				last->next_edge = next;
			}

			last_edge = inverse_edge;
			inverse_edge->next_edge = this;
		}

		// traversal methods

		Edge* getNextEdge() const {
			return next_edge;
		}
		Edge* getLastEdge() const {
			return last_edge;
		}
		Edge* getInverseEdge() const {
			return inverse_edge;
		}

		Edge* getCWOfRoot() const {
			if (last_edge == nullptr) {
				return nullptr;
			}
			return last_edge->inverse_edge;
		}

		Edge* getCCWofRoot() const {
			if (inverse_edge == nullptr) {
				return nullptr;
			}
			return inverse_edge->next_edge;
		}

		Point* getStartPoint() const {
			return root_point;
		}

		Point* getEndPoint() const {
			if (inverse_edge == nullptr) {
				return nullptr;
			}
			return inverse_edge->root_point;
		}

		Face* getFace() const {
			return face;
		}

		void setInverse(Edge* target) {
			if (inverse_edge == nullptr && target->inverse_edge == nullptr) {
				target->inverse_edge = this;
				inverse_edge = target;
				next_edge = inverse_edge;
				last_edge = inverse_edge;
				inverse_edge->next_edge = this;
				inverse_edge->last_edge = this;
			}
		}

		// modification methods
		void insertAfter(Edge* neighbor) {
			neighbor->setRoot(next_edge->root_point);

			neighbor->inverse_edge->next_edge = next_edge;
			neighbor->last_edge = this;

			next_edge->last_edge = neighbor->inverse_edge;
			next_edge = neighbor;
		}

		//removes this edge and its inverse from the graph
		void removeFromGraph() {
			//before
			last_edge->next_edge = inverse_edge->next_edge;
			inverse_edge->next_edge->last_edge = last_edge;

			//after
			next_edge->last_edge = inverse_edge->last_edge;
			inverse_edge->last_edge->next_edge = next_edge;
		}

		void insertBefore(Edge* neighbor) {
			last_edge->insertAfter(neighbor->inverse_edge);
		}

		void subdivide(const _P &location);

		TArray<_P> listPoints() const {
			TArray<_P> result;
			const Edge* focus = this;

			do {
				result.Push(focus->getStartPoint()->getPosition());

				focus = focus->getNextEdge();
			} while (focus != this);

			return result;
		}
	};

	//represents a connected face, may not be genus 0
	class Face {
		friend F_DCEL;
		F_DCEL* dad;

		Edge* root_edge;
		TArray<Edge*> hole_edges;

		Face(F_DCEL* creator) {
			dad = creator;
			root_edge = nullptr;
		}

		struct interact_point {
			interaction_state state;
			int region;
			_P point;
			interact_point* next;
			int mid_state;
			interact_point(const _P p) {
				region = -1;
				state = unknown_region;
				point = p;
				next = nullptr;
				mid_state = -1;
			}
			interact_point(const _P p, interact_point* n) {
				region = -1;
				state = unknown_region;
				point = p;
				next = n;
				mid_state = -1;
			}
		};
		struct strand {
			Edge* interior_edge;
			Edge* exterior_edge;
			bool unique_interior;
			bool unique_exterior;
			strand() {
				unique_interior = true;
				unique_exterior = true;
			}
		};
	public:
		Edge * getRootEdge() const {
			return root_edge;
		}
		int getHoleCount() const {
			return hole_edges.Num();
		}
		Edge* getHole(int index) const {
			return hole_edges[index];
		}
		F_DCEL* getDad() const {
			return dad;
		}
		TArray<Face*> getNeighbors() const {
			TArray<Face*> result;
			const Edge* focus = root_edge;

			do {
				Face* canidate = focus->inverse_edge->face;
				if (!result.Contains(canidate)) {
					result.Push(canidate);
				}
				focus = focus->next_edge;
			} while (focus != root_edge);

			return result;
		}

		interaction_state getPointState(const _P &test_point) const;

		interaction_state getFirstIntersect(const _P &start, const _P &end, _P &intersect) const;

		Edge* getContainingSegment(const _P &test_point) const;
		//splits this face into two sets of faces, the intersect components of this face and the supposed border, and the difference components of this face minus the supposed border
		//returns false if no non-trivial intersect exists
		bool subAllocateFace(const TArray<_P> &border, TArray<Face*> &interior_regions, TArray<Face*> &exterior_regions);

		bool mergeWithFace(Face* target);

		bool contains(const _P &test_point);

		int borderCount(int safety) const {
			if (root_edge == nullptr) {
				return 0;
			}
			const Edge* focus = root_edge;
			int length = 0;

			do {
				focus = focus->getNextEdge();
				length++;
			} while (focus != root_edge && length <= safety);

			return length;
		}
		int holeBorderCount(int hole, int safety) const {
			const Edge* focus = hole_edges[hole];
			int length = 0;

			do {
				focus = focus->getNextEdge();
				length++;
			} while (focus != hole_edges[hole] && length <= safety);

			return length;
		}

		void cleanBorder() {
			//merge parrallel sections that do not have joints build in

			// BOUNDARY
			auto focus = root_edge;
			do {
				auto next = focus->next_edge;
				//parrallel check

				auto start = focus->getStartPoint()->position;
				auto end = focus->getEndPoint()->position;
				auto test = next->getEndPoint()->position;

				auto state = test.getState(start, end);

				//end.Normalize();
				//test.Normalize();

				bool is_unjointed = (focus->inverse_edge->last_edge == focus->next_edge->inverse_edge);
				bool is_parrallel = (state == after_segment);//((1 - _P::Dot(end, test)) < FLT_EPSILON);

				if (is_unjointed && is_parrallel) {
					if (root_edge == next) {
						root_edge = focus;
					}
					auto neighbor = next->inverse_edge->face;

					if (neighbor->root_edge == next->inverse_edge) {
						neighbor->root_edge = focus->inverse_edge;
					}
					for (int ii = 0; ii < neighbor->hole_edges.Num(); ii++) {
						if (neighbor->hole_edges[ii] == next->inverse_edge) {
							neighbor->hole_edges[ii] = focus->inverse_edge;
						}
					}

					//MERGE
					focus->next_edge = next->next_edge;
					next->next_edge->last_edge = focus;

					focus->inverse_edge->last_edge = next->inverse_edge->last_edge;
					next->inverse_edge->last_edge->next_edge = focus->inverse_edge;

					focus->inverse_edge->root_point = next->inverse_edge->root_point;

					dad->removePoint(next->root_point);
					dad->removeEdge(next->inverse_edge);
					dad->removeEdge(next);
				}
				else {
					focus = focus->next_edge;
				}
			} while (focus != root_edge);

			for (int kk = 0; kk < hole_edges.Num(); kk++) {
				auto focus = hole_edges[kk];
				do {
					auto next = focus->next_edge;
					//parrallel check

					auto start = focus->getStartPoint()->position;
					auto end = focus->getEndPoint()->position;
					auto test = next->getEndPoint()->position;
					//end.Normalize();
					//test.Normalize();

					auto state = test.getState(start, end);

					bool is_unjointed = (focus->inverse_edge->last_edge == focus->next_edge->inverse_edge);
					bool is_parrallel = (state == after_segment);//((1 - _P::Dot(end, test)) < FLT_EPSILON); //MARK FLOAT USED

					if (is_unjointed && is_parrallel) {
						if (hole_edges[kk] == next) {
							hole_edges[kk] = focus;
						}
						auto neighbor = next->inverse_edge->face;

						if (neighbor->root_edge == next->inverse_edge) {
							neighbor->root_edge = focus->inverse_edge;
						}
						for (int ii = 0; ii < neighbor->hole_edges.Num(); ii++) {
							if (neighbor->hole_edges[ii] == next->inverse_edge) {
								neighbor->hole_edges[ii] = focus->inverse_edge;
							}
						}

						//MERGE
						focus->next_edge = next->next_edge;
						next->next_edge->last_edge = focus;

						focus->inverse_edge->last_edge = next->inverse_edge->last_edge;
						next->inverse_edge->last_edge->next_edge = focus->inverse_edge;

						focus->inverse_edge->root_point = next->inverse_edge->root_point;

						dad->removePoint(next->root_point);
						dad->removeEdge(next->inverse_edge);
						dad->removeEdge(next);
					}
					else {
						focus = focus->next_edge;
					}
				} while (focus != hole_edges[kk]);
			}

			dad->sanityCheck();
		}
	};

	TArray<Point*> points;
	TArray<Edge*> edges;
	TArray<Face*> faces;

	void CLEAN() {
		for (auto point : points) {
			delete point;
		}
		for (auto edge : edges) {
			delete edge;
		}
		for (auto face : faces) {
			delete face;
		}
		points.Empty();
		edges.Empty();
		faces.Empty();
	}
	Point* makePoint() {
		Point* temp = new Point(this);
		points.Push(temp);
		return temp;
	}
	Point* makePoint(const _P &p) {
		Point* temp = new Point(this, p);
		points.Push(temp);
		return temp;
	}
	Edge* makeEdge() {
		Edge* temp = new Edge(this);
		edges.Push(temp);
		return temp;
	}
	Face* makeFace() {
		Face* temp = new Face(this);
		faces.Push(temp);
		return temp;
	}

	void removePoint(Point* target) {
		//is it ACTUALLY disconnected? AHHHHHHHHH
		points.Remove(target);

		delete target;
	}

	void removeEdge(Edge* target) {
		//is it ACTUALLY disconnected? AHHHHHHHHH
		edges.Remove(target);

		delete target;
	}

	void removeFace(Face* target) {
		//is it ACTUALLY disconnected? AHHHHHHHHH
		faces.Remove(target);

		delete target;
	}

	//assumes simple polygon, will set negative area polygons to inverse orientation, creating them as if they were positive
	//this is the same as creating a universe, then suballocating the region from it
	Face* createFace(const TArray<_P> &boundary, Face* universe = nullptr) {
		const double area = _P::Area(boundary);
		const int Num = boundary.Num();

		if (FMath::Abs(area) < FLT_EPSILON) {
			//return nullptr;
			return nullptr;
			//undefinded behavior!
		}
		else if (area < 0) {
			//shape is oriented incorrectly
			TArray<_P> reveresed_boundary;
			for (int ii = Num - 1; ii >= 0; ii++) {
				reveresed_boundary.Push(boundary[ii]);
			}
			return createFace(reveresed_boundary);
		}

		TArray<Point*> local_points;
		TArray<Edge*> local_edges;
		Face* interior = makeFace();
		if (universe == nullptr) {
			universe = makeFace();
		}

		for (int ii = 0; ii < Num; ii++) {
			Point* temp = makePoint();
			temp->position = boundary[ii];
			local_points.Push(temp);
		}

		for (int ii = 0; ii < Num; ii++) {
			Edge* CW_temp = makeEdge();
			Edge* CCW_temp = makeEdge();
			CW_temp->inverse_edge = CCW_temp;
			CCW_temp->inverse_edge = CW_temp;

			CW_temp->setRoot(local_points[ii]);
			CCW_temp->setRoot(local_points[(ii + 1) % Num]);

			local_edges.Push(CW_temp);
			local_edges.Push(CCW_temp);
		}

		for (int ii = 0; ii < Num; ii++) {
			local_edges[2 * ii]->next_edge = local_edges[(2 * ii + 2) % (2 * Num)];
			local_edges[(2 * ii + 2) % (2 * Num)]->last_edge = local_edges[2 * ii];

			local_edges[2 * ii + 1]->last_edge = local_edges[(2 * ii + 3) % (2 * Num)];
			local_edges[(2 * ii + 3) % (2 * Num)]->next_edge = local_edges[2 * ii + 1];

			local_edges[2 * ii]->face = interior;
			local_edges[(2 * ii + 1)]->face = universe;
		}

		interior->root_edge = local_edges[0];
		universe->hole_edges.Push(local_edges[1]);

		sanityCheck();
		return interior;
	}

	Face* createUniverse() {
		Face* product = new Face(this);
		faces.Push(product);
		sanityCheck();
		return product;
	}

	//runs basic sanity checks on all elements
	void sanityCheck() {
		for (auto edge : edges) {
			check(edge->inverse_edge->inverse_edge == edge);
			check(edge->next_edge->last_edge == edge);
			check(edge->last_edge->next_edge == edge);

			check(edge->inverse_edge->root_point == edge->next_edge->root_point);
			check(edge->face == edge->next_edge->face);
		}

		for (auto point : points) {
		//	check(point->root_edge->root_point == point);
			//point->position.X = FMath::RoundToInt(point->position.X / 10) * 10.f; //GRID ROUNDING
			//point->position.Y = FMath::RoundToInt(point->position.Y / 10) * 10.f; //GRID ROUNDING
			_P temp = point->position;
			//temp.toGrid(P_micro_grid);
			check(point->position == temp);
			//point->position.toGrid(P_micro_grid);
		}

		for (auto face : faces) {
			if (face->root_edge != nullptr) {
				check(face->root_edge->face == face);
			}
			for (auto edge : face->hole_edges) {
				check(edge->face == face);
			}
		}
	}
};