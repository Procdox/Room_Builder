#pragma once

#include "CoreMinimal.h"
#include "DCEL.generated.h"

struct _P {
	int markup;
	float X;
	float Y;
	_P() {
		X = 0;
		Y = 0;
		markup = 0;
	}
	_P(int m) {
		X = 0;
		Y = 0;
		markup = m;
	}
	_P(float x, float y) {
		X = x;
		Y = y;
		markup = 0;
	}
	_P(float x, float y, int m) {
		X = x;
		Y = y;
		markup = m;
	}

	bool Equals(const _P &test) const {
		return (std::abs(test.X - X) < DBL_EPSILON && std::abs(test.Y - Y) < DBL_EPSILON);
	}
	_P operator+(const _P &add) const {
		return _P(X + add.X, Y + add.Y);
	}
	_P operator-(const _P &sub) const {
		return _P(X - sub.X, Y - sub.Y);
	}
	_P operator*(float mul) const {
		return _P(X * mul, Y * mul);
	}
	_P operator/(float div) const {
		return _P(X / div, Y / div);
	}
	bool operator==(const _P &compare) const {
		return Equals(compare);
	}
	float SizeSquared() {
		return X * X + Y * Y;
	}
	float Size() {
		return FMath::Sqrt(SizeSquared());
	}
	void Normalize() {
		const float size = Size();
		X /= size;
		Y /= size;
	}
	static float Dot(const _P &a, const _P &b) {
		return a.X*b.X + a.Y*b.Y;
	}
	static float Area(const TArray<_P> &boundary) {
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

	static bool isOnSegment(const _P &test, const _P &a, const _P &b) {
		if (test.Equals(a)) {
			return true;
		}
		_P A = test - a;
		_P B = b - a;


		if (A.Size() < B.Size()) {
			A.Normalize();
			B.Normalize();
			return 1 - _P::Dot(A, B) < .0000001;
		}
		return false;
	}
	static bool inRegionCW(const _P &left, const _P &right, const _P &test) {
		_P bisector = left + right;
		bisector.Normalize();

		//test and fix for concavity
		const _P test_concave(left.Y, -left.X);
		const float orientation = _P::Dot(right, test_concave);
		if (orientation < 0) {
			bisector = bisector * -1;
		}
		else if (orientation == 0) {
			bisector = test_concave;
		}


		return _P::Dot(bisector, left) <= _P::Dot(bisector, test);
	}

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
					if (test_point.Equals(focus->getStartPoint()->getPosition()) ||
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
			interact_point(const _P p) {
				region = -1;
				state = unknown_region;
				point = p;
				next = nullptr;
			}
			interact_point(const _P p, interact_point* n) {
				region = -1;
				state = unknown_region;
				point = p;
				next = n;
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

		interaction_state getPointState(const _P &test_point) const;

		interaction_state getFirstIntersect(const _P &start, const _P &end, _P &intersect) const;

		Edge* getContainingSegment(const _P &test_point) const;
		//splits this face into two sets of faces, the intersect components of this face and the supposed border, and the difference components of this face minus the supposed border
		//returns false if no non-trivial intersect exists
		void subAllocateFace(const TArray<_P> &border, TArray<Face*> &interior_regions, TArray<Face*> &exterior_regions);

		bool mergeWithFace(Face* target);

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

		if (area == 0) {
			//return nullptr;
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

		return interior;
	}

	Face* createUniverse() {
		Face* product = new Face(this);
		faces.Push(product);
		return product;
	}
};