#pragma once
#include "FLL.h"

/*

Contains definition for a spacial representation agnostic template of a DCEL structure
_P is not directly interacted with, therefor any data type you like can be appended
The structure specifically details how regions who have intersecting boundaries are related.

*/

template <class _P> class Point;
template <class _P> class Edge;
template <class _P> class Face;
template <class _P> class Region;
template <class _P> class DCEL;

// Represents a point in space, the ends of edges, and corners of faces
template <class _P>
class Point {
	//friend Face;
	friend DCEL<_P>;
	friend Edge<_P>;

	// The associated DCEL system
	DCEL<_P> * universe;

	// An edge leaving this, nullptr if no edges reference this point
	Edge<_P> * root;

	// The position of this point of course!
	_P position;

	// This can only be created through DCEL system functions
	Point(DCEL<_P> * uni){
		universe = uni;
	};
	Point(Point<_P> &&) = delete;
	Point(Point<_P> const &) = delete;

	~Point() {

	}
public:
	void setPosition(_P p) {
		position = p;
	};
	_P getPosition() const {
		return position;
	};

	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	//         Traversal Methods

	Edge<_P> * getRoot() {
		return root;
	}

	Edge<_P> const * getRoot() const {
		return root;
	}
};

enum EdgeModType { face_destroyed, faces_preserved, face_created};

template <class _P>
class EdgeModResult {
	friend Edge<_P>;
	//this can only be created by modifications
	EdgeModResult(EdgeModType t, Face<_P> * f) {
		type = t;
		relevant = f;
	}
public:

	EdgeModType type;
	Face<_P> * relevant;
};

// Represents a one way connection between two points
template <class _P>
class Edge {
	//friend Face;
	friend DCEL<_P>;
	friend Face<_P>;

	// The associated DCEL system
	DCEL<_P> * universe;

	// The point this edge originates from
	Point<_P> * root;

	// The next edge on the boundary this edge forms
	Edge<_P> * next;
	// The previous edge on the boundary this edge forms
	Edge<_P> * last;
	// The inverse edge of this one.
	Edge<_P> * inv;

	// The face this edge forms a boundary for
	Face<_P> * loop;

	//this can only be created through DCEL system functions
	Edge(DCEL<_P> * uni) {
		universe = uni;
	}
	Edge(Edge<_P> &&) = delete;
	Edge(Edge<_P> const &) = delete;

	~Edge() {

	}
public:
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	//         Traversal Methods

	Point<_P> * getStart() {
		return root;
	}
	Point<_P> * getEnd() {
		return inv->root;
	}

	Edge<_P> * getNext() {
		return next;
	}
	Edge<_P> * getLast() {
		return last;
	}
	Edge<_P> * getInv() {
		return inv;
	}

	Face<_P> * getFace() {
		return loop;
	}

	//get the next edge cw around the root point
	Edge<_P> * getCW() {
		return last->inv;
	}
	//get the next edge ccw around the root point
	Edge<_P> * getCCW() {
		return inv->next;
	}

	Point<_P> const * getStart() const {
		return root;
	}
	Point<_P> const * getEnd() const {
		return inv->root;
	}

	Edge<_P> const * getNext() const {
		return next;
	}
	Edge<_P> const * getLast() const {
		return last;
	}
	Edge<_P> const * getInv() const {
		return inv;
	}

	Face<_P> const * getFace() const {
		return loop;
	}

	//get the next edge cw around the root
	Edge<_P> const * getCW() const {
		return inv->next;
	}
	//get the next edge ccw around the root
	Edge<_P> const * getCCW() const {
		return last->inv;
	}

	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	//         Modification

	//subdivides this point, maintains this edges root
	void subdivide(_P mid_point) {
		Edge<_P>* adjoint = universe->createEdge();
		Point<_P>* mid = universe->createPoint();

		//we need to connect mid - root, position

		//20 variables internal to our 4 edges
		//4 of these are inverses, can be ignored
		//5: root, last, inv->next, loop, inv->loop don't change
		//- adjoint->(next, last, loop, root)
		//- adoint->inv->(next, last, loop, root)
		//- next
		//- inv->(last, root)

		//11 internal changes

		//6 possible external references
		//root->root doesnt change
		//last->next doesnt change
		//inv->next->last doesnt change
		//- inv->root->root
		//- last->next
		//- inv->next->last

		//3 external changes

		mid->position = mid_point;
		mid->root = adjoint;

		if (next != inv) {
			inv->root->root = next;

			adjoint->next = next;
			adjoint->last = this;
			adjoint->loop = loop;
			adjoint->root = mid;

			adjoint->inv->next = inv;
			adjoint->inv->last = inv->last;
			adjoint->inv->loop = inv->loop;
			adjoint->inv->root = inv->root;

			next->last = adjoint;
			inv->last->next = adjoint->inv;

			next = adjoint;
			inv->last = adjoint->inv;
			inv->root = mid;
		}
		else {
			inv->root->root = adjoint->inv;

			adjoint->next = adjoint->inv;
			adjoint->last = this;
			adjoint->loop = loop;
			adjoint->root = mid;

			adjoint->inv->next = inv;
			adjoint->inv->last = adjoint;
			adjoint->inv->loop = loop;
			adjoint->inv->root = inv->root;

			next = adjoint;
			inv->last = adjoint->inv;
			inv->root = mid;
		}
	}

	//detach from the current root point in favor of a novel point
	//just moves root if it is isolated
	//returns the face left at og root if it exists
	EdgeModResult<_P> moveRoot(_P p) {
		Point<_P>* og = root;
		if (last == inv) {
			//root is isolated

			root->setPosition(p);
			return EdgeModResult<_P>(EdgeModType::faces_preserved, nullptr);
		}
		else {
			Point<_P>* end = universe->createPoint();
			Edge<_P>* old = inv->next;
			end->root = this;
			end->position = p;

			root->root = old;

			last->next = old;
			old->last = last;

			last = inv;
			inv->next = this;
			root = end;

			//reface this and old
			if (loop == inv->loop) {
				//we have disconnected a loop
				loop->root = this;

				Face<_P>* novel = universe->createFace();
				novel->root = old;
				novel->reFace();

				return EdgeModResult<_P>(EdgeModType::face_created, novel);
			}
			else {
				//we have merged two loops
				universe->removeFace(inv->loop);
				loop->reFace();

				return EdgeModResult<_P>(EdgeModType::face_destroyed, inv->loop);
			}
		}
	}

	//detaches from the current root and inserts to the end of target
	//deletes og root if thereafter isolated
	//returns the face left at og root if it exists
	EdgeModResult<_P> insertAfter(Edge<_P>* target) {
		//remove from og

		Face<_P>* novel_a = nullptr;
		Face<_P>* novel_b = nullptr;

		if (last == inv) {
			universe->removePoint(root);
		}
		else {
			Edge<_P>* old = inv->next;

			root->root = old;

			last->next = old;
			old->last = last;

			//reface this and old
			if (loop == inv->loop) {
				//we have disconnected a loop
				loop->root = this;

				novel_a = universe->createFace();
				novel_a->root = old;
				novel_a->reFace();
			}
			else {
				//we have merged two loops
				universe->removeFace(inv->loop);
				loop->reFace();
			}
		}

		//insert elsewhere
		root = target->inv->root;

		inv->next = target->next;
		target->next->last = inv;

		target->next = this;
		last = target;

		if (target->loop == loop) {
			//we have split a loop
			loop->root = this;

			novel_b = universe->createFace();
			novel_b->root = inv;
			novel_b->reFace();

			return EdgeModResult<_P>(EdgeModType::face_created, novel_b);
		}
		else {
			//we have joined two loops

			//is it the loop we just disconnected?
			if (target->loop == novel_a) {
				novel_a = nullptr;
			}

			universe->removeFace(target->loop);
			loop->reFace();

			return EdgeModResult<_P>(EdgeModType::face_destroyed, target->loop);
		}
	}

	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	//         Removal

	//remove this edge, its inverse, and either end points that are thereafter isolated
	//returns novel face if created
	EdgeModResult<_P> remove() {

		//if either point is isolated, we dont need to reface
		bool loose_strand = false;
		bool isolated = true;
		Face<_P>* novel = nullptr;
		EdgeModResult<_P> product(EdgeModType::faces_preserved, nullptr);

		if (next == inv) {
			loose_strand = true;

			universe->removePoint(inv->root);
		}
		else {
			isolated = false;

			inv->root->root = next;
			next->last = inv->last;
			inv->last->next = next;
		}

		if (last == inv) {
			loose_strand = true;

			universe->removePoint(root);
		}
		else {
			isolated = false;

			root->root = inv->next;
			last->next = inv->next;
			inv->next->last = last;
		}

		//we may be connecting or disconnecting two loops
		if (!loose_strand) {
			if (loop == inv->loop) {
				//we have disconnected a loop
				loop->root = next;
				loop->reFace();

				novel = universe->createFace();
				novel->root = last;
				novel->reFace();

				product.type = EdgeModType::face_created;
				product.relevant = novel;

			}
			else {
				//we have merged two loops
				loop->root = next;
				universe->removeFace(inv->loop);
				loop->reFace();

				product.type = EdgeModType::face_destroyed;
				product.relevant = inv->loop;
			}
		}
		else if (isolated) {
			product.type = EdgeModType::face_destroyed;
			product.relevant = loop;
		}

		universe->removeEdge(this);
		return product;
	}

	//contracts this edge, the resulting point position is this edges root position
	void contract() {

		Edge<_P>* focus = next;

		do {
			focus->root = root;
			focus = focus->inv->next;
		} while (focus != inv);

		root->root = next;

		last->next = next;
		next->last = last;

		inv->next->last = inv->last;
		inv->last->next = inv->next;

		if (loop->root == this) {
			loop->root = next;
		}
		if (inv->loop->root == inv) {
			inv->loop->root = inv->last;
		}

		universe->removePoint(inv->root);
		universe->removeEdge(this);
	}

};

//represents a loop formed by a series of edges
template <class _P>
class Face {
	friend DCEL<_P>;
	friend Region<_P>;
	friend Edge<_P>;

	DCEL<_P> * universe;

	Edge<_P> * root;

	Region<_P> * group;

	//this can only be created through DCEL system functions
	Face(DCEL<_P> * uni) {
		universe = uni;
		group = nullptr;
	}
	Face(DCEL<_P> * uni, Region<_P> * grp) {
		universe = uni;
	}
	Face(Face<_P> &&) = delete;
	Face(Face<_P> const &) = delete;

	~Face() {

	}

	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	//         Modifiers
	//sets all edges of this loop to reference this
	void reFace() {
		Edge<_P> * focus = root;
		do {
			focus->loop = this;
			focus = focus->next;
		} while (focus != root);
	}

public:
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	//         Traversal Methods

	Edge<_P> * getRoot() {
		return root;
	}

	Edge<_P> const * getRoot() const {
		return root;
	}

	Region<_P> * getGroup() {
		return group;
	}

	Region<_P> const * getGroup() const {
		return group;
	}

	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	//         Queries

	//get the count of edges in the boundary
	int getLoopSize() const {
		Edge<_P> const * focus = root;
		int count = 0;

		do {
			count++;

			focus = focus->next;
		} while (focus != root);

		return count;
	}

	//return a list of the points in the loop
	FLL<_P> getLoopPoints() const {
		FLL<_P> target;
		Edge<_P> const * focus = root;

		do {
			_P p = focus->root->getPosition();
			target.append(p);

			focus = focus->next;
		} while (focus != root);

		return target;
	}

	//return a list of the edges in the loop
	FLL<Edge<_P> *> getLoopEdges() {
		Edge<_P> * focus = root;
		FLL<Edge<_P> *> target;

		do {
			target.append(focus);

			focus = focus->next;
		} while (focus != root);

		return target;
	}

	FLL<Edge<_P> const *> getLoopEdges() const {
		Edge<_P> * focus = root;
		FLL<Edge<_P> *> target;

		do {
			target.append(focus);

			focus = focus->next;
		} while (focus != root);

		return target;
	}

	FLL<Face<_P> *> getNeighbors() {
		FLL<Face<_P> *> target;
		Edge<_P> * focus = root;

		do {
			Face<_P> * canidate = focus->inv->loop;
			if (!target.contains(canidate)) {
				target.append(canidate);
			}
			focus = focus->next;
		} while (focus != root);

		return target;
	}

	//return a list of the faces that share a boundary in the loop
	FLL<Face<_P> const *> getNeighbors() const {
		FLL<Face<_P> const *> target;
		Edge<_P> const * focus = root;

		do {
			Face<_P> const * canidate = focus->inv->loop;
			if (!target.contains(canidate)) {
				target.append(canidate);
			}
			focus = focus->next;
		} while (focus != root);

		return target;
	}

	/*struct interact_point {
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

	int getPointState(const _P &test_point) const;

	int getFirstIntersect(const _P &start, const _P &end, _P &intersect) const;*/

	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	//         Regioning

	//takes in a simple region boundary and divides this face into faces interior to, and faces exterior to that boundary
	//void subAllocateFace(FLL<_P> const &boundary, FLL<Face*> &interiors, FLL<Face*> &exteriors);

	FLL<Face<_P> *> mergeWithFace(Face<_P>* target) {
		FLL<Edge<_P> *> markToRemove;
		FLL<Face<_P> *> product;

		Edge<_P> * focus = root;
		do {

			if (focus->inv->loop == target) {
				markToRemove.append(focus);
			}

			focus = focus->next;
		} while (focus != root);

		product.append(this);

		while (!markToRemove.empty()) {
			EdgeModResult<_P> result = markToRemove.pop()->remove();
			if (result.type == EdgeModType::face_created) {
				product.append(result.relevant);
			}
			else if (result.type == EdgeModType::face_destroyed) {
				product.remove(result.relevant);
			}
		}
		return product;
	}
};

template <class _P>
class Region {
	friend DCEL<_P>;

	DCEL<_P> * universe;

	Region(DCEL<Pint> * uni) {
		universe = uni;
	}
	~Region() {

	}
	FLL<Face<_P> *> Boundaries;

public:
	FLL<Face<_P> *> const & getBounds() {
		return Boundaries;
	}
	Face<_P> * operator[](int a) {
		return Boundaries[a];
	}

	int size() const {
		return Boundaries.size();
	}

	void append(Face<Pint> * border) {
		if (border->group != this) {
			if (border->group != nullptr) {
				border->group->remove(border);
			}

			border->group = this;
			Boundaries.push(border);
		}
	}
	void remove(Face<Pint> * border) {
		if (border->group == this) {
			border->group = nullptr;
			Boundaries.remove(border);
		}
	}
	void clear() {
		for (auto border : Boundaries) {
			border->group = nullptr;
		}

		Boundaries.clear();
	}

	DCEL<_P> * getUni() {
		return universe;
	}

	FLL<Region *> getNeighbors() {
		FLL<Region *> product;

		for (auto border : Boundaries) {
			auto canidates = border->getNeighbors();
			for (auto suggest : canidates) {
				if (!product.contains(suggest->group)) {
					product.push(suggest->group);
				}
			}
		}

		return product;
	}

};



template <class _P>
class DCEL {
	FLL<Point<_P> *> points;
	FLL<Edge<_P> *> edges;
	FLL<Face<_P> *> faces;
	FLL<Region<_P> *> regions;

	friend Edge<_P>;

	//creates a point
	//no parameters are initialized
	Point<_P> * createPoint() {
		Point<_P> * result = new Point<_P>(this);
		points.push(result);
		return result;
	}
	//creates an edge and its inverse
	//no parameters are initialized
	Edge<_P> * createEdge() {
		Edge<_P> * result = new Edge<_P>(this);
		Edge<_P> * inverse = new Edge<_P>(this);

		edges.push(result);
		edges.push(inverse);

		result->inv = inverse;
		inverse->inv = result;

		return result;
	}
	//creates a face
	//no parameters are initialized
	Face<_P> * createFace() {
		Face<_P> * result = new Face<_P>(this);
		faces.push(result);
		return result;
	}

	//removes a point
	//does NOT check to see if referenced elsewhere
	void removePoint(Point<_P> * target) {
		points.remove(target);

		delete target;
	}
	//removes an edge and its inverse
	//does NOT check to see if referenced elsewhere
	void removeEdge(Edge<_P> * target) {
		edges.remove(target);
		edges.remove(target->inv);

		delete target->inv;
		delete target;
	}
	//removes a face
	//does NOT check to see if referenced elsewhere
	void removeFace(Face<_P> * target) {
		faces.remove(target);

		delete target;
	}

public:
	~DCEL() {
		for(auto focus_point : points) {
			delete focus_point;
		}

		for (auto focus_edge : edges) {
			delete focus_edge;
		}

		for (auto focus_face : faces) {
			delete focus_face;
		}

		for (auto focus_region : regions) {
			delete focus_region;
		}
	}

	int pointCount() const {
		return points.size();
	}
	int edgeCount() const {
		return edges.size();
	}
	int faceCount() const {
		return faces.size();
	}
	int regionCount() const {
		return regions.size();
	}

	//creates an edge and its inverse connecting two novel points
	Edge<_P> * addEdge(_P a, _P b) {
		Edge<_P> * result = createEdge();
		Point<_P> * A = createPoint();
		Point<_P> * B = createPoint();
		Face<_P> * loop = createFace();

		result->next = result->inv;
		result->last = result->inv;
		result->inv->next = result;
		result->inv->last = result;

		result->root = A;
		A->root = result;
		A->position = a;

		result->inv->root = B;
		B->root = result->inv;
		B->position = b;

		result->loop = loop;
		result->inv->loop = loop;

		loop->root = result;

		return result;
	}
	//creates an edge and its inverse connecting after an edge to a novel point
	Edge<_P> * addEdge(Edge<_P> * a, _P b) {
		Edge<_P> * result = createEdge();
		Point<_P> * B = createPoint();

		result->next = result->inv;
		result->inv->last = result;

		result->root = a->inv->root;

		result->inv->root = B;
		B->root = result->inv;
		B->position = b;

		a->next->last = result->inv;
		result->inv->next = a->next;

		a->next = result;
		result->last = a;

		result->loop = a->loop;
		result->inv->loop = a->loop;

		return result;
	}
	//creates an edge and its inverse connecting after an two edges
	Edge<_P> * addEdge(Edge<_P> * a, Edge<_P> * b) {
		Edge<_P> * result = createEdge();
		Face<_P> * novel = nullptr;

		a->next->last = result->inv;
		result->inv->next = a->next;

		a->next = result;
		result->last = a;

		b->next->last = result;
		result->next = b->next;

		b->next = result->inv;
		result->inv->last = b;

		result->root = a->inv->root;
		result->inv->root = b->inv->root;

		if (a->loop == b->loop) {
			//we have split a loop

			a->loop->root = a;
			result->loop = a->loop;

			novel = createFace();
			novel->root = b;
			novel->reFace();
		}
		else {
			//we have joined two loops
			removeFace(b->loop);
			a->loop->reFace();
		}

		return result;
	}

	Region<_P> * region() {
		Region<_P> * product = new Region<_P>(this);
		regions.append(product);
		return product;
	}

	Region<_P> * region(Face<_P> * face) {
		Region<_P> * product = new Region<_P>(this);
		product->append(face);
		regions.append(product);
		return product;
	}
	Region<_P> * region(FLL<_P> const &boundary) {
		Region<_P> * product = new Region<_P>(this);
		product->append(draw(boundary));
		regions.append(product);
		return product;
	}

	//creates a circular chain of edges forming a loop with the given boundary
	//returns a pointer to the clock-wise oriented interior of the boundary
	Face<_P> * draw(FLL<_P> const &boundary) {
		auto track = boundary.begin();

		_P a = *track;
		++track;
		_P b = *track;
		++track;

		Edge<_P> * start = addEdge(a, b);
		Edge<_P> * strand = start;

		while (track != boundary.end()) {
			b = *track;
			++track;

			strand = addEdge(strand, b);
		}

		addEdge(strand, start->inv);

		start->loop->root = start;

		return start->loop;
	}

	//removes a region
	//does NOT check to see if referenced elsewhere
	void removeRegion(Region<_P> * target) {
		regions.remove(target);

		delete target;
	}
};