// F_DCEL.cpp : Defines the exported functions for the DLL application.
//
#include "DCEL.h"

bool testIntersect() {
	return false;
};

void F_DCEL::Edge::subdivide(const _P &location) {
	Edge* follows = dad->makeEdge();
	Edge* follows_inverse = dad->makeEdge();

	//follows->setInverse(follows_inverse);

	//insertAfter(follows_inverse);
	//inverse_edge->setRoot);
	//dinsertAfter(follows);

	follows->face = face;
	follows_inverse->face = inverse_edge->face;

	F_DCEL::Point *mid = dad->makePoint(location);

	//follows
	follows->root_point = mid;

	follows->next_edge = next_edge;
	next_edge->last_edge = follows;

	next_edge = follows;
	follows->last_edge = this;

	//follows inverse
	follows_inverse->root_point = mid;

	follows_inverse->next_edge = inverse_edge->next_edge;
	inverse_edge->next_edge->last_edge = follows_inverse;

	inverse_edge->next_edge = follows_inverse;
	follows_inverse->last_edge = inverse_edge;

	//inverses
	follows->inverse_edge = inverse_edge;
	follows_inverse->inverse_edge = this;

	inverse_edge->inverse_edge = follows;
	inverse_edge = follows_inverse;
}

//inclusive containment check (on border returns true)
//checks for nearest x+ intersect, then checks if it is internal to it
bool isPointContained(const _P &test_point, const F_DCEL::Edge* start_edge) {
	bool polarity = true;
	if (start_edge->loopArea() < 0) {
		polarity = false;
	}
	const F_DCEL::Edge* focus = start_edge;
	int count = 0;

	const F_DCEL::Edge* best_edge = nullptr;
	double best_distance = DBL_MAX;

	do {
		const _P &start_vector = focus->getStartPoint()->getPosition();
		const _P &end_vector = focus->getEndPoint()->getPosition();


		//does it sit on 
		if (start_vector.Y == end_vector.Y) {
			if (test_point.Y == start_vector.Y) {
				if ((start_vector.X <= test_point.X && test_point.X <= end_vector.X) ||
					(start_vector.X >= test_point.X && test_point.X >= end_vector.X)) {
					return true; //lies on border
				}
			}
			focus = focus->getNextEdge();
			continue;
		}

		double y_offset = test_point.Y - start_vector.Y;
		double y_length = end_vector.Y - start_vector.Y;
		double y_ratio = y_offset / y_length;

		if (y_ratio <= 1.f && y_ratio >= 0.f) {

			double x_length = end_vector.X - start_vector.X;
			double x = start_vector.X + x_length * y_ratio;

			if (x == test_point.X) {
				return true; //lies on border
			}

			double distance = x - test_point.X;
			if (distance > 0 && distance < best_distance) {
				best_distance = distance;
				best_edge = focus;
			}

		}


		focus = focus->getNextEdge();
	} while (focus != start_edge);

	if (best_edge == nullptr) {
		return false;
	}

	const _P start = best_edge->getStartPoint()->getPosition();
	const _P end = best_edge->getEndPoint()->getPosition();

	//does it sit on the ends?
	if (test_point.Y == start.Y) {
		//check before
		_P offset = test_point - start;
		_P left = end - start;
		_P right = best_edge->getLastEdge()->getStartPoint()->getPosition() - start;
		offset.Normalize();
		left.Normalize();
		right.Normalize();

		if (_P::inRegionCW(left, right, offset) != polarity) {
			return false;
		}
	}
	else if (test_point.Y == end.Y) {
		_P offset = test_point - end;
		_P left = best_edge->getNextEdge()->getEndPoint()->getPosition() - start;
		_P right = start - end;
		offset.Normalize();
		left.Normalize();
		right.Normalize();

		if (_P::inRegionCW(left, right, offset) != polarity) {
			return false;
		}
	}
	else {
		//check if planar to the closest
		_P best_direction = end - start;

		best_direction.Normalize();

		_P inwards(best_direction.Y, -best_direction.X);

		if ((_P::Dot(inwards, (test_point - start)) > 0) != polarity) {
			return false;
		}
	}


	return true;
}

//interaction code
//even are in region
//odd are boundary
// 0 external, 1 main boundary, 2 interior, 2n+3 hole boundary, 2n+4 hole interior
interaction_state F_DCEL::Face::getPointState(const _P &test_point) const {
	//is it on a border
	if (root_edge != nullptr) {
		Edge* focus = root_edge;
		do {

			if (_P::isOnSegment(test_point, focus->getStartPoint()->getPosition(), focus->getEndPoint()->getPosition())) {
				return external_boundary;
			}

			focus = focus->getNextEdge();
		} while (focus != root_edge);
	}

	//is it on a hole border
	int ii = -1;
	for (auto root : hole_edges) {
		ii++;
		Edge* focus = root;
		do {

			if (_P::isOnSegment(test_point, focus->getStartPoint()->getPosition(), focus->getEndPoint()->getPosition())) {
				return 3 + 2 * ii;
			}

			focus = focus->getNextEdge();
		} while (focus != root);
	}

	//is it interior to the external border
	if (root_edge == nullptr || isPointContained(test_point, root_edge)) {

		//is it interior to a hole
		int ii = -1;
		for (auto root : hole_edges) {
			ii++;
			Edge* focus = root;
			if (isPointContained(test_point, root)) {
				return 4 + 2 * ii;
			}
		}

		//interior to border but not a hole
		return internal_region;
	}

	//not interior
	return external_region;
}



double getIntersectRatio(const _P &A_S, const _P &A_E, const _P &B_S, const _P &B_E) {
	double ua, ub, denom;

	denom = (B_E.Y - B_S.Y)*(A_E.X - A_S.X) - (B_E.X - B_S.X)*(A_E.Y - A_S.Y);
	if (denom == 0) {
		return -1.f;
	}

	ua = ((B_E.X - B_S.X)*(A_S.Y - B_S.Y) - (B_E.Y - B_S.Y)*(A_S.X - B_S.X)) / denom;
	ub = ((A_E.X - A_S.X)*(A_S.Y - B_S.Y) - (A_E.Y - A_S.Y)*(A_S.X - B_S.X)) / denom;

	if (ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1) {
		return ua;
	}

	return -1.f;
}

//interaction code
//even are in region
//odd are boundary
// 0 external, 1 main boundary, 2 interior, 2n+3 hole boundary, 2n+4 hole interior
interaction_state F_DCEL::Face::getFirstIntersect(const _P &start, const _P &end, _P &intersect) const {
	double best_ratio = 1.0;
	interaction_state result = unknown_region;
	if (root_edge != nullptr) {
		Edge* focus = root_edge;
		do {
			double ratio = getIntersectRatio(start, end, focus->getStartPoint()->getPosition(), focus->getEndPoint()->getPosition());
			if (ratio > 0.0 && ratio < best_ratio) {
				best_ratio = ratio;
				result = external_boundary;
			}
			focus = focus->getNextEdge();
		} while (focus != root_edge);
	}

	int ii = -1;
	for (auto root : hole_edges) {
		ii++;
		Edge* focus = root;
		do {
			double ratio = getIntersectRatio(start, end, focus->getStartPoint()->getPosition(), focus->getEndPoint()->getPosition());
			if (ratio > 0.0 && ratio < best_ratio) {
				best_ratio = ratio;
				result = 3 + 2 * ii;
			}
			focus = focus->getNextEdge();
		} while (focus != root);
	}

	_P direction = end - start;
	intersect = start + direction * best_ratio;
	return result;
}

F_DCEL::Edge* F_DCEL::Face::getContainingSegment(const _P &test_point) const {
	//is it on the exterior border
	//[a,b)
	Edge* result = nullptr;
	if (root_edge != nullptr) {
		result = root_edge->getContainingSegment(test_point);
		if (result != nullptr) {
			return result;
		}
	}
	for (auto root : hole_edges) {
		result = root->getContainingSegment(test_point);
		if (result != nullptr) {
			return result;
		}
	}

	return nullptr;
}


//culls a polygon to the region represented by this face, then culls from tjos face that region to create a set of new regions
//interior regions are relevant to the proposed polygon
//exterior are regions not relevant to the proposed, but members of this still
void F_DCEL::Face::subAllocateFace(const TArray<_P> &border, TArray<Face*> &interior_regions, TArray<Face*> &exterior_regions) {
	//any point is exterior, interior, or border-intersecting
	int f_i = 0;
	const int Num = border.Num();
	const double area = _P::Area(border);

	if (area == 0) {
		//return nullptr;
		//undefinded behavior!
	}
	else if (area < 0) {
		//shape is oriented incorrectly
		TArray<_P> reveresed_boundary;
		for (int ii = Num - 1; ii >= 0; ii++) {
			reveresed_boundary.Push(border[ii]);
		}
		return subAllocateFace(reveresed_boundary, interior_regions, exterior_regions);
	}

	TArray<interact_point*> interactions;
	TArray<bool> touched_holes;

	for (auto temp : hole_edges) {
		touched_holes.Push(false);
	}

	for (int ii = 0; ii < Num; ii++) {
		interactions.Push(new interact_point(border[ii]));
	}

	for (int ii = 0; ii < Num; ii++) {
		interactions[ii]->next = interactions[(ii + 1) % Num];
	}

	interact_point* focus = interactions[0];
	do {
		//test point
		if (focus->state == unknown_region) {
			focus->state = getPointState(focus->point);
		}

		if (focus->state > 2) {
			touched_holes[(focus->state - 3) / 2] = true;
		}

		//test for intersects
		_P intersect_point;
		interaction_state intersect_result = getFirstIntersect(focus->point, focus->next->point, intersect_point);
		if (intersect_result != unknown_region) {

			//create new point
			interact_point* temp = new interact_point(intersect_point, focus->next);

			interactions.Push(temp);
			focus->next = temp;
			focus->next->state = intersect_result;
		}
		focus = focus->next;

	} while (focus != interactions[0]);

	/*struct strand {
	interact_point* start;
	interact_point* end;
	Edge* start_edge;
	Edge* end_edge;
	bool visited;
	bool unique_interior;
	bool unique_exterior;
	strand() {
	visited = false;
	unique_interior = true;
	unique_exterior = true;
	}
	};*/

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

	//test for trivial containment cases
	bool all_exterior = true;
	bool all_interior = true;
	bool all_hole = true;
	bool boundary_connected = false;

	int discovered_hole_index = -1;

	for (int index = 0; index < interactions.Num(); index++) {
		const int state = interactions[index]->state;

		if (state == 0) {
			all_interior = false;
			all_hole = false;
		}
		else if (state == 1) {
			all_hole = false;
			boundary_connected = true;
		}
		else if (state == 2) {
			all_exterior = false;
			all_hole = false;
		}
		else {
			all_exterior = false;
			all_interior = false;

			if (state % 2 == 1) {
				all_hole = false;
			}
			else {
				int hole_index = (state - 3) / 2;

				if (discovered_hole_index == -1) {
					discovered_hole_index = hole_index;
				}

				if (discovered_hole_index != hole_index) {
					all_hole = false;
				}
			}
		}

		if (state == 0) {
			all_interior = false;
			all_hole = false;
		}
		else {
			all_exterior = false;
			if (state == 1) {
				all_hole = false;
				all_exterior = false;
			}
			else if (state == 2) {
				all_hole = false;
			}
			else
			{
				all_interior = false;

				int hole_index = (state - 3) / 2;
				if (discovered_hole_index == -1) {
					discovered_hole_index = hole_index;
				}
				if (discovered_hole_index != hole_index) {
					all_hole = false;
				}
			}
		}
	}
	//if suggested contains existing, return existing as interior
	//if existing contains suggested, make a new hole and that is your result
	//if contained in a hole, return null
	if (all_exterior && !boundary_connected) {
		//if that boundary contains this, then it becomes this, otherwise it is culled to null
		//check for containment
		Face* test = dad->createFace(border);

		if (root_edge != nullptr && isPointContained(root_edge->getStartPoint()->getPosition(), test->getRootEdge())) {
			interior_regions.Push(this);
		}
		else {
			exterior_regions.Push(this);
		}
		return;
	}

	//is the suggested region contained entirely within the target region
	if (all_interior && !boundary_connected) {
		Face* interior = dad->createFace(border, this);
		interior_regions.Push(interior);
		exterior_regions.Push(this);
		return;
	}

	//is the suggested region entirely contained in another set of regions represented here as a hole
	if (all_hole) {
		exterior_regions.Push(this);
		return;
	}

	TArray<strand*> strands;

	strand* current = new strand();

	bool mid_strand = false;

	bool made_loop = false;

	F_DCEL::Edge* last_edge = nullptr;


	TArray<F_DCEL::Edge*> eligible_edges;
	if (boundary_connected) {
		root_edge->listFaceLoop(eligible_edges);
	}
	for (auto edge : hole_edges) {
		edge->listFaceLoop(eligible_edges);
	}


	while (true) {
		if (mid_strand) {
			if (focus->state != internal_region) {

				mid_strand = false;

				//for the end, we test inversely, to find the entering segment
				Edge* local_a = nullptr; //getContainingSegment(focus->point);
				TArray<_P> debug_compare;

				for (auto edge : eligible_edges) {
					debug_compare.Push(edge->getStartPoint()->getPosition());
					debug_compare.Push(edge->getEndPoint()->getPosition());

					if (_P::isOnSegment(focus->point, edge->getEndPoint()->getPosition(), edge->getStartPoint()->getPosition())) {
						local_a = edge;
						break;
					}
				}

				if (focus->point.Equals(local_a->getEndPoint()->getPosition())) {
					//find appropriate owning edge from root
					F_DCEL::Edge* rotation_focus = local_a;
					_P test = last_edge->getEndPoint()->getPosition() - focus->point;
					test.Normalize();
					do {
						//get the bisector and range
						_P right = rotation_focus->getStartPoint()->getPosition() - rotation_focus->getEndPoint()->getPosition();
						_P left = rotation_focus->getNextEdge()->getEndPoint()->getPosition() - rotation_focus->getEndPoint()->getPosition();
						left.Normalize();
						right.Normalize();

						if (_P::inRegionCW(left, right, test)) {
							local_a = rotation_focus;
							break;
						}

						rotation_focus = rotation_focus->getNextEdge()->getInverseEdge();
					} while (rotation_focus != local_a);
				}
				else {
					local_a->subdivide(focus->point);

					eligible_edges.Push(local_a->getNextEdge());
				}

				local_a->insertAfter(last_edge);

				strands.Push(current);
				current = new strand();
			}
			else {
				Edge* temp = dad->makeEdge();
				Edge* temp_inverse = dad->makeEdge();

				temp->setInverse(temp_inverse);

				temp->setRoot(dad->makePoint(focus->point));

				temp_inverse->insertAfter(last_edge);

				last_edge = temp_inverse;
			}
		}
		if (made_loop && !mid_strand) {
			break;
		}
		if (!mid_strand && focus->state % 2 == 1) { //are we on a border

			Edge* local = nullptr; //getContainingSegment(focus->point);
			TArray<_P> debug_compare;

			for (auto edge : eligible_edges) {
				debug_compare.Push(edge->getStartPoint()->getPosition());
				debug_compare.Push(edge->getEndPoint()->getPosition());

				if (_P::isOnSegment(focus->point, edge->getStartPoint()->getPosition(), edge->getEndPoint()->getPosition())) {
					local = edge;
					break;
				}
			}

			if (focus->next->state == focus->state) { //is the next point on the same border

				if (!(focus->next->point.Equals(local->getEndPoint()->getPosition()) ||
					_P::isOnSegment(focus->next->point, local->getStartPoint()->getPosition(), local->getEndPoint()->getPosition()))) { //is the next point on a different edge

					_P mid = (focus->point + focus->next->point) / 2;

					if (getPointState(mid) == 2) { //is the mid point of this section interior to the region, or the hole

						mid_strand = true;

						if (focus->point.Equals(local->getStartPoint()->getPosition())) {
							//find appropriate owning edge from root
							F_DCEL::Edge* rotation_focus = local;
							_P test = focus->next->point - focus->point;
							test.Normalize();
							do {
								//get the bisector and range
								_P left = rotation_focus->getEndPoint()->getPosition() - rotation_focus->getStartPoint()->getPosition();
								_P right = rotation_focus->getLastEdge()->getStartPoint()->getPosition() - rotation_focus->getStartPoint()->getPosition();
								left.Normalize();
								right.Normalize();

								if (_P::inRegionCW(left, right, test)) {
									local = rotation_focus;
									break;
								}

								rotation_focus = rotation_focus->getCWOfRoot();
							} while (rotation_focus != local);
						}
						else {
							local->subdivide(focus->point);

							local = local->getNextEdge();
							eligible_edges.Push(local);
						}

						current->interior_edge = dad->makeEdge();
						current->exterior_edge = dad->makeEdge();


						current->interior_edge->setInverse(current->exterior_edge);


						local->insertBefore(current->exterior_edge);

						last_edge = current->exterior_edge;
					}
				}
			}
			else if (focus->next->state == internal_region || focus->next->state % 2 == 1) { //is the next point interior, or crossing the interior to another boundary

				mid_strand = true;

				if (focus->point.Equals(local->getStartPoint()->getPosition())) {
					//find appropriate owning edge from root
					F_DCEL::Edge* rotation_focus = local;
					_P test = focus->next->point - focus->point;
					test.Normalize();
					do {
						//get the bisector and range
						_P left = rotation_focus->getEndPoint()->getPosition() - rotation_focus->getStartPoint()->getPosition();
						_P right = rotation_focus->getLastEdge()->getStartPoint()->getPosition() - rotation_focus->getStartPoint()->getPosition();
						left.Normalize();
						right.Normalize();

						if (_P::inRegionCW(left, right, test)) {
							local = rotation_focus;
							break;
						}

						rotation_focus = rotation_focus->getCWOfRoot();
					} while (rotation_focus != local);
				}
				else {
					local->subdivide(focus->point);

					local = local->getNextEdge();
					eligible_edges.Push(local);
				}

				current->interior_edge = dad->makeEdge();
				current->exterior_edge = dad->makeEdge();

				current->interior_edge->setInverse(current->exterior_edge);


				local->insertBefore(current->exterior_edge);

				last_edge = current->exterior_edge;
			}
		}

		focus = focus->next;
		if (focus == interactions[0]) {
			made_loop = true;
		}
		if (made_loop && !mid_strand) {
			break;
		}
	}

	delete current;
	if (strands.Num() == 0) {
		for (auto focus : interactions) {
			delete focus;
		}
		return;
	}

	//find unique sections

	if (!boundary_connected) {
		exterior_regions.Push(this);
	}
	for (int ii = hole_edges.Num() - 1; ii >= 0; ii--) {
		if (touched_holes[ii]) {
			hole_edges.RemoveAt(ii);
		}
	}

	TArray<Edge*> existing_holes;
	for (auto hole : hole_edges) {
		existing_holes.Push(hole);
	}
	hole_edges.Empty();

	for (int index = 0; index < strands.Num(); index++) {
		if (strands[index]->unique_interior) {
			//trace from end to start and mark encountered starts, doesn't skips strands
			Edge* focus = strands[index]->interior_edge;

			do {
				for (int compare = index; compare < strands.Num(); compare++) {
					if (focus == strands[compare]->interior_edge) {

						strands[compare]->unique_interior = false;
						break;
					}
				}

				focus = focus->getNextEdge();
			} while (focus != strands[index]->interior_edge);

			Face* interior_face = dad->makeFace();

			interior_face->root_edge = strands[index]->interior_edge;
			interior_face->root_edge->reFaceLoop(interior_face);

			interior_regions.Push(interior_face);

		}

		if (strands[index]->unique_exterior) {
			//trace from start inverse to end inverse and mark encountered end inverses, skips strands
			Edge* focus = strands[index]->exterior_edge;

			do {
				bool found = false;

				for (int compare = index; compare < strands.Num(); compare++) {
					if (focus == strands[compare]->exterior_edge) {

						strands[compare]->unique_exterior = false;

						break;
					}
				}

				focus = focus->getNextEdge();
			} while (focus != strands[index]->exterior_edge);

			if (boundary_connected) {
				Face* exterior_face = dad->makeFace();

				exterior_face->root_edge = strands[index]->exterior_edge;
				exterior_face->root_edge->reFaceLoop(exterior_face);

				exterior_regions.Push(exterior_face);
			}
			else {
				hole_edges.Push(strands[index]->exterior_edge);
				strands[index]->exterior_edge->reFaceLoop(this);
			}
		}
	}

	//find hole positions
	for (auto hole : existing_holes) {

		// find containing region
		// insert into region holes
		bool found = false;
		for (auto region : interior_regions) {
			if (isPointContained(hole->getStartPoint()->getPosition(), region->root_edge)) {
				found = true;
				region->hole_edges.Push(hole);
				hole->reFaceLoop(region);
				break;
			}
		}
		if (!found) {
			for (auto region : exterior_regions) {
				if (isPointContained(hole->getStartPoint()->getPosition(), region->root_edge)) {
					found = true;
					region->hole_edges.Push(hole);
					hole->reFaceLoop(region);
					break;
				}
			}
		}
	}

	if (boundary_connected) {
		delete this;
	}
	return;
}

//attempts to merge target faces to self by continuity across a shared edge section.
bool F_DCEL::Face::mergeWithFace(Face* target) {
	//cases
	//a) completely sorrounded by them (we are a hole in them)
	//b) completely sorrounding them (they are a hole in us)
	//c) partially meeting them (mark all boundary edges, find uniques, determine external boundary by wrapping
	//d) no meeting (no union occurs)

	bool found_merge = false;

	Edge* focus = root_edge;
	Edge* first_free = nullptr;

	do {
		if (focus->getInverseEdge()->getFace() == target) {
			found_merge = true;
			if (first_free != nullptr) { break; }
		}
		else if (first_free == nullptr) {
			first_free = focus;
			if (found_merge) { break; }
		}
		focus = focus->getNextEdge();
	} while (focus != root_edge);
	if (found_merge) {
		if (first_free == nullptr) { //------------------------------------------
									 //we are a hole, FILL IT
									 //list boundary edges, and points
			TArray<Edge*> edges;
			TArray<Point*> points;

			focus = root_edge;
			do {
				edges.Push(focus);
				points.Push(focus->getStartPoint());

				focus = focus->getNextEdge();
			} while (focus != root_edge);

			root_edge = target->root_edge;
			root_edge->reFaceLoop(this);

			//add their holes that aren't this to us
			for (auto edge : target->hole_edges) {
				if (edge->getInverseEdge()->getFace() != this) {
					edge->reFaceLoop(this);
					hole_edges.Push(edge);
				}
			}

			//delete ALL
			for (auto edge : edges) {
				dad->removeEdge(edge->getInverseEdge());
				dad->removeEdge(edge);
			}
			for (auto point : points) {
				dad->removePoint(point);
			}
			dad->removeFace(target);
		}
		else { //------------------------------------------
			   //they reside on our border


			   //add their holes that aren't this to us
			bool found_this = false;
			bool is_hole = false;
			for (auto edge : target->hole_edges) {

				if (!found_this) {
					focus = edge;

					do {
						if (focus->getInverseEdge()->getFace() == this) {
							found_this = true;
							is_hole = true;
							break;
						}
					} while (focus != edge);

					if (found_this) {
						continue;
					}
				}

				edge->reFaceLoop(this);
				hole_edges.Push(edge);
			}

			focus = first_free;
			bool mid_strand = false;
			bool made_loop = false;
			TArray<Edge*> strands;

			do {
				Edge* next = focus->getNextEdge();

				if (!mid_strand) {
					if (focus->getInverseEdge()->getFace() == target) {
						//strand starts
						//wrap the edges before, and the edge after

						focus->removeFromGraph();
						dad->removeEdge(focus->getInverseEdge());
						dad->removeEdge(focus);

						mid_strand = true;

						focus = next;
					}
					else {
						focus = next;
					}
				}
				else {
					if (focus->getInverseEdge()->getFace() == target) {
						//strand continues

						//remove the last point
						dad->removePoint(focus->getStartPoint());

						focus->removeFromGraph();
						dad->removeEdge(focus->getInverseEdge());
						dad->removeEdge(focus);

						focus = next;
					}
					else {
						//strand ends
						strands.Push(focus);

						mid_strand = false;

						focus = next;
					}
				}

				if (focus == first_free) {
					made_loop = true;
				}
				if (made_loop && !mid_strand) {
					break;
				}
			} while (true);


			if (is_hole) {
				root_edge = target->root_edge;
				root_edge->reFaceLoop(this);

				for (auto edge : strands) {
					edge->reFaceLoop(this);
					hole_edges.Push(edge);
				}
			}
			else {
				found_this = false;
				for (auto edge : strands) {
					edge->reFaceLoop(this);

					if (!found_this) {
						if (edge->loopArea() > 0) {
							found_this = true;
							root_edge = edge;
							continue;
						}
					}

					hole_edges.Push(edge);
				}
			}

			dad->removeFace(target);
		}

		return true;
	}
	else { //they are bound to a hole, or not at all ============================================
		for (auto root : hole_edges) {

			found_merge = false;
			first_free = nullptr; //redundant, oh well

			focus = root;
			do {
				if (focus->getInverseEdge()->getFace() == target) {
					found_merge = true;
					if (first_free != nullptr) { break; }
				}
				else if (first_free == nullptr) {
					first_free = focus;
					if (found_merge) { break; }
				}
				focus = focus->getNextEdge();
			} while (focus != root);
			if (found_merge) {
				if (first_free == nullptr) {//------------------------------------------
											//they are a hole, FILL IT
											//list boundary edges, and points
					TArray<Edge*> edges;
					TArray<Point*> points;

					focus = root;
					do {
						edges.Push(focus);
						points.Push(focus->getStartPoint());

						focus = focus->getNextEdge();
					} while (focus != root);

					//add their holes that aren't this to us
					for (auto edge : target->hole_edges) {
						edge->reFaceLoop(this);
						hole_edges.Push(edge);
					}

					//delete ALL
					for (auto edge : edges) {
						dad->removeEdge(edge->getInverseEdge());
						dad->removeEdge(edge);
					}
					for (auto point : points) {
						dad->removePoint(point);
					}
					dad->removeFace(target);

					//remove the reference to this hole
					hole_edges.Remove(root);

				}
				else { //------------------------------------------
					   //add their holes to us
					bool found_this = false;
					bool is_hole = false;

					hole_edges.Remove(root);

					for (auto edge : target->hole_edges) {
						edge->reFaceLoop(this);
						hole_edges.Push(edge);
					}

					focus = first_free;
					bool mid_strand = false;
					bool made_loop = false;
					TArray<Edge*> strands;

					do {
						Edge* next = focus->getNextEdge();

						if (!mid_strand) {
							if (focus->getInverseEdge()->getFace() == target) {
								//strand starts
								//wrap the edges before, and the edge after

								focus->removeFromGraph();
								dad->removeEdge(focus->getInverseEdge());
								dad->removeEdge(focus);

								mid_strand = true;

								focus = next;
							}
							else {
								focus = next;
							}
						}
						else {
							if (focus->getInverseEdge()->getFace() == target) {
								//strand continues

								//remove the last point
								dad->removePoint(focus->getStartPoint());

								focus->removeFromGraph();
								dad->removeEdge(focus->getInverseEdge());
								dad->removeEdge(focus);

								focus = next;
							}
							else {
								//strand ends
								strands.Push(focus);

								mid_strand = false;

								focus = next;
							}
						}

						if (focus == first_free) {
							made_loop = true;
						}
						if (made_loop && !mid_strand) {
							break;
						}
					} while (true);


					for (auto edge : strands) {
						edge->reFaceLoop(this);
						hole_edges.Push(edge);
					}

					dad->removeFace(target);
				}

				return true;
			}
		}

		return false;
	}
}