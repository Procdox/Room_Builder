#include "Ratio_Region.h"
#include <cmath>

#ifdef using_unreal
#include "CoreMinimal.h"
#endif


FaceRelation const getPointRelation(Face<Pint> & rel, Pint const &test_point) {
	Edge<Pint> * focus = rel.getRoot();
	int count = 0;

	bool is_best = false;
	rto best_distance = 0;
	bool inside = Pint::area(rel.getLoopPoints()) < 0;

	do {
		const Pint &start_vector = focus->getStart()->getPosition();
		const Pint &end_vector = focus->getEnd()->getPosition();

		//does it sit on
		rto y_length = end_vector.Y - start_vector.Y;
		rto y_offset = test_point.Y - start_vector.Y;

		if (y_length == 0) {
			if (y_offset == 0) {
				if ((start_vector.X <= test_point.X && test_point.X <= end_vector.X) ||
					(start_vector.X >= test_point.X && test_point.X >= end_vector.X)) {

					return FaceRelation(FaceRelationType::point_on_boundary, focus);
				}
			}
			focus = focus->getNext();
			continue;
		}
		else{
			rto ratio = y_offset / y_length;
			if (ratio <= 1 && ratio >= 0) {
				rto x_length = end_vector.X - start_vector.X;

				rto x = start_vector.X + x_length * ratio;
				rto distance = test_point.X - x;

				if (distance == 0) {
					return FaceRelation(FaceRelationType::point_on_boundary, focus);
				}
				else if (distance > 0 && (distance < best_distance || !is_best)) {
					is_best = true;
					best_distance = distance;

					inside = y_length > 0;
				}
			}
		}
		focus = focus->getNext();
	} while (focus != rel.getRoot());

	if (inside) {
		return FaceRelation(FaceRelationType::point_interior, nullptr);
	}
	else {
		return FaceRelation(FaceRelationType::point_exterior, nullptr);
	}
}

FaceRelationType const getPointRelation(FLL<Pint> const & rel, Pint const &test_point) {
	int count = 0;

	bool is_best = false;
	rto best_distance = 0;
	bool inside = Pint::area(rel) < 0;

	for (auto start = rel.begin(); start != rel.end(); ++start) {
		Pint start_vector = *start;
		Pint end_vector = *start.cyclic_next();

		//does it sit on
		rto y_length = end_vector.Y - start_vector.Y;
		rto y_offset = test_point.Y - start_vector.Y;

		if (y_length == 0) {
			if (y_offset == 0) {
				if ((start_vector.X <= test_point.X && test_point.X <= end_vector.X) ||
					(start_vector.X >= test_point.X && test_point.X >= end_vector.X)) {

					return FaceRelationType::point_on_boundary;
				}
			}
			continue;
		}
		else {
			rto ratio = y_offset / y_length;
			if (ratio <= 1 && ratio >= 0) {
				rto x_length = end_vector.X - start_vector.X;

				rto x = start_vector.X + x_length * ratio;
				rto distance = test_point.X - x;

				if (distance == 0) {
					return FaceRelationType::point_on_boundary;
				}
				else if (distance > 0 && (distance < best_distance || !is_best)) {
					is_best = true;
					best_distance = distance;

					inside = y_length > 0;
				}
			}
		}
	}

	if (inside) {
		return FaceRelationType::point_interior;
	}
	else {
		return FaceRelationType::point_exterior;
	}
}

FaceRelation contains(Region<Pint> * target, Pint const & test_point) {
	for(auto focus : target->getBounds()) {

		FaceRelation result = getPointRelation(*focus, test_point);
		if (result.type != FaceRelationType::point_interior) {
			return result;
		}
	}

	return FaceRelation(FaceRelationType::point_interior, nullptr);
}

bool merge(Region<Pint> * a, Region<Pint> * b) {
	//since both areas are continuous, its trivial that only one or no boundary pair can touch
	
	//regions are either strictly internal, strictly external, or weakly external to boundaries

	//if a boundary contains any part of a region, it can't touch that region (
	
	Face<Pint> * local_face = nullptr;
	Face<Pint> * target_face = nullptr;
	for(auto focus_local : a->getBounds()) {

		auto neighbors = focus_local->getNeighbors();

		for (auto focus_target : b->getBounds()) {

			if (neighbors.contains(focus_target)) {
				local_face = focus_local;
				target_face = focus_target;
				break;
			}
		}

		if (local_face != nullptr) {
			break;
		}
	}

	if (local_face == nullptr) return false;

	//we have found a boundary pair that neighbors one another, merge them

	auto tba = local_face->mergeWithFace(target_face);

	//now edit the boundary lists of both regions
	a->remove(local_face);

	for (auto face : tba) {
		b->append(face);
	}

	a->clear();

	return true;
}

struct intersect {

	Pint location;
	Edge<Pint>* mark;
	rto distance;
};

bool intersectSort(intersect *a, intersect *b) {
	return a->distance > b->distance;
}

//returns a list of intersects sorted by distance
FLL<intersect *> findIntersects(Pint const & start, Pint const & stop,
	FLL<Edge<Pint> *> const & canidates) {

	//detect intersect
	FLL<intersect *> product;

	for(auto target : canidates) {

		Pint intersect_location;

		Pint test_start = target->getStart()->getPosition();
		Pint test_stop = target->getEnd()->getPosition();

		bool valid = Pint::getIntersect(start, stop, test_start, test_stop, intersect_location);

		if (valid) {
			intersect * output = new intersect();

			output->location = intersect_location;
			output->mark = target;
			output->distance = (intersect_location - start).SizeSquared();

			product.qInsert(output, intersectSort);

		}
		else {
			//parrallel test
			Pint a = stop - start;
			Pint b = test_stop - test_start;

			rto x, y;
			if (a.Y != 0 && b.Y != 0) {
				x = a.X / a.Y;
				y = b.X / b.Y;
			}
			else if(a.Y == 0 && b.Y == 0){
				x = a.X / a.X;
				y = b.X / b.X;
			}

			if (x == y || x == -y) {

				//create an interesect for the ends of each segment, that lie on the other segment
				if (Pint::isOnSegment(start, test_start, test_stop)) {
					intersect * output = new intersect();

					output->location = start;
					output->mark = target;
					output->distance = 0;

					product.qInsert(output, intersectSort);
				}

				if (Pint::isOnSegment(stop, test_start, test_stop)) {
					intersect * output = new intersect();

					output->location = stop;
					output->mark = target;
					output->distance = (stop - start).SizeSquared();

					product.qInsert(output, intersectSort);
				}

				if (Pint::isOnSegment(test_start, start, stop) && test_start != start && test_start != stop) {
					intersect * output = new intersect();

					output->location = test_start;
					output->mark = target;
					output->distance = (test_start - start).SizeSquared();

					product.qInsert(output, intersectSort);
				}

				if (Pint::isOnSegment(test_stop, start, stop) && test_stop != start && test_stop != stop) {
					intersect * output = new intersect();

					output->location = test_stop;
					output->mark = target;
					output->distance = (test_stop - start).SizeSquared();

					product.qInsert(output, intersectSort);
				}
			}
		}
	}


	return product;
}

//finds interact features for a suballocation, and subidivides region edges where needed
//returns true if boundary is entirely external
bool markRegion(Region<Pint> * target, FLL<Pint> const & boundary, FLL<interact *>  & details) {
	
	bool exterior = true;

	{
		FLL<Edge<Pint> *> canidates;

		for(auto canidate_focus : target->getBounds()) {
			auto tba = canidate_focus->getLoopEdges(); //TODO: rvalues
			canidates.absorb(tba);
		}

		auto last = boundary.last();
		for(auto next : boundary) {
			//find and perform on all intersects

			auto intersects = findIntersects(last, next, canidates);

			bool end_collision = false;

			for(auto const & intersect_focus : intersects) {

				auto mark = intersect_focus->mark;

				//ignore hits at the start of either segment
				if (intersect_focus->location != last && intersect_focus->location != mark->getStart()->getPosition()) {


					interact* feature = new interact();

					feature->location = intersect_focus->location;
					feature->type = FaceRelationType::point_on_boundary;


					if (intersect_focus->location == mark->getEnd()->getPosition()) {
						//no need to subdivide

						feature->mark = mark;
					}
					else {
						//subdivide mark

						mark->getInv()->subdivide(intersect_focus->location);

						feature->mark = mark->getLast();

						canidates.push(feature->mark);
					}

					if (intersect_focus->location == next) {
						//prevents duplicate features for ends of segments
						end_collision = true;
					}

					exterior = false;
					details.append(feature);
				}
			}

			if (!end_collision) {

				auto state = contains(target, next);



				interact* feature = new interact();

				feature->location = next;
				feature->type = state.type;
				feature->mark = state.relevant;

				exterior = exterior && (state.type == FaceRelationType::point_exterior);
				details.append(feature);
			}

			last = next;
		}
	}

	//calculate mid inclusion

	{
		auto last = details.last();
		for(auto next : details) {
			last->mid_location = (last->location + next->location) / 2;

			auto result = contains(target, last->mid_location);

			last->mid_type = result.type;

			last = next;
		}
	}
	
	return exterior;
}

//returns if test is between A and B clockwise (right about the origin from A, left about from B)
bool between(Pint A, Pint B, Pint test) {

	Pint A_inward(A.Y, -A.X);
	Pint B_inward(-B.Y, B.X);

	rto bounds_relation = A_inward.Dot(B);

	if (bounds_relation > 0) {
		//the angle between bounds is in (0,180)
		return A_inward.Dot(test) >= 0 && B_inward.Dot(test) >= 0;
	}
	else if (bounds_relation == 0) {
		//the angle between bounds is 180 or 0, or one bound is length 0
		//any case other than 180 is due to an error as used for determine interiors

		return A_inward.Dot(test) >= 0;
	}
	else {
		//the angle between bounds is in (180,360)
		return A_inward.Dot(test) >= 0 || B_inward.Dot(test) >= 0;
	}
}

//insert strands into target, and determine face inclusions
void determineInteriors(Region<Pint> * target, FLL<interact *> & details,
	FLL<Face<Pint> *> & exteriors, FLL<Face<Pint> *> & interiors) {

	exteriors = target->getBounds();

	auto last = details.begin();
	auto next = last.cyclic_next();
	

	//consider first segment, if entirely internal, we need to create an edge from scratch
	if (last->type == FaceRelationType::point_interior) {

		auto into = *next;
		auto from = *last;

		if (into->type == FaceRelationType::point_interior) {

			into->mark = target->getUni()->addEdge(from->location, into->location);

			from->mark = into->mark->getInv();
		}
		else if (into->type == FaceRelationType::point_on_boundary) {

			from->mark = target->getUni()->addEdge(into->mark, from->location);

			into->mark = from->mark->getInv();
		}

		from->type = FaceRelationType::point_on_boundary;

		++last;
	}

	while (last != details.end()) {
		next = last.cyclic_next();

		auto into = *next;
		auto from = *last;

		if (from->type != FaceRelationType::point_exterior) {
			if (into->type == FaceRelationType::point_interior) {

				into->mark = target->getUni()->addEdge(from->mark, into->location);

			}
			else if (into->type == FaceRelationType::point_on_boundary) {
				if (from->mid_type == FaceRelationType::point_interior) {
					//if (from->mark->getNext() != into->mark) {
					if (into->mark->getFace() == from->mark->getFace()) {

						exteriors.remove(into->mark->getFace());

						auto created = target->getUni()->addEdge(from->mark, into->mark);

						//dot(mid-created_end, (next_end-created_end).cw(90) ) > 0 and dot(mid-created_end, (created_start-created_end).ccw(90) )
						Pint next_vector = created->getNext()->getEnd()->getPosition() - into->location;
						Pint created_vector = created->getStart()->getPosition() - into->location;
						Pint orientation = into->mid_location - into->location;

						if (between(next_vector, created_vector, orientation))
							into->mark = created;

						if (!interiors.contains(created->getFace()))
							interiors.push(created->getFace());

						exteriors.push(created->getInv()->getFace());
					}
					else {
						exteriors.remove(into->mark->getFace());

						into->mark = target->getUni()->addEdge(from->mark, into->mark);
					}
					//}
				}
			}
		}

		++last;
	}

	//does the entire boundary lie on a loop
	if (interiors.empty()) {
		auto from = *details.begin();


		if (from->type == FaceRelationType::point_on_boundary) {
			auto target = from->mark->getFace();

			exteriors.remove(target);

			interiors.push(target);
		}
	}
}

void subAllocate(Region<Pint> * target, FLL<Pint> const & boundary,
	FLL<Region<Pint> *> & exteriors, FLL<Region<Pint> *> & interiors) {

#ifdef using_unreal
	UE_LOG(LogTemp, Warning, TEXT("SA"));
	UE_LOG(LogTemp, Warning, TEXT("Boundary"));
	for (auto point : boundary) {
		UE_LOG(LogTemp, Warning, TEXT("(%f,%f)"),point.X.toFloat(),point.Y.toFloat());
	}

	for (auto face : target->getBounds()) {
		UE_LOG(LogTemp, Warning, TEXT("Face"));
		for (auto point : face->getLoopPoints()) {
			UE_LOG(LogTemp, Warning, TEXT("(%f,%f)"), point.X.toFloat(), point.Y.toFloat());
		}
	}
#endif
	//subdivide all edges based on intersects
	//this means all boundary edges are either
	//exterior
	//on point (with previous edge noted)
	//interior
	//subdivides are performed on inverse to preserve marks

	FLL<interact *> details;

	bool exterior = markRegion(target, boundary, details);

	FLL<Face<Pint> *> exterior_faces;
	FLL<Face<Pint> *> interior_faces;

#ifdef using_unreal
	for (auto detail : details) {
		if (detail->type == FaceRelationType::point_exterior) {
			UE_LOG(LogTemp, Warning, TEXT("(%f,%f) : exterior"), detail->location.X.toFloat(), detail->location.Y.toFloat());
		}else if(detail->type == FaceRelationType::point_interior) {
			UE_LOG(LogTemp, Warning, TEXT("(%f,%f) : interior"), detail->location.X.toFloat(), detail->location.Y.toFloat());
		}
		else {
			UE_LOG(LogTemp, Warning, TEXT("(%f,%f) : bound"), detail->location.X.toFloat(), detail->location.Y.toFloat());
		}

	}
#endif

	if (exterior) {

		auto test = (*target->getBounds().begin())->getRoot()->getStart()->getPosition();
		if (getPointRelation(boundary, test) == FaceRelationType::point_interior) {

			interiors.push(target);
		}
		else {
			exteriors.push(target);
		}
	}
	else {
		determineInteriors(target, details, exterior_faces, interior_faces);

	//find regions, place holes

	//determine clockwise faces
	//determine clockwise containment tree
	//insert counterclockwise containment at deepest symmetric level

	//determine interior face + exterior face sets with universal containment, create regions out of these
	//determine exterior face sets with universal containment, create regions out of these\

	//for each interior face, create a region, add any symmetricly contained exterior faces to that region
		for(auto interior_face : interior_faces) {
			Region<Pint> * novel = target->getUni()->region();

			auto interior_root = interior_face->getRoot()->getStart()->getPosition();

			getPointRelation(*interior_face, interior_root);

			for (auto exterior_focus = exterior_faces.begin(); exterior_focus != exterior_faces.end();) {

				auto exterior_face = *exterior_focus;
				auto exterior_root = exterior_face->getRoot()->getStart()->getPosition();

				auto ex_contains_in = getPointRelation(*exterior_face, interior_root);
				auto in_contains_ex = getPointRelation(*interior_face, exterior_root);

				++exterior_focus;

				if (ex_contains_in.type == FaceRelationType::point_interior && in_contains_ex.type == FaceRelationType::point_interior) {
					novel->append(exterior_face);

					exterior_faces.remove(exterior_face);
				}
			}

			novel->append(interior_face);

			interiors.push(novel);
		}

		//for each exterior face, see which faces are symmetric with it and create regions

		for (auto exterior_focus = exterior_faces.begin(); exterior_focus != exterior_faces.end(); ++exterior_focus) {
			Region<Pint> * novel = target->getUni()->region();

			auto base_face = *exterior_focus;
			auto base_root = base_face->getRoot()->getStart()->getPosition();

			for (auto compare = exterior_focus.next(); compare != exterior_faces.end();) {

				auto comp_face = *compare;
				auto comp_root = comp_face->getRoot()->getStart()->getPosition();

				auto comp_contains_base = getPointRelation(*comp_face, base_root);
				auto base_contains_comp = getPointRelation(*base_face, comp_root);

				++compare;

				if (comp_contains_base.type == FaceRelationType::point_interior && base_contains_comp.type == FaceRelationType::point_interior) {
					novel->append(comp_face);

					exterior_faces.remove(comp_face);
				}
			}

			novel->append(base_face);

			exteriors.push(novel);
		}

		target->getUni()->removeRegion(target);
	}
}

void cleanRegion(Region<Pint> * target) {
	for (auto border : target->getBounds()) {
		auto og_root = border->getRoot();
		auto focus = og_root;

		while(true) {
			auto next = focus->getNext();
			
			// mid point degree is two test
			if (focus->getInv()->getLast() == next->getInv()) {

				bool parallel = false;

				// parallel test
				Pint a = focus->getEnd()->getPosition() - focus->getStart()->getPosition();
				Pint b = next->getEnd()->getPosition() - next->getStart()->getPosition();

				if (a.Y != 0 && b.Y != 0) {
					rto x = a.X / a.Y;
					rto y = b.X / b.Y;
					parallel = x == y;
				}
				else if (a.Y == 0 && b.Y == 0) {
					parallel = true;
				}

				if (parallel) {
					next->getInv()->contract();
				}
			}

			if (next == og_root) {
				break;
			}

			focus = focus->getNext();
		}
	}
}
