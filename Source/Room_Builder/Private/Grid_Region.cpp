#include "Grid_Region.h"
#include <cmath>

#define debug_suballocate
//#define debug_merge
//#define debug_clean

#if defined(debug_suballocate) || defined(debug_merge) || defined(debug_clean)
#define using_unreal
#endif

#ifdef using_unreal
#include "CoreMinimal.h"
#endif


FaceRelation const getPointRelation(Face<Pgrd> & rel, Pgrd const &test_point) {

	Edge<Pgrd> * focus = rel.getRoot();

	bool inside = Pgrd::area(rel.getLoopPoints()) < 0;

	do {
		const Pgrd &start_vector = focus->getStart()->getPosition();
		const Pgrd &end_vector = focus->getEnd()->getPosition();

		//does it sit on
		grd y_length = end_vector.Y - start_vector.Y;
		grd y_offset = test_point.Y - start_vector.Y;

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
		else {
			grd ratio = y_offset / y_length;
			if (ratio <= 1 && ratio >= 0) {
				grd x_length = end_vector.X - start_vector.X;

				grd x = start_vector.X + x_length * ratio;
				grd distance = test_point.X - x;

				if (distance == 0) {
					return FaceRelation(FaceRelationType::point_on_boundary, focus);
				}
				else if (distance > 0) {
					if (ratio == 1) {
						if (y_offset > 0)
							inside = !inside;
					}
					else if (ratio == 0) {
						if (y_length < 0)
							inside = !inside;
					}
					else {
						inside = !inside;
					}
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

FaceRelationType const getPointRelation(FLL<Pgrd> const & rel, Pgrd const &test_point) {

	bool inside = Pgrd::area(rel) < 0;

	for (auto start = rel.begin(); start != rel.end(); ++start) {
		Pgrd start_vector = *start;
		Pgrd end_vector = *start.cyclic_next();

		//does it sit on
		grd y_length = end_vector.Y - start_vector.Y;
		grd y_offset = test_point.Y - start_vector.Y;

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
			grd ratio = y_offset / y_length;
			if (ratio <= 1 && ratio >= 0) {
				grd x_length = end_vector.X - start_vector.X;

				grd x = start_vector.X + x_length * ratio;
				grd distance = test_point.X - x;

				if (distance == 0) {
					return FaceRelationType::point_on_boundary;
				}
				else if (distance > 0) {
					if (ratio == 1) {
						if (y_offset > 0)
							inside = !inside;
					}
					else if (ratio == 0) {
						if (y_length < 0)
							inside = !inside;
					}
					else {
						inside = !inside;
					}
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

FaceRelation contains(Region<Pgrd> * target, Pgrd const & test_point) {
	for (auto focus : target->getBounds()) {

		FaceRelation result = getPointRelation(*focus, test_point);
		if (result.type != FaceRelationType::point_interior) {
			return result;
		}
	}

	return FaceRelation(FaceRelationType::point_interior, nullptr);
}

bool merge(Region<Pgrd> * a, Region<Pgrd> * b) {
	//since both areas are continuous, its trivial that only one or no boundary pair can touch

	//regions are either strictly internal, strictly external, or weakly external to boundaries

	//if a boundary contains any part of a region, it can't touch that region (


	if (a != b) {


		Face<Pgrd> * local_face = nullptr;
		Face<Pgrd> * target_face = nullptr;
		for (auto focus_local : a->getBounds()) {

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

#ifdef debug_merge
		UE_LOG(LogTemp, Warning, TEXT("merging"));
		UE_LOG(LogTemp, Warning, TEXT("a"));
		for (auto bound : a->getBounds()) {
			UE_LOG(LogTemp, Warning, TEXT("face >k-"));
			for (auto point : bound->getLoopPoints()) {
				UE_LOG(LogTemp, Warning, TEXT("(%f,%f)"), point.X.n, point.Y.n);
			}
		}
		UE_LOG(LogTemp, Warning, TEXT("b"));
		for (auto bound : b->getBounds()) {
			UE_LOG(LogTemp, Warning, TEXT("face >b-"));
			for (auto point : bound->getLoopPoints()) {
				UE_LOG(LogTemp, Warning, TEXT("(%f,%f)"), point.X.n, point.Y.n);
			}
		}
#endif

		//we have found a boundary pair that neighbors one another, merge them

		a->remove(local_face);
		b->remove(target_face);

		auto tba = local_face->mergeWithFace(target_face);

		auto t(b->getBounds());
		for (auto face : t)
			a->append(face);

		for (auto face : tba)
			a->append(face);

#ifdef debug_merge
		UE_LOG(LogTemp, Warning, TEXT("result"));
		for (auto bound : a->getBounds()) {
			UE_LOG(LogTemp, Warning, TEXT("face >r:"));
			for (auto point : bound->getLoopPoints()) {
				UE_LOG(LogTemp, Warning, TEXT("(%f,%f)"), point.X.n, point.Y.n);
			}
		}
#endif

		return true;
	}
	else {
#ifdef debug_merge
		UE_LOG(LogTemp, Warning, TEXT("merging"));
		for (auto bound : a->getBounds()) {
			UE_LOG(LogTemp, Warning, TEXT("face >k:"));
			for (auto point : bound->getLoopPoints()) {
				UE_LOG(LogTemp, Warning, TEXT("(%f,%f)"), point.X.n, point.Y.n);
			}
		}
#endif

		auto bounds = a->getBounds();
		for (auto focus_local = bounds.begin(); focus_local != bounds.end();) {

			for (auto focus_compare = focus_local; focus_compare != bounds.end();) {

				if (focus_local->neighbors(*focus_compare)) {
					auto tba = focus_local->mergeWithFace(*focus_compare);

					for (auto face : tba)
						if (face != *focus_local)
							a->append(face);

					focus_compare = focus_local;
				}

				++focus_compare;
			}
			++focus_local;
		}
#ifdef debug_merge
		UE_LOG(LogTemp, Warning, TEXT("result"));
		for (auto bound : a->getBounds()) {
			UE_LOG(LogTemp, Warning, TEXT("face >r:"));
			for (auto point : bound->getLoopPoints()) {
				UE_LOG(LogTemp, Warning, TEXT("(%f,%f)"), point.X.n, point.Y.n);
			}
		}
#endif

		return false;
	}

}

struct intersect {

	Pgrd location;
	Edge<Pgrd>* mark;
	grd distance;
};

bool intersectSort(intersect *a, intersect *b) {
	return a->distance > b->distance;
}

//returns a list of intersects sorted by distance
FLL<intersect *> findIntersects(Pgrd const & start, Pgrd const & stop,
	FLL<Edge<Pgrd> *> const & canidates) {

	//detect intersect
	FLL<intersect *> product;

	for (auto target : canidates) {

		Pgrd intersect_location;

		Pgrd test_start = target->getStart()->getPosition();
		Pgrd test_stop = target->getEnd()->getPosition();

		bool valid = Pgrd::getIntersect(start, stop, test_start, test_stop, intersect_location);

		if (valid) {
			intersect * output = new intersect();

			output->location = intersect_location;
			output->mark = target;
			output->distance = (intersect_location - start).SizeSquared();

			product.qInsert(output, intersectSort);

		}
		else {
			//parrallel test
			Pgrd a = stop - start;
			Pgrd b = test_stop - test_start;

			grd x, y;
			if (a.Y != 0 && b.Y != 0) {
				x = a.X / a.Y;
				y = b.X / b.Y;
			}
			else if (a.Y == 0 && b.Y == 0) {
				x = a.X / a.X;
				y = b.X / b.X;
			}

			if (x == y || x == -y) {

				//create an interesect for the ends of each segment, that lie on the other segment
				if (Pgrd::isOnSegment(start, test_start, test_stop)) {
					intersect * output = new intersect();

					output->location = start;
					output->mark = target;
					output->distance = 0;

					product.qInsert(output, intersectSort);
				}

				if (Pgrd::isOnSegment(stop, test_start, test_stop)) {
					intersect * output = new intersect();

					output->location = stop;
					output->mark = target;
					output->distance = (stop - start).SizeSquared();

					product.qInsert(output, intersectSort);
				}

				if (Pgrd::isOnSegment(test_start, start, stop) && test_start != start && test_start != stop) {
					intersect * output = new intersect();

					output->location = test_start;
					output->mark = target;
					output->distance = (test_start - start).SizeSquared();

					product.qInsert(output, intersectSort);
				}

				if (Pgrd::isOnSegment(test_stop, start, stop) && test_stop != start && test_stop != stop) {
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
bool markRegion(Region<Pgrd> * target, FLL<Pgrd> const & boundary, FLL<interact *>  & details) {

	bool exterior = true;

	{
		FLL<Edge<Pgrd> *> canidates;

		for (auto canidate_focus : target->getBounds()) {
			auto tba = canidate_focus->getLoopEdges(); //TODO: rvalues
			canidates.absorb(tba);
		}

		auto last = boundary.last();
		for (auto next : boundary) {
			//find and perform on all intersects

			auto intersects = findIntersects(last, next, canidates);

			bool end_collision = false;

			for (auto const & intersect_focus : intersects) {

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
		for (auto next : details) {
			last->mid_location = (last->location + next->location) / 2;

			auto result = contains(target, last->mid_location);

			last->mid_type = result.type;

			last = next;
		}
	}

	return exterior;
}

//returns if test is between A and B clockwise (right about the origin from A, left about from B)
bool between(Pgrd const &A, Pgrd const &B, Pgrd const &test) {

	Pgrd A_inward(A.Y, -A.X);
	Pgrd B_inward(-B.Y, B.X);

	grd bounds_relation = A_inward.Dot(B);

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
void determineInteriors(Region<Pgrd> * target, FLL<interact *> & details,
	FLL<Face<Pgrd> *> & exteriors, FLL<Face<Pgrd> *> & interiors) {

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
						Pgrd next_vector = created->getNext()->getEnd()->getPosition() - into->location;
						Pgrd created_vector = created->getStart()->getPosition() - into->location;
						Pgrd orientation = into->mid_location - into->location;

						if (between(next_vector, created_vector, orientation))
							into->mark = created;

						if (!interiors.contains(created->getFace()))
							interiors.push(created->getFace());

						exteriors.push(created->getInv()->getFace());
					}
					else if (next == details.begin()) {
						auto relevant = into->mark->getNext()->getInv();
						exteriors.remove(relevant->getFace());

						auto created = target->getUni()->addEdge(from->mark, relevant);

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

void subAllocate(Region<Pgrd> * target, FLL<Pgrd> const & boundary,
	FLL<Region<Pgrd> *> & exteriors, FLL<Region<Pgrd> *> & interiors) {

#ifdef debug_suballocate
	UE_LOG(LogTemp, Warning, TEXT("SA"));
	UE_LOG(LogTemp, Warning, TEXT("Boundary >g:"));
	for (auto point : boundary) {
		UE_LOG(LogTemp, Warning, TEXT("(%f,%f)"), point.X.n, point.Y.n);
	}

	for (auto face : target->getBounds()) {
		UE_LOG(LogTemp, Warning, TEXT("Face >k-"));
		for (auto point : face->getLoopPoints()) {
			UE_LOG(LogTemp, Warning, TEXT("(%f,%f)"), point.X.n, point.Y.n);
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

	FLL<Face<Pgrd> *> exterior_faces;
	FLL<Face<Pgrd> *> interior_faces;

#ifdef debug_suballocate
	for (auto detail : details) {
		if (detail->type == FaceRelationType::point_exterior) {
			UE_LOG(LogTemp, Warning, TEXT("(%f,%f) : exterior"), detail->location.X.n, detail->location.Y.n);
		}
		else if (detail->type == FaceRelationType::point_interior) {
			UE_LOG(LogTemp, Warning, TEXT("(%f,%f) : interior"), detail->location.X.n, detail->location.Y.n);
		}
		else {
			UE_LOG(LogTemp, Warning, TEXT("(%f,%f) : bound"), detail->location.X.n, detail->location.Y.n);
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
		for (auto interior_face : interior_faces) {
			Region<Pgrd> * novel = target->getUni()->region();

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
			Region<Pgrd> * novel = target->getUni()->region();

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

void cleanRegion(Region<Pgrd> * target) {

#ifdef debug_clean
	UE_LOG(LogTemp, Warning, TEXT("Clean Region"));
#endif
	for (auto border : target->getBounds()) {
		auto og_root = border->getRoot();
		auto focus = og_root;

#ifdef debug_clean
		UE_LOG(LogTemp, Warning, TEXT("Face >k:"));
		for (auto point : border->getLoopPoints()) {
			UE_LOG(LogTemp, Warning, TEXT("(%f,%f)"), point.X.n, point.Y.n);
		}
#endif

		while (true) {
			auto next = focus->getNext();

			// mid point degree is two test
			if (focus->getInv()->getLast() == next->getInv()) {

				bool parallel = false;

				// parallel test

				Pgrd const start = focus->getStart()->getPosition();
				Pgrd const mid = focus->getEnd()->getPosition();
				Pgrd const end = next->getEnd()->getPosition();

				Pgrd const a = mid - start;
				Pgrd const b = end - mid;

				if (a.Y != 0 && b.Y != 0) {
					grd x = a.X / a.Y;
					grd y = b.X / b.Y;
					parallel = x == y;
				}
				else if (a.Y == 0 && b.Y == 0) {
					parallel = true;
				}

				if (parallel) {
#ifdef debug_clean
					UE_LOG(LogTemp, Warning, TEXT("contract >r:"));
					UE_LOG(LogTemp, Warning, TEXT("(%f,%f)"), start.X.n, start.Y.n);
					UE_LOG(LogTemp, Warning, TEXT("(%f,%f)"), mid.X.n, mid.Y.n);
					UE_LOG(LogTemp, Warning, TEXT("contract >g:"));
					UE_LOG(LogTemp, Warning, TEXT("(%f,%f)"), mid.X.n, mid.Y.n);
					UE_LOG(LogTemp, Warning, TEXT("(%f,%f)"), end.X.n, end.Y.n);
#endif
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

Region<Pgrd> * RegionAdd(Region<Pgrd> * target, Edge<Pgrd> * A, Edge<Pgrd> * B) {
	auto A_face = A->getFace();
	auto B_face = B->getFace();


	Region<Pgrd> * result = nullptr;

	if (A_face->getGroup() != target || B_face->getGroup() != target)
		return nullptr;

	if (B_face == A_face) {
		//we will be splitting the region

		target->getUni()->addEdge(A, B);

		B_face = B->getFace();

		result = target->getUni()->region();

		result->append(B_face);

		FLL<Face<Pgrd> *> transfers;

		for (auto edge : target->getBounds())
			if (edge == A_face)
				continue;
			else if (getPointRelation(*edge, B_face->getRoot()->getStart()->getPosition()).type == FaceRelationType::point_interior
				&& getPointRelation(*B_face, edge->getRoot()->getStart()->getPosition()).type == FaceRelationType::point_interior)
				transfers.append(edge);

		for (auto edge : transfers)
			result->append(edge);
	}
	else {
		//we will be connecting two boundaries
		target->remove(B_face);

		target->getUni()->addEdge(A, B);
	}

	return result;
}