#include "Grid_Tools.h"
#include "CoreMinimal.h"

//==========================================================================================================
//========================================== transforms ====================================================
//==========================================================================================================

FVector2D convert(Pgrd const &target) {
	return FVector2D(target.X.n * 10, target.Y.n * 10);
}

TArray<FVector2D> convert(FLL<Pgrd> const &target) {
	TArray<FVector2D> result;

	for (auto x : target)
		result.Push(convert(x));

	return result;
}

TArray<FVector2D> toFVector(FLL<Pgrd> const &target) {
	TArray<FVector2D> product;

	for (auto point : target) {
		product.Push(convert(point));
	}

	return product;
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

	grd minDiameter(FLL<Edge<Pgrd> *> const &relevants) {
		//if no pair is small enough to split, add to ins and return
		grd diameter = 0;
		bool found = false;
		grd min_offset, max_offset;

		//check region size
		for (auto edge : relevants) {

			Pgrd const A_start = edge->getEnd()->getPosition();
			Pgrd const A_before = edge->getStart()->getPosition();
			Pgrd const A = A_start - A_before;

			min_offset = linear_offset(A, A_start);
			max_offset = min_offset;

			for (auto compare : relevants) {
				Pgrd const B_start = compare->getStart()->getPosition();

				grd const raw = linear_offset(A, B_start);

				if (min_offset > raw)
					min_offset = raw;

				if (max_offset < raw)
					max_offset = raw;
			}

			grd const offset = max_offset - min_offset;

			if (offset < diameter || !found) {
				found = true;
				diameter = offset;
			}
		}
		return diameter;
	}

	grd minDiameter(Region<Pgrd> * target) {
		//if no pair is small enough to split, add to ins and return

		FLL<Edge<Pgrd> *> relevants;

		for (auto border : target->getBounds()) {
			border->getLoopEdges(relevants);
		}

		return minDiameter(relevants);
	}

	void chord_clean(Region<Pgrd> * target, grd const & thresh, Region_List & sections) {
		//if no pair is small enough to split, add to ins and return

		FLL<Edge<Pgrd> *> relevants;

		for (auto border : target->getBounds()) {
			border->getLoopEdges(relevants);
		}

		if (minDiameter(relevants) < thresh) {

			sections.push(target);
			return;
		}

		split_canidate result;
		bool found_option = false;

		for (auto edge : relevants) {

			Pgrd const A_start = edge->getEnd()->getPosition();
			Pgrd const A_before = edge->getStart()->getPosition();
			Pgrd const A_after = edge->getNext()->getEnd()->getPosition();

			//Pgrd const A = A_start - A_before;

			Pgrd const A_last = A_before - A_start;
			Pgrd const A_next = A_after - A_start;

			//only clip concave corners
			{
				Pgrd const inner(A_last.Y, -A_last.X);
				if (inner.Dot(A_next) <= 0)
					continue;
			}

			bvs const A_angle(A_next, A_last);

			for (auto compare : relevants) {
				Pgrd const B_start = compare->getStart()->getPosition();
				Pgrd const B_end = compare->getEnd()->getPosition();
				Pgrd const B_segment = B_end - B_start;

				if (compare == edge || compare == edge->getNext()) {
					continue;
				}

				//single convex corner cut
				/*
				if(A_angle.bounds_relation < 0){

					//project A_last onto B
					if(!Pgrd::areParrallel(A_before, A_start, B_start, B_end))
					{
						Pgrd intersect;
						Pgrd::getIntersect(A_before, A_start, B_start, B_end, intersect);
					
						Pgrd const segment = intersect - A_start;

						//point is after A
						if (A_last.Dot(segment) < 0) {

							//point lies on B
							if (B_segment.Dot(intersect - B_start) >= 0 && B_segment.Dot(intersect - B_end) <= 0) {
								grd const distance = segment.Size();

								if (distance < thresh)
									if (!found_option || distance < result.distance) {
										found_option = true;
										result.distance = distance;

										result.A = A_start;
										result.B = intersect;

										result.A_edge = edge;
										result.B_edge = compare;
									}
							}
						}
					}
				
					//project A_next onto B
					if (!Pgrd::areParrallel(A_after, A_start, B_start, B_end))
					{
						Pgrd intersect;
						Pgrd::getIntersect(A_after, A_start, B_start, B_end, intersect);
						Pgrd const segment = intersect - A_start;

						//point is after A
						if (A_next.Dot(segment) < 0) {

							//point lies on B
							if (B_segment.Dot(intersect - B_start) >= 0 && B_segment.Dot(intersect - B_end) <= 0) {
								grd const distance = segment.Size();

								if (distance < thresh)
									if (!found_option || distance < result.distance) {
										found_option = true;
										result.distance = distance;

										result.A = A_start;
										result.B = intersect;

										result.A_edge = edge;
										result.B_edge = compare;
									}
							}
						}
					}
				}
				*/

				//convex cut attempt

				{
					Pgrd const B_last = compare->getLast()->getStart()->getPosition() - B_start;
					bvs const B_angle(B_segment, B_last);
					//if A convex
					if (A_angle.bounds_relation < 0) {
						//	if B_start convex
						if (B_angle.bounds_relation < 0) {
							Pgrd segment = B_start - A_start;
							grd const distance = segment.Size();

							if (A_angle.test(segment))
								if(B_angle.test(segment * -1))
									if (distance < thresh)
										if (!found_option || distance < result.distance) {
											found_option = true;
											result.distance = distance;

											result.A = A_start;
											result.B = B_start;

											result.A_edge = edge;
											result.B_edge = compare;
										}
						}
					}
				}




				//get shortest distance (projection of A onto -B- )
				{
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
					else if ((A_start - B_end).Dot(B_start - B_end) <= 0) {
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

					grd const distance = segment.Size();

					if (distance < thresh)
						if (!found_option || distance < result.distance) {
							found_option = true;
							result.distance = distance;

							result.A = A_start;
							result.B = intersect;

							result.A_edge = edge;
							result.B_edge = compare;
						}
				}
			}
		}

		if (found_option) {
			if (result.B == result.B_edge->getStart()->getPosition()) {
				result.B_edge = result.B_edge->getLast();
			}
			else if (result.B != result.B_edge->getEnd()->getPosition()) {
				//in-line, subdivide
				result.B_edge->subdivide(result.B);
			}
			//split now occurs at end of A_edge and B_edge
			auto P = RegionAdd(target, result.A_edge, result.B_edge);

			chord_clean(target, thresh, sections);

			if (P != nullptr) {
				chord_clean(P, thresh, sections);
			}
		}
		else {
			sections.push(target);
		}
	}
}

void sortSmalls(Region_List &source, grd const &width, Region_List &smalls) {
	Region_List bigs;

	for (auto target : source) {
		grd diameter = chord_splits::minDiameter(target);
		UE_LOG(LogTemp, Warning, TEXT("D: %f"), diameter.n);
		if (diameter < width)
			smalls.append(target);
		else
			bigs.append(target);
	}

	source.clear();
	source.absorb(bigs);
}

void mergeGroup(Region_List & nulls) {
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

//cuts the region along any chord less than width, returns simple regions
Region_List Cut(Region<Pgrd> * target, grd const &width) {

	Region_List parts;

	chord_splits::chord_clean(target, width, parts);

	for (auto region : parts) {
		merge(region, region);
	}

	return parts;
}

//cuts the regions along any chord less than width, returns simple regions
void Cut(Region_List &targets, grd const &width) {

	Region_List parts;

	for (auto target : targets) {
		auto p = Cut(target, width);
		parts.absorb(p);
	}

	targets.clear();
	targets.absorb(parts);
}

void removeSmallSections(Region_List &target, grd const &min_width, Region_List &smalls) {
	mergeGroup(target);

	Cut(target, min_width);

	sortSmalls(target, min_width, smalls);

	mergeGroup(target);

	mergeGroup(smalls);
}

void sizeRestrict(Region_List &target, grd const &min_width, Region_List &outs) {
	Cut(target, min_width);

	sortSmalls(target, min_width, outs);

	for (auto region : target)
		merge(region, region);

	mergeGroup(outs);
}

Region_List allocateBoundaryFrom(FLL<Pgrd> const &boundary, Region_List &set) {
	Region_List _ins;
	Region_List _outs;

	for (auto member : set) {
		subAllocate(member, boundary, _outs, _ins);
	}

	mergeGroup(_ins);

	mergeGroup(_outs);

	set.clear();

	set.absorb(_outs);

	return _ins;
}
 
void allocateBoundaryFromInto(FLL<Pgrd> const &boundary, Region_List &set, Region_List &result) {
	Region_List _outs;

	for (auto member : set) {
		subAllocate(member, boundary, _outs, result);
	}

	mergeGroup(result);

	UE_LOG(LogTemp, Warning, TEXT("results"));
	for(auto a : result)
		for (auto bound : a->getBounds()) {
			UE_LOG(LogTemp, Warning, TEXT("face >r:"));
			for (auto point : bound->getLoopPoints()) {
				UE_LOG(LogTemp, Warning, TEXT("(%f,%f)"), point.X.n, point.Y.n);
			}
		}

	mergeGroup(_outs);

	set.clear();

	set.absorb(_outs);
}

Region_List allocateCleanedBoundaryFrom(FLL<Pgrd> const &boundary, grd const &min_width, Region_List &set) {
	Region_List _ins;
	Region_List _outs;

	for (auto member : set) {
		subAllocate(member, boundary, _outs, _ins);
	}

	mergeGroup(_ins);

	sizeRestrict(_ins, min_width, _outs);

	set.clear();

	set.absorb(_outs);

	return _ins;
}

void allocateCleanedBoundaryFromInto(FLL<Pgrd> const &boundary, grd const &min_width, Region_List &set, Region_List &result) {
	Region_List _outs;

	for (auto member : set) {
		subAllocate(member, boundary, _outs, result);
	}

	mergeGroup(result);

	sizeRestrict(result, min_width, _outs);

	set.clear();

	set.absorb(_outs);
}