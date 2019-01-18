#pragma once
#include "Ratio_Point.h"
#include "DCEL.h"

/*

Contains a definition for a specific DCEL representation that 
allows for continuous regional allocations in the discrete 2d plane

continuous implies that any boundary is "inside" every other boundary
therefor there can only be one clockwise boundary and any number of discrete counter
clockwise boundaries contained within the clockwise boundary.

*/

//the relational types between a query point and a face
enum FaceRelationType { point_exterior, point_on_boundary, point_interior };

struct interact {
	Pint location;
	FaceRelationType type;
	
	FaceRelationType mid_type;
	Pint mid_location;

	Edge<Pint> * mark;

	interact() {
		mark = nullptr;
	}
};

//represents the relation a query point has to a face
//returned by getPointRelation
struct FaceRelation {
	//this can only be created by queris / copying 
	FaceRelation(FaceRelationType t, Edge<Pint> * e) {
		type = t;
		relevant = e;
	}
	FaceRelationType type;
	//if type is point_on_boundary, this is the edge that contains the point
	//root inclusive, end exclusive
	Edge<Pint> * relevant;
};

FaceRelation const getPointRelation(Face<Pint> &rel, Pint const &test_point);

bool merge(Region<Pint> * a, Region<Pint> * b);

void determineInteriors(Region<Pint> *, FLL<interact *> &, FLL<Face<Pint> *> &, 
	FLL<Face<Pint> *> &);

bool markRegion(Region<Pint> *, FLL<Pint> const &, FLL<interact *> &);

//type dependent
void subAllocate(Region<Pint> * target, FLL<Pint> const & boundary, 
	FLL<Region<Pint> *> &exteriors, FLL<Region<Pint> *> & interiors);

FaceRelation contains(Region<Pint> * target, Pint const & test_point);

FLL<Region<Pint> *> getNeighbors(Region<Pint> * target);

void cleanRegion(Region<Pint> * target);

//adds an edge between two edges within this region
	//fails if edges do not belong to this region
	//returns novel region if this splits the region
Region<Pint> * RegionAdd(Region<Pint> * target, Edge<Pint> * A, Edge<Pint> * B);