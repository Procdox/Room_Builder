#pragma once
#include "Grid_Point.h"
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
	Pgrd location;
	FaceRelationType type;

	FaceRelationType mid_type;
	Pgrd mid_location;

	Edge<Pgrd> * mark;

	interact() {
		mark = nullptr;
	}
};

//represents the relation a query point has to a face
//returned by getPointRelation
struct FaceRelation {
	//this can only be created by queris / copying 
	FaceRelation(FaceRelationType t, Edge<Pgrd> * e) {
		type = t;
		relevant = e;
	}
	FaceRelationType type;
	//if type is point_on_boundary, this is the edge that contains the point
	//root inclusive, end exclusive
	Edge<Pgrd> * relevant;
};

FaceRelation const getPointRelation(Face<Pgrd> &rel, Pgrd const &test_point);

bool merge(Region<Pgrd> * a, Region<Pgrd> * b);

void determineInteriors(Region<Pgrd> *, FLL<interact *> &, FLL<Face<Pgrd> *> &,
	FLL<Face<Pgrd> *> &);

bool markRegion(Region<Pgrd> *, FLL<Pgrd> const &, FLL<interact *> &);

//type dependent
void subAllocate(Region<Pgrd> * target, FLL<Pgrd> const & boundary,
	FLL<Region<Pgrd> *> &exteriors, FLL<Region<Pgrd> *> & interiors);

FaceRelation contains(Region<Pgrd> * target, Pgrd const & test_point);

FLL<Region<Pgrd> *> getNeighbors(Region<Pgrd> * target);

void cleanRegion(Region<Pgrd> * target);

//adds an edge between two edges within this region
	//fails if edges do not belong to this region
	//returns novel region if this splits the region
Region<Pgrd> * RegionAdd(Region<Pgrd> * target, Edge<Pgrd> * A, Edge<Pgrd> * B);