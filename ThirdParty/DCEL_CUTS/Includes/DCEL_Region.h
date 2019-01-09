#pragma once
#include "DCEL_point.h"
#include "DCEL_types.h"

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
	Edge<Pint> * mark;
	bool mid_interior;
	Pint mid;

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

class Region {
	FLL<Face<Pint> *> Boundaries;
	DCEL<Pint> * universe;

	friend void determineInteriors(Region *, FLL<interact *> &,
		FLL<Face<Pint> *> &, FLL<Face<Pint> *> &);

	friend bool markRegion(Region *, FLL<Pint> const &, FLL<interact *> &);


	friend void subAllocate(Region* target, FLL<Pint> const & boundary, FLL<Region*> &exteriors, FLL<Region *> & interiors);

public:
	Region(DCEL<Pint> *);
	Region(DCEL<Pint> *, FLL<Pint> const &);
	Region(DCEL<Pint> *, Face<Pint> *);

	FLL< Face<Pint> *> const * getBounds() const {
		return &Boundaries;
	}

	//returns nullptr if out of bounds
	Face<Pint> * operator[](int index) {
		return Boundaries[index];
	}

	int size() const {
		return Boundaries.size();
	}

	FaceRelation contains(Pint const & test_point) ;

	//if non-trivially connected, will absorb the target region into this one via face merging
	bool merge(Region*);

};


