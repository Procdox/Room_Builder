#pragma once
#include "Grid.h"
#include "FLL.h"

/*

Contains defintion for an integer bound 2d vector.
DCEL_types operate independent of spacial representation. You may substitute your own depending on your needs.
DCEL_region requires this representation as currently.

*/

enum point_near_segment_state { left_of_segment, right_of_segment, before_segment, after_segment, on_start, on_end, on_segment };
enum intersect_state { no_intersect, at_start_of_a, at_start_of_b, };


struct Pgrd {
	grd X;
	grd Y;

	Pgrd();
	Pgrd(grd x, grd y);
	//Pgrd(Pgrd &&target);
	//Pgrd(Pgrd const &target);
	//Pgrd & operator=(Pgrd const & target);

	Pgrd operator+(const Pgrd &target) const;
	Pgrd operator-(const Pgrd &target) const;
	Pgrd operator*(double factor) const;
	Pgrd operator*(grd factor) const;
	Pgrd operator*(const Pgrd &target) const;
	Pgrd operator/(double factor) const;
	Pgrd operator/(grd factor) const;
	Pgrd operator/(const Pgrd &target) const;

	Pgrd& operator+=(const Pgrd &target);
	Pgrd& operator-=(const Pgrd &target);
	Pgrd& operator*=(double factor);
	Pgrd& operator*=(grd factor);
	Pgrd& operator*=(const Pgrd &target);
	Pgrd& operator/=(double factor);
	Pgrd& operator/=(grd factor);
	Pgrd& operator/=(const Pgrd &target);

	bool operator==(const Pgrd &test) const;
	bool operator!=(const Pgrd &test) const;

	grd SizeSquared() const;
	grd Size() const;

	void Normalize();

	grd Dot(const Pgrd &b) const;

	Pgrd projectToSegment(Pgrd const &A, Pgrd const &B) const;

	point_near_segment_state getState(const Pgrd &start, const Pgrd &end) const;

	static bool areParrallel(const Pgrd &A_S, const Pgrd &A_E, const Pgrd &B_S, const Pgrd &B_E);

	static bool isOnSegment(const Pgrd &test, const Pgrd &a, const Pgrd &b);

	static bool inRegionCW(const Pgrd &test, const Pgrd &before, const Pgrd &corner, const Pgrd &after);

	static bool getIntersect(const Pgrd &A_S, const Pgrd &A_E, const Pgrd &B_S, const Pgrd &B_E, Pgrd &Result);

	static grd area(FLL<Pgrd> const &boundary);
};

struct PBox {
	Pgrd Min;
	Pgrd Max;

	Pgrd getExtent() const {
		return (Max - Min) / 2;
	}
	Pgrd getCenter() const {
		return (Max + Min) / 2;
	}
};

grd linear_offset(Pgrd const &A, Pgrd const &B);