#pragma once
#include "Ratio.h"
#include "FLL.h"
typedef signed long long int64;
/*

Contains defintion for an integer bound 2d vector.
DCEL_types operate independent of spacial representation. You may substitute your own depending on your needs.
DCEL_region requires this representation as currently.

*/

enum point_near_segment_state { left_of_segment, right_of_segment, before_segment, after_segment, on_start, on_end, on_segment };
enum intersect_state {no_intersect, at_start_of_a, at_start_of_b, };


struct Pint {
	rto X;
	rto Y;

	Pint();
	Pint(rto x, rto y);
	//Pint(Pint &&target);
	//Pint(Pint const &target);
	//Pint & operator=(Pint const & target);

	Pint operator+(const Pint &target) const;
	Pint operator-(const Pint &target) const;
	Pint operator*(int64 factor) const;
	Pint operator*(rto factor) const;
	Pint operator*(const Pint &target) const;
	Pint operator/(int64 factor) const;
	Pint operator/(rto factor) const;
	Pint operator/(const Pint &target) const;

	Pint& operator+=(const Pint &target);
	Pint& operator-=(const Pint &target);
	Pint& operator*=(int64 factor);
	Pint& operator*=(rto factor);
	Pint& operator*=(const Pint &target);
	Pint& operator/=(int64 factor);
	Pint& operator/=(rto factor);
	Pint& operator/=(const Pint &target);

	bool operator==(const Pint &test) const;
	bool operator!=(const Pint &test) const;
	rto SizeSquared() const;
	float Size() const;
	//void Normalize() {
	//	const rto sizeSq = SizeSquared();
	//	const rto sizeSq = SizeSquared();
	//	if (sizeSq == 0) {
	//		return;
	//	}
	//	X /= size;
	//	Y /= size;
	//}

	rto Dot(const Pint &b) const;

	point_near_segment_state getState(const Pint &start, const Pint &end) const;

	static bool areParrallel(const Pint &A_S, const Pint &A_E, const Pint &B_S, const Pint &B_E);

	static bool isOnSegment(const Pint &test, const Pint &a, const Pint &b);

	static bool inRegionCW(const Pint &test, const Pint &before, const Pint &corner, const Pint &after);

	static bool getIntersect(const Pint &A_S, const Pint &A_E, const Pint &B_S, const Pint &B_E, Pint &Result);

	static rto area(FLL<Pint> const &boundary);

};

struct PBox {
	Pint Min;
	Pint Max;

	Pint getExtent() const {
		return (Max - Min) / 2;
	}
	Pint getCenter() const {
		return (Max + Min) / 2;
	}
};

rto linear_offset(Pint &A, Pint &B);