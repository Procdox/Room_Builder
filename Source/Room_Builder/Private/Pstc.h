#pragma once
#include "FLL.h"
typedef signed long long int64;
/*

Contains defintion for an integer bound 2d vector.
DCEL_types operate independent of spacial representation. You may substitute your own depending on your needs.
DCEL_region requires this representation as currently.

*/

enum point_near_segment_state { left_of_segment, right_of_segment, before_segment, after_segment, on_start, on_end, on_segment };
enum intersect_state { no_intersect, at_start_of_a, at_start_of_b, };


struct Pstc {
	int64 X;
	int64 Y;

	Pstc();
	Pstc(int64 x, int64 y);
	//Pstc(Pstc &&target);
	//Pstc(Pstc const &target);
	//Pstc & operator=(Pstc const & target);

	Pstc operator+(const Pstc &target) const;
	Pstc operator-(const Pstc &target) const;
	Pstc operator*(int64 factor) const;
	Pstc operator*(const Pstc &target) const;
	Pstc operator/(int64 factor) const;
	Pstc operator/(const Pstc &target) const;

	Pstc& operator+=(const Pstc &target);
	Pstc& operator-=(const Pstc &target);
	Pstc& operator*=(int64 factor);
	Pstc& operator*=(const Pstc &target);
	Pstc& operator/=(int64 factor);
	Pstc& operator/=(const Pstc &target);

	bool operator==(const Pstc &test) const;
	bool operator!=(const Pstc &test) const;

	int64 SizeSquared() const;
	//int64 Size() const;

	//void Normalize();

	int64 Dot(const Pstc &b) const;

	point_near_segment_state getState(const Pstc &start, const Pstc &end) const;

	static bool areParrallel(const Pstc &A_S, const Pstc &A_E, const Pstc &B_S, const Pstc &B_E);

	static bool isOnSegment(const Pstc &test, const Pstc &a, const Pstc &b);

	static bool inRegionCW(const Pstc &test, const Pstc &before, const Pstc &corner, const Pstc &after);

	static bool getIntersect(const Pstc &A_S, const Pstc &A_E, const Pstc &B_S, const Pstc &B_E, Pstc &Result);

	static int64 area(FLL<Pstc> const &boundary);

};

struct PBox {
	Pstc Min;
	Pstc Max;

	Pstc getExtent() const {
		return (Max - Min) / 2;
	}
	Pstc getCenter() const {
		return (Max + Min) / 2;
	}
};

int64 linear_offset(Pstc const &A, Pstc const &B);