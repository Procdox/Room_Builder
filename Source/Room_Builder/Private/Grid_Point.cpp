#include "Grid_Point.h"

Pgrd::Pgrd() {
	X = 0;
	Y = 0;
}
Pgrd::Pgrd(grd x, grd y) {
	X = x;
	Y = y;
}

Pgrd Pgrd::operator+(const Pgrd &target) const {
	return Pgrd(X + target.X, Y + target.Y);
}
Pgrd Pgrd::operator-(const Pgrd &target) const {
	return Pgrd(X - target.X, Y - target.Y);
}
Pgrd Pgrd::operator*(double factor) const
{
	return Pgrd(X * factor, Y * factor);
}
Pgrd Pgrd::operator*(grd factor) const {
	return Pgrd(X * factor, Y * factor);
}
Pgrd Pgrd::operator*(const Pgrd &target) const {
	return Pgrd(X * target.X, Y * target.Y);
}
Pgrd Pgrd::operator/(double factor) const
{
	return Pgrd(X / factor, Y / factor);
}
Pgrd Pgrd::operator/(grd factor) const {
	return Pgrd(X / factor, Y / factor);
}
Pgrd Pgrd::operator/(const Pgrd &target) const {
	return Pgrd(X / target.X, Y / target.Y);
}

Pgrd& Pgrd::operator+=(const Pgrd &target) {
	X += target.X;
	Y += target.Y;
	return *this;
}
Pgrd& Pgrd::operator-=(const Pgrd &target) {
	X -= target.X;
	Y -= target.Y;
	return *this;
}
Pgrd & Pgrd::operator*=(double factor)
{
	X *= factor;
	Y *= factor;

	return *this;
}
Pgrd& Pgrd::operator*=(grd factor) {
	X *= factor;
	Y *= factor;

	return *this;
}
Pgrd& Pgrd::operator*=(const Pgrd &target) {
	X *= target.X;
	Y *= target.Y;
	return *this;
}
Pgrd & Pgrd::operator/=(double factor)
{
	X /= factor;
	Y /= factor;

	return *this;
}
Pgrd& Pgrd::operator/=(grd factor) {
	//check(X % div == 0);
	//check(Y % div == 0);
	X /= factor;
	Y /= factor;
	return *this;
}
Pgrd& Pgrd::operator/=(const Pgrd &target) {
	//check(X % div.X == 0);
	//check(Y % div.Y == 0);
	X /= target.X;
	Y /= target.Y;
	return *this;
}

bool Pgrd::operator==(const Pgrd &test) const {
	return test.X == X && test.Y == Y;
}
bool Pgrd::operator!=(const Pgrd &test) const {
	return test.X != X || test.Y != Y;
}
grd Pgrd::SizeSquared() const {
	return X * X + Y * Y;
}
grd Pgrd::Size() const {
	return SizeSquared().sqrt();
}

void Pgrd::Normalize() {
	grd const size = Size();
	if (size == 0) {
		return;
	}

	X /= size;
	Y /= size;
}

grd Pgrd::Dot(const Pgrd &b) const {
	return X * b.X + Y * b.Y;
}

Pgrd Pgrd::projectToSegment(Pgrd const & A, Pgrd const & B) const
{
	Pgrd const segment = B - A;
	Pgrd const perp(-segment.Y, segment.X);

	Pgrd result;

	Pgrd::getIntersect(A, B, *this, *this + perp, result);

	return result;
}

point_near_segment_state Pgrd::getState(const Pgrd &start, const Pgrd &end) const {
	if (*this == start) {
		return on_start;
	}
	if (*this == end) {
		return on_end;
	}

	const grd TWAT = (X * start.Y - Y * start.X) + (start.X * end.Y - start.Y * end.X) + (end.X * Y - end.Y * X);
	if (TWAT > 0) {
		return left_of_segment;
	}
	if (TWAT < 0) {
		return right_of_segment;
	}

	const grd target_size = (start - end).SizeSquared();
	if ((*this - end).SizeSquared() > target_size) {
		return before_segment;
	}
	if ((*this - start).SizeSquared() > target_size) {
		return after_segment;
	}

	return on_segment;
}

bool Pgrd::areParrallel(const Pgrd &A_S, const Pgrd &A_E, const Pgrd &B_S, const Pgrd &B_E) {
	const Pgrd A = A_E - A_S;
	const Pgrd A_R = A + B_S;

	//if A lies on B or after it they are parrallel
	auto state = A_R.getState(B_S, B_E);
	return (state == on_segment || state == on_end || state == after_segment);

}

bool Pgrd::isOnSegment(const Pgrd &test, const Pgrd &a, const Pgrd &b) {
	auto state = test.getState(a, b);
	return (state == on_segment || state == on_start || state == on_end);
}

bool Pgrd::inRegionCW(const Pgrd &test, const Pgrd &before, const Pgrd &corner, const Pgrd &after) {
	return (test.getState(before, corner) == right_of_segment && test.getState(corner, after) == right_of_segment);
}

bool Pgrd::getIntersect(const Pgrd &A_S, const Pgrd &A_E, const Pgrd &B_S, const Pgrd &B_E, Pgrd &Result) {
	const auto A = A_E - A_S;
	const auto B = B_E - B_S;

	const auto D = A_S - B_S;

	const auto denom = A.X * B.Y - A.Y * B.X;

	if (denom == 0) { //REFACTOR
		return false;
	}

	const auto s = (A.X * D.Y - A.Y * D.X) / denom;
	const auto t = (B.X * D.Y - B.Y * D.X) / denom;

	Result = (A * t) + A_S;

	return (s >= 0 && s <= 1 && t >= 0 && t <= 1);
}

grd Pgrd::area(FLL<Pgrd> const & boundary) {
	grd total = 0;

	Pgrd A = boundary.last();

	for (auto B : boundary) {

		grd width = B.X - A.X;
		grd avg_height = (A.Y + B.Y) / 2;

		total += width * avg_height;

		A = B;
	}

	return total;
}

grd linear_offset(Pgrd const &A, Pgrd const &B) {

	grd denom = (A.X * A.X + A.Y * A.Y);
	grd top = (A.Y * B.Y + A.X * B.X);

	return top / denom;
}
