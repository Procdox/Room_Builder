#include "Ratio_Point.h"

Pint::Pint() {
	X = 0;
	Y = 0;
}
Pint::Pint(rto x, rto y) {
	X = x;
	Y = y;
}

Pint Pint::operator+(const Pint &target) const {
	return Pint(X + target.X, Y + target.Y);
}
Pint Pint::operator-(const Pint &target) const {
	return Pint(X - target.X, Y - target.Y);
}
Pint Pint::operator*(int64 factor) const
{
	return Pint(X * factor, Y * factor);
}
Pint Pint::operator*(rto factor) const {
	return Pint(X * factor, Y * factor);
}
Pint Pint::operator*(const Pint &target) const {
	return Pint(X * target.X, Y * target.Y);
}
Pint Pint::operator/(int64 factor) const
{
	return Pint(X / factor, Y / factor);
}
Pint Pint::operator/(rto factor) const {
	return Pint(X / factor, Y / factor);
}
Pint Pint::operator/(const Pint &target) const {
	return Pint(X / target.X, Y / target.Y);
}

Pint& Pint::operator+=(const Pint &target) {
	X += target.X;
	Y += target.Y;
	return *this;
}
Pint& Pint::operator-=(const Pint &target) {
	X -= target.X;
	Y -= target.Y;
	return *this;
}
Pint & Pint::operator*=(int64 factor)
{
	X *= factor;
	Y *= factor;

	return *this;
}
Pint& Pint::operator*=(rto factor) {
	X *= factor;
	Y *= factor;

	return *this;
}
Pint& Pint::operator*=(const Pint &target) {
	X *= target.X;
	Y *= target.Y;
	return *this;
}
Pint & Pint::operator/=(int64 factor)
{
	X /= factor;
	Y /= factor;

	return *this;
}
Pint& Pint::operator/=(rto factor) {
	//check(X % div == 0);
	//check(Y % div == 0);
	X /= factor;
	Y /= factor;
	return *this;
}
Pint& Pint::operator/=(const Pint &target) {
	//check(X % div.X == 0);
	//check(Y % div.Y == 0);
	X /= target.X;
	Y /= target.Y;
	return *this;
}

bool Pint::operator==(const Pint &test) const {
	return test.X == X && test.Y == Y;
}
bool Pint::operator!=(const Pint &test) const {
	return test.X != X || test.Y != Y;
}
rto Pint::SizeSquared() const {
	return X * X + Y * Y;
}
float Pint::Size() const {
	//return FMath::Sqrt(SizeSquared()); //MARK FLOAT USED
	return 0;
}

//void Normalize() {
//	const rto sizeSq = SizeSquared();
//	const rto sizeSq = SizeSquared();
//	if (sizeSq == 0) {
//		return;
//	}
//	X /= size;
//	Y /= size;
//}

rto Pint::Dot(const Pint &b) const {
	return X * b.X + Y * b.Y;
}

point_near_segment_state Pint::getState(const Pint &start, const Pint &end) const {
	if (*this == start) {
		return on_start;
	}
	if (*this == end) {
		return on_end;
	}

	const rto TWAT = (X * start.Y - Y * start.X) + (start.X * end.Y - start.Y * end.X) + (end.X * Y - end.Y * X);
	if (TWAT > 0) {
		return left_of_segment;
	}
	if (TWAT < 0) {
		return right_of_segment;
	}

	const rto target_size = (start - end).SizeSquared();
	if ((*this - end).SizeSquared() > target_size) {
		return before_segment;
	}
	if ((*this - start).SizeSquared() > target_size) {
		return after_segment;
	}

	return on_segment;
}

bool Pint::areParrallel(const Pint &A_S, const Pint &A_E, const Pint &B_S, const Pint &B_E) {
	const Pint A = A_E - A_S;
	const Pint A_R = A + B_S;

	//if A lies on B or after it they are parrallel
	auto state = A_R.getState(B_S, B_E);
	return (state == on_segment || state == on_end || state == after_segment);

}

bool Pint::isOnSegment(const Pint &test, const Pint &a, const Pint &b) {
	auto state = test.getState(a, b);
	return (state == on_segment || state == on_start || state == on_end);
}

bool Pint::inRegionCW(const Pint &test, const Pint &before, const Pint &corner, const Pint &after) {
	return (test.getState(before, corner) == right_of_segment && test.getState(corner, after) == right_of_segment);
}

bool Pint::getIntersect(const Pint &A_S, const Pint &A_E, const Pint &B_S, const Pint &B_E, Pint &Result) {
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

rto Pint::area(FLL<Pint> const & boundary) {
	rto total = 0;

	Pint A = boundary.last();

	for(auto B : boundary){

		rto width = B.X - A.X;
		rto avg_height = (A.Y + B.Y) / 2;

		total += width * avg_height;

		A = B;
	}

	return total;
}