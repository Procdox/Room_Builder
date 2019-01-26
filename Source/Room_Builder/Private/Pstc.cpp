#include "Pstc.h"

#define epsilon 32

Pstc::Pstc() {
	X = 0;
	Y = 0;
}
Pstc::Pstc(int64 x, int64 y) {
	X = x;
	Y = y;
}

Pstc Pstc::operator+(const Pstc &target) const {
	return Pstc(X + target.X, Y + target.Y);
}
Pstc Pstc::operator-(const Pstc &target) const {
	return Pstc(X - target.X, Y - target.Y);
}
Pstc Pstc::operator*(int64 factor) const
{
	return Pstc(X * factor, Y * factor);
}
Pstc Pstc::operator*(const Pstc &target) const {
	return Pstc(X * target.X, Y * target.Y);
}
Pstc Pstc::operator/(int64 factor) const
{
	return Pstc(X / factor, Y / factor);
}
Pstc Pstc::operator/(const Pstc &target) const {
	return Pstc(X / target.X, Y / target.Y);
}

Pstc& Pstc::operator+=(const Pstc &target) {
	X += target.X;
	Y += target.Y;
	return *this;
}
Pstc& Pstc::operator-=(const Pstc &target) {
	X -= target.X;
	Y -= target.Y;
	return *this;
}
Pstc & Pstc::operator*=(int64 factor)
{
	X *= factor;
	Y *= factor;

	return *this;
}
Pstc& Pstc::operator*=(const Pstc &target) {
	X *= target.X;
	Y *= target.Y;
	return *this;
}
Pstc & Pstc::operator/=(int64 factor)
{
	X /= factor;
	Y /= factor;

	return *this;
}
Pstc& Pstc::operator/=(const Pstc &target) {
	//check(X % div.X == 0);
	//check(Y % div.Y == 0);
	X /= target.X;
	Y /= target.Y;
	return *this;
}

bool Pstc::operator==(const Pstc &test) const {
	bool X_v = (X + epsilon > X && X - epsilon < X);
	bool Y_v = (Y + epsilon > Y && Y - epsilon < Y);
	return X_v && Y_v;
}
bool Pstc::operator!=(const Pstc &test) const {
	bool X_v = (X + epsilon < X || X - epsilon > X);
	bool Y_v = (Y + epsilon < Y || Y - epsilon > Y);
	return X_v || Y_v;
}
int64 Pstc::SizeSquared() const {
	return X * X + Y * Y;
}
//float Pstc::Size() const {
//	return FMath::Sqrt(SizeSquared()); //MARK FLOAT USED
//	return 0;
//}

//void Normalize() {
//	const rto sizeSq = SizeSquared();
//	const rto sizeSq = SizeSquared();
//	if (sizeSq == 0) {
//		return;
//	}
//	X /= size;
//	Y /= size;
//}

int64 Pstc::Dot(const Pstc &b) const {
	return X * b.X + Y * b.Y;
}

point_near_segment_state Pstc::getState(const Pstc &start, const Pstc &end) const {
	if (*this == start) {
		return on_start;
	}
	if (*this == end) {
		return on_end;
	}

	const int64 TWAT = (X * start.Y - Y * start.X) + (start.X * end.Y - start.Y * end.X) + (end.X * Y - end.Y * X);
	if (TWAT > epsilon) {
		return left_of_segment;
	}
	if (TWAT < -epsilon) {
		return right_of_segment;
	}

	const int64 target_size = (start - end).SizeSquared();
	if ((*this - end).SizeSquared() + epsilon > target_size) {
		return before_segment;
	}
	if ((*this - start).SizeSquared() + epsilon > target_size) {
		return after_segment;
	}

	return on_segment;
}

bool Pstc::areParrallel(const Pstc &A_S, const Pstc &A_E, const Pstc &B_S, const Pstc &B_E) {
	const Pstc A = A_E - A_S;
	const Pstc A_R = A + B_S;

	//if A lies on B or after it they are parrallel
	auto state = A_R.getState(B_S, B_E);
	return (state == on_segment || state == on_end || state == after_segment);

}

bool Pstc::isOnSegment(const Pstc &test, const Pstc &a, const Pstc &b) {
	auto state = test.getState(a, b);
	return (state == on_segment || state == on_start || state == on_end);
}

bool Pstc::inRegionCW(const Pstc &test, const Pstc &before, const Pstc &corner, const Pstc &after) {
	return (test.getState(before, corner) == right_of_segment && test.getState(corner, after) == right_of_segment);
}

bool Pstc::getIntersect(const Pstc &A_S, const Pstc &A_E, const Pstc &B_S, const Pstc &B_E, Pstc &Result) {
	const auto A = A_E - A_S;
	const auto B = B_E - B_S;

	const auto D = A_S - B_S;

	const auto denom = A.X * B.Y - A.Y * B.X;

	if (denom < epsilon && denom > -epsilon) { //REFACTOR
		return false;
	}

	const auto s = (A.X * D.Y - A.Y * D.X) / denom;
	const auto t = (B.X * D.Y - B.Y * D.X) / denom;

	Result = (A * t) + A_S;

	return (s + epsilon >= 0 && s - epsilon <= 1 && t + epsilon >= 0 && t - epsilon <= 1);
}

int64 Pstc::area(FLL<Pstc> const & boundary) {
	int64 total = 0;

	Pstc A = boundary.last();

	for (auto B : boundary) {

		int64 width = B.X - A.X;
		int64 avg_height = (A.Y + B.Y) / 2;

		total += width * avg_height;

		A = B;
	}

	return total;
}

int64 linear_offset(Pstc const &A, Pstc const &B) {

	int64 denom = (A.X * A.X + A.Y * A.Y);
	int64 top = (A.Y * B.Y + A.X * B.X);

	return top / denom;
}
