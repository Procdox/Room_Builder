#include "Ratio.h"


int64 gcd(int64 a, int64 b) {
	if (a == 0)
		return b;
	return gcd(b % a, a);
}

rto::rto()
{
	n = 0;
	d = 1;
}

rto::rto(int64 num)
{
	n = num;
	d = 1;
}

rto::rto(int64 num, int64 den)
{
	n = num;
	d = den;
}

rto::rto(rto && target)
{
	n = target.n;
	d = target.d;
}

rto::rto(rto const & target) {
	n = target.n;
	d = target.d;
}

rto & rto::operator=(int64 const & target)
{
	n = target;
	d = 1;
	return *this;
}

rto & rto::operator=(rto const & target)
{
	n = target.n;
	d = target.d;

	return *this;
}

rto rto::operator-() const
{
	return rto(-n,d);
}

rto rto::operator+(int64 factor) const
{
	int64 x = n + factor * d;
	int64 y = d;

	int64 z = gcd(x, y);

	if (y / z < 0) {
		z *= -1;
	}

	return rto(x / z, y / z);
}

rto rto::operator+(const rto & target) const
{
	int64 g = gcd(target.d, d);

	int64 x = n * (target.d / g) + target.n * (d / g);
	int64 y = d * (target.d / g);

	int64 z = gcd(x, y);

	if (y / z < 0) {
		z *= -1;
	}

	return rto(x / z, y / z);
}

rto rto::operator-(int64 factor) const
{
	int64 x = n - factor * d;
	int64 y = d;

	int64 z = gcd(x, y);

	if (y / z < 0) {
		z *= -1;
	}

	return rto(x / z, y / z);
}

rto rto::operator-(const rto & target) const
{
	int64 g = gcd(target.d, d);

	int64 x = n * (target.d / g) - target.n * (d / g);
	int64 y = d * (target.d / g);

	int64 z = gcd(x, y);

	if (y / z < 0) {
		z *= -1;
	}

	return rto(x / z, y / z);
}

rto rto::operator*(int64 factor) const
{
	int64 x = n * factor;
	int64 y = d;

	int64 z = gcd(x, y);

	if (y / z < 0) {
		z *= -1;
	}

	return rto(x / z, y / z);
}

rto rto::operator*(const rto & target) const
{
	int64 x = n * target.n;
	int64 y = d * target.d;

	int64 z = gcd(x, y);

	if (y / z < 0) {
		z *= -1;
	}

	return rto(x / z, y / z);
}

rto rto::operator/(int64 factor) const
{
	int64 x = n;
	int64 y = d * factor;

	int64 z = gcd(x, y);

	if (y / z < 0) {
		z *= -1;
	}

	return rto(x / z, y / z);
}

rto rto::operator/(const rto & target) const
{
	int64 x = n * target.d;
	int64 y = d * target.n;

	int64 z = gcd(x, y);

	if (y / z < 0) {
		z *= -1;
	}

	return rto(x / z, y / z);
}

rto & rto::operator+=(int64 factor)
{
	int64 x = n + factor * d;
	int64 y = d;

	int64 z = gcd(x, y);

	n = x / z;
	d = y / z;

	if (d < 0) {
		n *= -1;
		d *= -1;
	}

	return *this;
}

rto & rto::operator+=(const rto & target)
{
	int64 g = gcd(target.d, d);

	int64 x = n * (target.d/g) + target.n * (d/g);
	int64 y = d * (target.d/g);

	int64 z = gcd(x, y);

	n = x / z;
	d = y / z;

	if (d < 0) {
		n *= -1;
		d *= -1;
	}

	return *this;
}

rto & rto::operator-=(int64 factor)
{
	int64 x = n - factor * d;
	int64 y = d;

	int64 z = gcd(x, y);

	n = x / z;
	d = y / z;

	if (d < 0) {
		n *= -1;
		d *= -1;
	}

	return *this;
}

rto & rto::operator-=(const rto & target)
{
	int64 g = gcd(target.d, d);

	int64 x = n * (target.d / g) - target.n * (d / g);
	int64 y = d * (target.d / g);

	int64 z = gcd(x, y);

	n = x / z;
	d = y / z;

	if (d < 0) {
		n *= -1;
		d *= -1;
	}

	return *this;
}

rto & rto::operator*=(int64 factor)
{
	int64 x = n * factor;
	int64 y = d;

	int64 z = gcd(x, y);

	n = x / z;
	d = y / z;

	if (d < 0) {
		n *= -1;
		d *= -1;
	}

	return *this;
}

rto & rto::operator*=(const rto & target)
{
	int64 x = n * target.n;
	int64 y = d * target.d;

	int64 z = gcd(x, y);

	n = x / z;
	d = y / z;

	if (d < 0) {
		n *= -1;
		d *= -1;
	}

	return *this;
}

rto & rto::operator/=(int64 factor)
{
	int64 x = n;
	int64 y = d * factor;

	int64 z = gcd(x, y);

	n = x / z;
	d = y / z;

	if (d < 0) {
		n *= -1;
		d *= -1;
	}

	return *this;
}

rto & rto::operator/=(const rto & target)
{
	int64 x = n * target.d;
	int64 y = d * target.n;

	int64 z = gcd(x, y);

	n = x / z;
	d = y / z;

	if (d < 0) {
		n *= -1;
		d *= -1;
	}

	return *this;
}

bool rto::operator==(const int64 & test) const
{
	return (n == test && d == 1);
}

bool rto::operator==(const rto & target) const
{
	return (n == target.n && d == target.d);
}

bool rto::operator!=(const int64 & test) const
{
	return (n != test || d != 1);
}

bool rto::operator!=(const rto & target) const
{
	return (n != target.n || d != target.d);
}

bool rto::operator>(const int64 & test) const
{
	return n > test * d;
}

bool rto::operator>(const rto & test) const
{
	return n * test.d > test.n * d;
}

bool rto::operator<(const int64 & test) const
{
	return n < test * d;
}

bool rto::operator<(const rto & test) const
{
	return n * test.d < test.n * d;
}

bool rto::operator>=(const int64 & test) const
{
	return n >= test * d;
}

bool rto::operator>=(const rto & test) const
{
	return n * test.d >= test.n * d;
}

bool rto::operator<=(const int64 & test) const
{
	return n <= test * d;
}

bool rto::operator<=(const rto & test) const
{
	return n * test.d <= test.n * d;
}

float rto::toFloat() const
{
	return (float)n / (float)d;
}
