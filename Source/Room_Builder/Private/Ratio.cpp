#include "Ratio.h"


int64 gcd(int64 a, int64 b) {
	if (a == 0)
		return b;
	return gcd(b % a, a);
}

rto::rto()
{
	o = 0;
	n = 0;
	d = 1;
}

rto::rto(int64 num)
{
	o = num;
	n = 0;
	d = 1;
}

rto::rto(int64 offset, int64 num, int64 den)
{
	o = offset;
	n = num;
	d = den;
}

rto::rto(rto && target)
{
	o = target.o;
	n = target.n;
	d = target.d;
}

rto::rto(rto const & target) {
	o = target.o;
	n = target.n;
	d = target.d;
}

rto & rto::operator=(int64 const & target)
{
	o = target;
	n = 0;
	d = 1;
	return *this;
}

rto & rto::operator=(rto const & target)
{
	o = target.o;
	n = target.n;
	d = target.d;

	return *this;
}

rto rto::operator-() const
{
	return rto(-o,-n,d);
}

rto rto::operator+(int64 factor) const
{
	return rto(o + factor, n, d);
}

rto rto::operator+(const rto & target) const
{
	int64 t = n * target.d + target.n * d;
	int64 y = target.d * d;

	int64 c = t / y;
	int64 x = t - c * y;

	int64 z = gcd(x, y);

	if (y / z < 0) {
		z *= -1;
	}
	x /= z;
	y /= z;

	return rto(o + target.o + c, x, y);
}

rto rto::operator-(int64 factor) const
{
	return rto(o - factor, n, d);
}

rto rto::operator-(const rto & target) const
{
	int64 t = n * target.d - target.n * d;
	int64 y = target.d * d;

	int64 c = t / y;
	int64 x = t - c * y;

	int64 z = gcd(x, y);

	if (y / z < 0) {
		z *= -1;
	}
	x /= z;
	y /= z;

	return rto(o - target.o + c, x, y);
}

rto rto::operator*(int64 factor) const
{
	int64 t = n * factor;
	int64 y = d;

	int64 c = t / y;
	int64 x = t - c * y;

	int64 z = gcd(x, y);

	if (y / z < 0) {
		z *= -1;
	}
	x /= z;
	y /= z;

	return rto(o * factor + c, x, y);
}

rto rto::operator*(const rto & target) const
{
	int64 t_a = o * target.n;

	int64 c_a = t_a / target.d;
	int64 r_a = t_a - c_a * target.d;

	int64 t_b = target.o * n;

	int64 c_b = t_b / d;
	int64 r_b = t_b - c_b * d;

	int64 t_c = r_a * d + r_b * target.d + n * target.n;
	int64 y = d * target.d;
	
	int64 c_c = t_c / y;
	int64 x = t_c - c_c * y;

	int64 z = gcd(x, y);

	if (y / z < 0) {
		z *= -1;
	}
	x /= z;
	y /= z;

	return rto(o*target.o + c_a + c_b + c_c, x, y);
}

rto rto::operator/(int64 factor) const
{
	int64 t = o * d + n;
	int64 y = d * factor;

	int64 c = t / y;
	int64 x = t - c * y;

	int64 z = gcd(x, y);

	if (y / z < 0) {
		z *= -1;
	}
	x /= z;
	y /= z;

	return rto(c, x, y);
}

rto rto::operator/(const rto & target) const
{
	int64 t_a = o * d + n;
	int64 t_b = target.o* target.d + target.n;

	int64 i = gcd(t_a, t_b);
	int64 j = gcd(d, target.d);

	int64 x = (t_a / i)*(target.d*j);
	int64 y = (t_b / i)*(d*j);

	int64 z = gcd(x, y);

	if (y / z < 0) {
		z *= -1;
	}
	x /= z;
	y /= z;

	int64 c = x / y;
	x -= c * y;

	return rto(c, x, y);

}

rto & rto::operator+=(int64 factor)
{
	o += factor;

	return *this;
}

rto & rto::operator+=(const rto & target)
{
	int64 t = n * target.d + target.n * d;
	int64 y = target.d * d;

	int64 c = t / y;
	int64 x = t - c * y;

	int64 z = gcd(x, y);

	if (y / z < 0) {
		z *= -1;
	}
	x /= z;
	y /= z;

	o = o + target.o + c;
	n = x;
	d = y;
	return *this;
}

rto & rto::operator-=(int64 factor)
{
	o -= factor;

	return *this;
}

rto & rto::operator-=(const rto & target)
{
	int64 t = n * target.d - target.n * d;
	int64 y = target.d * d;

	int64 c = t / y;
	int64 x = t - c * y;

	int64 z = gcd(x, y);

	if (y / z < 0) {
		z *= -1;
	}
	x /= z;
	y /= z;

	o = o - target.o + c;
	n = x;
	d = y;
	return *this;
}

rto & rto::operator*=(int64 factor)
{
	int64 t = n * factor;
	int64 y = d;

	int64 c = t / y;
	int64 x = t - c * y;

	int64 z = gcd(x, y);

	if (y / z < 0) {
		z *= -1;
	}
	x /= z;
	y /= z;

	o = o * factor + c;
	n = x;
	d = y;
	return *this;
}

rto & rto::operator*=(const rto & target)
{
	int64 t_a = o * target.n;

	int64 c_a = t_a / target.d;
	int64 r_a = t_a - c_a * target.d;

	int64 t_b = target.o * n;

	int64 c_b = t_b / d;
	int64 r_b = t_b - c_b * d;

	int64 t_c = r_a * d + r_b * target.d + n * target.n;
	int64 y = d * target.d;

	int64 c_c = t_c / y;
	int64 x = t_c - c_c * y;

	int64 z = gcd(x, y);

	if (y / z < 0) {
		z *= -1;
	}
	x /= z;
	y /= z;

	o = o * target.o + c_a + c_b + c_c;
	n = x;
	d = y;
	return *this;
}

rto & rto::operator/=(int64 factor)
{
	int64 t = o * d + n;
	int64 y = d * factor;

	int64 c = t / y;
	int64 x = t - c * y;

	int64 z = gcd(x, y);

	if (y / z < 0) {
		z *= -1;
	}
	x /= z;
	y /= z;

	o = c;
	n = x;
	d = y;
	return *this;
}

rto & rto::operator/=(const rto & target)
{
	int64 t_a = o * d + n;
	int64 t_b = target.o* target.d + target.n;

	int64 i = gcd(t_a, t_b);
	int64 j = gcd(d, target.d);

	int64 x = (t_a / i)*(target.d*j);
	int64 y = (t_b / i)*(d*j);

	int64 z = gcd(x, y);

	if (y / z < 0) {
		z *= -1;
	}
	x /= z;
	y /= z;

	int64 c = x / y;
	x -= c * y;

	o = c;
	n = x;
	d = y;
	return *this;
}

bool rto::operator==(const int64 & test) const
{
	return (o == test && n == 0);
}

bool rto::operator==(const rto & target) const
{
	return (o*d + n == target.o*target.d + target.n) && (d == target.d);
}

bool rto::operator!=(const int64 & test) const
{
	return (o != test || n != 0);
}

bool rto::operator!=(const rto & target) const
{
	return (o*d + n != target.o*target.d + target.n) || (d != target.d);
}

bool rto::operator>(const int64 & test) const
{
	return (o > test) || (o == test && n > 0);
}

bool rto::operator>(const rto & test) const
{
	if (o > test.o)
		if (o > test.o + 1)
			return true;
		else
			return test.n*d - n * test.d < d*test.d;
	else if(o < test.o)
		if (o < test.o - 1)
			return false;
		else
			return test.n*d - n * test.d < -d*test.d;
	else
		return test.n*d < n * test.d;
}

bool rto::operator<(const int64 & test) const
{
	return (o < test) || (o == test && n < 0);
}

bool rto::operator<(const rto & test) const
{
	if (o > test.o)
		if (o > test.o + 1)
			return false;
		else
			return test.n*d - n * test.d > d*test.d;
	else if (o < test.o)
		if (o < test.o - 1)
			return true;
		else
			return test.n*d - n * test.d > -d * test.d;
	else
		return test.n*d > n * test.d;
}

bool rto::operator>=(const int64 & test) const
{
	return (o > test) || (o == test && n >= 0);
}

bool rto::operator>=(const rto & test) const
{
	if (o > test.o)
		if (o > test.o + 1)
			return true;
		else
			return test.n*d - n * test.d <= d*test.d;
	else if (o < test.o)
		if (o < test.o - 1)
			return false;
		else
			return test.n*d - n * test.d <= -d * test.d;
	else
		return test.n*d <= n * test.d;
}

bool rto::operator<=(const int64 & test) const
{
	return (o < test) || (o == test && n <= 0);
}

bool rto::operator<=(const rto & test) const
{
	if (o > test.o)
		if (o > test.o + 1)
			return false;
		else
			return test.n*d - n * test.d >= d*test.d;
	else if (o < test.o)
		if (o < test.o - 1)
			return true;
		else
			return test.n*d - n * test.d >= -d * test.d;
	else
		return test.n*d >= n * test.d;
}

float rto::toFloat() const
{
	return (float)o + ((float)n / (float)d);
}
