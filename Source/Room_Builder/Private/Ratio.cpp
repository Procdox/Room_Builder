#include "Ratio.h"
#include "NumericLimits.h"

int64 safe_add(int64 const &x, int64 const &y) {
	if (y > 0 && TNumericLimits<int64>::Max() - y < x)
		return 0;
	if (y < 0 && TNumericLimits<int64>::Min() - y > x)
		return 0;

	return x + y;
}

int64 safe_sub(int64 const &x, int64 const &y) {
	if (y > 0 && TNumericLimits<int64>::Min() + y > x)
		return 0;
	if (y < 0 && TNumericLimits<int64>::Max() + y < x)
		return 0;
	return x - y;
}

int64 safe_mul(int64 const &x, int64 const &y) {
	if (y == 0 || x == 0)
		return 0;

	//two's complement check
	if (x == -1 && y == TNumericLimits<int64>::Min())
		return 0;
	if (y == -1) {
		if (x == TNumericLimits<int64>::Min())
			return 0;
	}
	else {
		if (y > 0) {
			if (TNumericLimits<int64>::Min() / y > x)
				return 0;
			if (TNumericLimits<int64>::Max() / y < x)
				return 0;
		}
		else{
			if (TNumericLimits<int64>::Min() / y < x)
				return 0;
			if (TNumericLimits<int64>::Max() / y > x)
				return 0;
		}
	}

	return x * y;
}

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
	return rto(safe_add(o, factor), n, d);
}



rto rto::operator+(const rto & target) const
{
	int64 t = safe_add(safe_mul(n, target.d), safe_mul(target.n, d));
	int64 y = safe_mul(target.d, d);

	int64 c = t / y;
	int64 x = safe_sub(t,safe_mul(c, y));

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
	return rto(safe_sub(o,factor), n, d);
}

rto rto::operator-(const rto & target) const
{
	int64 t = safe_sub(safe_mul(n, target.d), safe_mul(target.n, d));
	int64 y = safe_mul(target.d, d);

	int64 c = t / y;
	int64 x = safe_sub(t, safe_mul(c, y));

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
	int64 t = safe_mul(n, factor);
	int64 y = d;

	int64 c = t / y;
	int64 x = safe_sub(t, safe_mul(c, y));

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
	int64 t_a = safe_mul(o, target.n);

	int64 c_a = t_a / target.d;
	int64 r_a = safe_sub(t_a, safe_mul(c_a, target.d));

	int64 t_b = safe_mul(target.o, n);

	int64 c_b = t_b / d;
	int64 r_b = safe_sub(t_b,safe_mul(c_b, d));

	int64 t_c = safe_add(safe_add(safe_mul(r_a, d), safe_mul(r_b, target.d)), safe_mul(n, target.n));
	int64 y = safe_mul(d, target.d);
	
	int64 c_c = t_c / y;
	int64 x = t_c - safe_mul(c_c, y);

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
	int64 t = safe_add(safe_mul(o, d), n);
	int64 y = safe_mul(d, factor);

	int64 c = t / y;
	int64 x = safe_sub(t, safe_mul(c, y));

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
	int64 t_a = safe_add(safe_mul(o,d), n);
	int64 t_b = safe_add(safe_mul(target.o,target.d),target.n);

	int64 i = gcd(t_a, t_b);
	int64 j = gcd(d, target.d);

	int64 x = safe_mul((t_a / i), (target.d / j));
	int64 y = safe_mul((t_b / i), (d / j));

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
	o = safe_add(o, factor);

	return *this;
}

rto & rto::operator+=(const rto & target)
{
	int64 t = safe_add(safe_mul(n, target.d), safe_mul(target.n, d));
	int64 y = safe_mul(target.d, d);

	int64 c = t / y;
	int64 x = safe_sub(t, safe_mul(c, y));

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
	o = safe_sub(o, factor);

	return *this;
}

rto & rto::operator-=(const rto & target)
{
	int64 t = safe_sub(safe_mul(n, target.d), safe_mul(target.n, d));
	int64 y = safe_mul(target.d, d);

	int64 c = t / y;
	int64 x = safe_sub(t, safe_mul(c, y));

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
	int64 t = safe_mul(n, factor);
	int64 y = d;

	int64 c = t / y;
	int64 x = safe_sub(t, safe_mul(c, y));

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
	int64 t_a = safe_mul(o, target.n);

	int64 c_a = t_a / target.d;
	int64 r_a = safe_sub(t_a, safe_mul(c_a, target.d));

	int64 t_b = safe_mul(target.o, n);

	int64 c_b = t_b / d;
	int64 r_b = safe_sub(t_b, safe_mul(c_b, d));

	int64 t_c = safe_add(safe_add(safe_mul(r_a, d), safe_mul(r_b, target.d)), safe_mul(n, target.n));
	int64 y = safe_mul(d, target.d);

	int64 c_c = t_c / y;
	int64 x = t_c - safe_mul(c_c, y);

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
	int64 t = safe_add(safe_mul(o, d), n);
	int64 y = safe_mul(d, factor);

	int64 c = t / y;
	int64 x = safe_sub(t, safe_mul(c, y));

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
	int64 t_a = safe_add(safe_mul(o, d), n);
	int64 t_b = safe_add(safe_mul(target.o, target.d), target.n);

	int64 i = gcd(t_a, t_b);
	int64 j = gcd(d, target.d);

	int64 x = safe_mul((t_a / i), (target.d / j));
	int64 y = safe_mul((t_b / i), (d / j));

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
			return safe_sub(safe_mul(test.n,d), safe_mul(n, test.d)) < safe_mul(d, test.d);
	else if(o < test.o)
		if (o < test.o - 1)
			return false;
		else
			return safe_sub(safe_mul(test.n,d), safe_mul(n, test.d)) < safe_mul(-d,test.d);
	else
		return safe_mul(test.n, d) < safe_mul(n, test.d);
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
			return safe_sub(safe_mul(test.n, d), safe_mul(n, test.d)) > safe_mul(d, test.d);
	else if (o < test.o)
		if (o < test.o - 1)
			return true;
		else
			return safe_sub(safe_mul(test.n, d), safe_mul(n, test.d)) > safe_mul(-d, test.d);
	else
		return safe_mul(test.n, d) > safe_mul(n, test.d);
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
			return safe_sub(safe_mul(test.n, d), safe_mul(n, test.d)) <= safe_mul(d, test.d);
	else if (o < test.o)
		if (o < test.o - 1)
			return false;
		else
			return safe_sub(safe_mul(test.n, d), safe_mul(n, test.d)) <= safe_mul(-d, test.d);
	else
		return safe_mul(test.n, d) <= safe_mul(n, test.d);
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
			return safe_sub(safe_mul(test.n, d), safe_mul(n, test.d)) >= safe_mul(d, test.d);
	else if (o < test.o)
		if (o < test.o - 1)
			return true;
		else
			return safe_sub(safe_mul(test.n, d), safe_mul(n, test.d)) >= safe_mul(-d, test.d);
	else
		return safe_mul(test.n, d) >= safe_mul(n, test.d);
}

rto rto::round() const {
	return rto(o, 0, 1);
}

float rto::toFloat() const
{
	return (float)o + ((float)n / (float)d);
}
