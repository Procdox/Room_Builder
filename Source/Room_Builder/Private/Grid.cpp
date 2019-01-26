#include "Grid.h"
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

grd::grd(int64 num, unchecked_t)
{
	n = num;
}

grd::grd()
{
	n = 0;
}

grd::grd(int64 num)
{
	n = safe_mul(num, grid_size);
}

grd::grd(grd && target)
{
	n = target.n;
}

grd::grd(grd const & target) {
	n = target.n;
}

grd & grd::operator=(int64 const & target)
{
	n = safe_mul(target, grid_size);

	return *this;
}

grd & grd::operator=(grd const & target)
{
	n = target.n;

	return *this;
}

grd grd::operator-() const
{
	return grd(-n, unchecked);
}

grd grd::operator+(int64 factor) const
{
	return grd(safe_add(n, safe_mul(factor, grid_size)), unchecked);
}



grd grd::operator+(const grd & target) const
{
	return grd(safe_add(n, target.n), unchecked);
}

grd grd::operator-(int64 factor) const
{
	return grd(safe_sub(n, safe_mul(factor, grid_size)), unchecked);
}

grd grd::operator-(const grd & target) const
{
	return grd(safe_sub(n, target.n), unchecked);
}

grd grd::operator*(int64 factor) const
{
	return grd(safe_mul(n, factor), unchecked);
}

grd grd::operator*(const grd & target) const
{
	return grd(safe_mul(n, target.n) / grid_size, unchecked);
}

grd grd::operator/(int64 factor) const
{
	return grd(n / factor, unchecked);
}

grd grd::operator/(const grd & target) const
{
	return grd(safe_mul(n, grid_size) / target.n, unchecked);
}

grd & grd::operator+=(int64 factor)
{
	n = safe_add(n, safe_mul(factor, grid_size));

	return *this;
}

grd & grd::operator+=(const grd & target)
{
	n = safe_add(n, target.n);

	return *this;
}

grd & grd::operator-=(int64 factor)
{
	n = safe_sub(n, safe_mul(factor, grid_size));

	return *this;
}

grd & grd::operator-=(const grd & target)
{
	n = safe_sub(n, target.n);

	return *this;
}

grd & grd::operator*=(int64 factor)
{
	n = safe_mul(n, factor);

	return *this;
}

grd & grd::operator*=(const grd & target)
{
	n = safe_mul(n, target.n) / grid_size;

	return *this;
}

grd & grd::operator/=(int64 factor)
{
	n = n / factor;

	return *this;
}

grd & grd::operator/=(const grd & target)
{
	n = safe_mul(n, grid_size) / target.n;

	return *this;
}

bool grd::operator==(const int64 & test) const
{
	int64 p = test * grid_size;

	return (n + grid_epsilon > p && n - grid_epsilon < p);
}

bool grd::operator==(const grd & target) const
{
	return (n + grid_epsilon > target.n && n - grid_epsilon < target.n);
}

bool grd::operator!=(const int64 & test) const
{
	int64 p = test * grid_size;

	return (n + grid_epsilon < p || n - grid_epsilon > p);
}

bool grd::operator!=(const grd & target) const
{
	return (n + grid_epsilon < target.n || n - grid_epsilon > target.n);
}

bool grd::operator>(const int64 & test) const
{
	return n - grid_epsilon > test * grid_size;
}

bool grd::operator>(const grd & test) const
{
	return n - grid_epsilon > test.n;
}

bool grd::operator<(const int64 & test) const
{
	return n + grid_epsilon < test * grid_size;
}

bool grd::operator<(const grd & test) const
{
	return n + grid_epsilon < test.n;
}

bool grd::operator>=(const int64 & test) const
{
	return n + grid_epsilon > test * grid_size;
}

bool grd::operator>=(const grd & test) const
{
	return n + grid_epsilon > test.n;
}

bool grd::operator<=(const int64 & test) const
{
	return n - grid_epsilon < test * grid_size;
}

bool grd::operator<=(const grd & test) const
{
	return n - grid_epsilon < test.n;
}

grd grd::round() const {
	return grd((n / grid_size));
}

float grd::toFloat() const
{
	return (float)n / (float)grid_size;
}
