#include "Grid.h"

grd::grd()
{
	n = 0.f;
}

grd::grd(double num)
{
	n = num;
}

grd::grd(grd && target)
{
	n = target.n;
}

grd::grd(grd const & target) {
	n = target.n;
}

grd & grd::operator=(double const & target)
{
	n = target;

	return *this;
}

grd & grd::operator=(grd const & target)
{
	n = target.n;

	return *this;
}

grd grd::operator-() const
{
	return grd(-n);
}

grd grd::operator+(double factor) const
{
	return grd(n + factor);
}



grd grd::operator+(const grd & target) const
{
	return grd(n + target.n);
}

grd grd::operator-(double factor) const
{
	return grd(n - factor);
}

grd grd::operator-(const grd & target) const
{
	return grd(n -target.n);
}

grd grd::operator*(double factor) const
{
	return grd(n * factor);
}

grd grd::operator*(const grd & target) const
{
	return grd(n * target.n);
}

grd grd::operator/(double factor) const
{
	return grd(n / factor);
}

grd grd::operator/(const grd & target) const
{
	return grd(n / target.n);
}

grd & grd::operator+=(double factor)
{
	n += factor;

	return *this;
}

grd & grd::operator+=(const grd & target)
{
	n += target.n;

	return *this;
}

grd & grd::operator-=(double factor)
{
	n -= factor;

	return *this;
}

grd & grd::operator-=(const grd & target)
{
	n -= target.n;

	return *this;
}

grd & grd::operator*=(double factor)
{
	n *= factor;

	return *this;
}

grd & grd::operator*=(const grd & target)
{
	n *= target.n;

	return *this;
}

grd & grd::operator/=(double factor)
{
	n /= factor;

	return *this;
}

grd & grd::operator/=(const grd & target)
{
	n /= target.n;

	return *this;
}

bool grd::operator==(const double & test) const
{
	return (n + grid_epsilon > test && n - grid_epsilon < test);
}

bool grd::operator==(const grd & target) const
{
	return (n + grid_epsilon > target.n && n - grid_epsilon < target.n);
}

bool grd::operator!=(const double & test) const
{
	return (n + grid_epsilon < test || n - grid_epsilon > test);
}

bool grd::operator!=(const grd & target) const
{
	return (n + grid_epsilon < target.n || n - grid_epsilon > target.n);
}

bool grd::operator>(const double & test) const
{
	return n - grid_epsilon > test;
}

bool grd::operator>(const grd & test) const
{
	return n - grid_epsilon > test.n;
}

bool grd::operator<(const double & test) const
{
	return n + grid_epsilon < test;
}

bool grd::operator<(const grd & test) const
{
	return n + grid_epsilon < test.n;
}

bool grd::operator>=(const double & test) const
{
	return n + grid_epsilon > test;
}

bool grd::operator>=(const grd & test) const
{
	return n + grid_epsilon > test.n;
}

bool grd::operator<=(const double & test) const
{
	return n - grid_epsilon < test;
}

bool grd::operator<=(const grd & test) const
{
	return n - grid_epsilon < test.n;
}